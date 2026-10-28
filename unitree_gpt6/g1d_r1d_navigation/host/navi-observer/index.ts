/**
 * index.ts -- navi-observer: DSH host plugin ("observation push while the model is thinking").
 *
 * Problem: while the model thinks, the robot may already have stopped or been blocked by the radar reflex layer,
 * and the model cannot see it until its next tool call, wasting a full inference. While the model is running, this
 * plugin polls navi_mcp_server's read-only status endpoint (:3082) once per second; policy.ts decides whether a
 * push is worthwhile, and if so agent.steer() puts one observation line into the running turn's inbox (seen at the
 * next step boundary). If the same obstacle keeps blocking for over interruptS, it escalates to agent.cancel() to
 * abort the stale inference, then sends a followup once idle so the model is sure to see it.
 *
 * Run log: model speech, reasoning summaries, user instructions, [观测] messages and stopped/errored rounds are
 * extracted via narration.ts and POSTed to the status endpoint's /runlog (the sidecar logs tool calls itself),
 * shown in order below the camera view and saved to disk.
 *
 * Also a host-side fix for DSH goal continuation (policy in continuation.ts):
 *   - after an upstream-error round, if the goal is still active but disarmed, re-arm it after a backoff
 *     (10s/30s/60s, at most 3 times);
 *   - while a goal is armed, the interrupt above becomes a steer: cancel would make goal-round-driver disarm
 *     continuation and the task would stall halfway (the very problem being fixed). Human cancellations
 *     (UI stop, /goal pause) are never touched.
 *
 * Boundaries (safety and restraint matter equally):
 *   - Read-only: calls no robot tools and issues no motion commands. Its only actions are adding session messages /
 *     interrupting inference, re-arming the model's own goal after upstream failures, and appending display-only
 *     run-log notes to the status endpoint.
 *   - Navi sessions only (scope.ts): a host-layer plugin sees every session in the process; someone else's coding
 *     session must never receive an observation.
 *   - Polls only while agent.status === 'running': pushing while the model is not thinking would just start a
 *     needless new inference.
 *   - Status endpoint unreachable (robot not up / wrong port): return silently; never let it affect the session.
 *   - try/catch throughout: any plugin error goes to the log only and must not bubble into the agent's driver loop.
 *
 * Switches (env vars; the port must match the sidecar's navi.observer.port, default 3082):
 *   NAVI_OBSERVER_PORT=0        Disable pushes (the plugin returns at startup, no timers)
 *   NAVI_OBSERVER_INTERVAL_MS   Poll interval, default 1000
 *   NAVI_OBSERVER_TIMEOUT_MS    Per-request HTTP timeout, default 900
 *   NAVI_OBSERVER_IMAGES=0      No images (text lines only)
 *   NAVI_OBSERVER_IDLE_S        Notify after the base has been still this long, default 4 (0 = immediately)
 *   NAVI_OBSERVER_INTERRUPT_IDLE_S  Also interrupt inference after this many seconds still, default 0 = notify only
 */
import type { Context } from "@deepseek-ai/cordis";
import type { Agent } from "@deepseek-ai/dsh-agent";
import { boundContextSummary, createUserMessage } from "@deepseek-ai/dsh-llm";
import type { ContentBlock } from "@deepseek-ai/dsh-llm";

import { canRearm, ErrorRetry, goalArmed, type GoalLike } from "./continuation.ts";
import { isBlocked, ObservePolicy, type Decision, type ObsState } from "./policy.ts";
import { notesOf, type SessionEventLike } from "./narration.ts";
import { fetchFrame, fetchState, postNote, resolveBase } from "./poll.ts";
import { isNaviSession, presetFromLog } from "./scope.ts";

export const name = "navi-observer";
export const inject = ["agents"];

/** Give up permanently after this many failed Navi checks (avoid scanning other long sessions' logs every second). */
const GIVE_UP_CHECKS = 60;

/** Max wait for the driver to go idle after an interrupt before falling back to steer delivery. */
const INTERRUPT_WAIT_MS = 5000;

interface Options {
  base: string;
  intervalMs: number;
  timeoutMs: number;
  images: boolean;
  interruptIdleS: number;
  idleNotifyS: number;
}

/** The subset of ctx.goals (@deepseek-ai/dsh-goal) used here; the plugin does not depend on that package's types. */
interface GoalsLike {
  get(agent: Agent): GoalLike | undefined;
  resume(agent: Agent, ref: { id: string; revision: number }): unknown;
}

/** The turn/end session/event this plugin cares about. */
interface TurnEndEvent {
  type: string;
  data?: { reason?: { kind?: string; error?: { message?: string } } };
}

interface Runtime {
  policy: ObservePolicy;
  retry: ErrorRetry;
  /** undefined = undecided, keep checking; true/false = decided. */
  navi: boolean | undefined;
  checks: number;
  wasBlocked: boolean;
  /** When the base became still during this inference (ms); the plugin measures still time itself, see idle_s in policy.ts. */
  stillSince: number | undefined;
}

function envInt(key: string, fallback: number, min: number, max: number): number {
  const raw = process.env[key];
  if (raw === undefined || raw === "") return fallback;
  const value = Number(raw);
  if (!Number.isFinite(value)) return fallback;
  return Math.min(max, Math.max(min, Math.trunc(value)));
}

function loadOptions(): Options | null {
  const base = resolveBase();                       // NAVI_OBSERVER_PORT, default 3082; <= 0 -> null
  if (base === null) return null;
  return {
    base,
    intervalMs: envInt("NAVI_OBSERVER_INTERVAL_MS", 1000, 200, 10000),
    timeoutMs: envInt("NAVI_OBSERVER_TIMEOUT_MS", 900, 200, 10000),
    images: process.env.NAVI_OBSERVER_IMAGES !== "0",
    interruptIdleS: envInt("NAVI_OBSERVER_INTERRUPT_IDLE_S", 0, 0, 600),
    idleNotifyS: envInt("NAVI_OBSERVER_IDLE_S", 4, 0, 600),
  };
}

function describe(error: unknown): string {
  return error instanceof Error ? `${error.name}: ${error.message}` : String(error);
}

export function apply(ctx: Context): void {
  const opts = loadOptions();
  if (opts === null) {
    ctx.logger.info("navi-observer: observation push disabled (NAVI_OBSERVER_PORT <= 0)");
    return;
  }

  const runtimes = new Map<Agent, Runtime>();
  /** Broadcast by agent-preset-registry (agent-preset/selected) on preset mount/switch; covers cases missing from the header. */
  const presetBySession = new Map<string, string>();
  /** Pending auto-continue timers: cleared on unload so no session is touched afterwards. */
  const retryTimers = new Set<ReturnType<typeof setTimeout>>();
  let busy = false;
  /** Serial chain for run-log notes. */
  let noteChain: Promise<void> = Promise.resolve();

  const stateFor = (agent: Agent): Runtime => {
    let runtime = runtimes.get(agent);
    if (runtime === undefined) {
      runtime = {
        policy: new ObservePolicy({ interruptIdleS: opts.interruptIdleS, idleNotifyS: opts.idleNotifyS }),
        retry: new ErrorRetry(),
        navi: undefined, checks: 0, wasBlocked: false, stillSince: undefined,
      };
      runtimes.set(agent, runtime);
    }
    return runtime;
  };

  /** Assembly-layer signal: preset in header -> broadcast selection -> session log (appended before our first poll). */
  const presetOf = (agent: Agent): string | undefined => {
    const fromHeader = agent.session.header.agentPreset;
    if (fromHeader !== undefined) return fromHeader;
    const broadcast = presetBySession.get(agent.session.id);
    if (broadcast !== undefined) return broadcast;
    try {
      return presetFromLog(agent.session.snapshotEvents()) ?? undefined;
    } catch {
      return undefined;                             // Unreadable log: retry on the next tick
    }
  };

  const detect = (agent: Agent, runtime: Runtime): boolean => {
    if (runtime.navi !== undefined) return runtime.navi;
    const probe = {
      presetId: presetOf(agent),
      systemText: agent.session.requestHeader()?.system ?? undefined,
    };
    if (isNaviSession(probe)) {
      runtime.navi = true;
      ctx.logger.info(`navi-observer: taking over observation push for session ${agent.session.id}`);
      return true;
    }
    runtime.checks += 1;
    if (runtime.checks >= GIVE_UP_CHECKS) runtime.navi = false;
    return false;
  };

  /** The session's goal; undefined when this composition has no goal service. */
  const goalsService = (): GoalsLike | undefined =>
    (ctx as unknown as { get(name: string): unknown }).get("goals") as GoalsLike | undefined;
  const goalOf = (agent: Agent): GoalLike | undefined => {
    try {
      return goalsService()?.get(agent);
    } catch {
      return undefined;
    }
  };

  /** After an upstream-error round: once the backoff elapses, if still idle and the goal is active but disarmed, re-arm it (the driver then starts a round). */
  const onTurnEnd = (agent: Agent, runtime: Runtime, event: TurnEndEvent): void => {
    const kind = event.data?.reason?.kind ?? "";
    const waitMs = runtime.retry.onTurnEnd(kind);
    if (kind === "error" && waitMs === null) {
      ctx.logger.warn(`navi-observer: ${String(runtime.retry.consecutive)} consecutive upstream failures, auto-continue stopped → ${agent.session.id}`);
    }
    if (waitMs === null || !canRearm(goalOf(agent))) return;
    const attempt = runtime.retry.consecutive;
    const handle = setTimeout(() => {
      retryTimers.delete(handle);
      try {
        if (ctx.agents.get(agent.session.id) !== agent || agent.status !== "idle") return;   // A human has taken over
        const goal = goalOf(agent);
        const goals = goalsService();
        if (goals === undefined || goal === undefined || !canRearm(goal)) return;
        goals.resume(agent, { id: goal.id, revision: goal.revision });
        ctx.logger.info(`navi-observer: auto-continue #${String(attempt)} after upstream error (${event.data?.reason?.error?.message ?? "?"}) → ${agent.session.id}`);
      } catch (error) {
        ctx.logger.warn(`navi-observer: auto-continue failed: ${describe(error)}`);
      }
    }, waitMs);
    handle.unref?.();
    retryTimers.add(handle);
  };

  const saveFrame = async (): Promise<ContentBlock | undefined> => {
    const store = ctx.get("attachments");
    if (store === undefined) return undefined;      // No attachment service in this composition: text only
    const bytes = await fetchFrame(opts.base, opts.timeoutMs);
    if (bytes === null) return undefined;
    try {
      const [ref] = await store.saveImages([{ data: bytes, mediaType: "image/jpeg", name: "frame.jpg" }]);
      return ref === undefined ? undefined : { type: "image", attachment: ref };
    } catch (error) {
      // Image rejected (type check / size limit): send text only, the observation line is still useful
      ctx.logger.warn(`navi-observer: failed to save frame attachment (sending text only): ${describe(error)}`);
      return undefined;
    }
  };

  const deliver = async (agent: Agent, decision: Decision, mode: "steer" | "followup"): Promise<void> => {
    const content: ContentBlock[] = [{ type: "text", text: decision.text }];
    if (decision.withImage && opts.images) {
      const image = await saveFrame();
      if (image !== undefined) content.push(image);
    }
    const message = createUserMessage({
      content,
      // Producer-owned source kind (= this plugin's name). Session format v4 rejects the v3-era
      // {kind:"plugin",plugin:name} ("producer-owned source kind"): in v4 each producer uses its own
      // module name as kind, with no shared plugin fallback (see dsh-llm MessageSourceMap).
      source: {
        kind: name,
        form: "notice",
        summary: boundContextSummary(decision.text),
      },
    });
    if (mode === "followup") agent.followup(message);
    else agent.steer(message);
    ctx.logger.info(`navi-observer: push ${decision.reason}${decision.interrupt ? " (interrupt)" : ""} → ${agent.session.id}`);
  };

  /**
   * Urgent path: the inference is stale (blocked long enough, or rest interrupt enabled). Abort it and followup
   * only once the driver has actually stopped: cancel lands asynchronously, and a message delivered before it takes
   * effect may be swallowed by the abort. If the driver never goes idle (e.g. other queued messages), fall back to
   * steer after INTERRUPT_WAIT_MS (it must be running then, and steer is picked up at the next step boundary).
   */
  const interrupt = (agent: Agent, decision: Decision): void => {
    try {
      agent.cancel({ kind: "hook", reason: `navi-observer: ${decision.reason} 观测判定这轮推理已过时` }, { keepInbox: true });
    } catch (error) {
      ctx.logger.warn(`navi-observer: interrupt failed: ${describe(error)}`);
      return;
    }
    const idle = agent.whenIdle().then(() => true);
    const timeout = new Promise<boolean>((resolve) => {
      const handle = setTimeout(() => { resolve(false); }, INTERRUPT_WAIT_MS);
      handle.unref?.();
    });
    void Promise.race([idle, timeout])
      .then(async (wentIdle) => {
        if (ctx.agents.get(agent.session.id) !== agent) return;      // Session has moved on
        await deliver(agent, decision, wentIdle ? "followup" : "steer");
      })
      .catch((error: unknown) => {
        ctx.logger.warn(`navi-observer: delivery after interrupt failed: ${describe(error)}`);
      });
  };

  const tick = async (): Promise<void> => {
    if (busy) return;                                 // Previous tick still pending: skip, do not queue
    busy = true;
    try {
      const state: ObsState | null = await fetchState(opts.base, opts.timeoutMs);
      if (state === null) return;                     // Status endpoint unreachable: silent
      const now = Date.now();
      for (const agent of ctx.agents.list()) {
        if (ctx.agents.get(agent.session.id) !== agent) continue;    // Stale reference
        const runtime = stateFor(agent);
        if (!detect(agent, runtime)) continue;
        if (agent.status !== "running") {             // Model not thinking: do not accumulate still time
          runtime.stillSince = undefined;
          continue;
        }

        // Still time is measured here (only while the model runs and the base is still): the status endpoint's
        // since_submit_s is time since the last submission, which is wrong right after a long sequence finishes.
        const parked = !state.navigation?.active && state.seq?.active === false && state.driver?.moving !== true;
        if (parked && runtime.stillSince === undefined) runtime.stillSince = now;
        if (!parked) runtime.stillSince = undefined;
        const observed: ObsState = runtime.stillSince === undefined
          ? state
          : { ...state, idle_s: (now - runtime.stillSince) / 1000 };

        const blocked = isBlocked(state);
        if (!blocked && runtime.wasBlocked) runtime.policy.reset();  // Obstacle cleared -> the next block can report immediately
        runtime.wasBlocked = blocked;

        const decision = runtime.policy.consider(observed, now);
        if (!decision.push) continue;
        // Do not cancel while a goal is armed: cancel would disarm continuation (see continuation.ts)
        if (decision.interrupt && agent.status === "running" && !goalArmed(goalOf(agent))) interrupt(agent, decision);
        else await deliver(agent, decision, "steer");
      }
    } catch (error) {
      ctx.logger.warn(`navi-observer: poll failed: ${describe(error)}`);
    } finally {
      busy = false;
    }
  };

  ctx.effect(function* () {
    const timer = setInterval(() => { void tick(); }, opts.intervalMs);
    const stopPreset = ctx.on("agent-preset/selected", (sessionId, agentPreset) => {
      presetBySession.set(sessionId, agentPreset);
    });
    const stopTurnEnd = ctx.on("session/event", (session: { id: string }, event: TurnEndEvent & SessionEventLike) => {
      try {
        const notes = notesOf(event);
        if (event.type !== "turn/end" && notes.length === 0) return;
        const agent = ctx.agents.get(session.id);
        if (agent === undefined || agent.session.id !== session.id) return;
        const runtime = stateFor(agent);
        if (!detect(agent, runtime)) return;
        // Post notes in event order through one chain so concurrent requests cannot reorder them; a failure drops only that note
        for (const note of notes) noteChain = noteChain.then(() => postNote(opts.base, note, opts.timeoutMs)).then(() => undefined);
        if (event.type === "turn/end") onTurnEnd(agent, runtime, event);
      } catch (error) {
        ctx.logger.warn(`navi-observer: session event handling failed: ${describe(error)}`);
      }
    });
    const stopDisposed = ctx.on("agent/disposed", ({ agent }) => {
      presetBySession.delete(agent.session.id);
      runtimes.delete(agent);
    });
    ctx.logger.info(`navi-observer: started (status endpoint ${opts.base}, every ${String(opts.intervalMs)}ms, images ${opts.images ? "on" : "off"})`);

    yield async () => {
      clearInterval(timer);
      for (const handle of retryTimers) clearTimeout(handle);
      retryTimers.clear();
      stopTurnEnd();
      stopPreset();
      stopDisposed();
      runtimes.clear();
      presetBySession.clear();
    };
  }, "navi-observer lifecycle");
}
