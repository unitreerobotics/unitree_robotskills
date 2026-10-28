/**
 * policy.ts -- decision core of navi-observer (pure functions, no DSH dependency, runs under `node --test`).
 *
 * Translates the robot state from `/state` into "push an observation to the thinking model now?" and
 * "interrupt the current inference?". The plugin shell (index.ts) only polls, calls this module and delivers
 * the decision to the inbox.
 *
 * Push triggers (all while the model is thinking and cannot see the robot):
 *   1. The robot has come to rest (sequence finished / watchdog starvation stop) while the model still thinks:
 *      state the fact and remind it to plan the next segment if the task is unfinished. Pushed once per rest; not
 *      pushed if the model's latest frame was already taken at rest (frame.moved_since_s ~ 0). Repeating every 25s
 *      would force another reply per push (one inference often takes 15-30s), so the next push lands as soon as it
 *      answers and the turn never converges;
 *   2. The radar reflex layer blocks forward motion (turning is not implied safe): resubmitting the same forward
 *      sequence is pointless;
 *   3. The model has thought longer than idleNotifyS with the robot motionless: remind it to check the task and
 *      pre-check state.
 * Interrupt: the same obstacle keeps blocking for over interruptS -- the inference is stale; abort and wake it.
 * (Resting long enough can also interrupt, but only with interruptIdleS explicitly enabled: by default it only
 * notifies, since the model's frame is usually current and cutting inference may not pay off; tune on hardware.)
 */

export interface Reflex {
  reason?: string;              // ok / slow / blocked / no_data / disabled
  clear_m?: number | null;
  blocked_s?: number;
}

export interface ObsState {
  ts?: number;
  latency_est_s?: number;
  /**
   * How long the base has been still (s), measured by the plugin: accumulates only while the model runs and the
   * base is still. Preferred over seq.since_submit_s (time since the last submission), which right after a long
   * sequence would overstate the still time.
   */
  idle_s?: number;
  /** corridor_front_m: clearance in a body-width travel corridor (key present only with the corridor enabled; null = no echo in it). */
  lidar?: {
    ok?: boolean; enabled?: boolean; state?: string; front_m?: number | null;
    corridor_front_m?: number | null; error?: string | null;
  };
  seq?: {
    active?: boolean; remaining_s?: number; starved?: boolean;
    since_submit_s?: number | null; cmd?: { vx?: number; vyaw?: number }; reflex?: Reflex;
  };
  navigation?: { active?: boolean; state?: string };
  /** Latest frame given to the model: seconds of velocity sequence executed since. ~0 = the model sees the current state. */
  frame?: { id?: number; age_s?: number; moved_since_s?: number };
  driver?: { connected?: boolean; moving?: boolean; position?: number[] | null; yaw?: number | null };
}

export interface Decision {
  push: boolean;
  interrupt: boolean;           // true = abort the running inference (urgent cases only)
  reason: string;               // For logging / dedup
  text: string;                 // One-line Chinese observation injected for the model
  withImage: boolean;
}

export interface PolicyOpts {
  minIntervalS?: number;        // Minimum interval between pushes (default 6)
  idleNotifyS?: number;         // Notify after the robot has rested this long (default 4)
  blockedNotifyS?: number;      // Notify after being blocked this long (default 1.2)
  interruptS?: number;          // Escalate to interrupting inference after being blocked this long (default 8)
  repeatS?: number;             // Dedup window for the same observation kind (default 25)
  imageEveryS?: number;         // Image attachment cooldown (default 12)
  interruptIdleS?: number;      // Also interrupt after resting this long (default 0 = off, see below)
  interruptCooldownS?: number;  // Minimum interval between interrupts, debounce (default 30)
}

const DEFAULTS: Required<PolicyOpts> = {
  minIntervalS: 6, idleNotifyS: 4, blockedNotifyS: 1.2,
  interruptS: 8, repeatS: 25, imageEveryS: 12,
  interruptIdleS: 0, interruptCooldownS: 30,
};

/** Clearance note for the rest reminder: use the travel-corridor reading if present (ignores side walls), else the +-30 deg sector. */
function clearanceText(s: ObsState): string {
  const lidar = s.lidar;
  if (lidar == null || lidar.ok === false) return "";
  if ("corridor_front_m" in lidar) {
    return lidar.corridor_front_m == null
      ? "，正前方行进走廊里没有障碍回波"
      : `，正前方行进走廊净空 ${Number(lidar.corridor_front_m).toFixed(2)}m`;
  }
  return lidar.front_m != null ? `，前方净空 ${Number(lidar.front_m).toFixed(2)}m` : "";
}

/**
 * Whether the reflex layer is currently blocking (including "no lidar data" blocks under require).
 * Only trust readings while a sequence is running: once it stops (finished / watchdog), reflex freezes at the last
 * tick. Otherwise a frozen "blocked at 0.03m" reading would push every 25s and interrupt every 30s, even
 * interrupting the user's retry, so the model could never move.
 */
export function isBlocked(s: ObsState): boolean {
  if (s.seq?.active !== true) return false;
  const reason = s.seq.reflex?.reason;
  return reason === "blocked" || reason === "no_data";
}

export class ObservePolicy {
  private o: Required<PolicyOpts>;
  private lastPushAt = -Infinity;
  private lastImageAt = -Infinity;
  private lastInterruptAt = -Infinity;
  private lastByReason = new Map<string, number>();
  /** This rest was already notified; cleared once the base moves again. */
  private idleNotified = false;

  constructor(opts: PolicyOpts = {}) {
    this.o = { ...DEFAULTS, ...opts };
  }

  /** nowMs is injectable for tests; defaults to Date.now(). */
  consider(s: ObsState, nowMs?: number): Decision {
    const now = nowMs ?? Date.now();
    const none: Decision = { push: false, interrupt: false, reason: "none", text: "", withImage: false };
    if (!s || !s.driver || s.driver.connected === false) return none;

    // Map planner owns the base, including stopping/fault: never suggest velocity takeover.
    if (s.navigation?.active) return none;
    const seq = s.seq ?? {};
    const reflex = seq.reflex ?? {};
    const blocked = isBlocked(s);
    const blockedS = Number(reflex.blocked_s ?? 0);
    const idleS = seq.active ? 0 : Number(s.idle_s ?? seq.since_submit_s ?? 0);
    const still = seq.active === false && s.driver.moving !== true;
    if (!still) this.idleNotified = false;
    // Base has not moved since the model's latest frame (0.3s slack for trailing ticks): it already sees the current state
    const frameCurrent = s.frame != null && Number(s.frame.moved_since_s ?? Infinity) < 0.3;

    let reason = "";
    let text = "";
    if (blocked && blockedS >= this.o.blockedNotifyS) {
      reason = reflex.clear_m == null ? "nodata" : "blocked";
      text = reflex.clear_m == null
        ? `[观测] 雷达无有效净空数据，前进被拦 ${blockedS.toFixed(1)}s——先 stop 恢复有效观测；反射层不拦转向不代表转向安全。`
        : `[观测] 前方 ${Number(reflex.clear_m).toFixed(2)}m 有障碍，前进被反射层拦住 ${blockedS.toFixed(1)}s`
          + `——继续提交前进段无效，先停下核对周边净空，再决定是否绕行；不盲转或后退。`;
    } else if (still && idleS >= this.o.idleNotifyS) {
      // Poller idle_s also accumulates before any command has ever been sent.
      // No submission history means there is no sequence completion to report.
      if (typeof seq.since_submit_s !== "number" || !Number.isFinite(seq.since_submit_s)) return none;
      reason = "idle";
      // State facts only and nudge toward the next segment; cautionary wording made the model re-check
      // in place even in an empty corridor.
      text = `[观测] 机器人已停稳 ${idleS.toFixed(1)}s${clearanceText(s)}。`
        + `任务还没做完就按你手里最新的帧直接规划下一段——正前方走廊是空的就提交前进段，`
        + `不必再 observe / get_status；前方确有阻碍或地图导航进行中时除外。`;
    } else {
      return none;
    }

    // Interrupt: blocked long enough (default 8s), or -- only with interruptIdleS enabled -- rested long enough.
    // The latter is off by default: the model's frame is usually current, so interrupting would discard useful
    // inference; tune on hardware (NAVI_OBSERVER_INTERRUPT_IDLE_S).
    const wantInterrupt = (blocked && blockedS >= this.o.interruptS)
      || (still && this.o.interruptIdleS > 0 && idleS >= this.o.interruptIdleS);
    // No back-to-back interrupts: the model re-infers after one, and interrupting again on the same situation just oscillates
    const interrupt = wantInterrupt && now - this.lastInterruptAt >= this.o.interruptCooldownS * 1000;
    // "At rest" is informational: skip if already notified or the model's frame is post-rest (unless rest interrupt is enabled)
    if (reason === "idle" && !interrupt && (this.idleNotified || frameCurrent)) return none;
    // Interrupts bypass dedup/rate limiting, so escalating the same obstacle from notice to interrupt is not suppressed
    if (!interrupt) {
      const dupAt = this.lastByReason.get(reason);
      if (dupAt !== undefined && now - dupAt < this.o.repeatS * 1000) return none;
      if (now - this.lastPushAt < this.o.minIntervalS * 1000) return none;
    }

    if (reason === "idle") this.idleNotified = true;
    this.lastByReason.set(reason, now);
    this.lastPushAt = now;
    if (interrupt) this.lastInterruptAt = now;
    const withImage = now - this.lastImageAt >= this.o.imageEveryS * 1000;
    if (withImage) this.lastImageAt = now;
    return { push: true, interrupt, reason, text, withImage };
  }

  /** Call after an interrupt is delivered and the model restarts: clears rate limits so the next round can report at once (interrupt cooldown is kept). */
  reset(): void {
    this.lastPushAt = -Infinity;
    this.lastByReason.clear();
  }
}
