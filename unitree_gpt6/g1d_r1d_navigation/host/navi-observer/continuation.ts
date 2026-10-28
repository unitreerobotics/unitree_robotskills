/**
 * continuation.ts -- host-side fix for DSH goal continuation (pure functions, no DSH dependency, `node --test`-able).
 *
 * Background: Navi sessions use DSH goal + goal-round-driver to stop the model from quitting mid-task: the model
 * calls create_goal when it starts, and whenever it goes idle with the goal still active the driver starts another
 * round. The driver is fail-closed:
 *   - a round ending in error (upstream 503 / gateway 403 / stream cut, emits agent/error) -> continuation disarmed;
 *   - a cancelled round (by anyone) -> continuation disarmed, and the goal is paused if it was a continuation round.
 * That is right for a human pressing stop, but in practice most interruptions are transient upstream failures
 * ("The service is temporarily unavailable" / "Upstream access forbidden") that would each need a manual retry.
 * This module decides:
 *   1. after an upstream-error round, if the goal is still active but disarmed, the host re-arms it after a
 *      backoff (bounded number of attempts);
 *   2. while a goal is armed, the observer's own interrupt becomes a steer, not a cancel, which would disarm it.
 * Human cancellations (UI stop, /goal pause) are never touched.
 */

export interface GoalLike {
  id: string;
  revision: number;
  /** active / paused / blocked / complete */
  phase: string;
  /** armed = auto-continue when idle; disarmed = off (until re-armed by a human or the host) */
  activation?: string;
}

/** Goal active and armed: the current round must not be cancelled. */
export function goalArmed(goal: GoalLike | undefined): boolean {
  return goal !== undefined && goal.phase === "active" && goal.activation === "armed";
}

/** Goal still active but disarmed: the host may re-arm it. Paused/blocked/complete goals are human or model decisions; leave them. */
export function canRearm(goal: GoalLike | undefined): boolean {
  return goal !== undefined && goal.phase === "active" && goal.activation !== "armed";
}

export interface RetryOpts {
  /** Wait (seconds) before continuing after the n-th consecutive failure; beyond the list = stop auto-continuing */
  backoffS?: number[];
}

/** Per-session upstream failure counter: consecutive failures follow the backoff table; a completed round resets it. */
export class ErrorRetry {
  private failures = 0;
  private readonly backoffS: number[];

  constructor(opts: RetryOpts = {}) {
    this.backoffS = opts.backoffS ?? [10, 30, 60];
  }

  /**
   * Call at turn end. kind = turn/end reason.kind.
   * Returns milliseconds to wait before re-arming; null = do not continue (completed / human cancel / limit reached).
   */
  onTurnEnd(kind: string): number | null {
    if (kind === "completed") {
      this.failures = 0;
      return null;
    }
    if (kind !== "error") return null;
    this.failures += 1;
    const wait = this.backoffS[this.failures - 1];
    return wait === undefined ? null : wait * 1000;
  }

  get consecutive(): number {
    return this.failures;
  }
}
