/**
 * continuation.test.ts -- regression tests for the goal-continuation fix.
 * Run (Node 22+, no install needed): node --experimental-strip-types --test host/navi-observer/continuation.test.ts
 */
import assert from "node:assert/strict";
import { test } from "node:test";

import { canRearm, ErrorRetry, goalArmed } from "./continuation.ts";

const goal = (phase: string, activation: string) => ({ id: "g1", revision: 3, phase, activation });

test("host re-arms only active goals whose continuation is disarmed", () => {
  assert.equal(canRearm(goal("active", "disarmed")), true);
  assert.equal(canRearm(goal("active", "armed")), false);         // already armed
  assert.equal(canRearm(goal("paused", "disarmed")), false);      // paused by human or model: leave it
  assert.equal(canRearm(goal("blocked", "disarmed")), false);
  assert.equal(canRearm(goal("complete", "disarmed")), false);
  assert.equal(canRearm(undefined), false);
});

test("armed goal: current round must not be cancelled", () => {
  assert.equal(goalArmed(goal("active", "armed")), true);
  assert.equal(goalArmed(goal("active", "disarmed")), false);
  assert.equal(goalArmed(undefined), false);
});

test("upstream errors follow the backoff table, at most three; a completed round resets", () => {
  const r = new ErrorRetry();
  assert.equal(r.onTurnEnd("error"), 10_000);
  assert.equal(r.onTurnEnd("error"), 30_000);
  assert.equal(r.onTurnEnd("error"), 60_000);
  assert.equal(r.onTurnEnd("error"), null);                        // fourth: no more auto-continue
  assert.equal(r.onTurnEnd("completed"), null);
  assert.equal(r.onTurnEnd("error"), 10_000);                      // counting restarts after reset
});

test("human cancel (aborted) neither continues nor resets the failure count", () => {
  const r = new ErrorRetry();
  r.onTurnEnd("error");
  assert.equal(r.onTurnEnd("aborted"), null);
  assert.equal(r.consecutive, 1);
});
