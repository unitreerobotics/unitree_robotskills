/**
 * policy.test.ts -- regression tests for ObservePolicy.
 * Run (Node 22+, no install needed): node --experimental-strip-types --test host/navi-observer/policy.test.ts
 */
import assert from "node:assert/strict";
import { test } from "node:test";

import { ObservePolicy, type ObsState } from "./policy.ts";

const T0 = 1_000_000;

function stopped(idleS: number, extra: Partial<ObsState> = {}): ObsState {
  return {
    driver: { connected: true, moving: false },
    seq: { active: false, remaining_s: 0, since_submit_s: idleS + 3, reflex: { reason: "ok", clear_m: 1.5, blocked_s: 0 } },
    lidar: { ok: true, front_m: 1.5 },
    idle_s: idleS,
    ...extra,
  } as ObsState;
}

const moving: ObsState = {
  driver: { connected: true, moving: false },
  seq: { active: true, remaining_s: 2, since_submit_s: 1, reflex: { reason: "ok", clear_m: 3, blocked_s: 0 } },
} as ObsState;

test("model already holds a post-rest frame: no at-rest push", () => {
  // submit_sequence observe_after_s >= motion duration, so the returned frame is post-rest
  const p = new ObservePolicy();
  for (const idle of [5, 30, 55, 80]) {
    const d = p.consider(stopped(idle, { frame: { id: 3, age_s: idle, moved_since_s: 0 } }), T0 + idle * 1000);
    assert.equal(d.push, false);
  }
});

test("frame taken while moving: notify once after rest, never repeat", () => {
  const p = new ObservePolicy();
  const frame = { id: 3, age_s: 9, moved_since_s: 2.5 };
  assert.equal(p.consider(stopped(2, { frame }), T0 + 2000).push, false);       // idleNotifyS not reached yet
  const first = p.consider(stopped(5, { frame }), T0 + 5000);
  assert.equal(first.push, true);
  assert.equal(first.reason, "idle");
  assert.match(first.text, /已停稳 5\.0s.*前方净空 1\.50m/);
  for (const idle of [30, 55, 80, 300]) {                                        // previously repeated every 25s
    assert.equal(p.consider(stopped(idle, { frame }), T0 + idle * 1000).push, false);
  }
});

test("base moves then stops again: counts as a new rest", () => {
  const p = new ObservePolicy();
  const frame = { id: 3, age_s: 9, moved_since_s: 2.5 };
  assert.equal(p.consider(stopped(5, { frame }), T0 + 5000).push, true);
  assert.equal(p.consider(moving, T0 + 40_000).push, false);
  assert.equal(p.consider(stopped(5, { frame: { id: 4, age_s: 8, moved_since_s: 3 } }), T0 + 50_000).push, true);
});

test("old server state without frame: still notifies, but only once", () => {
  const p = new ObservePolicy();
  assert.equal(p.consider(stopped(5), T0 + 5000).push, true);
  assert.equal(p.consider(stopped(30), T0 + 30_000).push, false);
});

test("explicitly enabled rest interrupt ignores the notify-once rule", () => {
  const p = new ObservePolicy({ interruptIdleS: 20 });
  const frame = { id: 3, age_s: 9, moved_since_s: 2.5 };
  assert.equal(p.consider(stopped(5, { frame }), T0 + 5000).interrupt, false);
  const d = p.consider(stopped(21, { frame }), T0 + 21_000);
  assert.equal(d.push, true);
  assert.equal(d.interrupt, true);
});

test("reflex-block notices: push when due, again after the dedup window", () => {
  const p = new ObservePolicy();
  const blocked = (s: number): ObsState => ({
    driver: { connected: true, moving: false },
    seq: { active: true, remaining_s: 3, since_submit_s: 2, reflex: { reason: "blocked", clear_m: 0.3, blocked_s: s } },
    frame: { id: 3, age_s: 1, moved_since_s: 0 },
  } as ObsState);
  assert.equal(p.consider(blocked(0.5), T0).push, false);
  const d = p.consider(blocked(1.5), T0 + 1000);
  assert.equal(d.push, true);
  assert.match(d.text, /前方 0\.30m 有障碍/);
  assert.equal(p.consider(blocked(5), T0 + 5000).push, false);                   // within dedup window
  assert.equal(p.consider(blocked(9), T0 + 9000).interrupt, true);              // blocked long enough: escalate to interrupt
});

test("sequence stopped: frozen blocked reading neither pushes nor interrupts", () => {
  // After the sequence stops, reflex freezes at the last blocked tick; acting on it would interrupt every 30s and the model could never move
  const p = new ObservePolicy();
  const frame = { id: 9, age_s: 40, moved_since_s: 0 };
  const parked = (t: number): ObsState => ({
    driver: { connected: true, moving: false },
    seq: { active: false, remaining_s: 0, since_submit_s: 60 + t, reflex: { reason: "blocked", clear_m: 0.03, blocked_s: 49.8 } },
    lidar: { ok: true, front_m: 0.05, corridor_front_m: 0.03 },
    idle_s: 60 + t,
    frame,
  } as ObsState);
  for (const t of [0, 30, 60, 90, 120]) {
    const d = p.consider(parked(t), T0 + t * 1000);
    assert.equal(d.interrupt, false);
    assert.notEqual(d.reason, "blocked");
  }
});

test("rest notice prefers travel-corridor clearance and prompts planning the next segment", () => {
  const p = new ObservePolicy();
  const frame = { id: 3, age_s: 9, moved_since_s: 2.5 };
  const d = p.consider(stopped(5, { frame, lidar: { ok: true, front_m: 0.8, corridor_front_m: 4.2 } }), T0 + 5000);
  assert.equal(d.push, true);
  assert.match(d.text, /行进走廊净空 4\.20m/);
  assert.doesNotMatch(d.text, /前方净空 0\.80m/);                // the 0.8m side-wall sector reading is not reported
  assert.match(d.text, /直接规划下一段/);
  const q = new ObservePolicy();
  const open = q.consider(stopped(5, { frame, lidar: { ok: true, front_m: 0.8, corridor_front_m: null } }), T0 + 5000);
  assert.match(open.text, /行进走廊里没有障碍回波/);
});
