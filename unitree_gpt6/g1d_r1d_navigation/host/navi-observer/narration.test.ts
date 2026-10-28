/**
 * narration.test.ts -- regression tests for run-log notes.
 * Run (Node 22+, no install needed): node --experimental-strip-types --test host/navi-observer/narration.test.ts
 */
import assert from "node:assert/strict";
import { test } from "node:test";

import { notesOf } from "./narration.ts";

test("logs model speech and reasoning summaries; tool-call blocks are left to the sidecar", () => {
  const notes = notesOf({
    type: "assistant/message",
    data: { message: { content: [
      { type: "reasoning", text: "**Turning right toward passage**\n\nThe TV is on the left…" },
      { type: "text", text: "已进入卧室并保存「卧室」点位，接下来找沙发右侧的通道。" },
      { type: "tool-call" },
    ] } },
  });
  assert.deepEqual(notes, [
    { kind: "think", text: "Turning right toward passage" },
    { kind: "say", text: "已进入卧室并保存「卧室」点位，接下来找沙发右侧的通道。" },
  ]);
});

test("a step with only tool calls and no text yields no note", () => {
  assert.deepEqual(notesOf({ type: "assistant/message", data: { message: { content: [{ type: "tool-call" }] } } }), []);
  assert.deepEqual(notesOf({ type: "assistant/message", data: { message: { content: [{ type: "reasoning", text: "  " }] } } }), []);
});

test("logs user instructions, observations and continuations; skips host-internal runtime-context", () => {
  const msg = (kind: string, text: string) => ({
    type: "user/message", data: { message: { source: { kind }, content: [{ type: "text", text }] } },
  });
  assert.deepEqual(notesOf(msg("user", "找到厨房并标记")), [{ kind: "user", text: "找到厨房并标记" }]);
  assert.deepEqual(notesOf(msg("navi-observer", "[观测] 机器人已停稳 4.0s")), [{ kind: "observe", text: "[观测] 机器人已停稳 4.0s" }]);
  assert.equal(notesOf(msg("goal", "<goal_round>…")).at(0)?.kind, "goal");
  assert.deepEqual(notesOf(msg("runtime-context", "Current runtime context…")), []);
});

test("round errored / stopped / completed", () => {
  assert.deepEqual(notesOf({ type: "turn/end", data: { reason: { kind: "error", error: { message: "Upstream service temporarily unavailable" } } } }),
    [{ kind: "error", text: "上游出错：Upstream service temporarily unavailable" }]);
  assert.deepEqual(notesOf({ type: "turn/end", data: { reason: { kind: "aborted", reason: { kind: "user" } } } }),
    [{ kind: "stop", text: "这一轮被用户停止" }]);
  assert.match(notesOf({ type: "turn/end", data: { reason: { kind: "aborted", reason: { kind: "hook" } } } })[0]?.text ?? "", /观测层打断/);
  assert.deepEqual(notesOf({ type: "turn/end", data: { reason: { kind: "completed" } } }), []);
});

test("goal changes and other events", () => {
  assert.deepEqual(notesOf({ type: "goal/change", data: { operation: "complete" } }), [{ kind: "goal", text: "任务目标完成" }]);
  assert.deepEqual(notesOf({ type: "step/start", data: {} }), []);
  assert.deepEqual(notesOf({ type: "tool/call", data: {} }), []);
});
