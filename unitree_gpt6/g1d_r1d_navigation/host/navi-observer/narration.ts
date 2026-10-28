/**
 * narration.ts -- the "what the model said" half of the run log (pure functions, no DSH dependency, `node --test`-able).
 *
 * The MCP sidecar logs tool calls and results itself (src/navi_agent/services/runlog.py). Model speech, reasoning
 * summaries, user instructions, [观测] messages and stopped/errored rounds exist only in the DSH session, invisible
 * to the sidecar. index.ts subscribes to session/event, extracts notes with notesOf() and POSTs them to the status
 * endpoint's /runlog, so the camera page shows everything in chronological order.
 */

/** One note; kind matches the sidecar's runlog.NOTE_KINDS. */
export interface Note {
  kind: "say" | "think" | "user" | "observe" | "goal" | "stop" | "error";
  text: string;
}

/** The session/event fields this module reads. */
export interface SessionEventLike {
  type?: string;
  data?: {
    message?: { content?: Array<{ type?: string; text?: string }>; source?: { kind?: string } };
    reason?: { kind?: string; error?: { message?: string }; reason?: { kind?: string } };
    operation?: string;
  };
}

const GOAL_OPS: Record<string, string> = {
  create: "建立", edit: "修改", complete: "完成", pause: "暂停", resume: "恢复", block: "受阻",
};

function texts(blocks: Array<{ type?: string; text?: string }> | undefined, type: string): string[] {
  return (blocks ?? [])
    .filter((b) => b.type === type && typeof b.text === "string")
    .map((b) => (b.text ?? "").trim())
    .filter((t) => t.length > 0);
}

/** Keep only the first line of a reasoning summary (GPT puts a **heading** there), without bold markers. */
function headline(text: string): string {
  return (text.split("\n").find((line) => line.trim() !== "") ?? "").replace(/\*\*/g, "").trim();
}

export function notesOf(event: SessionEventLike): Note[] {
  const d = event.data ?? {};
  switch (event.type) {
    case "assistant/message": {
      const blocks = d.message?.content;
      return [
        ...texts(blocks, "reasoning").map(headline).filter((t) => t !== "").map((text): Note => ({ kind: "think", text })),
        ...texts(blocks, "text").map((text): Note => ({ kind: "say", text })),
      ];
    }
    case "user/message": {
      const text = texts(d.message?.content, "text").join("\n");
      switch (d.message?.source?.kind) {
        case "user": return text ? [{ kind: "user", text }] : [];
        case "navi-observer": return text ? [{ kind: "observe", text }] : [];
        case "goal": return [{ kind: "goal", text: "任务还没做完，宿主自动续行一轮" }];
        default: return [];                       // Host-internal messages (runtime-context etc.) are not logged
      }
    }
    case "goal/change": {
      const op = d.operation ?? "";
      return [{ kind: "goal", text: `任务目标${GOAL_OPS[op] ?? op}` }];
    }
    case "turn/end": {
      const reason = d.reason ?? {};
      if (reason.kind === "error") return [{ kind: "error", text: `上游出错：${reason.error?.message ?? "未知错误"}` }];
      if (reason.kind === "aborted") {
        return [{ kind: "stop", text: reason.reason?.kind === "user" ? "这一轮被用户停止" : "这一轮推理被观测层打断" }];
      }
      return [];
    }
    default:
      return [];
  }
}
