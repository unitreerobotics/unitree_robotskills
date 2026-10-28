/**
 * poll.ts -- HTTP client for the status endpoint (Node built-in fetch only, no dependencies, unit-testable).
 *
 * Reads the read-only status endpoint exposed by navi_mcp_server (default 127.0.0.1:3082):
 *   /state       -> ObsState (what the robot is doing now)
 *   /frame.jpg   -> current camera frame
 *   POST /runlog -> append a run-log note (model speech, user instructions, etc.; the sidecar logs tool calls itself)
 * All failures return null: a dead status endpoint must never affect the session.
 */
import type { ObsState } from "./policy.ts";

export const DEFAULT_BASE = "http://127.0.0.1:3082";

export async function fetchState(base = DEFAULT_BASE, timeoutMs = 1500): Promise<ObsState | null> {
  try {
    const r = await fetch(`${base}/state`, { signal: AbortSignal.timeout(timeoutMs) });
    if (!r.ok) return null;
    return (await r.json()) as ObsState;
  } catch {
    return null;                      // Unreachable / timeout / truncated JSON: treat as "no data", silently
  }
}

export async function fetchFrame(base = DEFAULT_BASE, timeoutMs = 2500): Promise<Uint8Array | null> {
  try {
    const r = await fetch(`${base}/frame.jpg`, { signal: AbortSignal.timeout(timeoutMs) });
    if (!r.ok) return null;
    return new Uint8Array(await r.arrayBuffer());
  } catch {
    return null;
  }
}

/** Resolve the base URL: env var -> config -> default. Port 0 or negative means disabled. */
export function resolveBase(port?: number | null, host = "127.0.0.1"): string | null {
  const envPort = process.env.NAVI_OBSERVER_PORT;
  const p = envPort !== undefined ? Number(envPort) : (port ?? 3082);
  if (!Number.isFinite(p) || p <= 0) return null;
  return `http://${host}:${p}`;
}

/** Append a run-log note via /runlog. Returns false on failure instead of throwing: a missing log line must not affect the session. */
export async function postNote(base: string, note: { kind: string; text: string }, timeoutMs = 1000): Promise<boolean> {
  try {
    const r = await fetch(`${base}/runlog`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(note),
      signal: AbortSignal.timeout(timeoutMs),
    });
    return r.ok;
  } catch {
    return false;
  }
}
