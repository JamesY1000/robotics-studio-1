import { useRef, useState } from "react";
import type { SafetyState } from "./types";

export function safetyClear(safety?: SafetyState): boolean {
  return !!(
    safety?.fresh &&
    safety.service_ready &&
    safety.instance_id &&
    safety.husky_stopped === false &&
    safety.parrot_stopped === false
  );
}

export function SafetyControls({
  safety,
  connected,
}: {
  safety?: SafetyState;
  connected: boolean;
}) {
  const [resetting, setResetting] = useState(false);
  const [error, setError] = useState("");
  const requestSequence = useRef(0);
  const fresh = connected && safety?.fresh && safety.service_ready;

  async function change(robot: string, stopped: boolean) {
    const sequence = ++requestSequence.current;
    setError("");
    if (!stopped) setResetting(true);
    try {
      const response = await fetch("/api/safety/stop", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          robot,
          stopped,
          expected_instance_id: safety?.instance_id || "",
          expected_revision: safety?.revision || 0,
        }),
        signal: AbortSignal.timeout(5000),
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result.detail || "Stop request failed");
      // The live heartbeat confirms the state, never optimistically clear a stop here
    } catch (reason) {
      if (sequence === requestSequence.current)
        setError(
          `Request not confirmed: ${String(reason)}. Check the live stop state`,
        );
    } finally {
      if (!stopped) setResetting(false);
    }
  }

  return (
    <section className="safety-panel" aria-label="Software emergency stop">
      <div>
        <strong>Software E-stop</strong>
        <p>
          {!fresh
            ? "Stop state unavailable — motion is not confirmed"
            : safetyClear(safety)
              ? "Stop released"
              : "Stop latched — mission held"}
        </p>
      </div>
      <div className="safety-buttons">
        <button
          className="safety-stop"
          disabled={!connected}
          onClick={() => change("all", true)}
        >
          E-STOP BOTH ROBOTS
        </button>
        {(["husky1", "parrot1"] as const).map((robot) => {
          const stopped =
            robot === "husky1" ? safety?.husky_stopped : safety?.parrot_stopped;
          const label = robot === "husky1" ? "Husky" : "Parrot";
          return (
            <div key={robot}>
              <span>
                {label}: {!fresh ? "unknown" : stopped ? "stopped" : "released"}
              </span>
              <button
                className="safety-stop"
                disabled={!connected}
                onClick={() => change(robot, true)}
              >
                Stop {label}
              </button>
              <button
                disabled={
                  !fresh || !safety?.reset_ready || !stopped || resetting
                }
                onClick={() => change(robot, false)}
              >
                Reset {label}
              </button>
            </div>
          );
        })}
        <button
          disabled={
            !fresh ||
            !safety?.reset_ready ||
            resetting ||
            !(safety.husky_stopped || safety.parrot_stopped)
          }
          onClick={() => change("all", false)}
        >
          Reset both stops
        </button>
      </div>
      <p className="safety-help">
        Reset waits for navigation to stop. Then select Resume or manual
        assistance to move. Both robots start stopped after launch or
        safety-node restart. This is a simulation software stop, not a hardware
        E-stop.
      </p>
      {error && <p role="alert">{error}</p>}
    </section>
  );
}
