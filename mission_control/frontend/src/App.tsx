import React, { useEffect, useRef, useState } from "react";
import { TerrainMap, type Mode } from "./Map";
import type {
  GroundTruthObstacle,
  GroundTruthTree,
  Plan,
  Preset,
  State,
  XY,
} from "./types";
import { canStartAttached, missionRequest, spawnRequest } from "./workflow";
import { Telemetry } from "./Telemetry";
import { ManualControl } from "./ManualControl";
import { SafetyControls, safetyClear } from "./SafetyControls";

async function api<T>(
  path: string,
  body?: unknown,
  signal?: AbortSignal,
): Promise<T> {
  const response = await fetch(`/api/${path}`, {
    signal,
    method: body === undefined ? "GET" : "POST",
    headers:
      body instanceof FormData ? {} : { "Content-Type": "application/json" },
    body:
      body === undefined
        ? undefined
        : body instanceof FormData
          ? body
          : JSON.stringify(body),
  });
  if (!response.ok) {
    const error = await response.json();
    throw new Error(
      typeof error.detail === "string"
        ? error.detail
        : JSON.stringify(error.detail),
    );
  }
  return response.json();
}
function Icon({ kind }: { kind: string }) {
  const paths: Record<string, React.ReactNode> = {
    tree: (
      <>
        <path
          d="M12 2 5 12h4l-5 6h6v4h4v-4h6l-5-6h4Z"
          fill="currentColor"
          stroke="none"
        />
      </>
    ),
    mission: (
      <>
        <circle cx="12" cy="12" r="8" />
        <circle cx="12" cy="12" r="3" />
        <path d="M12 1v5m0 12v5M1 12h5m12 0h5" />
      </>
    ),
    robots: (
      <>
        <rect x="4" y="7" width="16" height="13" rx="3" />
        <path d="M12 7V3m-4 9h1m6 0h1M8 16h8" />
      </>
    ),
    telemetry: (
      <>
        <rect x="2" y="4" width="20" height="15" rx="2" />
        <path d="m4 12 4 0 2-4 4 8 2-4h4M9 22h6m-3-3v3" />
      </>
    ),
    settings: (
      <>
        <path d="M4 6h16M4 12h16M4 18h16" />
        <circle cx="9" cy="6" r="2" fill="white" />
        <circle cx="15" cy="12" r="2" fill="white" />
        <circle cx="9" cy="18" r="2" fill="white" />
      </>
    ),
    uav: (
      <>
        <path d="m6 6 12 12M6 18 18 6" />
        <circle cx="6" cy="6" r="4" />
        <circle cx="18" cy="6" r="4" />
        <circle cx="6" cy="18" r="4" />
        <circle cx="18" cy="18" r="4" />
      </>
    ),
    ugv: (
      <>
        <rect x="4" y="7" width="16" height="9" rx="2" />
        <path d="M8 7V4h8v3" />
        <circle cx="6" cy="18" r="2" />
        <circle cx="18" cy="18" r="2" />
      </>
    ),
  };
  return (
    <svg
      width="22"
      height="22"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.6"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      {paths[kind] || paths.mission}
    </svg>
  );
}
const format = (n?: number) =>
  n === undefined || !Number.isFinite(n) ? "—" : n.toFixed(1);
const words = (s: string) => s.replaceAll("_", " ");
type Draft = {
  start: XY | null;
  roi: XY | null;
  // Square side length in metres; roi is the bottom-left entry corner.
  roi_side: number;
  station_offset: XY;
  parrot_clearance: number;
  image_id: string;
};
const emptyDraft = (): Draft => ({
  start: null,
  roi: null,
  roi_side: 10,
  station_offset: { x: 0, y: 5 },
  parrot_clearance: 4,
  image_id: "forest-overhead",
});
const roiCorner = (entry: XY, side: number): XY => ({
  x: entry.x + side,
  y: entry.y - side,
});
function readDraft(): Draft {
  try {
    return {
      ...emptyDraft(),
      ...JSON.parse(localStorage.getItem("rs1-draft") || "{}"),
    };
  } catch {
    return emptyDraft();
  }
}

function Coordinates({
  label,
  point,
  disabled,
  onChange,
}: {
  label: string;
  point: XY | null;
  disabled: boolean;
  onChange: (p: XY) => void;
}) {
  return (
    <div className="coordinates">
      {(["x", "y"] as const).map((axis) => (
        <label key={axis}>
          <span>{axis.toUpperCase()}</span>
          <input
            aria-label={`${label} ${axis.toUpperCase()}`}
            type="number"
            step="0.1"
            disabled={disabled}
            value={
              point && Number.isFinite(point[axis])
                ? Math.round(point[axis] * 100) / 100
                : ""
            }
            placeholder="—"
            onChange={(e) =>
              onChange({
                ...(point || { x: 0, y: 0 }),
                [axis]: e.target.valueAsNumber,
              })
            }
          />
          <span>m</span>
        </label>
      ))}
    </div>
  );
}
function RobotCard({
  robot,
  state,
  connected,
}: {
  robot: "husky1" | "parrot1";
  state: State | null;
  connected: boolean;
}) {
  const uav = robot === "parrot1";
  const pose = state?.ros.robots[robot];
  const fresh = connected && pose?.fresh;
  const stage = state?.ros.mission?.[uav ? "parrot_stage" : "husky_stage"];
  return (
    <section className={`card robot-card ${uav ? "uav" : "ugv"}`}>
      <div className="card-heading">
        <div className="robot-name">
          <span className="robot-icon">
            <Icon kind={uav ? "uav" : "ugv"} />
          </span>
          <div>
            <h3>
              {uav ? "Parrot" : "Husky"} <span>{uav ? "UAV" : "UGV"}</span>
            </h3>
            <small>{uav ? "Aerial exploration" : "Ground traversal"}</small>
          </div>
        </div>
        <span className={`badge ${fresh ? "green" : ""}`}>
          <i />
          {fresh
            ? "Live estimate"
            : state?.plan ||
                state?.mode === "external" ||
                state?.deployment?.plan
              ? "No fresh TF"
              : "Not launched"}
        </span>
      </div>
      <div className="robot-metrics">
        <div>
          <small>WORLD X / Y</small>
          <strong>
            {format(pose?.x)} <span>/</span> {format(pose?.y)} <em>m</em>
          </strong>
        </div>
        <div>
          <small>HEADING</small>
          <strong>
            {pose?.yaw === undefined
              ? "—"
              : ((pose.yaw * 180) / Math.PI).toFixed(0)}
            <em>°</em>
          </strong>
        </div>
      </div>
      <div className="robot-stage">
        <span className="status-dot" />
        {stage || "Waiting for a mission"}
      </div>
    </section>
  );
}

export default function App() {
  const [state, setState] = useState<State | null>(null);
  const [connected, setConnected] = useState(false);
  const [presets, setPresets] = useState<Preset[]>([]);
  const [draft, setDraft] = useState(readDraft);
  const [externalRoi, setExternalRoi] = useState<XY | null>(null);
  const [externalRoiSecond, setExternalRoiSecond] = useState<XY | null>(null);
  const [groundTruth, setGroundTruth] = useState<GroundTruthTree[]>([]);
  const [obstacles, setObstacles] = useState<GroundTruthObstacle[]>([]);
  const [showGroundTruth, setShowGroundTruth] = useState(false);
  const [showObstacles, setShowObstacles] = useState(false);
  // Filters the overlay to the trees inside the ROI box. Defaults off so the
  // whole inventory is visible while the operator positions the box, and turns
  // on automatically once the mission starts (the ROI is then locked).
  const [roiOnlyGroundTruth, setRoiOnlyGroundTruth] = useState(false);
  const externalSession = useRef<string | null>(null);
  const externalRoiDirty = useRef(false);
  const previousExternalPhase = useRef("");
  const deploySession = useRef<string | null>(null);
  const [tab, setTab] = useState("mission");
  const [telemetryOpened, setTelemetryOpened] = useState(false);
  const [mode, setMode] = useState<Mode>("start");
  const [error, setError] = useState("");
  const [previewError, setPreviewError] = useState("");
  const [roiPreview, setRoiPreview] = useState<{
    key: string;
    error: string;
  } | null>(null);
  const [preview, setPreview] = useState<Record<
    string,
    XY & { z: number }
  > | null>(null);
  const [busy, setBusy] = useState(false);
  const [id, setId] = useState(() => crypto.randomUUID());
  const [uploadFile, setUploadFile] = useState<File | null>(null);
  const [calibration, setCalibration] = useState({
    x_min: "",
    x_max: "",
    y_min: "",
    y_max: "",
  });
  const [acknowledged, setAcknowledged] = useState(false);
  useEffect(() => {
    let alive = true;
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      try {
        const s = await api<State>(
          "state",
          undefined,
          AbortSignal.timeout(2000),
        );
        if (alive) {
          setState(s);
          setConnected(true);
        }
      } catch {
        if (alive) setConnected(false);
      } finally {
        if (alive) timer = setTimeout(poll, 500);
      }
    };
    void poll();
    return () => {
      alive = false;
      clearTimeout(timer);
    };
  }, []);
  useEffect(() => {
    if (connected)
      api<Preset[]>("presets")
        .then(setPresets)
        .catch((e) => setError(e.message));
  }, [connected, state?.plan?.image_id, state?.deployment?.plan?.image_id]);
  useEffect(() => {
    // Ground-truth inventories are static; fetch them once when connected.
    if (!connected) return;
    let alive = true;
    api<GroundTruthTree[]>("trees/ground_truth")
      .then((trees) => {
        if (alive) setGroundTruth(trees);
      })
      .catch(() => {
        /* overlay is optional; leave it empty */
      });
    api<GroundTruthObstacle[]>("obstacles/ground_truth")
      .then((items) => {
        if (alive) setObstacles(items);
      })
      .catch(() => {
        /* overlay is optional; leave it empty */
      });
    return () => {
      alive = false;
    };
  }, [connected]);
  useEffect(() => {
    localStorage.setItem("rs1-draft", JSON.stringify(draft));
  }, [draft]);
  const external = state?.mode === "external";
  const deploy = state?.mode === "deploy";
  const attached = external || deploy;
  const deployment = state?.deployment;
  const phase = state?.phase || "connecting";
  const configuredRoi = state?.configured_roi;
  const worldRoi = configuredRoi?.frame_id === "world" ? configuredRoi : null;
  useEffect(() => {
    if (!external) return;
    const session =
      state?.ros.mission?.instance_id ||
      state?.ros.mission?.session_id ||
      "external";
    const previousPhase = previousExternalPhase.current;
    const newMission =
      externalSession.current !== session ||
      (["complete", "cancelled", "failed"].includes(previousPhase) &&
        phase === "idle");
    if (newMission) {
      externalSession.current = session;
      externalRoiDirty.current = false;
      setExternalRoi(null);
      setExternalRoiSecond(null);
    }
    if (!externalRoiDirty.current && configuredRoi?.frame_id === "world") {
      setExternalRoi({ x: configuredRoi.x, y: configuredRoi.y });
      setExternalRoiSecond(
        configuredRoi.min && configuredRoi.max
          ? { x: configuredRoi.max.x, y: configuredRoi.min.y }
          : roiCorner(configuredRoi, 10),
      );
    }
    // Waiting during a coordinator restart must not hide its preceding outcome.
    if (phase !== "waiting") previousExternalPhase.current = phase;
  }, [
    external,
    state?.ros.mission?.instance_id,
    state?.ros.mission?.session_id,
    phase,
    configuredRoi?.x,
    configuredRoi?.y,
    configuredRoi?.frame_id,
  ]);
  useEffect(() => {
    if (!deploy) return;
    const session = deployment?.plan?.session_id || "unspawned";
    if (deploySession.current !== session) {
      deploySession.current = session;
      externalRoiDirty.current = false;
      setExternalRoi(null);
      setExternalRoiSecond(null);
    }
    // A newly spawned coordinator has a placeholder target. Do not present it
    // as an operator-selected ROI after a refresh or dashboard restart.
    if (
      !externalRoiDirty.current &&
      deployment?.roi_selected &&
      configuredRoi?.frame_id === "world"
    ) {
      setExternalRoi({ x: configuredRoi.x, y: configuredRoi.y });
      setExternalRoiSecond(
        configuredRoi.min && configuredRoi.max
          ? { x: configuredRoi.max.x, y: configuredRoi.min.y }
          : roiCorner(configuredRoi, 10),
      );
    }
  }, [
    deploy,
    deployment?.plan?.session_id,
    deployment?.roi_selected,
    configuredRoi?.x,
    configuredRoi?.y,
    configuredRoi?.frame_id,
  ]);
  useEffect(() => {
    if (external) setMode("pan");
  }, [external]);
  useEffect(() => {
    if (deploy) setMode(deployment?.phase === "deployed" ? "roi" : "start");
  }, [deploy, deployment?.phase]);
  const spawnLocked = deploy
    ? deployment?.phase !== "unspawned"
    : external || !!state?.plan;
  const roiLocked = attached
    ? phase !== "idle" || (deploy && deployment?.phase !== "deployed")
    : !!state?.plan;
  const locked =
    deploy && deployment?.phase !== "deployed" ? spawnLocked : roiLocked;
  useEffect(() => {
    // ROI-only filtering follows the mission: off while planning, on once the
    // mission starts, and back off when the ROI is unlocked again.
    setRoiOnlyGroundTruth(locked);
  }, [locked]);
  const selected = state?.plan || { ...draft, ...deployment?.plan };
  const preset = presets.find((p) => p.id === selected.image_id) || presets[0];
  const start = external ? null : selected.start;
  const savedRoi = deploy && !deployment?.roi_selected ? null : worldRoi;
  const savedRoiSecond =
    savedRoi?.min && savedRoi?.max
      ? { x: savedRoi.max.x, y: savedRoi.min.y }
      : null;
  const roi = attached ? externalRoi || savedRoi : selected.roi;
  const roiSecond = attached
    ? externalRoiSecond || savedRoiSecond
    : selected.roi
      ? ((selected.roi_second as XY | null) ??
        roiCorner(selected.roi, draft.roi_side))
      : null;
  const station =
    start && !deploy
      ? {
          x: start.x + selected.station_offset.x,
          y: start.y + selected.station_offset.y,
        }
      : null;
  // Ground-truth overlay. "ROI only" filters to trees inside the box; it is off
  // by default and switches on when the mission starts (see the effect above).
  const groundTruthFiltered = showGroundTruth
    ? groundTruth.filter((t) => {
        if (!roiOnlyGroundTruth || !roi || !roiSecond) return true;
        return (
          t.x >= Math.min(roi.x, roiSecond.x) &&
          t.x <= Math.max(roi.x, roiSecond.x) &&
          t.y >= Math.min(roi.y, roiSecond.y) &&
          t.y <= Math.max(roi.y, roiSecond.y)
        );
      })
    : [];
  const obstaclesShown = showObstacles ? obstacles : [];
  const plan: Plan | null =
    !attached && start && roi && preset
      ? {
          session_id: id,
          world_id: preset.world_id,
          image_id: preset.id,
          calibration_id: preset.calibration_id,
          start,
          roi,
          roi_second: roiSecond,
          station_offset: selected.station_offset,
          parrot_clearance: selected.parrot_clearance,
        }
      : null;
  const spawnPlan = deploy
    ? spawnRequest(id, preset, start, selected.parrot_clearance)
    : null;
  const previewPlan = deploy ? spawnPlan : plan;
  const planKey = JSON.stringify(previewPlan);
  useEffect(() => {
    setPreview(null);
    setPreviewError("");
    if (!state || external || !previewPlan || spawnLocked) return;
    let alive = true;
    const timer = setTimeout(() => {
      api<Record<string, XY & { z: number }>>(
        deploy ? "deployment/preview" : "plan/preview",
        previewPlan,
      )
        .then((p) => {
          if (alive) setPreview(p);
        })
        .catch((e) => {
          if (alive) setPreviewError(e.message);
        });
    }, 300);
    return () => {
      alive = false;
      clearTimeout(timer);
    };
  }, [planKey, spawnLocked, external, deploy, !!state]);

  // Deploy mode submits a selected world ROI. External mode only submits one
  // when edited; an untouched coordinator goal keeps its existing frame and Z.
  const roiNeedsPreview =
    attached && !!roi && (deploy || externalRoiDirty.current);
  const roiPreviewKey = roiNeedsPreview
    ? JSON.stringify([
        state?.mode,
        deployment?.plan?.session_id,
        roi.x,
        roi.y,
        roiSecond?.x,
        roiSecond?.y,
      ])
    : "";
  useEffect(() => {
    setRoiPreview(null);
    if (!roiPreviewKey || roiLocked || !roi) return;
    let alive = true;
    // Match the spawn preview delay so clicks and coordinate edits settle first.
    const timer = setTimeout(() => {
      api<XY & { z: number }>("roi/preview", {
        x: roi.x,
        y: roi.y,
        ...(roiSecond ? { second: { x: roiSecond.x, y: roiSecond.y } } : {}),
      })
        .then(() => {
          if (alive) setRoiPreview({ key: roiPreviewKey, error: "" });
        })
        .catch((e) => {
          if (alive) setRoiPreview({ key: roiPreviewKey, error: e.message });
        });
    }, 300);
    return () => {
      // A response for an older selection must never validate the new point.
      alive = false;
      clearTimeout(timer);
    };
  }, [roiPreviewKey, roiLocked]);
  const roiPreviewMatches = roiPreview?.key === roiPreviewKey;
  const roiPreviewError = roiPreviewMatches ? roiPreview.error : "";
  const roiValidationMessage =
    roiNeedsPreview && !roiLocked
      ? roiPreviewMatches
        ? roiPreviewError || "✓ ROI passed placement checks. Ready to start."
        : "Checking ROI…"
      : "";

  const place = (which: Mode, p: XY) => {
    if (which !== "start" && which !== "station") return;
    if (spawnLocked) return;
    if (external || (deploy && which === "station")) return;
    setDraft((d) =>
      which === "station" && d.start
        ? { ...d, station_offset: { x: p.x - d.start.x, y: p.y - d.start.y } }
        : which === "start"
          ? { ...d, start: p }
          : d,
    );
  };
  const placeRoi = (entry: XY, corner: XY) => {
    if (roiLocked) return;
    if (attached) {
      externalRoiDirty.current = true;
      setExternalRoi(entry);
      setExternalRoiSecond(corner);
      return;
    }
    setDraft((d) => ({
      ...d,
      roi: entry,
      roi_side: Math.max(2, corner.x - entry.x, entry.y - corner.y),
    }));
  };
  const roiEntryChange = (p: XY) => {
    if (roiLocked) return;
    if (attached) {
      externalRoiDirty.current = true;
      const side =
        externalRoi && externalRoiSecond
          ? Math.max(
              2,
              externalRoiSecond.x - externalRoi.x,
              externalRoi.y - externalRoiSecond.y,
            )
          : 10;
      setExternalRoi(p);
      setExternalRoiSecond(roiCorner(p, side));
      return;
    }
    setDraft((d) => ({ ...d, roi: p }));
  };
  const act = async (
    action:
      | "start"
      | "cancel"
      | "reset"
      | "spawn"
      | "stop"
      | "manual_husky"
      | "manual_parrot"
      | "input_keyboard"
      | "input_controller"
      | "resume",
  ) => {
    setBusy(true);
    setError("");
    try {
      const controlAction = [
        "manual_husky",
        "manual_parrot",
        "input_keyboard",
        "input_controller",
        "resume",
      ].includes(action);
      const next = await api<State>(
        controlAction
          ? `mission/control/${action}`
          : action === "reset"
            ? "session/reset"
            : action === "spawn" || action === "stop"
              ? `deployment/${action}`
              : `mission/${action}`,
        action === "start"
          ? missionRequest(
              state?.mode,
              roi,
              roiSecond,
              externalRoiDirty.current,
              plan,
            )
          : action === "spawn"
            ? spawnPlan
            : {},
      );
      setState(next);
      if (action === "stop" || (action === "reset" && !attached)) {
        setId(crypto.randomUUID());
        setMode("start");
      }
    } catch (e) {
      setError((e as Error).message);
    } finally {
      setBusy(false);
    }
  };
  const upload = async () => {
    if (!uploadFile) return;
    setBusy(true);
    setError("");
    try {
      const form = new FormData();
      form.append("file", uploadFile);
      form.append(
        "registration",
        JSON.stringify({
          world_id: "v2_forest",
          calibration: Object.fromEntries(
            Object.entries(calibration).map(([k, v]) => [k, Number(v)]),
          ),
        }),
      );
      const p = await api<Preset>("images", form);
      setPresets((all) => [...all, p]);
      setDraft((d) => ({ ...d, image_id: p.id }));
      setTab("mission");
    } catch (e) {
      setError((e as Error).message);
    } finally {
      setBusy(false);
    }
  };
  const active =
    [
      "preparing",
      "ready",
      "executing",
      "sampling",
      "returning",
      "manual_assistance",
      "cancellation_requested",
    ].includes(phase) &&
    (!deploy || deployment?.phase === "deployed");
  const placements = deployment?.placements || state?.placements || preview;
  const motionEnabled = safetyClear(state?.ros.safety);
  const canStart = attached
    ? motionEnabled &&
      canStartAttached(state, roi) &&
      (!roiNeedsPreview || (roiPreviewMatches && !roiPreviewError))
    : !locked && !!preview && !!state?.ros.available;
  const canSpawn =
    deploy && !!deployment?.can_spawn && !spawnLocked && !!preview;
  const canStop = deploy && !!deployment?.plan && !active;
  const controlMode = state?.ros.mission?.control_mode;
  const returningHome =
    state?.ros.mission?.returning_home || phase === "returning";
  const manualRobot =
    controlMode === "husky1" || controlMode === "parrot1" ? controlMode : null;
  const sweepProgress = state?.ros.mission?.roi?.progress;
  const showSweep =
    ["sampling", "returning", "complete"].includes(phase) &&
    typeof sweepProgress === "number" &&
    Number.isFinite(sweepProgress);
  const sweepPct = showSweep
    ? Math.round(Math.min(1, Math.max(0, sweepProgress as number)) * 100)
    : 0;
  const treeCount = state?.ros.trees?.length ?? 0;
  return (
    <div className="shell">
      <aside className="sidebar">
        <div className="brand">
          <span>
            <Icon kind="tree" />
          </span>
          <div>
            RS1<small>FOREST ROBOTICS</small>
          </div>
        </div>
        <div className="nav-caption">WORKSPACE</div>
        <nav>
          {[
            ["mission", "Mission"],
            ["robots", "Robots & status"],
            ["telemetry", "Telemetry"],
            ["settings", "Settings"],
          ].map(([key, label]) => (
            <button
              className={tab === key ? "selected" : ""}
              key={key}
              aria-current={tab === key ? "page" : undefined}
              title={label}
              onClick={() => {
                setTab(key);
                if (key === "telemetry") setTelemetryOpened(true);
              }}
            >
              <Icon kind={key} />
              {label}
              {key === "mission" && <span className="nav-count">01</span>}
            </button>
          ))}
        </nav>
        <div className="sidebar-bottom">
          <div className="connection">
            <span className={`status-dot ${connected ? "on" : ""}`} />
            <strong>{connected ? "Backend connected" : "Reconnecting…"}</strong>
            <small>
              ROS{" "}
              {state?.ros.available
                ? "adapter available"
                : "adapter unavailable"}
            </small>
          </div>
          <div className="project-mark">
            UTS{" "}
            <span>
              Robotics Studio 1<br />
              Simulation workspace
            </span>
          </div>
        </div>
      </aside>
      <main>
        <header className="page-header">
          <div>
            <div className="eyebrow">
              FOREST SURVEY /{" "}
              {external
                ? "EXTERNAL ROS STACK"
                : deploy
                  ? "DEPLOY TO SIMULATION"
                  : "LOCAL SIMULATION"}
            </div>
            <h1>
              {tab === "mission"
                ? "Mission Control"
                : tab === "robots"
                  ? "Robots & status"
                  : tab === "telemetry"
                    ? "Robot telemetry"
                    : "Workspace settings"}
            </h1>
            <p>
              {tab === "mission"
                ? "Plan the survey. Keep both robots in view."
                : tab === "robots"
                  ? "Localization estimates and the interfaces that support your mission."
                  : tab === "telemetry"
                    ? "Inspect robot sensors, maps and transforms alongside your mission."
                    : "Calibrate terrain imagery and review session configuration."}
            </p>
          </div>
          <div className="header-actions">
            {[
              "executing",
              "sampling",
              "returning",
              "manual_assistance",
            ].includes(phase) && (
              <>
                <button
                  disabled={busy || !connected || !motionEnabled}
                  onClick={() => act("manual_husky")}
                >
                  Manual Husky
                </button>
                <button
                  disabled={busy || !connected || !motionEnabled}
                  onClick={() => act("manual_parrot")}
                >
                  Manual Parrot
                </button>
              </>
            )}
            {phase === "manual_assistance" && (
              <button
                className="primary"
                disabled={
                  busy ||
                  !connected ||
                  !motionEnabled ||
                  !state?.ros.mission?.cleanup_complete
                }
                title={
                  state?.ros.mission?.cleanup_complete
                    ? returningHome
                      ? "Recheck readiness and continue returning home"
                      : "Recheck readiness and continue the mission"
                    : "Waiting for previous navigation to stop"
                }
                onClick={() => act("resume")}
              >
                Resume autonomous mission
              </button>
            )}
            <span
              className={`badge ${["executing", "sampling", "returning", "complete"].includes(phase) ? "green" : ""}`}
            >
              {words(phase)}
            </span>
            {deploy && deployment?.phase !== "deployed" ? (
              <button
                className="primary"
                disabled={busy || !connected || !canSpawn}
                onClick={() => act("spawn")}
              >
                {deployment?.phase === "spawning"
                  ? "Spawning robots…"
                  : "Spawn robots"}
              </button>
            ) : active ? (
              <button
                className="cancel"
                disabled={
                  busy ||
                  !connected ||
                  phase === "cancellation_requested" ||
                  returningHome
                }
                onClick={() => act("cancel")}
              >
                {phase === "cancellation_requested"
                  ? "Stopping survey…"
                  : returningHome
                    ? phase === "manual_assistance"
                      ? "Return home paused"
                      : "Returning home…"
                    : phase === "preparing"
                      ? "Cancel preparation"
                      : "Cancel mission & return home"}
              </button>
            ) : (
              <button
                className="primary"
                disabled={busy || !connected || !canStart}
                onClick={() => act("start")}
              >
                ▶ Start Mission
              </button>
            )}
          </div>
        </header>
        <SafetyControls safety={state?.ros.safety} connected={connected} />
        {tab === "mission" &&
          phase === "manual_assistance" &&
          manualRobot &&
          connected &&
          !busy && (
            <>
              <div className="notice" role="status">
                {state?.ros.mission?.message}
                {!state?.ros.mission?.cleanup_complete &&
                  " Previous navigation is still stopping; manual control is available while you wait."}
              </div>
              <ManualControl
                key={`${state?.ros.mission?.instance_id}:${manualRobot}:${state?.ros.mission?.input_revision}`}
                robot={manualRobot}
                input={state?.ros.mission?.manual_input}
                controllerConnected={!!state?.ros.controller?.connected}
                onSelectInput={(input) =>
                  act(
                    input === "keyboard"
                      ? "input_keyboard"
                      : "input_controller",
                  )
                }
              />
            </>
          )}
        {(error || (!connected && state)) && (
          <div className="alert" role="alert">
            {error ||
              (attached
                ? "Dashboard connection lost. Your ROS stack continues running. Reconnecting does not spawn robots again."
                : "Connection lost. The backend continues to own this session. Reconnecting will not restart it.")}
            {error && (
              <button onClick={() => setError("")} aria-label="Dismiss error">
                ×
              </button>
            )}
          </div>
        )}
        {state?.ros.error && <div className="notice">{state.ros.error}</div>}
        {external && (
          <div className="notice">
            <strong>External ROS stack.</strong> Launch simulation and mission
            nodes in their own terminals. This dashboard connects to them and
            sends mission requests. Robot spawning and ground-station placement
            stay in your simulation launch.
          </div>
        )}
        {deploy && (
          <div className="notice">
            <strong>Deploy to the running world.</strong> Choose a calibrated
            image and robot start, spawn the robots, then select an ROI and
            start the mission. Closing this dashboard leaves the robots and
            mission nodes running. Images describe the current world; they do
            not replace it.
          </div>
        )}
        <div hidden={tab !== "mission"}>
          <div className="mission-grid">
            <section className="card map-card">
              <div className="map-header">
                <div>
                  <h2>Map & mission</h2>
                  <small>
                    {preset?.world_id || "Loading terrain"} <span>·</span> World
                    frame
                  </small>
                </div>
                <select
                  aria-label="Terrain image"
                  disabled={deploy ? spawnLocked : locked}
                  value={preset?.id || ""}
                  onChange={(e) =>
                    setDraft((d) => ({ ...d, image_id: e.target.value }))
                  }
                >
                  {presets.map((p) => (
                    <option key={p.id} value={p.id}>
                      {p.name}
                    </option>
                  ))}
                </select>
              </div>
              <div className="map-toolbar">
                {(["pan", "start", "roi", "station"] as const).map(
                  (m, i) =>
                    (!external || m === "pan" || m === "roi") &&
                    (!deploy || m !== "station") && (
                      <button
                        key={m}
                        disabled={
                          m !== "pan" && (m === "roi" ? roiLocked : spawnLocked)
                        }
                        className={mode === m ? "active" : ""}
                        onClick={() => setMode(m)}
                      >
                        <span>{["↔", "①", "②", "⌂"][i]}</span>
                        {["Pan", "Robot start", "ROI box", "Station"][i]}
                      </button>
                    ),
                )}
              </div>
              {preset ? (
                <TerrainMap
                  preset={preset}
                  start={start}
                  roi={roi}
                  roiSecond={roiSecond}
                  station={station}
                  mode={mode}
                  locked={locked}
                  state={state}
                  groundTruth={groundTruthFiltered}
                  obstacles={obstaclesShown}
                  onPlace={place}
                  onRoi={placeRoi}
                />
              ) : (
                <div className="loading-map">
                  Connecting to the terrain service…
                </div>
              )}
              <div className="map-footer">
                <span>
                  <i className="legend-dot uav-dot" />
                  Parrot
                </span>
                <span>
                  <i className="legend-dot ugv-dot" />
                  Husky
                </span>
                <span>
                  <i className="legend-dot roi-dot" />
                  ROI box
                </span>
                <span>
                  <i className="legend-dot tree-dot" />
                  Trees
                </span>
                <span className="map-footer-note">
                  {external || (deploy ? !!deployment?.plan : locked)
                    ? "Trails from estimated TF"
                    : "Dashed markers · spawn preview"}
                </span>
              </div>
              <label className="ground-truth-toggle">
                <input
                  type="checkbox"
                  checked={showGroundTruth}
                  aria-label="Show ground-truth trees"
                  onChange={(e) => setShowGroundTruth(e.target.checked)}
                />
                <i className="legend-dot gt-dot" />
                Ground-truth trees
                <small>
                  {groundTruthFiltered.length}
                  {roiOnlyGroundTruth && roi && roiSecond
                    ? " inside ROI"
                    : " total"}
                </small>
              </label>
              <label className="ground-truth-toggle">
                <input
                  type="checkbox"
                  checked={roiOnlyGroundTruth}
                  disabled={!showGroundTruth}
                  aria-label="Show only ground-truth trees inside the ROI"
                  onChange={(e) => setRoiOnlyGroundTruth(e.target.checked)}
                />
                ROI only
              </label>
              <label className="ground-truth-toggle">
                <input
                  type="checkbox"
                  checked={showObstacles}
                  aria-label="Show ground-truth bushes and rocks"
                  onChange={(e) => setShowObstacles(e.target.checked)}
                />
                Obstacles
                <i className="legend-dot bush-dot" />
                Bush
                <i className="legend-dot rock-dot" />
                Rock
                <small>{obstacles.length}</small>
              </label>
            </section>
            <aside className="plan-column">
              <div className="card plan-card">
                <div className="section-label">
                  {external
                    ? "MISSION ROI"
                    : deploy
                      ? "ROBOT DEPLOYMENT"
                      : "MISSION PLAN"}{" "}
                  <span>
                    {deploy
                      ? words(deployment?.phase || "connecting")
                      : locked
                        ? "Read only"
                        : "Draft"}
                  </span>
                </div>
                {!external && (
                  <div className="plan-section">
                    <h3>
                      <b>1</b> Shared robot start
                    </h3>
                    <p>Husky and Parrot share X / Y. Both spawn facing +X.</p>
                    <Coordinates
                      label="Start"
                      point={start}
                      disabled={spawnLocked}
                      onChange={(p) => place("start", p)}
                    />
                    <small className="height-info">
                      {placements
                        ? `Spawn Z · UGV ${format(placements.husky.z)} / UAV ${format(placements.parrot.z)} m`
                        : "Spawn heights use the collision terrain."}
                    </small>
                  </div>
                )}
                <div className="plan-section">
                  <h3>
                    <b>{external ? "◎" : "2"}</b> ROI box
                  </h3>
                  <p>
                    {deploy && deployment?.phase !== "deployed"
                      ? "Spawn the robots first, then choose the square to survey."
                      : "Both robots traverse to the square's bottom-left corner."}
                  </p>
                  <Coordinates
                    label="ROI entry"
                    point={roi}
                    disabled={roiLocked}
                    onChange={roiEntryChange}
                  />
                  {roiSecond && roi && (
                    <small className="height-info">
                      Square side{" "}
                      {format(
                        Math.max(roiSecond.x - roi.x, roi.y - roiSecond.y),
                      )}{" "}
                      m · corner {format(roiSecond.x)}, {format(roiSecond.y)} m
                    </small>
                  )}
                  {external &&
                    configuredRoi &&
                    configuredRoi.frame_id !== "world" &&
                    !roi && (
                      <small className="height-info">
                        Coordinator ROI: {format(configuredRoi.x)},{" "}
                        {format(configuredRoi.y)} m in {configuredRoi.frame_id}.
                        Start uses this target, or select a world ROI on the
                        map.
                      </small>
                    )}
                </div>
                {!attached && (
                  <div className="plan-section">
                    <h3>
                      <b>⌂</b> Ground station
                    </h3>
                    <p>Offset from the shared start. Drag to adjust.</p>
                    <Coordinates
                      label="Station offset"
                      point={selected.station_offset}
                      disabled={locked}
                      onChange={(p) =>
                        setDraft((d) => ({ ...d, station_offset: p }))
                      }
                    />
                    {station && (
                      <small className="height-info">
                        World {format(station.x)}, {format(station.y)} m
                      </small>
                    )}
                  </div>
                )}
                <div
                  className={`validation ${previewError || roiPreviewError ? "invalid" : ""}`}
                  role="status"
                >
                  {deploy
                    ? previewError ||
                      roiValidationMessage ||
                      (deployment?.phase === "unspawned"
                        ? !deployment.can_spawn
                          ? deployment.message
                          : preview
                            ? "✓ Robot placements validated. Spawn robots to continue."
                            : "Choose the shared robot start to validate terrain clearance."
                        : deployment?.phase === "deployed"
                          ? phase !== "idle"
                            ? state?.message
                            : !roi
                              ? "Select an ROI box on the map or enter coordinates. No mission starts automatically."
                              : "Start sends the selected square ROI to the coordinator. Terrain supplies Z."
                          : deployment?.message ||
                            "Connecting to the running world…")
                    : external
                      ? locked
                        ? "ROI editing is available when the mission coordinator is idle."
                        : roiValidationMessage ||
                          (externalRoiDirty.current
                            ? "Start sends your edited world X / Y to the coordinator. Terrain supplies Z."
                            : "Start uses the coordinator’s configured ROI, including its frame and Z. Edit X / Y to select a world ROI with terrain height.")
                      : locked
                        ? "Plan saved for this session."
                        : previewError ||
                          (preview
                            ? "✓ Placements validated against terrain."
                            : "Place both a start and an ROI to validate.")}
                </div>
                {deploy && deployment?.plan && (
                  <button
                    className="deployment-stop"
                    disabled={busy || !connected || !canStop}
                    onClick={() => act("stop")}
                  >
                    Stop robots
                  </button>
                )}
              </div>
              <div className="plan-note">
                <Icon kind="mission" />
                <span>
                  {deploy
                    ? "Stop robots removes this deployment and its mission nodes. Gazebo stays running; restart the world terminal before deploying again. Cancel an active mission to return both robots home, then Stop robots."
                    : external
                      ? "After a mission ends, restart the mission launch in its terminal to run another. Restarting this dashboard does not reset the mission."
                      : "Terrain following stays active. Horizontal movement begins after alignment and navigation are ready."}
                </span>
              </div>
            </aside>
          </div>
          <div className="lower-grid">
            <RobotCard robot="parrot1" state={state} connected={connected} />
            <RobotCard robot="husky1" state={state} connected={connected} />
            <section className="card progress-card">
              <div className="card-heading">
                <h3>Mission status</h3>
                <span className="tiny-label">
                  {phase === "returning"
                    ? "RETURNING HOME"
                    : phase === "sampling"
                      ? "SAMPLING"
                      : phase === "complete"
                        ? "COMPLETE"
                        : phase === "cancelled"
                          ? "CANCELLED"
                          : "TRAVERSAL"}
                </span>
              </div>
              <p className="phase-message" aria-live="polite">
                {state?.message || "Connecting to mission service…"}
              </p>
              {showSweep && (
                <div
                  className="sweep-progress"
                  role="progressbar"
                  aria-valuemin={0}
                  aria-valuemax={100}
                  aria-valuenow={sweepPct}
                >
                  <div className="sweep-progress-track">
                    <div
                      className="sweep-progress-fill"
                      style={{ width: `${sweepPct}%` }}
                    />
                  </div>
                  <div className="sweep-progress-label">
                    {sweepPct}% swept · {treeCount} tree(s) found
                  </div>
                </div>
              )}
              <div className="steps">
                {(deploy
                  ? ["Place", "Spawn", "Traverse", "Finish"]
                  : external
                    ? ["Connect", "Ready", "Traverse", "Finish"]
                    : ["Plan", "Prepare", "Traverse", "Finish"]
                ).map((label, i) => (
                  <div
                    key={label}
                    className={
                      (phase === "complete"
                        ? 3
                        : state?.events.some((e) => e.phase === "executing")
                          ? 2
                          : deploy
                            ? deployment?.phase === "deployed" ||
                              deployment?.phase === "spawning"
                              ? 1
                              : 0
                            : external
                              ? state?.can_start
                                ? 1
                                : 0
                              : locked
                                ? 1
                                : 0) >= i
                        ? "reached"
                        : ""
                    }
                  >
                    <i />
                    {label}
                  </div>
                ))}
              </div>
              <small>
                {treeCount > 0
                  ? `${treeCount} tree(s) mapped; markers shown on the terrain map.`
                  : "Sweep progress updates live; tree markers appear once the tree map is published."}
              </small>
            </section>
          </div>
        </div>
        {telemetryOpened && (
          <div hidden={tab !== "telemetry"}>
            <Telemetry />
          </div>
        )}
        {tab === "robots" && (
          <>
            <div className="robot-grid">
              <RobotCard robot="parrot1" state={state} connected={connected} />
              <RobotCard robot="husky1" state={state} connected={connected} />
            </div>
            <section className="card detail-card">
              <h2>Readiness checks</h2>
              <p>
                Start waits for fresh TF, verified alignment, active navigation
                nodes and mission services.
              </p>
              <div className="checks">
                {Object.entries(state?.ros.checks || {}).map(([key, ready]) => (
                  <div key={key}>
                    <span
                      className={`status-dot ${ready && connected ? "on" : ""}`}
                    />
                    <code>{words(key)}</code>
                    <span>{ready && connected ? "Ready" : "Waiting"}</span>
                  </div>
                ))}
              </div>
              <p className="muted">
                XY and heading come from localization through the shared world
                frame. TF Z is not a measured altitude.
              </p>
            </section>
            <section className="card detail-card">
              <h2>Session events</h2>
              {state?.events.length ? (
                state.events.map((e, i) => (
                  <div className="event" key={i}>
                    <time>{new Date(e.at * 1000).toLocaleTimeString()}</time>
                    <strong>{words(e.phase)}</strong>
                    <span>{e.message}</span>
                  </div>
                ))
              ) : (
                <p>No mission has been started.</p>
              )}
            </section>
          </>
        )}
        {tab === "settings" && (
          <div className="settings-grid">
            <section className="card detail-card">
              <h2>Upload a terrain image</h2>
              <p>
                Associate an image with the existing <strong>v2_forest</strong>{" "}
                world. Uploading does not create or change Gazebo terrain.
              </p>
              <label className="upload-zone">
                Choose PNG or JPEG
                <input
                  aria-label="Terrain image upload"
                  type="file"
                  accept="image/png,image/jpeg"
                  disabled={deploy ? spawnLocked : locked}
                  onChange={(e) => setUploadFile(e.target.files?.[0] || null)}
                />
                <small>
                  {uploadFile?.name || "Up to 10 MiB · 16 megapixels"}
                </small>
              </label>
              <h3>Image edge calibration</h3>
              <p>
                Enter world coordinates at the image edges. +X is right, +Y is
                down. Rotated imagery must be registered before uploading.
              </p>
              <div className="calibration-fields">
                {Object.keys(calibration).map((key) => (
                  <label key={key}>
                    {
                      (
                        {
                          x_min: "Left edge X",
                          x_max: "Right edge X",
                          y_min: "Top edge Y",
                          y_max: "Bottom edge Y",
                        } as Record<string, string>
                      )[key]
                    }
                    <input
                      type="number"
                      aria-label={key}
                      disabled={deploy ? spawnLocked : locked}
                      value={calibration[key as keyof typeof calibration]}
                      onChange={(e) =>
                        setCalibration((c) => ({ ...c, [key]: e.target.value }))
                      }
                    />
                  </label>
                ))}
              </div>
              <label className="check-label">
                <input
                  type="checkbox"
                  checked={acknowledged}
                  disabled={deploy ? spawnLocked : locked}
                  onChange={(e) => setAcknowledged(e.target.checked)}
                />
                I have verified this image’s registration to v2_forest.
              </label>
              <button
                className="primary"
                disabled={
                  busy ||
                  (deploy ? spawnLocked : locked) ||
                  !uploadFile ||
                  !acknowledged ||
                  Object.values(calibration).some((v) => v === "")
                }
                onClick={upload}
              >
                Save calibrated image
              </button>
            </section>
            <section className="card detail-card">
              <h2>Session & configuration</h2>
              <dl>
                <dt>World</dt>
                <dd>v2_forest</dd>
                <dt>Coordinate frame</dt>
                <dd>world · metres</dd>
                <dt>Terrain footprint</dt>
                <dd>200 × 200 m</dd>
                <dt>Launch ownership</dt>
                <dd>
                  {external
                    ? "External ROS terminals"
                    : deploy
                      ? "External world · persistent robot deployment"
                      : "Dashboard backend"}
                </dd>
                <dt>Session</dt>
                <dd className="session-id">
                  {state?.plan?.session_id ||
                    deployment?.plan?.session_id ||
                    state?.ros.mission?.session_id ||
                    "No active session"}
                </dd>
              </dl>
              <hr />
              <h3>{attached ? "Dashboard view" : "New session"}</h3>
              <p>
                {attached
                  ? "Clear recorded trails and dashboard events. Simulation, alignment and mission nodes continue running."
                  : "Stops this application’s simulation, clears alignment and trails, and unlocks placement editing. Saved session plans and logs remain on disk."}
              </p>
              <button
                disabled={busy || !connected}
                onClick={() => act("reset")}
              >
                ↻ {attached ? "Clear trails" : "Reset / New session"}
              </button>
              {external && (
                <p>
                  For another mission, restart the mission launch in its
                  terminal after completion or cancellation.
                </p>
              )}
              {deploy && (
                <>
                  <h3>Robot deployment</h3>
                  <p>
                    {deployment?.message || "Waiting for the simulation world."}{" "}
                    Stop robots leaves the world running. Before deploying
                    again, restart its terminal with rs1_sim.launch.py ui:=true.
                    The dashboard can stay open.
                  </p>
                  <button
                    disabled={busy || !connected || !canStop}
                    onClick={() => act("stop")}
                  >
                    Stop robots
                  </button>
                </>
              )}
              <p className="muted">
                Cancellation is a mission request, not an emergency stop. The
                mission traverses to the ROI, samples trees and returns to the
                start poses.
              </p>
            </section>
          </div>
        )}
        <footer className="page-footer">
          <span>
            RS1 <b> / </b> Forest survey workspace
          </span>
          <span>
            {state?.ros.sim_time !== undefined
              ? `Simulation time ${state.ros.sim_time.toFixed(1)} s`
              : "Awaiting simulation clock"}{" "}
            <b>·</b> ROS 2 Humble
          </span>
          {(attached || locked) && (
            <button disabled={busy || !connected} onClick={() => act("reset")}>
              ↻ {attached ? "Clear trails" : "New session"}
            </button>
          )}
        </footer>
      </main>
    </div>
  );
}
