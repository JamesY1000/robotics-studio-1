import type { Plan, Preset, SpawnPlan, State, XY } from "./types";

// Keep deployment independent of any ROI or station retained in a managed draft.
export function spawnRequest(
  sessionId: string,
  preset: Preset | undefined,
  start: XY | null,
  clearance: number,
): SpawnPlan | null {
  if (!preset || !start) return null;
  return {
    session_id: sessionId,
    world_id: preset.world_id,
    image_id: preset.id,
    calibration_id: preset.calibration_id,
    start,
    parrot_clearance: clearance,
  };
}

export function missionRequest(
  mode: State["mode"] | undefined,
  roi: XY | null,
  roiSecond: XY | null,
  roiEdited: boolean,
  plan: Plan | null,
): Plan | { roi?: XY; roi_second?: XY } | null {
  const roiBox = (roi: XY): { roi: XY; roi_second?: XY } =>
    roiSecond ? { roi, roi_second: roiSecond } : { roi };
  if (mode === "deploy") return roi ? roiBox(roi) : null;
  if (mode === "external")
    return roiEdited && roi ? roiBox(roi) : {};
  return plan;
}

export function canStartAttached(state: State | null, roi: XY | null): boolean {
  if (!state?.can_start || state.phase !== "idle") return false;
  if (state.mode === "deploy" && state.deployment?.phase !== "deployed")
    return false;
  return roi
    ? Number.isFinite(roi.x) && Number.isFinite(roi.y)
    : state.mode === "external" && !!state.configured_roi;
}
