import { describe, expect, it } from "vitest";
import { canStartAttached, missionRequest, spawnRequest } from "./workflow";
import type { Preset, State } from "./types";

const preset: Preset = {
  id: "forest-overhead",
  calibration_id: "forest-edges-v1",
  world_id: "v2_forest",
  name: "Forest",
  calibration: { x_min: -100, x_max: 100, y_min: -100, y_max: 100 },
  url: "/api/images/forest-overhead",
};
const state: State = {
  mode: "deploy",
  phase: "idle",
  message: "Ready",
  can_start: true,
  configured_roi: { x: 90, y: -48, z: 4, frame_id: "world" },
  plan: null,
  placements: null,
  deployment: {
    phase: "deployed",
    can_spawn: false,
    message: "Robots ready",
    plan: spawnRequest("session-id", preset, { x: 90, y: -48 }, 4),
    placements: null,
    roi_selected: false,
  },
  trails: {},
  events: [],
  process_running: true,
  ros: { available: true, checks: {}, robots: {} },
};

describe("staged deployment", () => {
  it("can construct a spawn request before any ROI or station is chosen", () => {
    const spawn = spawnRequest("session-id", preset, { x: 90, y: -48 }, 4);
    expect(spawn).toMatchObject({
      session_id: "session-id",
      image_id: "forest-overhead",
      calibration_id: "forest-edges-v1",
      world_id: "v2_forest",
      start: { x: 90, y: -48 },
      parrot_clearance: 4,
    });
    expect(spawn).not.toHaveProperty("roi");
    expect(spawn).not.toHaveProperty("station_offset");
    expect(spawnRequest("session-id", preset, null, 4)).toBeNull();
  });

  it("requires an operator ROI even when the new coordinator has a placeholder target", () => {
    expect(canStartAttached(state, null)).toBe(false);
    expect(missionRequest("deploy", null, null, false, null)).toBeNull();
    expect(canStartAttached(state, { x: 88, y: -48 })).toBe(true);
    expect(missionRequest("deploy", { x: 88, y: -48 }, null, true, null)).toEqual({
      roi: { x: 88, y: -48 },
    });
    expect(
      missionRequest(
        "deploy",
        { x: 88, y: -48 },
        { x: 98, y: -58 },
        true,
        null,
      ),
    ).toEqual({
      roi: { x: 88, y: -48 },
      roi_second: { x: 98, y: -58 },
    });
  });

  it("cannot start while spawning, disconnected from ready ROS, or already executing", () => {
    const roi = { x: 88, y: -48 };
    expect(
      canStartAttached(
        { ...state, deployment: { ...state.deployment!, phase: "spawning" } },
        roi,
      ),
    ).toBe(false);
    expect(canStartAttached({ ...state, can_start: false }, roi)).toBe(false);
    expect(canStartAttached({ ...state, phase: "executing" }, roi)).toBe(false);
    expect(canStartAttached(state, { x: NaN, y: 1 })).toBe(false);
  });

  it("preserves the externally configured frame and altitude when the ROI is untouched", () => {
    const external: State = {
      ...state,
      mode: "external",
      configured_roi: { x: 2, y: 3, z: 7, frame_id: "parrot1/map" },
    };
    expect(canStartAttached(external, null)).toBe(true);
    expect(missionRequest("external", null, null, false, null)).toEqual({});
    expect(missionRequest("external", { x: 88, y: -48 }, null, true, null)).toEqual({
      roi: { x: 88, y: -48 },
    });
  });
});
