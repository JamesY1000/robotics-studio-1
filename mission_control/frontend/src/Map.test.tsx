import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { TerrainMap } from "./Map";
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
  mode: "external",
  phase: "idle",
  message: "Ready",
  can_start: true,
  plan: null,
  placements: null,
  trails: {},
  events: [],
  process_running: false,
  ros: {
    available: true,
    checks: {},
    robots: {
      husky1: { x: 90, y: -48, yaw: 0, stamp: 10, fresh: true },
      parrot1: { x: 89, y: -47, yaw: 0.5, stamp: 10, fresh: false },
    },
  },
};

function renderMap(value: State) {
  return renderToStaticMarkup(
    <TerrainMap
      preset={preset}
      start={{ x: 1, y: 2 }}
      roi={{ x: 88, y: -48 }}
      station={null}
      mode="roi"
      locked={false}
      state={value}
      onPlace={() => {}}
    />,
  );
}

describe("map with independently launched robots", () => {
  it("renders estimated robot positions without a backend-owned plan", () => {
    const markup = renderMap(state);
    expect(markup).toContain("translate(90 -48)");
    expect(markup).toContain("translate(89 -47)");
    expect(markup).toContain('opacity="0.45"');
    expect(markup).not.toContain("translate(1 2)");
    expect(markup).not.toContain("stroke-dasharray");
    expect(markup).toContain("Click to place the point ROI");
  });

  it("keeps spawn previews for an unstarted managed session", () => {
    const markup = renderMap({ ...state, mode: "managed" });
    expect(markup).toContain("translate(1 2)");
    expect(markup).toContain("stroke-dasharray");
    expect(markup).not.toContain("translate(90 -48)");
  });

  it("switches deploy previews to live TF when a deployment exists", () => {
    const unspawned = renderMap({ ...state, mode: "deploy" });
    expect(unspawned).toContain("translate(1 2)");
    expect(unspawned).toContain("stroke-dasharray");
    const deployed = renderMap({
      ...state,
      mode: "deploy",
      deployment: {
        phase: "deployed",
        can_spawn: false,
        message: "Ready",
        placements: null,
        plan: {
          session_id: "session-id",
          world_id: preset.world_id,
          image_id: preset.id,
          calibration_id: preset.calibration_id,
          start: { x: 1, y: 2 },
          parrot_clearance: 4,
        },
      },
    });
    expect(deployed).toContain("translate(90 -48)");
    expect(deployed).not.toContain("translate(1 2)");
    expect(deployed).not.toContain("stroke-dasharray");
  });
});
