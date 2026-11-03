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
      roiSecond={null}
      station={null}
      mode="roi"
      locked={false}
      state={value}
      groundTruth={[]}
      obstacles={[]}
      onPlace={() => {}}
      onRoi={() => {}}
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
    expect(markup).toContain("Click to place the ROI box");
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

  it("renders tree markers from the published tree map", () => {
    const markup = renderMap({
      ...state,
      ros: {
        ...state.ros,
        trees: [
          { x: 10, y: 20, z: 1, type: "OAK_TREE", observations: 3 },
          { x: 15, y: 25, z: 1, type: "PINE_TREE", observations: 2 },
        ],
      },
    });
    expect(markup).toContain("tree-marker");
    expect(markup).toContain("translate(10 20)");
    expect(markup).toContain("translate(15 25)");
    expect(markup).toContain("oak");
    expect(markup).toContain("pine");
  });

  it("shows the located sampling target and live range while assisting", () => {
    const value: State = {
      ...state,
      phase: "manual_assistance",
      ros: {
        ...state.ros,
        sampling_target: {
          x: 87,
          y: -48,
          tree_index: 2,
          refined: true,
          min_range_m: 1,
          max_range_m: 4,
        },
      },
    };
    const markup = renderMap(value);
    expect(markup).toContain('data-testid="sampling-target"');
    expect(markup).toContain("Tree 2 · located · 3.00 m");
    expect(markup).toContain('r="4"');
    expect(markup).toContain('r="1"');
    value.ros.robots = { husky1: { fresh: false } };
    expect(renderMap(value)).not.toContain("3.00 m");
    value.ros.sampling_target!.refined = false;
    expect(renderMap(value)).toContain("mapped estimate");
  });

  it("renders ground-truth trees and clutter in distinct colours", () => {
    const markup = renderToStaticMarkup(
      <TerrainMap
        preset={preset}
        start={{ x: 1, y: 2 }}
        roi={{ x: 88, y: -48 }}
        roiSecond={null}
        station={null}
        mode="roi"
        locked={false}
        state={state}
        groundTruth={[{ name: "t1", type: "oak", x: 5, y: 6, z: 0 }]}
        obstacles={[
          { name: "b1", type: "bush", x: -5, y: -6, z: 0 },
          { name: "r1", type: "rock", x: -7, y: -8, z: 0 },
        ]}
        onPlace={() => {}}
        onRoi={() => {}}
      />,
    );
    expect(markup).toContain("ground-truth-marker");
    expect(markup).toContain('fill="#e91e63"');
    expect(markup).toContain("obstacle-marker");
    expect(markup).toContain("translate(-5 -6)");
    expect(markup).toContain("translate(-7 -8)");
    // Bushes are circles, rocks are polygons, each in its own colour.
    expect(markup).toContain('fill="#ff9800"');
    expect(markup).toContain('fill="#6a1b9a"');
  });
});

describe("AGB from the current sampled ROI", () => {
  const biomass: NonNullable<State["ros"]["biomass"]> = {
    valid: true,
    total_agb_kg: 123.4,
    tree_count: 3,
    measured_count: 1,
    estimated_count: 1,
    message:
      "Uses measured DBH with configured species heights and wood densities",
  };
  function renderBiomass(value: typeof biomass | null, edited = false) {
    const withSurvey: State = {
      ...state,
      ros: {
        ...state.ros,
        biomass: value,
        mission: {
          state: "sampling",
          parrot_stage: "Done",
          husky_stage: "Measuring",
          roi: {
            frame_id: "world",
            x: 88,
            y: -48,
            z: 0,
            progress: 1,
            min: { x: 88, y: -58 },
            max: { x: 98, y: -48 },
          },
        },
      },
    };
    return renderToStaticMarkup(
      <TerrainMap
        preset={preset}
        start={null}
        roi={{ x: edited ? 87 : 88, y: -48 }}
        roiSecond={{ x: 98, y: -58 }}
        station={null}
        mode="roi"
        locked={true}
        state={withSurvey}
        groundTruth={[]}
        obstacles={[]}
        onPlace={() => {}}
        onRoi={() => {}}
      />,
    );
  }

  it("labels partial coverage and keeps the terrain visible", () => {
    const markup = renderBiomass(biomass);
    expect(markup).toContain("Sampled AGB estimate: 123.4 kg");
    expect(markup).toContain("1/3 trees included");
    expect(markup).toContain("configured species heights and wood densities");
    expect(markup).not.toContain('fill="#22a447"');
    expect(markup).not.toContain("Total AGB");
  });

  it("uses the total label only when every tree has an estimate", () => {
    const markup = renderBiomass({
      ...biomass,
      measured_count: 3,
      estimated_count: 3,
    });
    expect(markup).toContain("Total AGB estimate: 123.4 kg");
    expect(markup).toContain("3/3 trees included");
    expect(markup).toContain('fill="#22a44733"');
  });

  it("shows missing measurements and invalid estimates without inventing zero biomass", () => {
    const empty = renderBiomass({
      ...biomass,
      total_agb_kg: null,
      measured_count: 0,
      estimated_count: 0,
    });
    expect(empty).toContain("Awaiting DBH samples");
    expect(empty).not.toContain("0 kg");
    const invalid = renderBiomass({
      ...biomass,
      valid: false,
      total_agb_kg: NaN,
    });
    expect(invalid).toContain("AGB unavailable");
    expect(invalid).not.toContain("NaN");
  });

  it("does not attach old totals to an edited ROI", () => {
    expect(renderBiomass(biomass, true)).not.toContain(
      'data-testid="roi-agb-label"',
    );
    expect(renderBiomass(null)).not.toContain('data-testid="roi-agb-label"');
  });
});
