export type XY = { x: number; y: number };
export type Calibration = {
  x_min: number;
  x_max: number;
  y_min: number;
  y_max: number;
};
export type Preset = {
  id: string;
  calibration_id: string;
  world_id: string;
  name: string;
  calibration: Calibration;
  url: string;
};
export type Plan = {
  session_id: string;
  world_id: string;
  image_id: string;
  calibration_id: string;
  start: XY;
  roi: XY;
  // Opposite (top-right) corner of the axis-aligned square ROI box. roi is the
  // first-placed entry corner (bottom-left) the robots reach in phase 1.
  roi_second?: XY | null;
  station_offset: XY;
  parrot_clearance: number;
};
export type SpawnPlan = Omit<Plan, "roi" | "station_offset">;
export type Deployment = {
  phase: "unspawned" | "spawning" | "deployed" | "failed" | "stale";
  can_spawn: boolean;
  message: string;
  plan: SpawnPlan | null;
  placements: Record<string, XY & { z: number }> | null;
  roi_selected?: boolean;
  requires_world_restart?: boolean;
};
export type Pose = XY & { yaw: number; stamp: number; fresh: boolean };
export type TreeMarker = {
  x: number;
  y: number;
  z: number;
  type: string;
  observations: number;
};
export type GroundTruthTree = {
  name: string;
  type: string;
  x: number;
  y: number;
  z: number;
};
export type GroundTruthObstacle = GroundTruthTree;
export type SafetyState = {
  fresh: boolean;
  service_ready: boolean;
  reset_ready: boolean;
  instance_id?: string;
  revision?: number;
  husky_stopped?: boolean;
  parrot_stopped?: boolean;
};
export type State = {
  mode: "managed" | "external" | "deploy";
  deployment?: Deployment;
  can_start?: boolean;
  configured_roi?: XY & {
    z: number;
    frame_id: string;
    min?: XY;
    max?: XY;
    progress?: number;
  };
  phase: string;
  message: string;
  plan: Plan | null;
  placements: Record<string, XY & { z: number }> | null;
  trails: Record<string, number[][]>;
  events: { phase: string; message: string; at: number }[];
  process_running: boolean;
  ros: {
    safety?: SafetyState;
    available: boolean;
    error?: string;
    sim_time?: number;
    robots: Record<string, Partial<Pose>>;
    checks: Record<string, boolean>;
    controller?: { connected: boolean };
    mission?: {
      session_id?: string;
      instance_id?: string;
      state?: string;
      parrot_stage: string;
      husky_stage: string;
      control_mode?: "auto" | "husky1" | "parrot1" | "hold";
      manual_input?: "keyboard" | "controller";
      input_revision?: number;
      cleanup_complete?: boolean;
      returning_home?: boolean;
      message?: string;
      roi?: {
        frame_id: string;
        x: number;
        y: number;
        z: number;
        min?: XY;
        max?: XY;
        progress: number;
      };
    };
    total_agb_kg?: number | null; // Aggregated AGB (kg)
    biomass?: {
      valid: boolean;
      total_agb_kg: number | null;
      tree_count: number;
      measured_count: number;
      estimated_count: number;
      message: string;
    } | null;
    trees?: TreeMarker[];
    sampling_target?:
      | (XY & {
          tree_index: number;
          refined: boolean;
          min_range_m: number;
          max_range_m: number;
        })
      | null;
    alignment?: {
      ready?: Record<string, boolean>;
      errors?: Record<string, { xy_m: number; yaw_rad: number }>;
    };
  };
};
