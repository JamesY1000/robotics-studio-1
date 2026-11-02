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
export type State = {
  mode: "managed" | "external" | "deploy";
  deployment?: Deployment;
  can_start?: boolean;
  configured_roi?: XY & { z: number; frame_id: string };
  phase: string;
  message: string;
  plan: Plan | null;
  placements: Record<string, XY & { z: number }> | null;
  trails: Record<string, number[][]>;
  events: { phase: string; message: string; at: number }[];
  process_running: boolean;
  ros: {
    available: boolean;
    error?: string;
    sim_time?: number;
    robots: Record<string, Partial<Pose>>;
    checks: Record<string, boolean>;
    mission?: {
      session_id?: string;
      instance_id?: string;
      state?: string;
      parrot_stage: string;
      husky_stage: string;
      control_mode?: "auto" | "husky1" | "parrot1" | "hold";
      cleanup_complete?: boolean;
      message?: string;
    };
    alignment?: {
      ready?: Record<string, boolean>;
      errors?: Record<string, { xy_m: number; yaw_rad: number }>;
    };
  };
};
