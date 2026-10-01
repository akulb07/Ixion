export type Pose = { x: number; y: number; theta: number };
export type PIDConfig = {
  kp: number;
  ki: number;
  kd: number;
  output_min: number;
  output_max: number;
  derivative_time_constant: number;
  integral_limit: number;
};
export type Navigation = {
  path: { x: number; y: number }[];
  encoder: string;
  max_steps: number;
  max_speed: number;
  lookahead: number;
  clearance: number;
  max_yaw_rate?: number;
  goal_tolerance?: number;
  max_sensor_age?: number;
};
export type State = {
  pose: Pose;
  time: number;
  wheels: { left: number; right: number };
  twist: { linear: number; angular: number };
};
export type Sensor = {
  type: string;
  name: string;
  rate_hz: number;
  latency: number;
  frame?: string;
  rays?: number;
};
export type Fault = {
  name: string;
  kind: string;
  target: string;
  start: number;
  end: number | null;
  magnitude: number;
  delay_steps: number;
};
export type Config = {
  name: string;
  seed: number;
  robot: {
    name: string;
    initial_pose: Pose;
    footprint_radius: number;
    wheel_radius: number;
    wheel_separation: number;
    mounts: { name: string; pose: Pose }[];
  };
  environment: {
    name: string;
    width: number;
    height: number;
    obstacles: (
      | {
          type: "rectangle";
          x: number;
          y: number;
          width: number;
          height: number;
        }
      | { type: "circle"; x: number; y: number; radius: number }
    )[];
  };
  simulation: { dt: number; integrator: string; collision: { mode: string } };
  commands: { left: number; right: number; steps: number }[];
  navigation?: Navigation | null;
  wheel_controller?: {
    encoder: string;
    feedforward: number;
    left: PIDConfig;
    right: PIDConfig;
  } | null;
  sensors: Sensor[];
  faults: Fault[];
};
export type Job = {
  provenance?: {
    captured_utc: string;
    capture_stage: string;
    python: { version: string; implementation: string };
    dependencies: Record<string, string>;
    source: {
      sha256: string | null;
      git_revision: string | null;
      git_dirty: boolean | null;
      notes: string[];
    };
  };
  id: string;
  name: string;
  status: string;
  seed: number;
  created_utc: string;
  error: string | null;
  metrics: Record<string, number> | null;
  cancel_requested: boolean;
  navigation_outcome?: string | null;
};
export type Reading = {
  sensor: string;
  kind: string;
  frame: string;
  capture_time: number;
  delivery_time: number;
  sequence: number;
  angles?: number[];
  ranges?: (number | null)[];
  hits?: boolean[];
  left_ticks?: number | null;
  right_ticks?: number | null;
  gyro_z?: number | null;
};
export type Frame = {
  navigation?: {
    time: number;
    estimate: Pose;
    estimate_time: number | null;
    measurement_fresh: boolean;
    tracking: {
      goal_distance: number;
      cross_track_error: number;
      reached: boolean;
    };
  } | null;
  state: State;
  sensors: Reading[];
  sensor_capture_poses: Record<string, Pose>;
  active_faults: Fault[];
};
export type Trajectory = {
  states: State[];
  sampled: boolean;
  total_states: number;
};
export type Preset = { id: string; title: string; config: Config };
export type PlanningDocument = {
  format_version: number;
  software_version: string;
  request_sha256: string;
  planning_radius_m: number;
  request: {
    environment: Config["environment"];
    start: { x: number; y: number };
    goal: { x: number; y: number };
    algorithm: string;
    seed: number;
    budget: number;
    footprint_radius: number;
    clearance: number;
    resolution: number;
    step_size: number;
    goal_bias: number;
    rewire_radius: number;
  };
  result: {
    algorithm: string;
    status: string;
    path: { x: number; y: number }[];
    length: number;
    expanded: number;
    collision_checks: number;
    seed: number | null;
  };
};
export type Comparison = {
  format_version: number;
  software_version: string;
  created_utc: string;
  baseline_id: string;
  same_setup: boolean;
  same_software: boolean;
  same_source?: boolean | null;
  same_dependencies?: boolean | null;
  runs: (Job & {
    config: Config;
    software_version: string;
    config_sha256: string;
  })[];
  metrics: {
    name: string;
    values: (number | null)[];
    deltas: (number | null)[];
  }[];
  config_differences: {
    path: string;
    values: { present: boolean; value: unknown }[];
  }[];
};
export type BatchTrial = {
  index: number;
  group: string;
  parameters: Record<string, unknown>;
  seed: number;
  status: string;
  error: string | null;
  run_id: string | null;
  config_sha256: string;
  metrics: Record<string, number>;
};
export type BatchPreview = {
  expected_trials: number;
  resources: {
    steps: number;
    duration_s: number;
    sensor_readings: number;
    lidar_rays: number;
  };
  trials: BatchTrial[];
};
export type BatchSummary = {
  id: string;
  name: string;
  status: string;
  created_utc: string;
  expected_trials: number;
};
export type Batch = BatchSummary &
  BatchPreview & {
    finished_trials: number;
    error: string | null;
    cancel_requested: boolean;
    groups: Record<
      string,
      {
        trials: number;
        failed_trials: number;
        failure_rate: number;
        failure_rate_wilson95: [number, number];
        metrics: Record<
          string,
          {
            measured_trials: number;
            mean: number;
            sample_stddev: number | null;
            median: number;
            p05: number;
            p95: number;
          }
        >;
      }
    >;
  };
