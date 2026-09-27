export type Pose = { x: number; y: number; theta: number };
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
  sensors: Sensor[];
  faults: Fault[];
};
export type Job = {
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
