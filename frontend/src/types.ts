export type Pose = { x: number; y: number; theta: number };
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
