# Physical robot description and mounting frames

## Purpose and scope

Existing radius, separation, footprint, name and initial pose remain valid.
`model: differential_drive` identifies the supported type; unknown types fail.
New immutable fields are `body_mass` (10 kg default), `wheel_mass` (.25 kg per
wheel), optional `body_yaw_inertia` (kg m^2), and named `mounts`. Mass/inertia
values must be finite and positive. They are descriptive and exported, and do
not alter ideal prescribed-velocity motion. Torque, delay, friction and motor
limits belong to future actuator/dynamics models.

## Inertia equations and assumptions

For a uniform circular body of footprint radius b, `I_body=m_body*b^2/2`, unless
overridden. Wheels are thin solid disks of radius r, with spin axes along base y:

```text
I_wheel_spin = m_wheel*r^2/2
I_wheel_yaw_about_centre = m_wheel*r^2/4
I_wheel_yaw_about_base = m_wheel*r^2/4 + m_wheel*(L/2)^2
I_total_yaw = I_body + 2*I_wheel_yaw_about_base
m_total = m_body + 2*m_wheel
```

The added offset term follows the parallel-axis theorem. Defaults produce total
mass 10.5 kg, body yaw inertia .2 kg m^2, wheel spin inertia .0003125 kg m^2,
and total yaw inertia .2115625 kg m^2. These are geometric estimates, not a
calibrated multibody model. Override body inertia when known. The footprint is
a user-declared collision envelope and must cover all relevant hardware.

## Frames

`DifferentialDriveRobot.frame_transforms(pose)` returns world-from-base, left
wheel, right wheel, then configured mounts in stable order. Wheel origins are
`(0,+L/2)` and `(0,-L/2)` in base. Defaults are LiDAR at (.1,0,0) and IMU at
(0,0,0). These are mounting coordinates, not sensor simulations.

```yaml
robot:
  body_mass: 10.0
  wheel_mass: 0.25
  mounts:
    - name: lidar
      pose: {x: 0.12, y: 0.0, theta: 0.0}
    - name: payload
      pose: {x: 0.0, y: -0.1, theta: 1.5707963267948966}
```

Names must be unique lowercase identifiers; `world`, `base`, `left_wheel`,
`right_wheel` are reserved. Existing SE(2) composition computes world transforms.
Dynamic joints, frame-tree lookup, cameras and hardware transports are deferred.

## Tests

Tests check independent mass/inertia values, overrides, invalid inputs, default
wheel/mount world coordinates, custom mounts and name conflicts. Existing motion
regressions confirm unchanged ideal kinematics.
