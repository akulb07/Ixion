# PID wheel feedback

PID accepts a setpoint, measurement, timestep and optional feedforward term.
`e = setpoint - measurement`, `P = kp*e`, and `I += ki*e*dt`.
The derivative is on measurement: `raw_D = -(measurement - previous)/dt`.
An exponential low-pass approximation uses `alpha = dt/(tau+dt)` and
`filtered_D += alpha*(raw_D-filtered_D)`; `D = kd*filtered_D`.
The first derivative sample is zero. A setpoint change causes no derivative kick.

Integral contribution is bounded by integral_limit. Conditional integration rejects
increments pushing further into output saturation. Output is the clamped sum of
P, I, D and feedforward. Anti-windup concerns the controller's output limits, not
hidden actuator saturation: configure compatible bounds. It does not compensate
for arbitrary actuator delay, traction limits, or unobservable faults.

The simulator's optional wheel_controller sees only delivered encoder readings.
Two complete cumulative tick pairs establish a wheel-rate measurement using capture
time differences. Dropouts bridge the whole gap. Left and right PID outputs are
held until the next complete pair. Startup output is zero; setpoint changes take
effect on the next encoder update. Delay means feedback is stale, intentionally.
Multiple delivered pairs are processed in delivery order with the current target.
Encoder quantization limits speed resolution; increase resolution or reduce sample
rate if differentiation produces excessive quantization noise.

Configured command rates become wheel-speed setpoints when feedback is enabled.
The actuator receives PID output. control.json exports timestamps and every PID
term, error, saturation flag, measurement and output. The independent PID class
can also control other scalar plants. reset clears all controller memory.
