# Wheel actuators

Each simulation tick samples the requested wheel rates in rad/s. A FIFO delays
commands by exactly `delay_steps * dt`, initially outputting zero. Per wheel,
commands within the inclusive deadzone become zero; other commands are multiplied
by gain. The result is clipped to the optional symmetric max_speed.

The response is `w_next = w + (target-w) * (1-exp(-dt/tau))`; tau=0 responds
immediately. The change is clipped to `max_acceleration * dt` when configured.
Speed is rad/s, acceleration rad/s², and time_constant is seconds. Left and right
parameters are independent. Defaults reproduce the ideal model exactly.

The new rate is held throughout that tick for kinematics, collision and sensors.
This endpoint hold is a discrete approximation, not continuous torque dynamics.
The velocity endpoint matches the unrestricted first-order analytical response;
position converges at first order as dt decreases. Combining lag and slew limits
is an explicitly discrete model. Mass does not determine motor torque or traction.

`actuators.json` records start time, requested/delayed/bounded target/applied rates,
and speed/acceleration limit flags for every attempted tick. A collision may
truncate the last attempted interval; use exported motion segments for actual
duration, and terminal state for the stopped rates. Encoder readings integrate
the applied motion. No controller accesses these internal actuator values yet.
