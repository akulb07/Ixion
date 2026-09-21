# Pure Pursuit path tracking

PurePursuit accepts an estimated Pose2, never a robot or world handle. It projects
onto the polyline with nondecreasing arc-length progress, selects a target one
lookahead distance along that polyline, and transforms it into the robot frame.
Curvature is `2*y_local / distance_to_target²`; angular rate is curvature times
forward speed. Speed is bounded by max_speed, distance-to-goal braking (gain 1/s),
and max_yaw_rate. Targets more than 60 degrees off the heading trigger bounded
rotation in place. Arrival requires both endpoint proximity and final-path progress.

This is arc-length lookahead, not circle/segment intersection lookahead. It does
not guarantee collision-free tracking of a collision-free polyline: corners are
cut, so planning needs clearance and execution still needs swept collision checks.
No final-heading constraint, reverse driving, dynamic-obstacle avoidance, or
delay compensation is implemented. Global projection may jump ahead on crossing
or overlapping paths; use non-self-intersecting routes for this implementation.

The milestone_8 laboratory connects A*, Pure Pursuit, wheel actuators, simulated
encoders, and encoder odometry through their public APIs. Only the physics/sensor
and collision layers access truth. The tracker uses the latest delivered odometry
estimate. This explicit laboratory loop is separate from the prescribed-command
CLI; a unified navigation scenario configuration is future work.
