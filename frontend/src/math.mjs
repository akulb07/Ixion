// Local scan -> mount -> base -> world, using the robot pose at capture time.
export function sensorMount(robot, frame) {
  if (frame === "left_wheel")
    return { x: 0, y: robot.wheel_separation / 2, theta: 0 };
  if (frame === "right_wheel")
    return { x: 0, y: -robot.wheel_separation / 2, theta: 0 };
  return (
    robot.mounts.find((m) => m.name === frame)?.pose ?? { x: 0, y: 0, theta: 0 }
  );
}
export function scanSegments(reading, pose, mount = { x: 0, y: 0, theta: 0 }) {
  if (!pose || !reading.ranges || !reading.angles) return [];
  const c = Math.cos(pose.theta),
    s = Math.sin(pose.theta);
  const x = pose.x + c * mount.x - s * mount.y;
  const y = pose.y + s * mount.x + c * mount.y;
  const stride = Math.max(1, Math.ceil(reading.ranges.length / 360));
  return reading.ranges.flatMap((range, i) => {
    if (i % stride || range === null || !Number.isFinite(range)) return [];
    const angle = pose.theta + mount.theta + reading.angles[i];
    return [
      {
        x,
        y,
        endX: x + range * Math.cos(angle),
        endY: y + range * Math.sin(angle),
        hit: reading.hits[i],
      },
    ];
  });
}
export function nextTime(time, duration, speed) {
  return Math.min(duration, Math.round((time + 0.1 * speed) * 1e6) / 1e6);
}
