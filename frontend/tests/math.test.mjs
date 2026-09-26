import test from "node:test";
import assert from "node:assert/strict";
import { scanSegments, nextTime, sensorMount } from "../src/math.mjs";
test("LiDAR uses capture pose and translated, rotated mount; null rays omitted", () => {
  const rays = scanSegments(
    { ranges: [2, null], angles: [0, 1], hits: [true, false] },
    { x: 3, y: 4, theta: Math.PI / 2 },
    { x: 1, y: 0, theta: -Math.PI / 2 },
  );
  assert.equal(rays.length, 1);
  assert.ok(Math.abs(rays[0].x - 3) < 1e-12);
  assert.ok(Math.abs(rays[0].y - 5) < 1e-12);
  assert.ok(Math.abs(rays[0].endX - 5) < 1e-12);
  assert.ok(Math.abs(rays[0].endY - 5) < 1e-12);
});
test("Replay reaches the exact terminal time without overshooting", () => {
  assert.equal(nextTime(1.9, 2, 4), 2);
  assert.equal(nextTime(0.1, 2, 0.5), 0.15);
});
test("Built-in wheel frames use signed half separation; rendering is bounded", () => {
  const robot = { wheel_separation: 0.3, mounts: [] };
  assert.equal(sensorMount(robot, "left_wheel").y, 0.15);
  assert.equal(sensorMount(robot, "right_wheel").y, -0.15);
  assert.deepEqual(sensorMount(robot, "base"), { x: 0, y: 0, theta: 0 });
  const scan = {
    ranges: Array(10000).fill(1),
    angles: Array(10000).fill(0),
    hits: Array(10000).fill(true),
  };
  assert.ok(scanSegments(scan, { x: 0, y: 0, theta: 0 }).length <= 360);
});
