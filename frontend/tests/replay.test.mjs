import { test } from "node:test";
import assert from "node:assert/strict";
import {
  deliveredEvents,
  replayTransform,
  sampleIndexAt,
} from "../src/replay.mjs";

test("cursor holds the last recorded boundary without revealing future samples", () => {
  const samples = [0, 0.001, 0.002, 0.0023].map((time_s) => ({ time_s }));
  assert.equal(sampleIndexAt(samples, -0.01), -1);
  assert.equal(sampleIndexAt(samples, 0), 0);
  assert.equal(sampleIndexAt(samples, 0.0019), 1);
  assert.equal(sampleIndexAt(samples, 0.0023), 3);
  assert.equal(sampleIndexAt(samples, 1), 3);
  assert.equal(sampleIndexAt([], 0), -1);
});
test("delivered events include simultaneous deliveries only at their timestamp", () => {
  const events = [0, 0.0012, 0.0012, 0.002].map((time_s, id) => ({
    time_s,
    id,
  }));
  assert.deepEqual(
    deliveredEvents(events, 0.001).map((e) => e.id),
    [0],
  );
  assert.deepEqual(
    deliveredEvents(events, 0.0012).map((e) => e.id),
    [0, 1, 2],
  );
  assert.deepEqual(
    deliveredEvents(events, 0.003, 2).map((e) => e.id),
    [2, 3],
  );
});
test("3D replay maps recorded travel onto the uphill fixture and derives no-slip spin", () => {
  const pose = replayTransform(2, 0.1, Math.PI / 6);
  assert.ok(Math.abs(pose.x - Math.sqrt(3)) < 1e-12);
  assert.ok(Math.abs(pose.z - 1) < 1e-12);
  assert.equal(pose.pitch, -Math.PI / 6);
  assert.equal(pose.wheelAngle, 20 % (2 * Math.PI));
  for (const args of [
    [Infinity, 0.1, 0],
    [0, 0, 0],
    [0, 0.1, Math.PI / 2],
    [1e8, 0.1, 0],
  ])
    assert.throws(() => replayTransform(...args), /display bounds/);
});
