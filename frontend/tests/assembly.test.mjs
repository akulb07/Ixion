import { test } from "node:test";
import assert from "node:assert/strict";
import { assemblyLayout } from "../src/assembly.mjs";

const project = {
  assembly: { wheel_radius_m: 0.0325, track_width_m: 0.18 },
  components: [
    { id: "left_wheel", category: "wheel" },
    { id: "right_wheel", category: "wheel" },
    { id: "body", category: "chassis" },
    { id: "board", category: "mcu" },
  ],
};
test("preview uses declared track and wheel radii while marking assumed shapes", () => {
  const result = assemblyLayout(project);
  assert.equal(result.parts[0].position[1], 0.09);
  assert.equal(result.parts[1].position[1], -0.09);
  assert.equal(result.parts[0].size[0], 0.0325);
  assert.equal(result.parts[0].declared, true);
  assert.equal(result.parts[2].declared, false);
  assert.equal(result.parts[3].id, "board");
  assert.equal(project.components[0].position, undefined);
});
test("preview dimensions follow edits and reject unsupported or unbounded input", () => {
  const edited = structuredClone(project);
  edited.assembly.track_width_m = 0.3;
  assert.equal(assemblyLayout(edited).parts[0].position[1], 0.15);
  for (const value of [0, -1, Infinity, 1e100]) {
    const invalid = structuredClone(project);
    invalid.assembly.wheel_radius_m = value;
    assert.throws(() => assemblyLayout(invalid), /dimensions/);
  }
  assert.throws(
    () => assemblyLayout({ ...project, components: [] }),
    /two wheel/,
  );
});
