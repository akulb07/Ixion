// The current project describes topology and a few dimensions, not CAD poses.
export function assemblyLayout(project) {
  const radius = project.assembly.wheel_radius_m;
  const track = project.assembly.track_width_m;
  if (
    ![radius, track].every(
      (value) => Number.isFinite(value) && value >= 0.00001 && value <= 1000,
    )
  )
    throw new Error(
      "Assembly preview dimensions must be between 0.00001 m and 1000 m.",
    );
  const wheels = project.components.filter((part) => part.category === "wheel");
  if (wheels.length !== 2)
    throw new Error(
      "The reference assembly preview needs two wheel components.",
    );
  const length = track * 1.6,
    width = track * 0.7,
    deck = radius * 1.65;
  const counters = new Map();
  const parts = project.components.map((part) => {
    const index = counters.get(part.category) || 0;
    counters.set(part.category, index + 1);
    const side = index % 2 ? -1 : 1;
    const base = {
      id: part.id,
      category: part.category,
      shape: "box",
      size: [0.04, 0.025, 0.008],
      position: [0, 0, deck + 0.015],
      declared: false,
    };
    switch (part.category) {
      case "wheel":
        return {
          ...base,
          shape: "cylinder",
          size: [radius, radius * 0.6],
          position: [0, (side * track) / 2, radius],
          declared: true,
        };
      case "chassis":
        return {
          ...base,
          size: [length, width, radius * 0.35],
          position: [0, 0, deck],
        };
      case "caster":
        return {
          ...base,
          shape: "sphere",
          size: [radius * 0.35],
          position: [-length * 0.4, 0, radius * 0.35],
        };
      case "motor":
        return {
          ...base,
          shape: "cylinder",
          size: [radius * 0.35, track * 0.25],
          position: [0, side * track * 0.28, radius],
        };
      case "encoder":
        return {
          ...base,
          size: [radius * 0.6, radius * 0.25, radius * 0.6],
          position: [0, side * track * 0.14, radius],
        };
      case "battery":
        return {
          ...base,
          size: [length * 0.35, width * 0.42, radius * 0.7],
          position: [-length * 0.22, 0, deck + radius * 0.6],
        };
      case "mcu":
        return {
          ...base,
          size: [length * 0.28, width * 0.3, radius * 0.15],
          position: [length * 0.15, 0, deck + radius * 0.3],
        };
      case "motor_driver":
        return {
          ...base,
          size: [length * 0.14, width * 0.3, radius * 0.15],
          position: [0, width * 0.28, deck + radius * 0.3],
        };
      case "sensor":
        return {
          ...base,
          size: [length * 0.1, width * 0.25, radius * 0.25],
          position: [
            length * (0.3 - index * 0.15),
            -width * 0.25,
            deck + radius * 0.3,
          ],
        };
      case "regulator":
        return {
          ...base,
          size: [length * 0.12, width * 0.22, radius * 0.15],
          position: [-length * 0.05, -width * 0.28, deck + radius * 0.3],
        };
      default:
        return {
          ...base,
          size: [length * 0.1, width * 0.18, radius * 0.15],
          position: [
            -length * 0.3,
            width * 0.28,
            deck + radius * (0.3 + index * 0.3),
          ],
        };
    }
  });
  return {
    parts,
    radius,
    track,
    span: Math.max(length, track + radius * 0.6, radius * 5),
    center: [0, 0, radius * 1.5],
  };
}
