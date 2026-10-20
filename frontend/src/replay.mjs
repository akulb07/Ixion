// The cursor holds recorded samples. It never interpolates sensor deliveries.
export function sampleIndexAt(samples, time) {
  if (!samples.length) return -1;
  let lo = 0,
    hi = samples.length;
  while (lo < hi) {
    const mid = Math.floor((lo + hi) / 2);
    if (samples[mid].time_s <= time) lo = mid + 1;
    else hi = mid;
  }
  return lo - 1;
}

export function deliveredEvents(events, time, limit = 8) {
  const end = sampleIndexAt(events, time) + 1;
  return events.slice(Math.max(0, end - limit), end);
}

export function replayTransform(distance, radius, grade = 0) {
  if (
    ![distance, radius, grade].every(Number.isFinite) ||
    radius <= 0 ||
    Math.abs(distance) > 1e6 ||
    grade < 0 ||
    grade >= Math.PI / 2
  )
    throw new Error("Recorded position is outside the 3D display bounds.");
  return {
    x: distance * Math.cos(grade),
    z: distance * Math.sin(grade),
    pitch: -grade,
    // Straight-line, no-slip fixture; this isn't decoded encoder feedback.
    wheelAngle: (distance / radius) % (2 * Math.PI),
  };
}
