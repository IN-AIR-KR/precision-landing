const XYZ = ["x", "y", "z"];
const POSE = XYZ.map((axis) => "pose_position_" + axis);

function px4(topics, name) {
  const prefix = "/fmu/out/" + name;
  return topics[prefix] || Object.entries(topics).find(([topic]) =>
    topic.startsWith(prefix + "_v") && /^\d+$/.test(topic.slice(prefix.length + 2)))?.[1];
}

function rows(topic, keys, valid = () => true) {
  return topic?.t.map((t, i) => {
    const values = keys.map((key) => topic.fields[key]?.[i]);
    return valid(topic.fields, i) && values.every(Number.isFinite) ? [t, ...values] : [t];
  }) ?? [];
}

export function sample(rows, t, maxGap, interpolate = true) {
  let lo = 0, hi = rows.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (rows[mid][0] <= t) lo = mid + 1;
    else hi = mid;
  }
  const a = rows[lo - 1], b = rows[lo];
  if (!a || a.length === 1 || t - a[0] > maxGap) return null;
  if (!interpolate || !b || b.length === 1 || b[0] - a[0] > maxGap) return a.slice(1);
  const alpha = (t - a[0]) / (b[0] - a[0]);
  return a.slice(1).map((v, i) => v + alpha * (b[i + 1] - v));
}

const toEnu = ([t, n, e, d]) => n === undefined ? [t] : [t, e, n, -d];

function recordingOrigin(local, status) {
  const valid = local.filter((row) => row.length === 4);
  if (!valid.length) return {position: [0, 0, 0], source: "local"};

  const states = status?.fields.arming_state;
  // VehicleStatus defines ARMED as 2. ARM-triggered bags usually start already armed.
  const armIndex = states?.findIndex((state, i) => state === 2 && states[i - 1] === 1) ?? -1;
  if (armIndex >= 0) {
    const t = status.t[armIndex];
    const after = local.findIndex((row) => row[0] >= t);
    if (after >= 0 && local[after].length === 4) {
      const next = local[after];
      const prev = local[after - 1];
      if (next[0] === t || (prev?.length === 4 && next[0] - prev[0] <= 0.3)) {
        return {position: sample(local, t, 0.3), source: "arm"};
      }
    }
    const nearest = valid.reduce((a, b) => Math.abs(a[0] - t) <= Math.abs(b[0] - t) ? a : b);
    return {position: nearest.slice(1), source: "nearArm"};
  }
  return {position: valid[0].slice(1), source: "first"};
}

export function sceneData(topics) {
  const local = rows(px4(topics, "vehicle_local_position"), XYZ, (f, i) =>
    (!f.xy_valid || f.xy_valid[i] === 1) && (!f.z_valid || f.z_valid[i] === 1)).map(toEnu);
  const gazeboVehicle = rows(topics["/pl/sim/vehicle_pose"], POSE);
  let offset = null;
  // Align Gazebo's origin to PX4's local origin once, at the first common position.
  // A fixed offset preserves subsequent position drift in the comparison.
  for (const [t, e, n, u] of local) {
    if (e === undefined) continue;
    const gazebo = sample(gazeboVehicle, t, 0.15);
    if (!gazebo) continue;
    offset = [e, n, u].map((v, i) => v - gazebo[i]);
    break;
  }
  const marker = rows(topics["/pl/sim/marker_pose"], POSE);
  const {position: origin, source: originSource} = recordingOrigin(local, px4(topics, "vehicle_status"));
  const shift = ([t, ...xyz]) => [t, ...xyz.map((v, i) => v - origin[i])];
  return {
    originSource,
    drone: local.map(shift),
    attitude: rows(px4(topics, "vehicle_attitude"), ["q_1", "q_2", "q_3", "q_0"]),
    estimate: rows(topics["/pl/target_pose_world"], POSE).map(toEnu).map(shift),
    hasTruth: marker.some((row) => row.length > 1),
    truth: offset ? marker.map(([t, ...xyz]) => shift([t, ...xyz.map((v, i) => v + offset[i])])) : [],
  };
}
