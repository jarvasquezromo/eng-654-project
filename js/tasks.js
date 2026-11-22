// Task layer shared by steps 5–7 (no DOM, so it also runs in workers and in Node):
//  * the closed full-pose contour and its 2-D placement family,
//  * all IK candidates along the contour, validated with our FK, periodic angles resolved, limits applied,
//  * persistent branches (continuation on the joint torus),
//  * per-start-branch feasibility: nearest-configuration continuation and branch-aware graph search,
//    with the failure reason when a start cannot finish.
import { pointJacobian, conditioning, LIMITS } from './gofa.js';
import { solveIK, branchIndex, branchLabel } from './ik.js';
import { rpy, rotX, rotY, rotZ, mul, homog, rad } from './math.js';

const TWO_PI = 2 * Math.PI;
export const wrap = (a) => a - TWO_PI * Math.floor((a + Math.PI) / TWO_PI);

export const ORIENTATIONS = {
  down: { name: 'Down −z', rpy: [0, 180, 0] },
  fwd: { name: 'Forward +x', rpy: [0, 90, 0] },
  side: { name: 'Side +y', rpy: [-90, 0, 0] },
  tilt45: { name: 'Down, tilted 45° to +x', rpy: [0, 135, 0] },
};

export const DEFAULTS = {
  a: 0.12, b: 0.08, tilt: 15, spin: 20, N: 40, tcp: 0,
  orientA: 'down', orientB: 'fwd',
  z: 0.30, xRange: [-0.9, 0.9], yRange: [-0.9, 0.9], gridN: 19,
  center: [0.45, 0.0],
  dqMax: 20, margin: 0.03, L: 0.5,
};
const KEY = 'gofa-task-v1';
export function loadSettings() {
  try { const s = JSON.parse(localStorage.getItem(KEY) || 'null'); if (s) return { ...DEFAULTS, ...s }; } catch { /* storage may be unavailable */ }
  return { ...DEFAULTS };
}
export function saveSettings(s) { try { localStorage.setItem(KEY, JSON.stringify(s)); } catch { /* ignore */ } }

export const orientR = (key) => rpy(ORIENTATIONS[key].rpy.map(rad));

// ---------------------------------------------------------------- contour
// Ellipse (semi-axes a, b) in the plane perpendicular to the global tool direction, centred at c.
// Orientation along the contour: R(s) = R_g · Rz(spin·sin s) · Rx(tilt·sin s) · Ry(tilt·cos s).
// Every term is 2π-periodic, so position and orientation both close.
export function contourPoses(S, center, orient) {
  const Rg = orientR(orient), poses = [];
  for (let k = 0; k < S.N; k++) {
    const s = (TWO_PI * k) / S.N;
    const loc = [S.a * Math.cos(s), S.b * Math.sin(s), 0];
    const p = [0, 1, 2].map((i) => center[i] + Rg[i][0] * loc[0] + Rg[i][1] * loc[1] + Rg[i][2] * loc[2]);
    const R = mul(Rg, rotZ(rad(S.spin) * Math.sin(s)), rotX(rad(S.tilt) * Math.sin(s)), rotY(rad(S.tilt) * Math.cos(s)));
    poses.push(homog(R, p));
  }
  return poses;
}
export const centerOf = (S, xy) => [xy[0], xy[1], S.z];
export function gridCenters(S) {
  const out = [], n = S.gridN;
  for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
    out.push([S.xRange[0] + ((S.xRange[1] - S.xRange[0]) * i) / (n - 1), S.yRange[0] + ((S.yRange[1] - S.yRange[0]) * j) / (n - 1)]);
  }
  return out;
}

// ---------------------------------------------------------------- candidates
// For every contour sample: all IK solutions for the TCP pose (tool0 pose = TCP pose shifted back by tcp along z).
// Each candidate: q (q1,q2,q4,q5 ∈ (−π, π], q3 shifted into its range, q6 ∈ (−π, π]), within, b (branch label index),
// invKappa (only for candidates within limits), and the FK residuals.
export function candidatesAlong(S, poses, { stopOnEmpty = false, samples = 64 } = {}) {
  const out = [];
  for (const T of poses) {
    const z = [T[0][2], T[1][2], T[2][2]], p = [T[0][3], T[1][3], T[2][3]];
    const T0 = T.map((r) => r.slice());
    for (let i = 0; i < 3; i++) T0[i][3] = p[i] - S.tcp * z[i];
    const sols = solveIK(T0, { samples }).map((s) => ({
      q: s.q, within: s.within, b: branchIndex(s.branch), label: s.label,
      invKappa: s.within ? conditioning(pointJacobian(s.q, p), S.L).invKappa : NaN,
      position_error: s.position_error, rotation_error: s.rotation_error,
    }));
    out.push(sols);
    if (stopOnEmpty && !sols.length) break;
  }
  return out;
}
const kappaOf = (S, T, q) => conditioning(pointJacobian(q, [T[0][3], T[1][3], T[2][3]]), S.L).invKappa;

// Physical representatives within limits. Only q6 has a range wider than 2π, so it may have two.
export function representatives(q) {
  const reps = [];
  for (const d6 of [0, -TWO_PI, TWO_PI]) {
    const r = q.slice(); r[5] += d6;
    if (r.every((x, i) => x >= LIMITS[i][0] - 1e-9 && x <= LIMITS[i][1] + 1e-9)) reps.push(r);
  }
  return reps;
}
const torusDist = (a, b) => Math.max(...a.map((x, i) => Math.abs(wrap(x - b[i]))));
const rawDist = (a, b) => Math.max(...a.map((x, i) => Math.abs(x - b[i])));

// ---------------------------------------------------------------- persistent branches (Step 5)
// Greedy matching between consecutive samples on the joint torus. A branch ends when no candidate is
// within `gate` of it (it merged with another one at a singularity or left the IK set); unmatched
// candidates start new branches. For a closed contour, the last sample is matched back to the first
// to see which branch each one returns on (monodromy).
export function trackBranches(cands, { gate, closed = true }) {
  const branches = [];
  let active = (cands[0] || []).map((c, j) => { const br = { id: branches.length, start: 0, end: 0, idx: [j], q: [c.q] }; branches.push(br); return br; });
  for (let k = 1; k < cands.length; k++) {
    const C = cands[k], pairs = [];
    active.forEach((br, a) => C.forEach((c, j) => { const d = torusDist(br.q[br.q.length - 1], c.q); if (d <= gate) pairs.push([d, a, j]); }));
    pairs.sort((x, y) => x[0] - y[0]);
    const usedA = new Set(), usedJ = new Set(), next = [];
    for (const [, a, j] of pairs) {
      if (usedA.has(a) || usedJ.has(j)) continue;
      usedA.add(a); usedJ.add(j);
      const br = active[a]; br.end = k; br.idx.push(j); br.q.push(C[j].q); next.push(br);
    }
    C.forEach((c, j) => {
      if (usedJ.has(j)) return;
      const br = { id: branches.length, start: k, end: k, idx: [j], q: [c.q] }; branches.push(br); next.push(br);
    });
    active = next;
  }
  if (closed && cands.length > 1) {
    const C0 = cands[0];
    for (const br of active) {
      if (br.end !== cands.length - 1) continue;
      let best = -1, bd = gate;
      C0.forEach((c, j) => { const d = torusDist(br.q[br.q.length - 1], c.q); if (d <= bd) { bd = d; best = j; } });
      br.closesTo = best >= 0 ? branches.find((x) => x.start === 0 && x.idx[0] === best)?.id ?? null : null;
    }
  }
  return branches;
}

// ---------------------------------------------------------------- feasibility (Step 7)
export const REASONS = ['ok', 'noik', 'limit', 'jump', 'margin'];
export const REASON_TEXT = { ok: 'Completes the contour', noik: 'Loss of IK', limit: 'Joint-limit violation', jump: 'Excessive joint discontinuity', margin: 'Insufficient singularity margin' };

// Nearest-configuration continuation from one start configuration (already a physical representative).
// At each next sample, take the candidate closest on the torus, unwrap it next to the previous q,
// then test: IK exists → step ≤ dqMax → within limits → 1/κ ≥ margin. The first test that fails is the reason.
export function nearestContinuation(S, cands, poses, q0, { keepPath = false } = {}) {
  const dq = rad(S.dqMax), N = cands.length;
  let q = q0.slice();
  const path = keepPath ? [{ q: q.slice(), invKappa: kappaOf(S, poses[0], q), step: 0 }] : null;
  for (let k = 1; k <= N; k++) {
    const C = cands[k % N];
    if (!C.length) return { ok: false, reason: 'noik', at: k, path };
    let best = null, bd = Infinity;
    for (const c of C) { const d = torusDist(c.q, q); if (d < bd) { bd = d; best = c; } }
    const qn = q.map((x, i) => x + wrap(best.q[i] - x));
    const ik = kappaOf(S, poses[k % N], qn);
    if (path) path.push({ q: qn.slice(), invKappa: ik, step: bd });
    if (bd > dq) return { ok: false, reason: 'jump', at: k, path };
    if (!qn.every((x, i) => x >= LIMITS[i][0] - 1e-9 && x <= LIMITS[i][1] + 1e-9)) return { ok: false, reason: 'limit', at: k, path };
    if (!(ik >= S.margin)) return { ok: false, reason: 'margin', at: k, path };
    q = qn;
  }
  return { ok: true, reason: 'ok', at: N, closes: rawDist(q, q0) < 1e-6, path };
}

// Branch-aware search: layered graph whose nodes are all physical representatives that respect the
// limits and the margin, with an edge when the raw joint step is ≤ dqMax. A start succeeds when some
// node of the final layer (the start pose again) is reachable. Returns, per start node, ok and closes.
export function branchAwareFeasible(S, cands) {
  const dq = rad(S.dqMax), N = cands.length;
  const layer = (k) => {
    const nodes = [];
    for (const c of cands[k % N]) if (c.within && c.invKappa >= S.margin) for (const r of representatives(c.q)) nodes.push(r);
    return nodes;
  };
  const L0 = layer(0), layers = [L0];
  for (let k = 1; k < N; k++) layers.push(layer(k));
  layers.push(L0);
  // reach[s] = Set of node indices in the current layer reachable from start s
  let reach = L0.map((_, s) => new Set([s]));
  for (let k = 1; k <= N; k++) {
    const prev = layers[k - 1], cur = layers[k];
    const adj = prev.map((a) => cur.map((b, j) => (rawDist(a, b) <= dq ? j : -1)).filter((j) => j >= 0));
    reach = reach.map((set) => { const nx = new Set(); for (const i of set) for (const j of adj[i]) nx.add(j); return nx; });
  }
  return L0.map((q, s) => ({ q, ok: reach[s].size > 0, closes: reach[s].has(s) }));
}

// One placement, both strategies, every start configuration. Compact result for maps.
export function evaluatePlacement(S, center, orient) {
  const poses = contourPoses(S, center, orient);
  const cands = candidatesAlong(S, poses, { stopOnEmpty: true });
  const counts = cands.map((c) => c.length);
  if (cands.length < poses.length || counts.some((n) => n === 0)) {
    return { noik: true, minCount: 0, starts: [], firstEmpty: counts.findIndex((n) => n === 0) };
  }
  const aware = branchAwareFeasible(S, cands);
  const starts = [];
  for (const c of cands[0]) {
    const reps = representatives(c.q);
    const qs = reps.length ? reps : [c.q];
    for (const q0 of qs) {
      let res;
      if (!reps.length) res = { ok: false, reason: 'limit', at: 0 };
      else if (!(c.invKappa >= S.margin)) res = { ok: false, reason: 'margin', at: 0 };
      else res = nearestContinuation(S, cands, poses, q0);
      const aw = aware.find((a) => rawDist(a.q, q0) < 1e-9);
      starts.push({ b: c.b, q0, nearest: { ok: res.ok, reason: res.reason, at: res.at, closes: !!res.closes }, aware: { ok: !!aw?.ok, closes: !!aw?.closes } });
    }
  }
  return { noik: false, minCount: Math.min(...counts), starts };
}

export { branchLabel };
