// GoFa topology atlas (the Lecture 06 picture, for the real 6R robot).
//
// Reduction: keep the tool approach axis vertical (z6 = ∓ẑ). That task is symmetric about the base
// axis (a base rotation q1 is absorbed by q6, which spins about the same vertical axis), so the
// workspace can be studied on the meridian section (ρ, z), exactly as in the lecture atlas.
//  * z6 = v needs axis 5 ⟂ v. With q1 = 0, z4 = (sin q4 sin s, cos q4, sin q4 cos s), s = q2 + q3, so
//    v · z4 = ∓ sin q4 · cos s = 0: the solution set is FOUR 2-D sheets,
//      A: q4 = 0 and B: q4 = 180°  (coordinates q2, q3),
//      U: s = −90° (forearm up) and D: s = +90° (forearm down)  (coordinates q2, q4).
//    On every sheet q5 is then unique, q6 absorbs q1.
//  * On each sheet, D = det J (the full 6×6 Jacobian; independent of q1 and q6).
//    Its zero set are the singular configurations; their images in (ρ, z) the critical curves.
//  * Every preimage of a target (ρ, z) on either sheet is one IK solution of the full pose, so the
//    regions between critical curves have constant IK counts — checked against solveIK.
import { DH, LIMITS } from './gofa.js';
import { solveIK } from './ik.js';
import { homog } from './math.js';

const TWO_PI = 2 * Math.PI, PI = Math.PI;
export const wrap = (a) => a - TWO_PI * Math.floor((a + PI) / TWO_PI);
export const APPROACH = { down: { name: 'Tool down (−z)', v: [0, 0, -1] }, up: { name: 'Tool up (+z)', v: [0, 0, 1] } };

// Fast D–H frames: returns origins o0..o6 and axes x0..x6, z0..z6
function frames(q) {
  let T = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0];
  const o = [[0, 0, 0]], z = [[0, 0, 1]], x = [[1, 0, 0]];
  for (let i = 0; i < 6; i++) {
    const { theta0, d, a, alpha } = DH[i];
    const t = q[i] + theta0, ct = Math.cos(t), st = Math.sin(t), ca = Math.cos(alpha), sa = Math.sin(alpha);
    const A = [ct, -st * ca, st * sa, a * ct, st, ct * ca, -ct * sa, a * st, 0, sa, ca, d];
    const N = new Array(12);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 4; c++) N[4 * r + c] = T[4 * r] * A[c] + T[4 * r + 1] * A[4 + c] + T[4 * r + 2] * A[8 + c] + (c === 3 ? T[4 * r + 3] : 0);
    T = N;
    o.push([T[3], T[7], T[11]]); z.push([T[2], T[6], T[10]]); x.push([T[0], T[4], T[8]]);
  }
  return { o, z, x };
}
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

// q5 that turns z5 onto v, given q1..q4 (v must be ⟂ z4, which every sheet guarantees)
function q5For(q, v) {
  const F = frames([q[0], q[1], q[2], q[3], 0, 0]);
  const x4 = F.x[4], z4 = F.z[4], y4 = cross(z4, x4);
  return Math.atan2(-dot(v, x4), dot(v, y4));
}
export const SHEETS = [
  { id: 'A', name: 'q₄ = 0', yName: 'q₃', q: (a, b) => [0, a, b, 0, 0, 0] },
  { id: 'B', name: 'q₄ = 180°', yName: 'q₃', q: (a, b) => [0, a, b, PI, 0, 0] },
  { id: 'U', name: 'forearm up, s = −90°', yName: 'q₄', q: (a, b) => [0, a, wrap(-PI / 2 - a), b, 0, 0] },
  { id: 'D', name: 'forearm down, s = +90°', yName: 'q₄', q: (a, b) => [0, a, wrap(PI / 2 - a), b, 0, 0] },
];
export function sheetConfig(sheet, a, b, v) { const q = sheet.q(a, b); q[4] = q5For(q, v); return q; }
// Which sheet a configuration lies on, and its plot coordinates there
export function sheetOf(q) {
  if (Math.abs(Math.cos(q[1] + q[2])) < 1e-6) return { id: Math.sin(q[1] + q[2]) < 0 ? 'U' : 'D', a: wrap(q[1]), b: wrap(q[3]) };
  return { id: Math.abs(wrap(q[3])) < 1e-5 ? 'A' : 'B', a: wrap(q[1]), b: wrap(q[2]) };
}

// det of the 6×6 geometric Jacobian (rows: linear, angular)
export function detJ(q) {
  const F = frames(q), p = F.o[6], M = [];
  for (let i = 0; i < 6; i++) {
    const zi = F.z[i], r = [p[0] - F.o[i][0], p[1] - F.o[i][1], p[2] - F.o[i][2]], lin = cross(zi, r);
    M.push([lin[0], lin[1], lin[2], zi[0], zi[1], zi[2]]); // column i stored as a row (det(Jᵀ) = det J)
  }
  let d = 1;
  for (let k = 0; k < 6; k++) {
    let piv = k;
    for (let i = k + 1; i < 6; i++) if (Math.abs(M[i][k]) > Math.abs(M[piv][k])) piv = i;
    if (M[piv][k] === 0) return 0;
    if (piv !== k) { [M[piv], M[k]] = [M[k], M[piv]]; d = -d; }
    d *= M[k][k];
    for (let i = k + 1; i < 6; i++) { const f = M[i][k] / M[k][k]; for (let j = k; j < 6; j++) M[i][j] -= f * M[k][j]; }
  }
  return d;
}
export function tool0(q) { return frames(q).o[6]; }
export function meridianOf(q) { const p = tool0(q); return [Math.hypot(p[0], p[1]), p[2]]; }

export function Dsheet(sheet, a, b, v) { return detJ(sheetConfig(sheet, a, b, v)); }

// ---------------------------------------------------------------- atlas
export function buildGofaAtlas(approach = 'down', n = 181) {
  const v = APPROACH[approach].v, step = TWO_PI / (n - 1);
  const sheets = SHEETS.map((sh) => {
    const D = new Float64Array(n * n);
    for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) D[j * n + i] = Dsheet(sh, -PI + i * step, -PI + j * step, v);
    const f = (a, b) => Dsheet(sh, a, b, v), phi = (a, b) => meridianOf(sheetConfig(sh, a, b, v));
    const curves = zeroCurves(D, n, step).map((c) => c.map(([a, b]) => refineOnZero(f, a, b)));
    const images = curves.map((c) => c.map(([a, b]) => phi(a, b)));
    let dMax = 0; for (const x of D) dMax = Math.max(dMax, Math.abs(x));
    // |∇D| must be a fair fraction of the sheet's scale: crossings of two singular curves (∇D ≈ 0) are not cusps
    const cusps = findCusps(f, phi, curves, 0.05 * dMax);
    return { ...sh, D, curves, images, cusps, dMax };
  });
  let rhoMax = 0, zMin = Infinity, zMax = -Infinity;
  for (const sh of SHEETS) for (let j = 0; j < 60; j++) for (let i = 0; i < 60; i++) {
    const [r, z] = meridianOf(sheetConfig(sh, -PI + (i * TWO_PI) / 59, -PI + (j * TWO_PI) / 59, v));
    rhoMax = Math.max(rhoMax, r); zMin = Math.min(zMin, z); zMax = Math.max(zMax, z);
  }
  return { approach, v, n, step, sheets, bounds: { rhoMax, zMin, zMax } };
}

// Full-pose IK for the target (ρ, z) on the meridian y = 0, tool vertical. Each solution is
// tagged with its sheet id and its plot coordinates (a, b) there.
export function solveTarget(approach, rho, z, opts = {}) { return solvePoint(approach, [rho, 0, z], opts); }
// Same, for any 3-D tool point (the count only depends on ρ and z)
export function solvePoint(approach, p, { yaw = 0 } = {}) {
  const v = APPROACH[approach].v;
  // tool frame: z6 = v, x6 = horizontal direction set by yaw
  const x6 = [Math.cos(yaw), Math.sin(yaw), 0], z6 = v, y6 = cross(z6, x6);
  const R = [[x6[0], y6[0], z6[0]], [x6[1], y6[1], z6[1]], [x6[2], y6[2], z6[2]]];
  const S = solveIK(homog(R, p)), out = S.map((s) => ({ q: s.q, within: s.within, family: s.family, ...sheetOf(s.q), det: detJ(s.q) }));
  out.continuum = !!S.continuum;
  return out;
}

// q3 joint-limit band on the (−π, π] axis: allowed are q3 ∈ [−π, 85°] ∪ [135°, π] (= −225° … −180°)
export const Q3_FORBIDDEN = [LIMITS[2][1], LIMITS[2][0] + TWO_PI];

// ---------------------------------------------------------------- curves, refinement, cusps
function zeroCurves(D, n, step) {
  const segs = [], at = (i, j) => D[j * n + i], X = (i) => -PI + i * step;
  const lerp = (i1, j1, i2, j2) => { const a = at(i1, j1), b = at(i2, j2), t = a / (a - b); return [X(i1) + t * (X(i2) - X(i1)), X(j1) + t * (X(j2) - X(j1))]; };
  for (let j = 0; j < n - 1; j++) for (let i = 0; i < n - 1; i++) {
    const c = [at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1)];
    if (c.some((x) => !Number.isFinite(x))) continue;
    const e = [];
    if ((c[0] > 0) !== (c[1] > 0)) e.push(lerp(i, j, i + 1, j));
    if ((c[1] > 0) !== (c[2] > 0)) e.push(lerp(i + 1, j, i + 1, j + 1));
    if ((c[3] > 0) !== (c[2] > 0)) e.push(lerp(i, j + 1, i + 1, j + 1));
    if ((c[0] > 0) !== (c[3] > 0)) e.push(lerp(i, j, i, j + 1));
    if (e.length === 2) segs.push(e); else if (e.length === 4) { segs.push([e[0], e[1]]); segs.push([e[2], e[3]]); }
  }
  const key = ([a, b]) => `${a.toFixed(9)},${b.toFixed(9)}`, ends = new Map();
  segs.forEach((s, idx) => s.forEach((p) => { const k = key(p); if (!ends.has(k)) ends.set(k, []); ends.get(k).push(idx); }));
  const used = new Uint8Array(segs.length), lines = [];
  for (let s0 = 0; s0 < segs.length; s0++) {
    if (used[s0]) continue;
    used[s0] = 1;
    const line = [segs[s0][0], segs[s0][1]];
    for (const dir of [1, 0]) for (;;) {
      const tip = dir ? line[line.length - 1] : line[0];
      const nxt = (ends.get(key(tip)) || []).find((x) => !used[x]);
      if (nxt === undefined) break;
      used[nxt] = 1;
      const s = segs[nxt], other = key(s[0]) === key(tip) ? s[1] : s[0];
      if (dir) line.push(other); else line.unshift(other);
    }
    lines.push(line);
  }
  return lines;
}
function refineOnZero(f, a, b) {
  for (let it = 0; it < 3; it++) {
    const h = 1e-6, v = f(a, b), ga = (f(a + h, b) - f(a - h, b)) / (2 * h), gb = (f(a, b + h) - f(a, b - h)) / (2 * h), gg = ga * ga + gb * gb;
    if (!Number.isFinite(v) || !Number.isFinite(gg) || gg < 1e-20) break;
    a -= (v * ga) / gg; b -= (v * gb) / gg;
  }
  return [a, b];
}
// Cusp test along each singular curve: the two image chords (±δ along the curve) leave the critical
// value in the same direction. Candidates come from sign changes of the image speed's direction.
function findCusps(f, phi, curves, gmin) {
  const out = [];
  for (const c of curves) {
    if (c.length < 5) continue;
    const img = c.map(([a, b]) => phi(a, b));
    for (let i = 1; i < c.length - 1; i++) {
      const u = [img[i][0] - img[i - 1][0], img[i][1] - img[i - 1][1]], w = [img[i + 1][0] - img[i][0], img[i + 1][1] - img[i][1]];
      const nu = Math.hypot(...u), nw = Math.hypot(...w);
      if (nu < 1e-12 || nw < 1e-12) continue;
      const jump = Math.hypot(c[i][0] - c[i - 1][0], c[i][1] - c[i - 1][1]) > 0.2 || Math.hypot(c[i + 1][0] - c[i][0], c[i + 1][1] - c[i][1]) > 0.2;
      if (jump) continue;
      if ((u[0] * w[0] + u[1] * w[1]) / (nu * nw) < -0.3 && isCusp(f, phi, c[i], gmin)) {
        if (!out.some((o) => Math.hypot(o.q[0] - c[i][0], o.q[1] - c[i][1]) < 0.05)) out.push({ q: c[i], image: img[i] });
      }
    }
  }
  return out;
}
function isCusp(f, phi, q, gmin, delta = 0.04) {
  const h = 1e-6, g = [(f(q[0] + h, q[1]) - f(q[0] - h, q[1])) / (2 * h), (f(q[0], q[1] + h) - f(q[0], q[1] - h)) / (2 * h)], gn = Math.hypot(...g);
  if (!(gn > Math.max(gmin, 1e-9))) return false;
  const t = [-g[1] / gn, g[0] / gn], c0 = phi(q[0], q[1]);
  const chord = (s) => { const r = refineOnZero(f, q[0] + s * delta * t[0], q[1] + s * delta * t[1]); const m = phi(r[0], r[1]); return [m[0] - c0[0], m[1] - c0[1]]; };
  const a = chord(1), b = chord(-1), na = Math.hypot(...a), nb = Math.hypot(...b);
  return na > 1e-9 && nb > 1e-9 && (a[0] * b[0] + a[1] * b[1]) / (na * nb) > 0.5;
}
