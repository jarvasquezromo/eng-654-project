// 3R position atlas (Lecture 06): det Jp over (q2, q3), its zero set, the critical curves in the
// meridian section (ρ, z), regular IK counts per region, cusps. No DOM, so it runs in Node tests.
//
// Chain: standard D–H, ⁱ⁻¹Tᵢ = Rz(qᵢ + θ0ᵢ) Tz(dᵢ) Tx(aᵢ) Rx(αᵢ), i = 1..3; the point is the
// frame-3 origin moved by d4 along z3 (d4 = 0 reproduces the lecture presets).

export const PRESETS = {
  zero: { name: 'a₁ = 0 m · intersecting axes', a: [0, 2, 1.5], d: [1, 1, 0], alpha: [90, 90, 0], theta0: [0, 0, 0], d4: 0, formula: 'det(Jₚ) = ¾(3c₃ + 4) c₂(c₃ − 2s₃)' },
  offset: { name: 'a₁ = 1 m · offset axes', a: [1, 2, 1.5], d: [1, 1, 0], alpha: [90, 90, 0], theta0: [0, 0, 0], d4: 0, formula: 'det(Jₚ) = ¾(3c₃ + 4)[c₂(c₃ − 2s₃) − s₃]' },
  gofa: { name: 'GoFa arm · wrist point O₄', a: [0, 0.444, 0.110], d: [0.265, 0, 0], alpha: [-90, 0, -90], theta0: [0, -90, 0], d4: 0.470, formula: 'GoFa first three joints, point O₄ = O₃ + d₄ z₃ (Step 1 D–H table)' },
};

const R = Math.PI / 180;
// Returns frame origins o0..o3, axes z0..z2 and the point p, for q = [q1, q2, q3] (radians)
export function chain(P, q) {
  let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]];
  const o = [[0, 0, 0]], z = [[0, 0, 1]];
  for (let i = 0; i < 3; i++) {
    const th = q[i] + P.theta0[i] * R, al = P.alpha[i] * R;
    const ct = Math.cos(th), st = Math.sin(th), ca = Math.cos(al), sa = Math.sin(al);
    const A = [[ct, -st * ca, st * sa, P.a[i] * ct], [st, ct * ca, -ct * sa, P.a[i] * st], [0, sa, ca, P.d[i]]];
    const N = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 4; c++) N[r][c] = T[r][0] * A[0][c] + T[r][1] * A[1][c] + T[r][2] * A[2][c] + (c === 3 ? T[r][3] : 0);
    }
    T = N;
    o.push([T[0][3], T[1][3], T[2][3]]);
    z.push([T[0][2], T[1][2], T[2][2]]);
  }
  const p = [0, 1, 2].map((k) => o[3][k] + P.d4 * z[3][k]);
  return { o, z, p };
}
export const point = (P, q) => chain(P, q).p;

// det of the 3×3 positional Jacobian (columns z_{i-1} × (p − o_{i-1})); independent of q1
export function detJp(P, q2, q3) {
  const { o, z, p } = chain(P, [0, q2, q3]);
  const c = [0, 1, 2].map((i) => {
    const r = [p[0] - o[i][0], p[1] - o[i][1], p[2] - o[i][2]], w = z[i];
    return [w[1] * r[2] - w[2] * r[1], w[2] * r[0] - w[0] * r[2], w[0] * r[1] - w[1] * r[0]];
  });
  return c[0][0] * (c[1][1] * c[2][2] - c[1][2] * c[2][1]) - c[1][0] * (c[0][1] * c[2][2] - c[0][2] * c[2][1]) + c[2][0] * (c[0][1] * c[1][2] - c[0][2] * c[1][1]);
}
// Meridian map Φ(q2, q3) = (ρ, z) with q1 eliminated
export function meridian(P, q2, q3) { const p = point(P, [0, q2, q3]); return [Math.hypot(p[0], p[1]), p[2]]; }

// ---------------------------------------------------------------- grid, zero set, critical curves
const TWO_PI = 2 * Math.PI;
export function buildAtlas(P, n = 241) {
  const step = TWO_PI / (n - 1), D = new Float64Array(n * n), RHO2 = new Float64Array(n * n), Z = new Float64Array(n * n);
  for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
    const q2 = -Math.PI + i * step, q3 = -Math.PI + j * step, k = j * n + i;
    D[k] = detJp(P, q2, q3);
    const p = point(P, [0, q2, q3]);
    RHO2[k] = p[0] * p[0] + p[1] * p[1]; Z[k] = p[2];
  }
  const curves = zeroCurves(D, n, step).map((pts) => pts.map(([q2, q3]) => refineOnZero(P, q2, q3)));
  const images = curves.map((c) => c.map(([q2, q3]) => meridian(P, q2, q3)));
  let rhoMax = 0, zMin = Infinity, zMax = -Infinity, dMax = 0;
  for (let k = 0; k < n * n; k++) { rhoMax = Math.max(rhoMax, Math.sqrt(RHO2[k])); zMin = Math.min(zMin, Z[k]); zMax = Math.max(zMax, Z[k]); dMax = Math.max(dMax, Math.abs(D[k])); }
  const atlas = { P, n, step, D, RHO2, Z, curves, images, bounds: { rhoMax, zMin, zMax }, dMax };
  atlas.cusps = findCusps(P, curves);
  return atlas;
}

// Marching squares on the periodic grid, joined into polylines of (q2, q3)
function zeroCurves(D, n, step) {
  const segs = [];
  const at = (i, j) => D[j * n + i], X = (i) => -Math.PI + i * step;
  const lerp = (i1, j1, i2, j2) => { const a = at(i1, j1), b = at(i2, j2), t = a / (a - b); return [X(i1) + t * (X(i2) - X(i1)), X(j1) + t * (X(j2) - X(j1))]; };
  for (let j = 0; j < n - 1; j++) for (let i = 0; i < n - 1; i++) {
    const e = [];
    if ((at(i, j) > 0) !== (at(i + 1, j) > 0)) e.push(lerp(i, j, i + 1, j));
    if ((at(i + 1, j) > 0) !== (at(i + 1, j + 1) > 0)) e.push(lerp(i + 1, j, i + 1, j + 1));
    if ((at(i, j + 1) > 0) !== (at(i + 1, j + 1) > 0)) e.push(lerp(i, j + 1, i + 1, j + 1));
    if ((at(i, j) > 0) !== (at(i, j + 1) > 0)) e.push(lerp(i, j, i, j + 1));
    if (e.length === 2) segs.push(e);
    else if (e.length === 4) { segs.push([e[0], e[1]]); segs.push([e[2], e[3]]); }
  }
  // join segments by shared endpoints
  const key = ([a, b]) => `${a.toFixed(9)},${b.toFixed(9)}`;
  const ends = new Map();
  segs.forEach((s, idx) => s.forEach((p) => { const k = key(p); if (!ends.has(k)) ends.set(k, []); ends.get(k).push(idx); }));
  const used = new Uint8Array(segs.length), lines = [];
  for (let s0 = 0; s0 < segs.length; s0++) {
    if (used[s0]) continue;
    used[s0] = 1;
    const line = [segs[s0][0], segs[s0][1]];
    for (const dir of [1, 0]) {
      for (;;) {
        const tip = dir ? line[line.length - 1] : line[0];
        const nxt = (ends.get(key(tip)) || []).find((x) => !used[x]);
        if (nxt === undefined) break;
        used[nxt] = 1;
        const s = segs[nxt], other = key(s[0]) === key(tip) ? s[1] : s[0];
        if (dir) line.push(other); else line.unshift(other);
      }
    }
    lines.push(line);
  }
  return lines;
}
// One Newton step along ∇D to sit on D = 0 (makes the critical-curve image accurate)
function refineOnZero(P, q2, q3) {
  for (let it = 0; it < 3; it++) {
    const h = 1e-6, f = detJp(P, q2, q3);
    const g2 = (detJp(P, q2 + h, q3) - detJp(P, q2 - h, q3)) / (2 * h), g3 = (detJp(P, q2, q3 + h) - detJp(P, q2, q3 - h)) / (2 * h);
    const gg = g2 * g2 + g3 * g3;
    if (gg < 1e-20) break;
    q2 -= (f * g2) / gg; q3 -= (f * g3) / gg;
  }
  return [q2, q3];
}

// Cusps: along a singular curve, the kernel k of DΦ becomes tangent to the curve (∇D · k = 0).
// Sign changes of ∇D · k are refined by bisection on the polyline and reported with their image.
// Geometric confirmation: step ±δ along the singular curve; at a cusp both image chords leave the
// critical value in the same direction, at a fold point (or a curve crossing) they are opposite.
function isCusp(P, q, delta = 0.03) {
  const h = 1e-6;
  const g = [(detJp(P, q[0] + h, q[1]) - detJp(P, q[0] - h, q[1])) / (2 * h), (detJp(P, q[0], q[1] + h) - detJp(P, q[0], q[1] - h)) / (2 * h)];
  const gn = Math.hypot(...g);
  if (gn < 1e-9) return false;
  const t = [-g[1] / gn, g[0] / gn], c = meridian(P, q[0], q[1]);
  const chord = (sgn) => { const r = refineOnZero(P, q[0] + sgn * delta * t[0], q[1] + sgn * delta * t[1]); const m = meridian(P, r[0], r[1]); return [m[0] - c[0], m[1] - c[1]]; };
  const a = chord(1), b = chord(-1), na = Math.hypot(...a), nb = Math.hypot(...b);
  if (na < 1e-12 || nb < 1e-12) return false;
  return (a[0] * b[0] + a[1] * b[1]) / (na * nb) > 0.5;
}

function findCusps(P, curves) {
  const out = [], h = 1e-6;
  const info = (q2, q3) => {
    const r2a = meridian(P, q2 + h, q3), r2b = meridian(P, q2 - h, q3), r3a = meridian(P, q2, q3 + h), r3b = meridian(P, q2, q3 - h);
    const J = [[(r2a[0] - r2b[0]) / (2 * h), (r3a[0] - r3b[0]) / (2 * h)], [(r2a[1] - r2b[1]) / (2 * h), (r3a[1] - r3b[1]) / (2 * h)]];
    const row = Math.hypot(...J[0]) > Math.hypot(...J[1]) ? J[0] : J[1];
    const k = [-row[1], row[0]], kn = Math.hypot(...k) || 1;
    const g = [(detJp(P, q2 + h, q3) - detJp(P, q2 - h, q3)) / (2 * h), (detJp(P, q2, q3 + h) - detJp(P, q2, q3 - h)) / (2 * h)], gn = Math.hypot(...g) || 1;
    return { k: [k[0] / kn, k[1] / kn], g: [g[0] / gn, g[1] / gn] };
  };
  for (const c of curves) {
    let prevK = null, prev = null;
    c.forEach(([q2, q3], i) => {
      const { k, g } = info(q2, q3);
      if (prevK && k[0] * prevK[0] + k[1] * prevK[1] < 0) { k[0] = -k[0]; k[1] = -k[1]; }
      const val = g[0] * k[0] + g[1] * k[1];
      if (prev && prev.val * val < 0 && Math.abs(c[i - 1][0] - q2) < 0.2 && Math.abs(c[i - 1][1] - q3) < 0.2) {
        const t = prev.val / (prev.val - val);
        const q = refineOnZero(P, c[i - 1][0] + t * (q2 - c[i - 1][0]), c[i - 1][1] + t * (q3 - c[i - 1][1]));
        if (isCusp(P, q) && !out.some((o) => Math.hypot(o.q[0] - q[0], o.q[1] - q[1]) < 0.02)) out.push({ q, image: meridian(P, q[0], q[1]) });
      }
      prevK = k; prev = { val };
    });
  }
  return out;
}

// ---------------------------------------------------------------- IK on the meridian section
// Solutions (q2, q3) of Φ(q2, q3) = (ρ, z): candidate cells of the precomputed grid where both
// residuals can vanish, then 2-D Newton; q1 recovered from the target azimuth (here azimuth 0).
export function solveMeridian(A, rho, z) {
  const { P, n, step, RHO2, Z } = A, sols = [];
  const r2 = rho * rho;
  for (let j = 0; j < n - 1; j++) for (let i = 0; i < n - 1; i++) {
    const ks = [j * n + i, j * n + i + 1, (j + 1) * n + i, (j + 1) * n + i + 1];
    let a = 0, b = 0, c = 0, d = 0;
    for (const k of ks) { const f = RHO2[k] - r2, g = Z[k] - z; if (f > 0) a++; else b++; if (g > 0) c++; else d++; }
    if (!a || !b || !c || !d) continue;
    let q2 = -Math.PI + (i + 0.5) * step, q3 = -Math.PI + (j + 0.5) * step, ok = false;
    for (let it = 0; it < 30; it++) {
      const h = 1e-7, F = res(P, q2, q3, r2, z);
      if (Math.abs(F[0]) < 1e-12 && Math.abs(F[1]) < 1e-12) { ok = true; break; }
      const F2 = res(P, q2 + h, q3, r2, z), F3 = res(P, q2, q3 + h, r2, z);
      const J = [[(F2[0] - F[0]) / h, (F3[0] - F[0]) / h], [(F2[1] - F[1]) / h, (F3[1] - F[1]) / h]];
      const det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
      if (Math.abs(det) < 1e-14) break;
      let d2 = (J[1][1] * F[0] - J[0][1] * F[1]) / det, d3 = (-J[1][0] * F[0] + J[0][0] * F[1]) / det;
      const m = Math.max(Math.abs(d2), Math.abs(d3)); if (m > 0.3) { d2 *= 0.3 / m; d3 *= 0.3 / m; }
      q2 -= d2; q3 -= d3;
    }
    if (!ok) { const F = res(P, q2, q3, r2, z); ok = Math.abs(F[0]) < 1e-9 && Math.abs(F[1]) < 1e-9; }
    if (!ok) continue;
    q2 = wrap(q2); q3 = wrap(q3);
    if (sols.some((s) => Math.abs(wrap(s.q2 - q2)) < 1e-6 && Math.abs(wrap(s.q3 - q3)) < 1e-6)) continue;
    const p = point(P, [0, q2, q3]);
    const q1 = wrap(-Math.atan2(p[1], p[0])); // rotate so the point lies on the +x half-plane (y = 0)
    sols.push({ q1, q2, q3, det: detJp(P, q2, q3) });
  }
  sols.sort((s, t) => s.q2 - t.q2);
  return sols;
}
function res(P, q2, q3, r2, z) { const p = point(P, [0, q2, q3]); return [p[0] * p[0] + p[1] * p[1] - r2, p[2] - z]; }
export const wrap = (a) => a - TWO_PI * Math.floor((a + Math.PI) / TWO_PI);
