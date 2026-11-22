// Analytical-numerical inverse kinematics for ABB GoFa (CRB 15000-5/0.95), written for the browser.
//
// Idea (same structure as the course solver, which sweeps t6 = tan(q6/2)):
//  * Given the tool0 pose and a value of q6, the D–H frame-4 origin O4 and the axis-5 direction z4
//    are fixed: O5 = p − d6·z6, x5 = −cos q6·x6 + sin q6·y6, O4 = O5 − a5·x5, z4 = x5 × z5.
//  * O4 depends only on (q1, q2, q3), so the arm has closed-form solutions:
//    2 shoulder (front/back) × 2 elbow (up/down).
//  * The one remaining condition is that axis 5 is perpendicular to the forearm axis z3:
//        f(q6) = z3(q1, q2, q3) · z4(q6) = 0.
//    Its roots are found per arm branch by sampling q6 and refining each sign change.
//  * q4 and q5 then follow from z4 and x5, and every candidate is checked with forward kinematics.
import { DH, poeFK, LIMITS } from './gofa.js';
import { posOf, rotOf } from './math.js';

const D1 = DH[0].d, A2 = DH[1].a, A3 = DH[2].a, D4 = DH[3].d, A5 = DH[4].a, D6 = DH[5].d;
const L3 = Math.hypot(A3, D4), BETA = Math.atan2(D4, A3);
const TWO_PI = 2 * Math.PI;
const wrap = (a) => a - TWO_PI * Math.floor((a + Math.PI) / TWO_PI);

// Closed-form arm: (q1, q2, q3) placing O4 at P. Returns up to 4 { q1, q2, q3, shoulder, elbow }.
export function armIK(P) {
  const rho = Math.hypot(P[0], P[1]), phi = Math.atan2(P[1], P[0]), h = P[2] - D1;
  const out = [];
  for (const shoulder of [1, -1]) {
    const r = shoulder * rho, q1 = shoulder === 1 ? phi : wrap(phi + Math.PI);
    const c = (r * r + h * h - A2 * A2 - L3 * L3) / (2 * A2 * L3);
    if (c > 1 || c < -1) continue;
    const g = Math.acos(c);
    for (const elbow of [1, -1]) {
      const q3 = elbow * g - BETA;
      // V = (0, A2) + R(q3)(D4, A3), with R(s)(x, z) = (x cos s + z sin s, −x sin s + z cos s) in the arm plane
      const cs = Math.cos(q3), sn = Math.sin(q3);
      const Vr = D4 * cs + A3 * sn, Vh = A2 - D4 * sn + A3 * cs;
      const q2 = wrap(Math.atan2(Vh, Vr) - Math.atan2(h, r));
      out.push({ q1, q2, q3, shoulder, elbow });
    }
  }
  return out;
}

// Pose data needed for a sweep over q6
function wristGeometry(T) {
  const p = posOf(T), R = rotOf(T);
  const x6 = [R[0][0], R[1][0], R[2][0]], y6 = [R[0][1], R[1][1], R[2][1]], z6 = [R[0][2], R[1][2], R[2][2]];
  const O5 = [p[0] - D6 * z6[0], p[1] - D6 * z6[1], p[2] - D6 * z6[2]];
  return { x6, y6, z6, O5 };
}
function atQ6(W, q6) {
  const c = Math.cos(q6), s = Math.sin(q6);
  const x5 = [-c * W.x6[0] + s * W.y6[0], -c * W.x6[1] + s * W.y6[1], -c * W.x6[2] + s * W.y6[2]];
  const z5 = W.z6;
  const O4 = [W.O5[0] - A5 * x5[0], W.O5[1] - A5 * x5[1], W.O5[2] - A5 * x5[2]];
  const z4 = [x5[1] * z5[2] - x5[2] * z5[1], x5[2] * z5[0] - x5[0] * z5[2], x5[0] * z5[1] - x5[1] * z5[0]];
  return { x5, z5, O4, z4 };
}
// Forearm axis z3 for an arm solution
function z3Of(q1, q2, q3) {
  const s = q2 + q3, cs = Math.cos(s);
  return [Math.cos(q1) * cs, Math.sin(q1) * cs, -Math.sin(s)];
}
// Residual for arm branch k (index into armIK output, order shoulder × elbow)
function residual(W, q6, branch) {
  const g = atQ6(W, q6);
  const sols = armIK(g.O4);
  const a = sols.find((x) => x.shoulder === branch.shoulder && x.elbow === branch.elbow);
  if (!a) return { f: NaN };
  const z3 = z3Of(a.q1, a.q2, a.q3);
  return { f: z3[0] * g.z4[0] + z3[1] * g.z4[1] + z3[2] * g.z4[2], arm: a, g };
}

// Recover q4, q5 once (q1, q2, q3, q6) are known
function wristAngles(arm, g) {
  const { q1, q2, q3 } = arm, s = q2 + q3;
  // frame 3 axes (D–H): x3 along the a3 offset, y3 = −(base-rotated y), z3 forearm
  const c1 = Math.cos(q1), s1 = Math.sin(q1), cs = Math.cos(s), ss = Math.sin(s);
  const z3 = [c1 * cs, s1 * cs, -ss];
  const x3 = [c1 * ss, s1 * ss, cs];
  const y3 = [z3[1] * x3[2] - z3[2] * x3[1], z3[2] * x3[0] - z3[0] * x3[2], z3[0] * x3[1] - z3[1] * x3[0]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  // z4 = R3 (sin θ4, −cos θ4, 0)
  const t4 = Math.atan2(dot(g.z4, x3), -dot(g.z4, y3));
  const c4 = Math.cos(t4), s4 = Math.sin(t4);
  // frame 4: x4 = R3 (cos θ4, sin θ4, 0); y4 = z4 × x4
  const x4 = [c4 * x3[0] + s4 * y3[0], c4 * x3[1] + s4 * y3[1], c4 * x3[2] + s4 * y3[2]];
  const z4 = g.z4;
  const y4 = [z4[1] * x4[2] - z4[2] * x4[1], z4[2] * x4[0] - z4[0] * x4[2], z4[0] * x4[1] - z4[1] * x4[0]];
  const t5 = Math.atan2(dot(g.x5, y4), dot(g.x5, x4));
  return [t4, t5];
}

function rootsOf(fn, samples) {
  const xs = [], fs = [];
  for (let i = 0; i <= samples; i++) { const x = -Math.PI + 0.0137 + (TWO_PI * i) / samples; xs.push(x); fs.push(fn(x)); }
  const roots = [];
  const refine = (a, b, fa, fb) => { // Illinois regula falsi
    let side = 0;
    for (let it = 0; it < 60; it++) {
      const c = (a * fb - b * fa) / (fb - fa), fc = fn(c);
      if (!Number.isFinite(fc)) return null;
      if (Math.abs(fc) < 1e-14 || Math.abs(b - a) < 1e-13) return c;
      if (fc * fb > 0) { b = c; fb = fc; if (side === -1) fa /= 2; side = -1; }
      else { a = c; fa = fc; if (side === 1) fb /= 2; side = 1; }
    }
    return (a + b) / 2;
  };
  for (let i = 0; i < samples; i++) {
    const fa = fs[i], fb = fs[i + 1];
    const okA = Number.isFinite(fa), okB = Number.isFinite(fb);
    if (okA !== okB) {
      // Reach boundary inside this interval (the two elbow branches meet there). Locate it and
      // look for a sign change between the reachable sample and the boundary.
      let g = okA ? xs[i] : xs[i + 1], bad = okA ? xs[i + 1] : xs[i];
      for (let it = 0; it < 60; it++) { const m = (g + bad) / 2; if (Number.isFinite(fn(m))) g = m; else bad = m; }
      // f behaves like a square root near the boundary, so sub-sample more densely towards it
      const x0 = okA ? xs[i] : xs[i + 1], K = 24;
      let xp = x0, fp = okA ? fa : fb;
      for (let k = K - 1; k >= 0; k--) {
        const x = g + (x0 - g) * (k / K) ** 2, f = k === 0 ? fn(g) : fn(x);
        if (!Number.isFinite(f)) break;
        if (Math.abs(f) < 1e-13) roots.push(x);
        else if (fp * f < 0) {
          const r = refine(Math.min(xp, x), Math.max(xp, x), xp < x ? fp : f, xp < x ? f : fp);
          if (r !== null) roots.push(r);
        }
        xp = x; fp = f;
      }
      continue;
    }
    if (!okA) continue;
    if (Math.abs(fa) < 1e-13) { roots.push(xs[i]); continue; }
    if (fa * fb < 0) { const r = refine(xs[i], xs[i + 1], fa, fb); if (r !== null) roots.push(r); }
    // tangential (double) root: local minimum of |f| close to zero
    else if (i > 0 && Number.isFinite(fs[i - 1]) && Math.abs(fa) < 2e-3 && Math.abs(fa) <= Math.abs(fs[i - 1]) && Math.abs(fa) <= Math.abs(fb)) {
      let a = xs[i - 1], b = xs[i + 1];
      for (let it = 0; it < 60; it++) {
        const m1 = a + (b - a) * 0.382, m2 = a + (b - a) * 0.618;
        if (Math.abs(fn(m1)) < Math.abs(fn(m2))) b = m2; else a = m1;
      }
      const x = (a + b) / 2;
      if (Math.abs(fn(x)) < 1e-9) roots.push(x);
    }
  }
  return roots;
}

// Fit joint 3 into its URDF range by whole turns; others wrapped to (−π, π]. Returns null if q3 cannot fit.
export function fitLimits(q) {
  const out = q.map(wrap);
  let q3 = out[2];
  while (q3 < LIMITS[2][0] - 1e-12) q3 += TWO_PI;
  while (q3 > LIMITS[2][1] + 1e-12) q3 -= TWO_PI;
  out[2] = q3;
  const ok = q3 >= LIMITS[2][0] - 1e-12 && q3 <= LIMITS[2][1] + 1e-12;
  return { q: out, within: ok };
}

function poseErr(T, q) {
  const F = poeFK(q);
  let ep = 0, er = 0;
  for (let i = 0; i < 3; i++) { ep += (F[i][3] - T[i][3]) ** 2; for (let j = 0; j < 3; j++) er = Math.max(er, Math.abs(F[i][j] - T[i][j])); }
  return { position: Math.sqrt(ep), rotation: er };
}

/**
 * All IK solutions for a tool0 pose T (4×4).
 * Returns [{ q, within, branch: { shoulder, elbow, wrist }, label, position_error, rotation_error }].
 * Options: samples (q6 grid, default 90), tol (FK acceptance, default 1e-7).
 */
export function solveIK(T, { samples = 90, tol = 1e-7 } = {}) {
  const W = wristGeometry(T), sols = [];
  for (const shoulder of [1, -1]) for (const elbow of [1, -1]) {
    const branch = { shoulder, elbow };
    const roots = rootsOf((q6) => residual(W, q6, branch).f, samples);
    for (const q6 of roots) {
      const { arm, g } = residual(W, q6, branch);
      if (!arm) continue;
      const [t4, t5] = wristAngles(arm, g);
      const raw = [arm.q1, arm.q2, arm.q3, t4, t5, q6];
      const err = poseErr(T, raw);
      if (err.position > tol || err.rotation > tol) continue;
      const { q, within } = fitLimits(raw);
      // drop duplicates (roots found twice at a sample boundary)
      if (sols.some((s) => s.q.every((x, i) => Math.abs(wrap(x - q[i])) < 1e-7))) continue;
      const wrist = q[4] >= 0 ? 1 : -1;
      sols.push({
        q, within, branch: { shoulder, elbow, wrist },
        label: branchLabel({ shoulder, elbow, wrist }),
        position_error: err.position, rotation_error: err.rotation,
      });
    }
  }
  return sols;
}

export const BRANCHES = [];
for (const shoulder of [1, -1]) for (const elbow of [1, -1]) for (const wrist of [1, -1]) BRANCHES.push({ shoulder, elbow, wrist });
export function branchIndex({ shoulder, elbow, wrist }) {
  return (shoulder === 1 ? 0 : 4) + (elbow === 1 ? 0 : 2) + (wrist === 1 ? 0 : 1);
}
export function branchLabel({ shoulder, elbow, wrist }) {
  return `${shoulder === 1 ? 'Front' : 'Back'} · ${elbow === 1 ? 'q₃ > q₃*' : 'q₃ < q₃*'} · ${wrist === 1 ? 'q₅ ≥ 0' : 'q₅ < 0'}`;
}
