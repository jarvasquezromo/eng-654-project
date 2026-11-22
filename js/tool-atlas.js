// Topology atlas for any tool (orientation R, length ℓ): numerical version used when the tool axis
// is not vertical (the (ρ, z) symmetry is gone). For each point of a plane grid: all IK solutions of
// the TCP pose, their wrist sheet (sign of q5; the 4-5-6 rotation has exactly two wrist solutions),
// det J and the joint-limit flag. No DOM, so it runs in workers and in Node.
import { solveIK } from './ik.js';
import { homog } from './math.js';
import { detJ } from './gofa-atlas.js';

export function solveTool(R, len, p) {
  const z = [R[0][2], R[1][2], R[2][2]];
  const T = homog(R, [p[0] - len * z[0], p[1] - len * z[1], p[2] - len * z[2]]);
  return solveIK(T).map((s) => ({ q: s.q, within: s.within, w: s.q[4] >= 0 ? 1 : -1, det: detJ(s.q) }));
}

// Plane grid: point(a, b) = origin + (u0 + (a + ½)·step)·eu + (v0 + (b + ½)·step)·ev
export function planePoint(G, a, b) {
  const u = G.u0 + (a + 0.5) * G.step, v = G.v0 + (b + 0.5) * G.step, p = [0, 0, 0];
  p[G.ui] = u; p[G.vi] = v; p[G.n] = G.h;
  return p;
}

// Rows j of the grid → counts and a flat list of solutions [pixel, q2, q3, w, det, within]
export function computeRows(R, len, G, rows) {
  const out = [];
  for (const j of rows) {
    const all = new Uint8Array(G.nu), inl = new Uint8Array(G.nu), sols = [];
    for (let i = 0; i < G.nu; i++) {
      const S = solveTool(R, len, planePoint(G, i, j));
      all[i] = S.length; inl[i] = S.filter((s) => s.within).length;
      for (const s of S) sols.push(j * G.nu + i, s.q[1], s.q[2], s.w, s.det, s.within ? 1 : 0);
    }
    out.push({ j, all, inl, sols: Float64Array.from(sols) });
  }
  return out;
}
