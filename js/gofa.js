// ABB GoFa (CRB 15000-5/0.95) kinematic models: URDF chain, standard D–H, Product of Exponentials,
// and screw-based Jacobians. Three independent FK paths so they can be cross-validated.
import {
  PI, eye, mul, matMul, matVec, transpose, homog, rotOf, posOf, trans, rpy, expSO3, expSE3,
  adjoint, cross, sub, add, scale, invT, det, singularValues, rotDistance, norm, skew,
} from './math.js';

// ---------------------------------------------------------------- URDF
const attr = (tag, name) => {
  const m = tag.match(new RegExp(`${name}\\s*=\\s*"([^"]*)"`));
  return m ? m[1] : null;
};
const nums = (s) => s.trim().split(/\s+/).map(Number);

// Minimal URDF parser (regex based so it also runs in Node). Returns joints in file order.
export function parseURDF(text) {
  const joints = [];
  for (const m of text.matchAll(/<joint\b([^>]*)>([\s\S]*?)<\/joint>/g)) {
    const head = m[1], body = m[2];
    const origin = body.match(/<origin\b[^>]*>/)?.[0] ?? '';
    const axis = body.match(/<axis\b[^>]*>/)?.[0];
    const limit = body.match(/<limit\b[^>]*>/)?.[0];
    joints.push({
      name: attr(head, 'name'),
      type: attr(head, 'type'),
      parent: body.match(/<parent\s+link="([^"]+)"/)[1],
      child: body.match(/<child\s+link="([^"]+)"/)[1],
      xyz: attr(origin, 'xyz') ? nums(attr(origin, 'xyz')) : [0, 0, 0],
      rpy: attr(origin, 'rpy') ? nums(attr(origin, 'rpy')) : [0, 0, 0],
      axis: axis ? nums(attr(axis, 'xyz')) : null,
      lower: limit ? Number(attr(limit, 'lower')) : null,
      upper: limit ? Number(attr(limit, 'upper')) : null,
      velocity: limit ? Number(attr(limit, 'velocity')) : null,
    });
  }
  const links = [...text.matchAll(/<link\s+name="([^"]+)"\s*(\/?)>([\s\S]*?)(?=<link|<joint|<\/robot>)/g)].map((m) => ({
    name: m[1],
    mesh: m[2] ? null : (m[3].match(/<visual>[\s\S]*?filename="([^"]+)"/)?.[1] ?? null),
    color: m[2] ? null : (m[3].match(/<visual>[\s\S]*?rgba="([^"]+)"/)?.[1] ?? null),
  }));
  return { name: text.match(/<robot\s+name="([^"]+)"/)?.[1], joints, links };
}

// Ordered chain base_link -> tool0
export function chainTo(model, tip = 'tool0', root = 'base_link') {
  const byChild = Object.fromEntries(model.joints.map((j) => [j.child, j]));
  const chain = [];
  for (let l = tip; l !== root; l = byChild[l].parent) chain.unshift(byChild[l]);
  return chain;
}

// URDF forward kinematics. Returns world transforms of every child link and the tool.
export function urdfFK(model, q) {
  const chain = chainTo(model);
  let T = eye(4), k = 0;
  const links = { base_link: T };
  const jointFrames = [];
  for (const j of chain) {
    T = mul(T, trans(j.xyz), homog(rpy(j.rpy)));
    if (j.type === 'revolute' || j.type === 'continuous') {
      jointFrames.push(T);
      T = matMul(T, homog(expSO3(j.axis, q[k++])));
    }
    links[j.child] = T;
  }
  return { links, jointFrames, tool: T };
}

// ---------------------------------------------------------------- standard D–H
// Rz(θ) Tz(d) Tx(a) Rx(α), with θ_i = q_i + θ_i,0. Frame i-1's z axis is joint i's axis,
// and each z points the same way as the URDF axis so joint signs match the solver.
export const DH = [
  { theta0: 0, d: 0.265, a: 0, alpha: -PI / 2 },
  { theta0: -PI / 2, d: 0, a: 0.444, alpha: 0 },
  { theta0: 0, d: 0, a: 0.110, alpha: -PI / 2 },
  { theta0: 0, d: 0.470, a: 0, alpha: PI / 2 },
  { theta0: 0, d: 0, a: 0.080, alpha: -PI / 2 },
  { theta0: PI, d: 0.101, a: 0, alpha: 0 },
];

export function dhLink({ theta0, d, a, alpha }, q) {
  const t = q + theta0, ct = Math.cos(t), st = Math.sin(t), ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1],
  ];
}
// Frames 0..6 (frame 0 = base, frame 6 = tool0)
export function dhFK(q, table = DH) {
  const frames = [eye(4)];
  table.forEach((row, i) => frames.push(matMul(frames[i], dhLink(row, q[i]))));
  return frames;
}

// ---------------------------------------------------------------- Product of Exponentials
// Space-frame screw axes at q = 0: unit direction ω and a point r on the axis, v = -ω × r.
export const SCREW_AXES = [
  { w: [0, 0, 1], r: [0, 0, 0.265] },
  { w: [0, 1, 0], r: [0, 0, 0.265] },
  { w: [0, 1, 0], r: [0, 0, 0.709] },
  { w: [1, 0, 0], r: [0, 0, 0.819] },
  { w: [0, 1, 0], r: [0.470, 0, 0.819] },
  { w: [1, 0, 0], r: [0.571, 0, 0.899] },
].map((s) => ({ ...s, v: scale(cross(s.w, s.r), -1) }));

// Home pose of tool0: rotation Ry(π/2), origin (0.571, 0, 0.899)
export const M_HOME = homog([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], [0.571, 0, 0.899]);

export function poeFK(q, screws = SCREW_AXES, M = M_HOME) {
  let T = eye(4);
  screws.forEach((s, i) => { T = matMul(T, expSE3(s.w, s.v, q[i])); });
  return matMul(T, M);
}

// ---------------------------------------------------------------- Jacobians
// Space Jacobian J_s (twists ordered ω on top, v below), column i = Ad(e^[S1]q1 … e^[S_{i-1}]q_{i-1}) S_i
export function spaceJacobian(q, screws = SCREW_AXES) {
  let T = eye(4);
  const cols = screws.map((s, i) => {
    const col = matVec(adjoint(T), [...s.w, ...s.v]);
    T = matMul(T, expSE3(s.w, s.v, q[i]));
    return col;
  });
  return transpose(cols);
}

// Current joint axes in world: direction ω_i and a point r_i on each axis
export function currentAxes(q, screws = SCREW_AXES) {
  let T = eye(4);
  return screws.map((s, i) => {
    const R = rotOf(T), p = posOf(T);
    const ax = { w: matVec(R, s.w), r: add(matVec(R, s.r), p) };
    T = matMul(T, expSE3(s.w, s.v, q[i]));
    return ax;
  });
}

// Geometric Jacobian about point p, resolved in basis R (world if omitted).
// Linear rows on top, angular rows below, matching the course's ^3J_5 layout.
export function pointJacobian(q, p, R = eye(3)) {
  const Rt = transpose(R);
  const cols = currentAxes(q).map(({ w, r }) => [...matVec(Rt, cross(w, sub(p, r))), ...matVec(Rt, w)]);
  return transpose(cols);
}
// Full-robot Jacobian at the tool point, world basis
export function toolJacobian(q) {
  return pointJacobian(q, posOf(poeFK(q)));
}
// Course subchain Jacobian ^3J_5: reference point O5 (D–H frame 5 origin), resolved in D–H frame 3
export function J35(q) {
  const F = dhFK(q);
  return pointJacobian(q, posOf(F[5]), rotOf(F[3]));
}

// Numerical Jacobian at tool point by central differences of PoE FK (for verification)
export function numericToolJacobian(q, h = 1e-6, fk = poeFK) {
  const p0 = posOf(fk(q));
  const cols = q.map((_, i) => {
    const qp = q.slice(), qm = q.slice(); qp[i] += h; qm[i] -= h;
    const Tp = fk(qp), Tm = fk(qm);
    const dp = scale(sub(posOf(Tp), posOf(Tm)), 1 / (2 * h));
    // ω from (dR/dq) R^T
    const dR = rotOf(Tp).map((row, a) => row.map((x, b) => (x - rotOf(Tm)[a][b]) / (2 * h)));
    const W = matMul(dR, transpose(rotOf(fk(q))));
    const w = [(W[2][1] - W[1][2]) / 2, (W[0][2] - W[2][0]) / 2, (W[1][0] - W[0][1]) / 2];
    return [...dp, ...w];
  });
  void p0;
  return transpose(cols);
}

// Conditioning of a 6×6 geometric Jacobian. Linear rows are divided by a characteristic length L
// so metres and radians are comparable.
export function conditioning(J, L = 0.5) {
  const Js = J.map((row, i) => (i < 3 ? row.map((x) => x / L) : row));
  const s = singularValues(Js);
  const smin = s[s.length - 1], smax = s[0];
  return {
    sigma: s,
    sigmaMin: smin,
    kappa: smin > 0 ? smax / smin : Infinity,
    invKappa: smax > 0 ? smin / smax : 0,
    manipulability: s.reduce((a, b) => a * b, 1),
    det: det(J),
  };
}

// ---------------------------------------------------------------- limits
export const LIMITS = [
  [-PI, PI], [-PI, PI], [-3.9269908169872414, 1.4835298641951802],
  [-PI, PI], [-PI, PI], [-4.71238898038469, 4.71238898038469],
];
export const JOINT_NAMES = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6'];
export const REFERENCE_Q = [0, 0.45, 0, -0.15, -1.55, 0]; // regular reference: kappa ≈ 5 with L = 0.5 m

export function poseError(TA, TB) {
  return { position: norm(sub(posOf(TA), posOf(TB))), rotation: rotDistance(rotOf(TA), rotOf(TB)) };
}

export { skew, invT };
