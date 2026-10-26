// Small dense linear-algebra and rigid-body helpers.
// Matrices are arrays of rows. Everything is plain JS so it runs in the browser and in Node.

export const PI = Math.PI;
export const deg = (r) => (r * 180) / PI;
export const rad = (d) => (d * PI) / 180;

export const eye = (n) => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
export const zeros = (r, c) => Array.from({ length: r }, () => new Array(c).fill(0));
export const clone = (A) => A.map((row) => row.slice());

export function matMul(A, B) {
  const n = A.length, m = B[0].length, k = B.length;
  const C = zeros(n, m);
  for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) {
    let s = 0;
    for (let t = 0; t < k; t++) s += A[i][t] * B[t][j];
    C[i][j] = s;
  }
  return C;
}
export const mul = (...Ms) => Ms.reduce((A, B) => matMul(A, B));
export const matVec = (A, v) => A.map((row) => row.reduce((s, a, j) => s + a * v[j], 0));
export const transpose = (A) => A[0].map((_, j) => A.map((row) => row[j]));

export const add = (a, b) => a.map((x, i) => x + b[i]);
export const sub = (a, b) => a.map((x, i) => x - b[i]);
export const scale = (a, s) => a.map((x) => x * s);
export const dot = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
export const norm = (a) => Math.sqrt(dot(a, a));
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const unit = (a) => scale(a, 1 / norm(a));

export const skew = ([x, y, z]) => [[0, -z, y], [z, 0, -x], [-y, x, 0]];

export const rotX = (t) => { const c = Math.cos(t), s = Math.sin(t); return [[1, 0, 0], [0, c, -s], [0, s, c]]; };
export const rotY = (t) => { const c = Math.cos(t), s = Math.sin(t); return [[c, 0, s], [0, 1, 0], [-s, 0, c]]; };
export const rotZ = (t) => { const c = Math.cos(t), s = Math.sin(t); return [[c, -s, 0], [s, c, 0], [0, 0, 1]]; };
// URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll)
export const rpy = ([r, p, y]) => mul(rotZ(y), rotY(p), rotX(r));

// Rodrigues: rotation of angle t about unit axis w
export function expSO3(w, t) {
  const W = skew(w), W2 = matMul(W, W), s = Math.sin(t), c = 1 - Math.cos(t);
  return [0, 1, 2].map((i) => [0, 1, 2].map((j) => (i === j ? 1 : 0) + s * W[i][j] + c * W2[i][j]));
}

// Homogeneous transforms
export function homog(R, p = [0, 0, 0]) {
  return [[...R[0], p[0]], [...R[1], p[1]], [...R[2], p[2]], [0, 0, 0, 1]];
}
export const rotOf = (T) => [T[0].slice(0, 3), T[1].slice(0, 3), T[2].slice(0, 3)];
export const posOf = (T) => [T[0][3], T[1][3], T[2][3]];
export const trans = (p) => homog(eye(3), p);
export function invT(T) {
  const Rt = transpose(rotOf(T));
  return homog(Rt, scale(matVec(Rt, posOf(T)), -1));
}

// Matrix exponential of a twist S = (w, v) (unit w, revolute) times angle t
export function expSE3(w, v, t) {
  const R = expSO3(w, t), W = skew(w), W2 = matMul(W, W);
  const s = Math.sin(t), c = Math.cos(t);
  const G = [0, 1, 2].map((i) => [0, 1, 2].map((j) => (i === j ? t : 0) + (1 - c) * W[i][j] + (t - s) * W2[i][j]));
  return homog(R, matVec(G, v));
}

// Adjoint of T acting on twists ordered (w, v)
export function adjoint(T) {
  const R = rotOf(T), P = matMul(skew(posOf(T)), R), A = zeros(6, 6);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    A[i][j] = R[i][j]; A[i + 3][j + 3] = R[i][j]; A[i + 3][j] = P[i][j];
  }
  return A;
}

// Rotation angle between two rotation matrices
export function rotDistance(R1, R2) {
  const R = matMul(transpose(R1), R2);
  const c = (R[0][0] + R[1][1] + R[2][2] - 1) / 2;
  const s = Math.hypot(R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]) / 2;
  return Math.atan2(s, c); // stable near 0, unlike acos
}
export const maxAbsDiff = (A, B) => Math.max(...A.flat().map((a, i) => Math.abs(a - B.flat()[i])));

// Quaternion [w,x,y,z] -> rotation matrix
export function quatToR([w, x, y, z]) {
  const n = Math.hypot(w, x, y, z); w /= n; x /= n; y /= n; z /= n;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ];
}
export function rToQuat(R) {
  const tr = R[0][0] + R[1][1] + R[2][2];
  let w, x, y, z;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = s / 4; x = (R[2][1] - R[1][2]) / s; y = (R[0][2] - R[2][0]) / s; z = (R[1][0] - R[0][1]) / s; }
  else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) { const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2; w = (R[2][1] - R[1][2]) / s; x = s / 4; y = (R[0][1] + R[1][0]) / s; z = (R[0][2] + R[2][0]) / s; }
  else if (R[1][1] > R[2][2]) { const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2; w = (R[0][2] - R[2][0]) / s; x = (R[0][1] + R[1][0]) / s; y = s / 4; z = (R[1][2] + R[2][1]) / s; }
  else { const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2; w = (R[1][0] - R[0][1]) / s; x = (R[0][2] + R[2][0]) / s; y = (R[1][2] + R[2][1]) / s; z = s / 4; }
  return [w, x, y, z];
}

// Determinant by partial-pivot LU
export function det(A) {
  const M = clone(A), n = M.length;
  let d = 1;
  for (let k = 0; k < n; k++) {
    let p = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(M[i][k]) > Math.abs(M[p][k])) p = i;
    if (M[p][k] === 0) return 0;
    if (p !== k) { [M[p], M[k]] = [M[k], M[p]]; d = -d; }
    d *= M[k][k];
    for (let i = k + 1; i < n; i++) {
      const f = M[i][k] / M[k][k];
      for (let j = k; j < n; j++) M[i][j] -= f * M[k][j];
    }
  }
  return d;
}

// Eigenvalues of a symmetric matrix (cyclic Jacobi). Returns sorted descending.
export function symEig(S) {
  const A = clone(S), n = A.length;
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += A[i][j] * A[i][j];
    if (off < 1e-26) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(A[p][q]) < 1e-300) continue;
      const th = (A[q][q] - A[p][p]) / (2 * A[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) {
        const akp = A[k][p], akq = A[k][q];
        A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < n; k++) {
        const apk = A[p][k], aqk = A[q][k];
        A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
      }
    }
  }
  return A.map((r, i) => r[i]).sort((a, b) => b - a);
}
// Symmetric eigen-decomposition with vectors. Returns { values, vectors } sorted descending;
// vectors[k] is the k-th eigenvector.
export function symEigVec(S) {
  const A = clone(S), n = A.length, V = eye(n);
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += A[i][j] * A[i][j];
    if (off < 1e-26) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(A[p][q]) < 1e-300) continue;
      const th = (A[q][q] - A[p][p]) / (2 * A[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) {
        const akp = A[k][p], akq = A[k][q]; A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
        const vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
      }
      for (let k = 0; k < n; k++) {
        const apk = A[p][k], aqk = A[q][k]; A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
      }
    }
  }
  const order = A.map((r, i) => [r[i], i]).sort((a, b) => b[0] - a[0]);
  return { values: order.map((o) => o[0]), vectors: order.map(([, i]) => V.map((row) => row[i])) };
}
// Singular values of a (square or tall) matrix, descending
export const singularValues = (J) => symEig(matMul(transpose(J), J)).map((l) => Math.sqrt(Math.max(0, l)));

export const fmt = (x, d = 4) => {
  if (!Number.isFinite(x)) return String(x);
  const v = Math.abs(x) < 0.5 * 10 ** -d ? 0 : x;
  return v.toFixed(d);
};
export const fmtSci = (x) => (x === 0 ? '0' : x.toExponential(1));
