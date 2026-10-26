// Numerical cross-checks: node tests/check.mjs
import { readFileSync } from 'node:fs';
import { parseURDF, urdfFK, dhFK, poeFK, toolJacobian, numericToolJacobian, J35, conditioning, LIMITS, REFERENCE_Q, poseError, SCREW_AXES, chainTo } from '../js/gofa.js';
import { maxAbsDiff, posOf, rotOf, quatToR, homog, rotDistance, norm, sub, cross, unit } from '../js/math.js';

const here = new URL('.', import.meta.url);
const model = parseURDF(readFileSync(new URL('../assets/abb_gofa/crb15000_5_95.urdf', here), 'utf8'));
let failed = 0;
const check = (name, value, tol) => {
  const ok = value <= tol;
  if (!ok) failed++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name.padEnd(52)} ${value.toExponential(2)}  (tol ${tol})`);
};
const randQ = () => LIMITS.map(([lo, hi]) => lo + Math.random() * (hi - lo));

// 1. URDF vs DH vs PoE on random configurations
let eDH = 0, ePoE = 0, eDHPoE = 0;
for (let n = 0; n < 2000; n++) {
  const q = randQ();
  const Tu = urdfFK(model, q).tool, Td = dhFK(q)[6], Tp = poeFK(q);
  eDH = Math.max(eDH, maxAbsDiff(Tu, Td));
  ePoE = Math.max(ePoE, maxAbsDiff(Tu, Tp));
  eDHPoE = Math.max(eDHPoE, maxAbsDiff(Td, Tp));
}
check('FK  URDF vs D–H   (2000 random q, max |ΔT|)', eDH, 1e-12);
check('FK  URDF vs PoE   (2000 random q, max |ΔT|)', ePoE, 1e-12);
check('FK  D–H  vs PoE   (2000 random q, max |ΔT|)', eDHPoE, 1e-12);

// 2. D–H z_{i-1} equals URDF joint axis i (direction and line)
let eAxis = 0;
for (let n = 0; n < 200; n++) {
  const q = randQ();
  const F = dhFK(q), U = urdfFK(model, q).jointFrames, chain = chainTo(model).filter((j) => j.type === 'revolute');
  U.forEach((T, i) => {
    const wU = rotOf(T).map((r) => r.reduce((s, x, k) => s + x * chain[i].axis[k], 0));
    const wD = rotOf(F[i]).map((r) => r[2]);
    const off = cross(wU, sub(posOf(F[i]), posOf(T)));
    eAxis = Math.max(eAxis, norm(sub(wU, wD)), norm(off));
  });
}
check('D–H z_{i-1} = URDF axis i (direction + line)', eAxis, 1e-12);

// 3. Jacobian vs finite differences
let eJ = 0;
for (let n = 0; n < 300; n++) {
  const q = randQ();
  eJ = Math.max(eJ, maxAbsDiff(toolJacobian(q), numericToolJacobian(q)));
}
check('Jacobian (screw) vs central differences', eJ, 1e-7);

// 4. ^3J_5 vs finite differences of O5 expressed in frame 3
let e35 = 0;
for (let n = 0; n < 300; n++) {
  const q = randQ(), h = 1e-6, J = J35(q), F0 = dhFK(q), R3t = rotOf(F0[3]);
  for (let i = 0; i < 6; i++) {
    const qp = q.slice(), qm = q.slice(); qp[i] += h; qm[i] -= h;
    const dp = sub(posOf(dhFK(qp)[5]), posOf(dhFK(qm)[5])).map((x) => x / (2 * h));
    const dpIn3 = [0, 1, 2].map((a) => R3t[0][a] * dp[0] + R3t[1][a] * dp[1] + R3t[2][a] * dp[2]);
    for (let a = 0; a < 3; a++) e35 = Math.max(e35, Math.abs(dpIn3[a] - J[a][i]));
  }
}
check('^3J_5 linear block vs finite differences of O5', e35, 1e-7);

// 5. Supplied IK example: every returned q must reproduce the target pose with our FK
const ik = JSON.parse(readFileSync(new URL('../data/ik-example.json', here), 'utf8'));
const target = homog(quatToR([0.7071067811865476, 0, 0.7071067811865476, 0]), [0.571, 0, 0.899]);
let eIK = 0;
for (const s of ik.solutions) { const e = poseError(poeFK(s.q), target); eIK = Math.max(eIK, e.position, e.rotation); }
check(`Supplied IK solutions (${ik.solutions.length}) reproduce target in our FK`, eIK, 1e-9);

// 6. Reference configuration is regular
const c = conditioning(toolJacobian(REFERENCE_Q));
console.log(`INFO  reference q: sigma_min=${c.sigmaMin.toFixed(4)} kappa=${c.kappa.toFixed(2)} det=${c.det.toExponential(3)}`);
console.log(`INFO  home q=0: kappa=${conditioning(toolJacobian([0, 0, 0, 0, 0, 0])).kappa}`);
console.log(failed ? `\n${failed} check(s) failed` : '\nAll checks passed');
process.exit(failed ? 1 : 0);
