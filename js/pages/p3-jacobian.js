import { RobotViewer } from '../viewer.js';
import { mountChrome, mountTabs, jointSliders, matrixTable, renderMath, $, $$ } from '../ui.js';
import { toolJacobian, numericToolJacobian, J35, currentAxes, poeFK, dhFK, conditioning, REFERENCE_Q, LIMITS } from '../gofa.js';
import { fmt, fmtSci, posOf, rotOf, norm, cross, sub, det, maxAbsDiff } from '../math.js';

mountChrome();
mountTabs();

const viewer = new RobotViewer($('#view'));
const state = { col: 4, axes: true, ghost: true, O5: true, tab: 'cols', L: 0.5 };
let sliders;
const ROWS = ['v<sub>x</sub>', 'v<sub>y</sub>', 'v<sub>z</sub>', 'ω<sub>x</sub>', 'ω<sub>y</sub>', 'ω<sub>z</sub>'];
const COLS = ['J₁', 'J₂', 'J₃', 'J₄', 'J₅', 'J₆'];
const randQ = () => LIMITS.map(([a, b]) => a + Math.random() * (b - a));
const zeroClass = (i, j, x) => (Math.abs(x) < 5e-4 ? 'zero' : '');

const JOINT_NOTES = [
  'Base yaw. ω is always vertical, so the linear part is horizontal and tangent to the circle the tool point draws around the base axis. Its length is the tool point\'s distance from that axis, and it vanishes when the point lies on the base axis (shoulder singularity).',
  'Shoulder pitch. ω₂ is always parallel to ω₃, so columns 2 and 3 have identical angular parts. Only their lever arms differ.',
  'Elbow pitch. Same ω as joint 2, so columns 2 and 3 become linearly dependent whenever the two lever arms are parallel (elbow-type singularity).',
  'Forearm roll. ω₄ lies along the forearm; its lever arm to the tool includes the 0.080 m wrist offset.',
  'Wrist pitch. ω₅ is perpendicular to ω₄. Axes 4 and 5 intersect, so about that intersection point column 5 has no linear part.',
  'Flange roll. Axis 6 passes through the tool point, so this column is a pure rotation: its linear part at the tool is exactly zero.',
];

function draw() {
  const q = sliders.get();
  viewer.setQ(q);
  viewer.clearOverlay();
  const axes = currentAxes(q), T = poeFK(q), F = dhFK(q);
  const ref = state.tab === 'j35' ? posOf(F[5]) : posOf(T);
  if (state.axes) axes.forEach(({ w, r }, i) => i !== state.col && viewer.addAxisLine(r, w, { length: 0.5, opacity: 0.6 }));
  const { w, r } = axes[state.col];
  viewer.addAxisLine(r, w, { color: 0xff0000, length: 0.9, radius: 0.006 });
  const v = cross(w, sub(ref, r));
  viewer.addPoint(ref, { color: 0x141414, radius: 0.01, label: state.tab === 'j35' ? 'O₅' : 'p' });
  viewer.addArrow(ref, v, { color: 0x2f6fe4, scale: 1.2, radius: 0.007 });
  viewer.addArrow(ref, w, { color: 0x9b51e0, scale: 0.18, radius: 0.006 });
  if (state.tab === 'j35' && state.O5) { viewer.addFrame(F[3], { size: 0.12, label: '{3}' }); viewer.addFrame(F[5], { size: 0.08, label: '{5}' }); }
  else viewer.addFrame(T, { size: 0.09, label: 'tool0' });

  // Columns tab
  const J = toolJacobian(q);
  $('#jTool').innerHTML = matrixTable(J, { digits: 3, rowLabels: ROWS, colLabels: COLS, highlightCol: state.col, cellClass: zeroClass });
  const lever = norm(v);
  $('#colTitle').textContent = `Column ${state.col + 1}`;
  $('#colInfo').innerHTML = `<p>${JOINT_NOTES[state.col]}</p>
    <table><tr><th>ω (world)</th><td>${w.map((x) => fmt(x, 3)).join(', ')}</td></tr>
    <tr><th>point on axis r</th><td>${r.map((x) => fmt(x, 3)).join(', ')} m</td></tr>
    <tr><th>distance p ↔ axis</th><td>${fmt(lever, 4)} m (= |linear part| per rad)</td></tr></table>`;

  // ³J₅ tab
  const K = J35(q);
  $('#j35').innerHTML = matrixTable(K, { digits: 3, rowLabels: ROWS, colLabels: COLS, highlightCol: state.tab === 'j35' ? state.col : -1, cellClass: (i, j, x) => `${zeroClass(i, j, x)} ${i < 3 && j >= 3 ? 'block' : ''}` });
  const lin = [3, 4, 5].map((j) => Math.hypot(K[0][j], K[1][j], K[2][j]));
  const A = [0, 1, 2].map((i) => K[i].slice(0, 3)), W = [3, 4, 5].map((i) => K[i].slice(3, 6));
  const dFull = det(K), dProd = det(A) * det(W);
  $('#j35Info').innerHTML = `<p>Upper-right block (wrist joints' linear velocity at O₅): column norms
    <b>${lin.map((x) => fmt(x, 4)).join(' · ')}</b> m/rad for joints 4 · 5 · 6.</p>
    <p>Axis 6 passes through O₅, so its column is zero. Axes 4 and 5 do not: O₅ sits 0.080 m off the 4–5 intersection, along the common normal of axes 5 and 6. The block is not zero, so the determinant does <b>not</b> split exactly:</p>
    <table><tr><th>det(³J₅)</th><td>${fmtSci(dFull)}</td></tr><tr><th>det(J<sub>A,v</sub>)·det(J<sub>W,ω</sub>)</th><td>${fmtSci(dProd)}</td></tr><tr><th>det(⁰J<sub>tool</sub>)</th><td>${fmtSci(det(J))}</td></tr></table>
    <p style="margin-top:8px">The product is still a useful approximation away from strong coupling, but GoFa's exact singularity set must come from the full determinant (Step 4).</p>`;

  // Conditioning tab
  const c = conditioning(J, state.L);
  $('#tiles').innerHTML = [['σ<sub>min</sub>', fmt(c.sigmaMin, 4)], ['κ', Number.isFinite(c.kappa) ? fmt(c.kappa, 1) : '∞'], ['1/κ', fmt(c.invKappa, 4)], ['w = Πσ', fmtSci(c.manipulability)], ['det J', fmtSci(c.det)]]
    .map(([k, v]) => `<div class="tile"><small>${k}</small><b>${v}</b></div>`).join('');
  const smax = c.sigma[0];
  $('#sigmas').innerHTML = c.sigma.map((s, i) => `<div class="row" style="gap:8px;margin:3px 0"><span style="width:22px">σ${'₁₂₃₄₅₆'[i]}</span><span style="flex:1;height:10px;background:var(--soft);border-radius:3px;overflow:hidden"><span style="display:block;height:100%;width:${(100 * s) / smax}%;background:${i === 5 ? 'var(--red)' : '#9a9a9a'}"></span></span><span class="mono" style="width:60px;text-align:right">${fmt(s, 4)}</span></div>`).join('');

  // Verify tab
  const N = numericToolJacobian(q);
  const colErr = COLS.map((_, j) => Math.max(...J.map((row, i) => Math.abs(row[j] - N[i][j]))));
  $('#fd').innerHTML = `<table><thead><tr><th>column</th>${COLS.map((c2) => `<th>${c2}</th>`).join('')}</tr></thead><tbody><tr><td>max |J − J<sub>FD</sub>|</td>${colErr.map((e) => `<td class="${e < 1e-7 ? 'pass' : 'fail'}">${fmtSci(e)}</td>`).join('')}</tr></tbody></table>`;
}

(async () => {
  await viewer.load();
  $('#status').hidden = true;
  viewer.setOpacity(0.4);
  sliders = jointSliders($('#sliders'), REFERENCE_Q, draw);
  $('#colpick').innerHTML = COLS.map((c, i) => `<button data-c="${i}">${c}</button>`).join('');
  $$('#colpick button').forEach((b) => (b.onclick = () => { state.col = Number(b.dataset.c); markCol(); draw(); }));
  markCol();
  draw();
  $('#ref').onclick = () => sliders.set(REFERENCE_Q);
  $('#home').onclick = () => sliders.set([0, 0, 0, 0, 0, 0]);
  $('#rand').onclick = () => sliders.set(randQ());
})();

function markCol() { $$('#colpick button').forEach((b) => b.classList.toggle('on', Number(b.dataset.c) === state.col)); }

document.querySelector('.tabs').addEventListener('tab', (e) => { state.tab = e.detail; if (sliders) draw(); });
$('#showO5').onchange = (e) => { state.O5 = e.target.checked; draw(); };
$('#L').oninput = (e) => { state.L = Number(e.target.value); $('#Lout').textContent = state.L.toFixed(2); draw(); };
$$('#views button').forEach((b) => (b.onclick = () => viewer.view(b.dataset.v)));
$$('#layers button').forEach((b) => (b.onclick = () => {
  const k = b.dataset.l; state[k] = !state[k]; b.classList.toggle('on', state[k]);
  if (k === 'ghost') viewer.setOpacity(state.ghost ? 0.4 : 1);
  draw();
}));
$('#batch').onclick = () => {
  let eJ = 0, e35 = 0;
  for (let n = 0; n < 1000; n++) {
    const q = randQ();
    eJ = Math.max(eJ, maxAbsDiff(toolJacobian(q), numericToolJacobian(q)));
    // ³J₅ vs re-resolving the tool Jacobian at O₅ in frame 3 by hand
    const F = dhFK(q), R3 = rotOf(F[3]), shift = sub(posOf(F[5]), posOf(poeFK(q)));
    const J = toolJacobian(q);
    const moved = J[0].map((_, j) => { const w = [J[3][j], J[4][j], J[5][j]]; const v = [J[0][j], J[1][j], J[2][j]]; return [...sub(v, cross(shift, w)), ...w]; });
    const K = J35(q);
    moved.forEach((col, j) => {
      const vl = [0, 1, 2].map((a) => R3[0][a] * col[0] + R3[1][a] * col[1] + R3[2][a] * col[2]);
      const wl = [0, 1, 2].map((a) => R3[0][a] * col[3] + R3[1][a] * col[4] + R3[2][a] * col[5]);
      [...vl, ...wl].forEach((x, i) => { e35 = Math.max(e35, Math.abs(x - K[i][j])); });
    });
  }
  $('#batchOut').innerHTML = `<table><tr><th>screw J vs finite differences</th><td class="${eJ < 1e-7 ? 'pass' : 'fail'}">${fmtSci(eJ)}</td></tr>
    <tr><th>³J₅ vs shifted + rotated tool J</th><td class="${e35 < 1e-12 ? 'pass' : 'fail'}">${fmtSci(e35)}</td></tr></table>
    <p class="note" style="margin-top:6px">Finite-difference error is about h² ≈ 10⁻¹⁰ by construction. The ³J₅ check is exact algebra, so it sits at machine precision.</p>`;
};

renderMath();
