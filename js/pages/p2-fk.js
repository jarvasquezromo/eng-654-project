import { RobotViewer, loadModel } from '../viewer.js';
import { mountChrome, mountTabs, jointSliders, renderMath, $, $$ } from '../ui.js';
import { dhFK, poeFK, urdfFK, poseError, REFERENCE_Q, LIMITS } from '../gofa.js';
import { fmt, fmtSci, posOf, homog, quatToR, deg } from '../math.js';

mountChrome();
mountTabs();

const viewer = new RobotViewer($('#view'));
const layers = { ghost: true, chain: true };
let model, sliders;
const randQ = () => LIMITS.map(([a, b]) => a + Math.random() * (b - a));
const errCell = (e) => `<span class="${e < 1e-12 ? 'pass' : 'fail'}">${fmtSci(e)}</span>`;

function draw(q) {
  viewer.setQ(q);
  viewer.clearOverlay();
  const Tu = urdfFK(model, q).tool, F = dhFK(q), Td = F[6], Tp = poeFK(q);
  if (layers.chain) F.slice(0, 6).forEach((T, i) => viewer.addFrame(T, { size: 0.07, label: `{${i}}`, lineWidth: 0.003, opacity: 0.7 }));
  viewer.addFrame(Tu, { size: 0.2, lineWidth: 0.004, opacity: 0.35 });
  viewer.addFrame(Td, { size: 0.14, lineWidth: 0.006, label: 'tool0' });
  viewer.addFrame(Tp, { size: 0.08, lineWidth: 0.009 });
  const rows = [['URDF', Tu], ['D–H', Td], ['PoE', Tp]];
  $('#live').innerHTML = `<table><thead><tr><th>model</th><th>x</th><th>y</th><th>z</th></tr></thead><tbody>${
    rows.map(([n, T]) => `<tr><td>${n}</td>${posOf(T).map((x) => `<td>${fmt(x, 6)}</td>`).join('')}</tr>`).join('')}</tbody></table>`;
  const pairs = [['URDF – D–H', Tu, Td], ['URDF – PoE', Tu, Tp], ['D–H – PoE', Td, Tp]];
  $('#liveErr').innerHTML = `<table><thead><tr><th>pair</th><th>position (m)</th><th>rotation (rad)</th></tr></thead><tbody>${
    pairs.map(([n, A, B]) => { const e = poseError(A, B); return `<tr><td>${n}</td><td>${errCell(e.position)}</td><td>${errCell(e.rotation)}</td></tr>`; }).join('')}</tbody></table>`;
}

(async () => {
  ({ model } = await loadModel());
  await viewer.load();
  $('#status').hidden = true;
  viewer.setOpacity(0.35);
  sliders = jointSliders($('#sliders'), REFERENCE_Q, draw);
  draw(REFERENCE_Q);
  $('#rand').onclick = () => sliders.set(randQ());
  $('#home').onclick = () => sliders.set([0, 0, 0, 0, 0, 0]);
  runIK();
})();

$$('#views button').forEach((b) => (b.onclick = () => viewer.view(b.dataset.v)));
$$('#layers button').forEach((b) => (b.onclick = () => {
  const k = b.dataset.l; layers[k] = !layers[k]; b.classList.toggle('on', layers[k]);
  if (k === 'ghost') viewer.setOpacity(layers.ghost ? 0.35 : 1);
  draw(sliders.get());
}));

// ---------------- batch test
$('#run').onclick = () => {
  const N = Math.max(100, Math.min(100000, Number($('#n').value) || 5000));
  const keys = ['URDF – D–H', 'URDF – PoE', 'D–H – PoE'];
  const stats = keys.map(() => ({ pMax: 0, rMax: 0, pSum: 0, rSum: 0 }));
  const all = [];
  const t0 = performance.now();
  for (let n = 0; n < N; n++) {
    const q = randQ(), Tu = urdfFK(model, q).tool, Td = dhFK(q)[6], Tp = poeFK(q);
    [[Tu, Td], [Tu, Tp], [Td, Tp]].forEach(([A, B], k) => {
      const e = poseError(A, B), s = stats[k];
      s.pMax = Math.max(s.pMax, e.position); s.rMax = Math.max(s.rMax, e.rotation);
      s.pSum += e.position; s.rSum += e.rotation;
      if (k === 1) all.push(e.position);
    });
  }
  const ms = performance.now() - t0;
  $('#batchOut').innerHTML = `<table><thead><tr><th>pair</th><th>max pos (m)</th><th>mean pos</th><th>max rot (rad)</th><th>mean rot</th></tr></thead><tbody>${
    keys.map((k, i) => { const s = stats[i]; return `<tr><td>${k}</td><td>${errCell(s.pMax)}</td><td>${fmtSci(s.pSum / N)}</td><td>${errCell(s.rMax)}</td><td>${fmtSci(s.rSum / N)}</td></tr>`; }).join('')
  }</tbody></table><p class="note" style="margin-top:6px">${N.toLocaleString()} configurations in ${ms.toFixed(0)} ms. Green = below 10⁻¹² (machine precision on metre-scale numbers).</p>`;
  histogram(all.map((e) => Math.log10(Math.max(e, 1e-18))));
};

function histogram(values) {
  const lo = -18, hi = -12, bins = 24, counts = new Array(bins).fill(0);
  values.forEach((v) => { const b = Math.min(bins - 1, Math.max(0, Math.floor(((v - lo) / (hi - lo)) * bins))); counts[b]++; });
  const W = 400, H = 120, max = Math.max(...counts), bw = W / bins;
  const bars = counts.map((c, i) => `<rect x="${i * bw + 1}" y="${H - 18 - (c / max) * (H - 30)}" width="${bw - 2}" height="${(c / max) * (H - 30)}" fill="#ff0000" opacity=".8"/>`).join('');
  const ticks = [-18, -16, -14, -12].map((t) => `<text x="${((t - lo) / (hi - lo)) * W}" y="${H - 4}" font-size="10" fill="#686868" text-anchor="middle">1e${t}</text>`).join('');
  $('#hist').innerHTML = `<div class="label">URDF – PoE position error distribution (log scale)</div><svg viewBox="-12 0 ${W + 24} ${H}" width="100%">${bars}${ticks}</svg>`;
}

// ---------------- supplied IK example
async function runIK() {
  const ik = await fetch('../data/ik-example.json').then((r) => r.json());
  const target = homog(quatToR([0.7071067811865476, 0, 0.7071067811865476, 0]), [0.571, 0, 0.899]);
  $('#ikOut').innerHTML = `<table><thead><tr><th>#</th><th>q (°)</th><th>in limits</th><th>PoE pos</th><th>PoE rot</th><th>D–H pos</th></tr></thead><tbody>${
    ik.solutions.map((s, i) => {
      const ep = poseError(poeFK(s.q), target), ed = poseError(dhFK(s.q)[6], target);
      return `<tr><td>${i + 1}</td><td class="mono">${s.q.map((x) => deg(x).toFixed(1)).join(', ')}</td><td>${s.within_limits ? '✓' : '✗'}</td><td>${errCell(ep.position)}</td><td>${errCell(ep.rotation)}</td><td>${errCell(ed.position)}</td></tr>`;
    }).join('')
  }</tbody></table>`;
}

renderMath();
