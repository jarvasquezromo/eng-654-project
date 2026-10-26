import { RobotViewer } from '../viewer.js';
import { mountChrome, mountTabs, jointSliders, renderMath, $, $$ } from '../ui.js';
import { toolJacobian, conditioning, REFERENCE_Q, LIMITS } from '../gofa.js';
import { deg, rad, fmt, fmtSci, det, symEigVec, matMul, transpose } from '../math.js';

mountChrome();
mountTabs();

const Q2 = [-Math.PI, Math.PI], Q3 = LIMITS[2];
const Q3STAR = -Math.PI / 2 + Math.atan2(0.110, 0.470);
const CLASS = [
  { key: 'shoulder', name: 'Shoulder (q₁)', color: '#2f6fe4' },
  { key: 'elbow', name: 'Elbow (q₂, q₃)', color: '#ff0000' },
  { key: 'wrist', name: 'Wrist-coupled (q₄–q₆)', color: '#d4a017' },
];
const state = { metric: 'invkappa', curves: true, elbow: true, res: 180, L: 0.5, fixed: REFERENCE_Q.slice(), probe: [REFERENCE_Q[1], REFERENCE_Q[2]] };
let grid = null, job = 0;

const canvas = $('#canvas'), ctx = canvas.getContext('2d');
const PAD = { l: 58, r: 16, t: 54, b: 92 };
const inset = new RobotViewer($('#inset'));
inset.camera.position.set(1.9, -1.9, 1.3);
inset.load().then(() => { inset.setOpacity(1); pose(); });

const sliders = jointSliders($('#sliders'), REFERENCE_Q, (q) => { state.fixed = q; compute(); }, { only: [3, 4] });
$('#ref').onclick = () => sliders.set(REFERENCE_Q);
$('#flat').onclick = () => sliders.set([0, 0, 0, 0, 0, 0]);

const qAt = (q2, q3) => [state.fixed[0], q2, q3, state.fixed[3], state.fixed[4], state.fixed[5]];
function classify(J) {
  const Js = J.map((row, i) => (i < 3 ? row.map((x) => x / state.L) : row));
  const { vectors } = symEigVec(matMul(transpose(Js), Js));
  const v = vectors[5];
  const e = [v[0] ** 2, v[1] ** 2 + v[2] ** 2, v[3] ** 2 + v[4] ** 2 + v[5] ** 2];
  return { v, e, cls: e.indexOf(Math.max(...e)) };
}

// ---------------- computation (chunked so the page stays responsive)
function compute() {
  const id = ++job, n = state.res, m = Math.round(n * 0.86);
  const val = { invkappa: new Float64Array((n + 1) * (m + 1)), sigmamin: new Float64Array((n + 1) * (m + 1)), det: new Float64Array((n + 1) * (m + 1)) };
  $('#progress').hidden = false;
  let a = 0;
  const step = () => {
    if (id !== job) return;
    const t0 = performance.now();
    for (; a <= n && performance.now() - t0 < 30; a++) {
      const q2 = Q2[0] + ((Q2[1] - Q2[0]) * a) / n;
      for (let b = 0; b <= m; b++) {
        const q3 = Q3[0] + ((Q3[1] - Q3[0]) * b) / m;
        const c = conditioning(toolJacobian(qAt(q2, q3)), state.L), k = a * (m + 1) + b;
        val.invkappa[k] = c.invKappa; val.sigmamin[k] = c.sigmaMin; val.det[k] = c.det;
      }
      $('#progress').textContent = `Computing ${Math.round((100 * a) / n)}%`;
    }
    if (a <= n) return requestAnimationFrame(step);
    // singular cells
    const curves = [];
    for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) {
      const d = val.det, k = i * (m + 1) + j;
      if (Math.sign(d[k]) !== Math.sign(d[k + 1]) || Math.sign(d[k]) !== Math.sign(d[k + m + 1])) {
        const q2 = Q2[0] + ((Q2[1] - Q2[0]) * (i + 0.5)) / n, q3 = Q3[0] + ((Q3[1] - Q3[0]) * (j + 0.5)) / m;
        curves.push({ q2, q3, cls: classify(toolJacobian(qAt(q2, q3))).cls });
      }
    }
    grid = { n, m, val, curves };
    $('#progress').hidden = true;
    summarize();
    paint();
  };
  requestAnimationFrame(step);
}

function summarize() {
  const v = grid.val.invkappa;
  let best = 0, bi = 0;
  v.forEach((x, i) => { if (x > best) { best = x; bi = i; } });
  const q2b = Q2[0] + ((Q2[1] - Q2[0]) * Math.floor(bi / (grid.m + 1))) / grid.n, q3b = Q3[0] + ((Q3[1] - Q3[0]) * (bi % (grid.m + 1))) / grid.m;
  const counts = CLASS.map((_, c) => grid.curves.filter((p) => p.cls === c).length);
  $('#summary').innerHTML = [
    ['best 1/κ', fmt(best, 3)], ['at (q₂, q₃)', `${deg(q2b).toFixed(0)}°, ${deg(q3b).toFixed(0)}°`],
    ...CLASS.map((c, i) => [`<span class="dot" style="background:${c.color}"></span>${c.name.split(' ')[0]} cells`, counts[i]]),
  ].map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
}

// ---------------- drawing
const MAGMA = [[0, [252, 253, 191]], [0.25, [252, 137, 97]], [0.5, [183, 55, 121]], [0.75, [81, 18, 124]], [1, [0, 0, 4]]];
function magma(t) { // t = 0 well conditioned … 1 singular
  t = Math.max(0, Math.min(1, t));
  for (let i = 1; i < MAGMA.length; i++) if (t <= MAGMA[i][0]) {
    const [t0, c0] = MAGMA[i - 1], [t1, c1] = MAGMA[i], u = (t - t0) / (t1 - t0);
    return c0.map((x, k) => x + u * (c1[k] - x));
  }
  return MAGMA[MAGMA.length - 1][1];
}
function colorFor(k) {
  const v = grid.val;
  if (state.metric === 'invkappa') return magma(1 - v.invkappa[k] / grid.maxInv);
  if (state.metric === 'sigmamin') return magma(1 - v.sigmamin[k] / grid.maxSig);
  const d = v.det[k], t = Math.min(1, Math.abs(d) / grid.maxDet) ** 0.5;
  return d >= 0 ? [255 - 120 * t, 255 - 200 * t, 255 - 200 * t] : [255 - 200 * t, 255 - 150 * t, 255 - 40 * t];
}

function paint() {
  const r = canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio, 2);
  canvas.width = r.width * dpr; canvas.height = r.height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, r.width, r.height);
  ctx.fillStyle = '#fff'; ctx.fillRect(0, 0, r.width, r.height);
  if (!grid) return;
  const W = r.width - PAD.l - PAD.r - (r.width > 980 ? 280 : 0), H = r.height - PAD.t - PAD.b;
  state.plot = { x: PAD.l, y: PAD.t, w: W, h: H };
  grid.maxInv = Math.max(...grid.val.invkappa); grid.maxSig = Math.max(...grid.val.sigmamin); grid.maxDet = Math.max(...grid.val.det.map(Math.abs));
  const img = ctx.createImageData(grid.n + 1, grid.m + 1);
  for (let a = 0; a <= grid.n; a++) for (let b = 0; b <= grid.m; b++) {
    const c = colorFor(a * (grid.m + 1) + b), p = ((grid.m - b) * (grid.n + 1) + a) * 4;
    img.data[p] = c[0]; img.data[p + 1] = c[1]; img.data[p + 2] = c[2]; img.data[p + 3] = 255;
  }
  const off = new OffscreenCanvas(grid.n + 1, grid.m + 1); off.getContext('2d').putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = true; ctx.drawImage(off, PAD.l, PAD.t, W, H);

  const X = (q2) => PAD.l + ((q2 - Q2[0]) / (Q2[1] - Q2[0])) * W, Y = (q3) => PAD.t + (1 - (q3 - Q3[0]) / (Q3[1] - Q3[0])) * H;
  state.X = X; state.Y = Y;
  if (state.curves) grid.curves.forEach((p) => { ctx.fillStyle = CLASS[p.cls].color; ctx.fillRect(X(p.q2) - 1.3, Y(p.q3) - 1.3, 2.6, 2.6); });
  if (state.elbow) {
    ctx.setLineDash([6, 4]); ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.4;
    ctx.beginPath(); ctx.moveTo(PAD.l, Y(Q3STAR)); ctx.lineTo(PAD.l + W, Y(Q3STAR)); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#fff'; ctx.font = '600 11px Inter, Arial'; ctx.fillText('q₃* = −76.83°', PAD.l + 6, Y(Q3STAR) - 5);
  }
  // reference + probe markers
  const mark = (q2, q3, col, size) => { ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(X(q2) - size, Y(q3)); ctx.lineTo(X(q2) + size, Y(q3)); ctx.moveTo(X(q2), Y(q3) - size); ctx.lineTo(X(q2), Y(q3) + size); ctx.stroke(); };
  mark(REFERENCE_Q[1], REFERENCE_Q[2], '#ffffff', 7);
  ctx.beginPath(); ctx.arc(X(state.probe[0]), Y(state.probe[1]), 6, 0, 2 * Math.PI); ctx.strokeStyle = '#00e0ff'; ctx.lineWidth = 2.2; ctx.stroke();
  // axes
  ctx.strokeStyle = '#141414'; ctx.lineWidth = 1; ctx.strokeRect(PAD.l, PAD.t, W, H);
  ctx.fillStyle = '#141414'; ctx.font = '11px Inter, Arial'; ctx.textAlign = 'center';
  for (let d = -180; d <= 180; d += 45) { ctx.fillText(`${d}°`, X(rad(d)), PAD.t + H + 16); }
  ctx.fillText('q₂ (shoulder)', PAD.l + W / 2, PAD.t + H + 34);
  ctx.textAlign = 'right';
  for (let d = -225; d <= 85; d += 45) { ctx.fillText(`${d}°`, PAD.l - 6, Y(rad(d)) + 4); }
  ctx.fillText(`85°`, PAD.l - 6, Y(Q3[1]) + 4);
  ctx.save(); ctx.translate(16, PAD.t + H / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText('q₃ (elbow) — full joint-3 range', 0, 0); ctx.restore();
  ctx.textAlign = 'left';
  legend();
}

function legend() {
  const lab = { invkappa: '1/κ: dark = singular, light = well conditioned', sigmamin: 'σ_min: dark = singular', det: 'det J: red > 0, blue < 0, white ≈ 0' }[state.metric];
  $('#legend').innerHTML = `<span>${lab}</span>${state.curves ? CLASS.map((c) => `<span><span class="dot" style="background:${c.color}"></span>${c.name}</span>`).join('') : ''}<span>✛ reference · ○ probe</span>`;
}

// ---------------- probe
function pose() {
  const q = qAt(...state.probe);
  inset.setQ(q);
  $('#insetCap').textContent = `q₂ = ${deg(q[1]).toFixed(1)}°, q₃ = ${deg(q[2]).toFixed(1)}°`;
  const J = toolJacobian(q), c = conditioning(J, state.L), { v, e, cls } = classify(J);
  $('#probeTiles').innerHTML = [['q₂', `${deg(q[1]).toFixed(1)}°`], ['q₃', `${deg(q[2]).toFixed(1)}°`], ['1/κ', fmt(c.invKappa, 4)], ['σ_min', fmt(c.sigmaMin, 4)], ['det J', fmtSci(det(J))]]
    .map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
  $('#nullvec').innerHTML = v.map((x, i) => `<div class="bar"><span>q${'₁₂₃₄₅₆'[i]}</span><span class="track"><span class="fill" style="width:${Math.abs(x) * 100}%;background:${i === 0 ? '#2f6fe4' : i < 3 ? '#ff0000' : '#d4a017'}"></span></span><span class="mono">${fmt(x, 2)}</span></div>`).join('');
  $('#nullNote').innerHTML = `Energy split: q₁ ${(100 * e[0]).toFixed(0)}% · q₂,q₃ ${(100 * e[1]).toFixed(0)}% · q₄–q₆ ${(100 * e[2]).toFixed(0)}%. Nearest family: <b style="color:${CLASS[cls].color}">${CLASS[cls].name}</b>. ${c.invKappa > 0.05 ? 'This point is regular, so the vector shows the <i>weakest</i> direction, not a lost one.' : 'Near-singular: this joint combination produces almost no tool twist.'}`;
  if (grid) paint();
}
let dragging = false;
const probeAt = (ev) => {
  if (!state.plot) return;
  const r = canvas.getBoundingClientRect(), x = ev.clientX - r.left, y = ev.clientY - r.top, P = state.plot;
  if (x < P.x || x > P.x + P.w || y < P.y || y > P.y + P.h) return;
  state.probe = [Q2[0] + ((x - P.x) / P.w) * (Q2[1] - Q2[0]), Q3[0] + (1 - (y - P.y) / P.h) * (Q3[1] - Q3[0])];
  pose();
};
canvas.addEventListener('pointerdown', (e) => { dragging = true; probeAt(e); });
canvas.addEventListener('pointermove', (e) => dragging && probeAt(e));
window.addEventListener('pointerup', () => { dragging = false; });
new ResizeObserver(() => paint()).observe($('#heat'));

// ---------------- controls
$$('#metric button').forEach((b) => (b.onclick = () => { state.metric = b.dataset.m; $$('#metric button').forEach((x) => x.classList.toggle('on', x === b)); paint(); }));
$$('#overlays button').forEach((b) => (b.onclick = () => { const k = b.dataset.o; state[k] = !state[k]; b.classList.toggle('on', state[k]); paint(); }));
$('#res').oninput = (e) => { $('#resOut').textContent = e.target.value; };
$('#res').onchange = (e) => { state.res = Number(e.target.value); compute(); };
$('#L').oninput = (e) => { $('#Lout').textContent = Number(e.target.value).toFixed(2); };
$('#L').onchange = (e) => { state.L = Number(e.target.value); compute(); pose(); };

// ---------------- invariance check (q1, q6 do not change det or σ)
(function invariance() {
  let eDet = 0, eSig = 0;
  for (let i = 0; i < 400; i++) {
    const q = LIMITS.map(([a, b]) => a + Math.random() * (b - a)), q2 = q.slice();
    q2[0] = Math.random() * 6.28 - 3.14; q2[5] = Math.random() * 9.4 - 4.7;
    const c1 = conditioning(toolJacobian(q)), c2 = conditioning(toolJacobian(q2));
    eDet = Math.max(eDet, Math.abs(c1.det - c2.det)); eSig = Math.max(eSig, ...c1.sigma.map((s, k) => Math.abs(s - c2.sigma[k])));
  }
  $('#inv').innerHTML = `400 random configurations, q₁ and q₆ re-drawn at random: max |Δ det J| = <b>${fmtSci(eDet)}</b>, max |Δσ| = <b>${fmtSci(eSig)}</b>. So a (q₂, q₃) slice at fixed (q₄, q₅) is complete for the conditioning question.`;
})();

compute();
renderMath();
