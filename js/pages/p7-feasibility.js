// Step 7: branch-resolved placement-feasibility and failure maps, for two global tool orientations.
import { mountChrome, mountTabs, renderMath, $, $$ } from '../ui.js';
import { loadSettings, saveSettings, gridCenters, REASONS, REASON_TEXT } from '../tasks.js';
import { bindSettings, branchName, orientName, REASON_COLORS } from '../taskui.js';
import { BRANCHES } from '../ik.js';
import { makePool } from '../pool.js';
import { useSupplied, solveVerified, statsHTML, getSource } from '../supplied.js';
import { mountIkSource } from '../ikui.js';
import { contourPoses, centerOf, tool0Of } from '../tasks.js';

mountChrome();
mountTabs();
mountIkSource();

const S = loadSettings();
const pool = makePool();
const state = { which: 'A', mode: 'reason', branch: 'all', probe: null };
const cache = {}; // key → { results: [], done }
const orient = () => (state.which === 'A' ? S.orientA : S.orientB);
const keyOf = (o) => `${o}|${JSON.stringify([getSource(), S.a, S.b, S.tilt, S.spin, S.N, S.tcp, S.z, S.xRange, S.yRange, S.gridN, S.dqMax, S.margin, S.L])}`;

$('#branch').innerHTML = '<option value="all">All start branches</option>' + BRANCHES.map((b, i) => `<option value="${i}">Start on ${branchName(i)}</option>`).join('');
$('#mode').onchange = (e) => { state.mode = e.target.value; paintAll(); };
$('#branch').onchange = (e) => { state.branch = e.target.value; paintAll(); };
$$('#orientBtns button').forEach((b) => (b.onclick = () => {
  state.which = b.dataset.o; $$('#orientBtns button').forEach((x) => x.classList.toggle('on', x === b)); state.probe = null; run();
}));
bindSettings(document.body, S, { onChange: () => { saveSettings(S); run(); } });

// ---------------------------------------------------------------- computation
let current = null;
async function run() {
  const o = orient(), key = keyOf(o);
  if (cache[key]?.done) { current = cache[key]; paintAll(); return; }
  pool.cancel();
  const centers = gridCenters(S), entry = { key, orient: o, centers, results: new Array(centers.length).fill(null), done: false, S: { ...S } };
  cache[key] = entry; current = entry;
  $('#progress').hidden = false;
  let got = 0, lastPaint = 0;
  const t0 = performance.now();
  // With the helper on: all contour poses of all placements go to the supplied solver in one batch,
  // every answer is verified with our FK, and the workers evaluate those candidates.
  const jobs = centers.map((xy, i) => ({ i, xy }));
  entry.source = 'browser IK';
  if (await useSupplied()) {
    try {
      const poses = centers.map((xy) => contourPoses(S, centerOf(S, xy), o).map((T) => tool0Of(S, T))).flat();
      $('#progress').textContent = `Supplied solver: ${poses.length} poses…`;
      const { sols, stats } = await solveVerified(poses, { chunk: 8000, onProgress: (d, n) => { $('#progress').textContent = `Supplied solver: ${d} / ${n} poses`; } });
      if (current !== entry) return;
      jobs.forEach((j, i) => { j.sols = sols.slice(i * S.N, (i + 1) * S.N); });
      entry.source = 'supplied'; entry.stats = stats; entry.solverSeconds = (performance.now() - t0) / 1000;
    } catch (e) { entry.source = `browser IK (supplied failed: ${e.message})`; }
  }
  const ok = await pool.run(S, o, jobs, (i, r) => {
    if (current !== entry) return;
    entry.results[i] = r; got++;
    $('#progress').textContent = `Placements ${got} / ${centers.length} · ${pool.size} worker${pool.size > 1 ? 's' : ''}`;
    if (performance.now() - lastPaint > 250) { lastPaint = performance.now(); paintMap(); }
  });
  if (!ok || current !== entry) return;
  entry.done = true; entry.seconds = (performance.now() - t0) / 1000;
  $('#progress').hidden = true;
  paintAll();
}

// ---------------------------------------------------------------- classification
const filterStarts = (r, f) => (f === 'all' ? r.starts : r.starts.filter((s) => s.b === Number(f)));
function classify(r, f = state.branch, mode = state.mode) {
  if (!r) return { cat: 'pending' };
  if (r.noik) return { cat: 'noik' };
  const st = filterStarts(r, f);
  if (!st.length) return { cat: 'absent' };
  const nN = st.filter((s) => s.nearest.ok).length, nA = st.filter((s) => s.aware.ok).length;
  if (mode === 'countN') return { cat: nN ? 'ok' : 'fail', n: nN };
  if (mode === 'countA') return { cat: nA ? 'ok' : 'fail', n: nA };
  if (mode === 'cmp') return { cat: nN ? 'both' : nA ? 'aware' : 'none' };
  if (nN) return { cat: 'ok', n: nN };
  const far = st.reduce((m, s) => (!m || s.nearest.at > m.nearest.at ? s : m), null);
  return { cat: far.nearest.reason, at: far.nearest.at };
}
const CMP_COLORS = { both: '#1f8a4c', aware: '#00a3c4', none: '#e8590c' };
const greens = (n, max) => { const t = Math.min(1, n / Math.max(max, 1)); return `rgb(${Math.round(190 - 160 * t)},${Math.round(230 - 110 * t)},${Math.round(190 - 130 * t)})`; };
function colorOf(c, maxN) {
  switch (c.cat) {
    case 'pending': return '#ffffff';
    case 'noik': return REASON_COLORS.noik;
    case 'absent': return 'absent';
    case 'ok': return greens(c.n, maxN);
    case 'fail': return '#f3d9d9';
    case 'both': case 'aware': case 'none': return CMP_COLORS[c.cat];
    default: return REASON_COLORS[c.cat];
  }
}

// ---------------------------------------------------------------- drawing
function drawGrid(canvas, entry, { f = state.branch, mode = state.mode, axes = true } = {}) {
  const r = canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio, 2);
  if (!r.width || !entry) return null;
  canvas.width = r.width * dpr; canvas.height = r.height * dpr;
  const ctx = canvas.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#fff'; ctx.fillRect(0, 0, r.width, r.height);
  const s = entry.S, n = s.gridN, phone = r.width < 600;
  const PAD = axes ? { l: phone ? 44 : 58, r: 12, t: phone ? 10 : 54, b: phone ? 40 : 96 } : { l: 2, r: 2, t: 2, b: 2 };
  const xs = s.xRange, ys = s.yRange, dx = (xs[1] - xs[0]) / (n - 1), dy = (ys[1] - ys[0]) / (n - 1);
  const uS = xs[1] - xs[0] + dx, vS = ys[1] - ys[0] + dy;
  const W0 = r.width - PAD.l - PAD.r, H0 = r.height - PAD.t - PAD.b, sc = Math.min(W0 / uS, H0 / vS);
  const W = uS * sc, H = vS * sc, x0 = PAD.l + (W0 - W) / 2, y0 = PAD.t + (H0 - H) / 2;
  const X = (x) => x0 + (x - xs[0] + dx / 2) * sc, Y = (y) => y0 + H - (y - ys[0] + dy / 2) * sc;
  const cls = entry.results.map((res) => classify(res, f, mode));
  const maxN = Math.max(1, ...cls.map((c) => c.n || 0));
  cls.forEach((c, i) => {
    const [cx, cy] = entry.centers[i], col = colorOf(c, maxN);
    const px = X(cx) - (dx * sc) / 2, py = Y(cy) - (dy * sc) / 2, w = dx * sc + 0.6, h = dy * sc + 0.6;
    if (col === 'absent') {
      ctx.fillStyle = '#f7f7f7'; ctx.fillRect(px, py, w, h);
      ctx.strokeStyle = '#d0d0d0'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(px, py + h); ctx.lineTo(px + w, py); ctx.stroke();
    } else { ctx.fillStyle = col; ctx.fillRect(px, py, w, h); }
  });
  // base axis
  ctx.strokeStyle = 'rgba(20,20,20,.6)'; ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(X(0), y0); ctx.lineTo(X(0), y0 + H); ctx.moveTo(x0, Y(0)); ctx.lineTo(x0 + W, Y(0)); ctx.stroke(); ctx.setLineDash([]);
  ctx.strokeStyle = '#141414'; ctx.strokeRect(x0, y0, W, H);
  if (axes) {
    ctx.fillStyle = '#141414'; ctx.font = `${phone ? 10 : 11}px Inter, Arial`; ctx.textAlign = 'center';
    const tick = phone ? 0.5 : 0.25;
    for (let u = Math.ceil(xs[0] / tick) * tick; u <= xs[1] + 1e-9; u += tick) ctx.fillText(u.toFixed(2).replace('-0.00', '0.00'), X(u), y0 + H + 14);
    ctx.fillText('contour centre x (m)', x0 + W / 2, y0 + H + 30);
    ctx.textAlign = 'right';
    for (let v = Math.ceil(ys[0] / tick) * tick; v <= ys[1] + 1e-9; v += tick) ctx.fillText(v.toFixed(2).replace('-0.00', '0.00'), x0 - 5, Y(v) + 4);
    ctx.save(); ctx.translate(12, y0 + H / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText('contour centre y (m)', 0, 0); ctx.restore();
    ctx.textAlign = 'left';
    ctx.font = '600 10.5px Inter, Arial'; ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,255,.9)';
    ctx.strokeText('base', X(0) + 5, Y(0) - 5); ctx.fillText('base', X(0) + 5, Y(0) - 5);
    if (state.probe !== null) {
      const [cx, cy] = entry.centers[state.probe];
      ctx.strokeStyle = '#00e0ff'; ctx.lineWidth = 2.4; ctx.strokeRect(X(cx) - (dx * sc) / 2, Y(cy) - (dy * sc) / 2, dx * sc, dy * sc);
      // contour footprint (to scale, ignoring the plane tilt)
      ctx.strokeStyle = '#141414'; ctx.lineWidth = 1; ctx.beginPath(); ctx.ellipse(X(cx), Y(cy), s.a * sc, s.b * sc, 0, 0, 2 * Math.PI); ctx.stroke();
    }
  }
  return { X, Y, x0, y0, W, H, sc, dx, dy };
}

const canvas = $('#canvas');
let geom = null;
function paintMap() { geom = drawGrid(canvas, current); legend(); }

function legend() {
  const items = [];
  if (state.mode === 'reason') {
    items.push(`<span><span class="dot" style="background:${greens(1, 2)}"></span>completes (darker = more starts)</span>`);
    ['limit', 'jump', 'margin', 'noik'].forEach((k) => items.push(`<span><span class="dot" style="background:${REASON_COLORS[k]}"></span>${REASON_TEXT[k]}</span>`));
  } else if (state.mode === 'cmp') {
    items.push(`<span><span class="dot" style="background:${CMP_COLORS.both}"></span>nearest succeeds</span><span><span class="dot" style="background:${CMP_COLORS.aware}"></span>only branch-aware succeeds</span><span><span class="dot" style="background:${CMP_COLORS.none}"></span>neither</span><span><span class="dot" style="background:${REASON_COLORS.noik}"></span>no IK</span>`);
  } else {
    items.push(`<span><span class="dot" style="background:${greens(1, 2)}"></span>number of feasible starts (darker = more)</span><span><span class="dot" style="background:#f3d9d9"></span>none feasible</span><span><span class="dot" style="background:${REASON_COLORS.noik}"></span>no IK</span>`);
  }
  if (state.branch !== 'all') items.push('<span><span class="dot" style="background:#f7f7f7;border:1px solid #ccc"></span>branch absent at s = 0</span>');
  $('#legend').innerHTML = `<span><b>${orientName(orient())}</b> · z = ${S.z.toFixed(2)} m · ${state.branch === 'all' ? 'all starts' : branchName(Number(state.branch))}</span>${items.join('')}`;
}

function summary() {
  const e = current;
  if (!e?.done) return;
  const R = e.results, n = R.length;
  const noik = R.filter((r) => r.noik).length;
  const feasN = R.filter((r) => !r.noik && r.starts.some((s) => s.nearest.ok)).length;
  const feasA = R.filter((r) => !r.noik && r.starts.some((s) => s.aware.ok)).length;
  const tally = Object.fromEntries(REASONS.map((k) => [k, 0]));
  R.forEach((r) => r.starts.forEach((s) => { tally[s.nearest.reason]++; }));
  const other = Object.values(cache).find((x) => x.done && x !== e && x.key.split('|')[1] === e.key.split('|')[1]);
  const pct = (x) => `${Math.round((100 * x) / n)}%`;
  $('#srcTiles').innerHTML = e.stats ? statsHTML(e.stats, e.solverSeconds) : `<div class="tile"><small>IK source</small><b>${e.source}</b></div>`;
  $('#sumTiles').innerHTML = [
    ['placements', n], ['no IK somewhere', pct(noik)], ['feasible · nearest', pct(feasN)], ['feasible · branch-aware', pct(feasA)],
    ...(other ? [[`other orientation (${orientName(other.orient)}) · aware`, pct(other.results.filter((r) => !r.noik && r.starts.some((s) => s.aware.ok)).length)]] : []),
    ['compute time', `${e.seconds.toFixed(1)} s`],
  ].map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
  const tot = Object.values(tally).reduce((a, b) => a + b, 0) || 1;
  $('#reasonBars').innerHTML = ['ok', 'limit', 'jump', 'margin'].map((k) => `<div class="row" style="gap:8px;margin-bottom:4px"><span style="width:170px;font-size:12px">${REASON_TEXT[k]}</span><span style="flex:1;background:#f2f2f2;height:12px;border-radius:3px;overflow:hidden"><span style="display:block;height:100%;width:${(100 * tally[k]) / tot}%;background:${REASON_COLORS[k]}"></span></span><span style="width:44px;text-align:right;font-size:12px">${tally[k]}</span></div>`).join('');
  $('#sumNote').textContent = `Counted over all start configurations of placements with IK everywhere (${tot} starts). Switch the orientation at the top to compute and compare the second one.`;
  $('#setTiles').innerHTML = [['contour', `${S.a.toFixed(2)} × ${S.b.toFixed(2)} m`], ['tilt / spin', `${S.tilt}° / ${S.spin}°`], ['height', `${S.z.toFixed(2)} m`], ['orientations', `${orientName(S.orientA)} · ${orientName(S.orientB)}`]]
    .map(([k, x]) => `<div class="tile"><small>${k}</small><b style="font-size:13px">${x}</b></div>`).join('');
}

function minimaps() {
  const box = $('#minimaps');
  if (!box.children.length) {
    box.innerHTML = BRANCHES.map((b, i) => `<div class="minimap" data-b="${i}"><canvas></canvas><small>${branchName(i)}</small></div>`).join('');
    $$('.minimap', box).forEach((m) => (m.onclick = () => { state.branch = m.dataset.b; state.mode = 'reason'; $('#branch').value = state.branch; $('#mode').value = 'reason'; paintAll(); }));
  }
  $$('.minimap', box).forEach((m) => { m.classList.toggle('on', m.dataset.b === state.branch); drawGrid($('canvas', m), current, { f: m.dataset.b, mode: 'reason', axes: false }); });
}

function inspect() {
  if (state.probe === null || !current?.results[state.probe]) { $('#insTiles').innerHTML = ''; $('#insTable').innerHTML = ''; return; }
  const r = current.results[state.probe], [x, y] = current.centers[state.probe];
  $('#insTiles').innerHTML = [['centre (m)', `${x.toFixed(2)}, ${y.toFixed(2)}, ${S.z.toFixed(2)}`], ['min #IK', r.minCount], ['starts', r.starts.length],
    ['nearest ok', r.starts.filter((s) => s.nearest.ok).length], ['aware ok', r.starts.filter((s) => s.aware.ok).length]]
    .map(([k, v]) => `<div class="tile"><small>${k}</small><b>${v}</b></div>`).join('');
  $('#insTable').innerHTML = r.noik ? `<p class="note">No IK solution at sample ${r.firstEmpty}: the contour leaves the reachable set for this orientation.</p>`
    : `<table class="starts"><thead><tr><th>#</th><th>start branch</th><th>nearest</th><th>branch-aware</th></tr></thead><tbody>${r.starts.map((s, i) => `<tr><td>${i + 1}</td><td>${branchName(s.b)}</td><td><span class="tag" style="background:${REASON_COLORS[s.nearest.reason]}">${s.nearest.ok ? `ok${s.nearest.closes ? '' : ' · ends on other q'}` : `${s.nearest.reason} @ k=${s.nearest.at}`}</span></td><td>${s.aware.ok ? (s.aware.closes ? '✓ closes' : '✓') : '—'}</td></tr>`).join('')}</tbody></table>`;
  $('#toStep6').onclick = () => { S.center = [x, y]; saveSettings(S); };
}

function paintAll() { paintMap(); summary(); minimaps(); inspect(); }

canvas.addEventListener('pointerdown', (ev) => {
  if (!geom || !current) return;
  const r = canvas.getBoundingClientRect(), px = ev.clientX - r.left, py = ev.clientY - r.top;
  let best = -1, bd = Infinity;
  current.centers.forEach(([cx, cy], i) => { const d = (geom.X(cx) - px) ** 2 + (geom.Y(cy) - py) ** 2; if (d < bd) { bd = d; best = i; } });
  if (bd > (geom.dx * geom.sc) ** 2) return;
  state.probe = best; paintMap(); inspect();
  $$('.tabs [data-tab]').find((b) => b.dataset.tab === 'ins')?.click();
});
new ResizeObserver(() => paintMap()).observe($('#heat'));
new ResizeObserver(() => { if (current) minimaps(); }).observe($('#minimaps'));
$$('.tabs').forEach((t) => t.addEventListener('tab', (e) => { if (e.detail === 'br' && current) requestAnimationFrame(minimaps); }));

window.addEventListener('ik-source', () => run());
run();
renderMath();
