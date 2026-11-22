// Extra page: the Lecture 06 atlas — det Jp over (q2, q3), critical curves in (ρ, z), IK counts, cusps.
import { mountChrome, mountTabs, renderMath, $, $$ } from '../ui.js';
import { PRESETS, buildAtlas, solveMeridian, meridian, chain, wrap } from '../atlas.js';
import { fmt } from '../math.js';

mountChrome();
mountTabs();

const IK_COLORS = ['#c62828', '#1f5fa8', '#2e7d32', '#7b3fa0', '#e8590c', '#00838f', '#6d4c41', '#ad1457'];
const COUNT_FILL = ['#ffffff', '#f6f8fc', '#eaf1fb', '#e2ecfa', '#d3e2f7', '#c8dbf5', '#b8d0f1', '#acc8ef', '#9dbeec'];
const clone = (p) => JSON.parse(JSON.stringify(p));
const state = { P: clone(PRESETS.offset), preset: 'offset', A: null, target: null, sols: [], sel: 0, regions: true, regionData: null };
const PI = Math.PI;

// ---------------------------------------------------------------- parameters
function syncInputs() {
  $$('[data-p]').forEach((inp) => { const [k, i] = inp.dataset.p.split('.'); inp.value = i === undefined ? state.P[k] : state.P[k][Number(i)]; });
  $$('#presetBtns button').forEach((b) => b.classList.toggle('on', b.dataset.preset === state.preset));
  $('#formula').innerHTML = state.preset === 'custom' ? 'custom parameters · det(Jₚ) evaluated numerically<br>independent of q₁, d₁' : `${PRESETS[state.preset].formula}<br>independent of q₁, d₁${state.P.d4 ? '' : ', α₃'}`;
}
$$('[data-p]').forEach((inp) => (inp.onchange = () => {
  const [k, i] = inp.dataset.p.split('.'), v = Number(inp.value);
  if (!Number.isFinite(v)) return;
  if (i === undefined) state.P[k] = v; else state.P[k][Number(i)] = v;
  state.preset = Object.entries(PRESETS).find(([, p]) => JSON.stringify({ ...p, name: 0, formula: 0 }) === JSON.stringify({ ...state.P, name: 0, formula: 0 }))?.[0] ?? 'custom';
  rebuild(false);
}));
$$('#presetBtns button').forEach((b) => (b.onclick = () => { state.preset = b.dataset.preset; state.P = clone(PRESETS[state.preset]); rebuild(true); }));
$('#regionsBtn').onclick = () => { state.regions = !state.regions; $('#regionsBtn').classList.toggle('on', state.regions); paintW(); };

function rebuild(resetTarget) {
  syncInputs();
  state.A = buildAtlas(state.P);
  state.regionData = null;
  const b = state.A.bounds;
  if (resetTarget || !state.target) {
    // default target: the lecture's example for the presets, otherwise a point in the middle of the reach
    state.target = state.preset === 'offset' || state.preset === 'zero' ? [1.498, 1.481] : [0.55 * b.rhoMax, (b.zMin + b.zMax) / 2];
  }
  solve();
  cusps();
}

// ---------------------------------------------------------------- IK
function solve() {
  state.sols = solveMeridian(state.A, ...state.target);
  state.sel = Math.min(state.sel, Math.max(0, state.sols.length - 1));
  const [rho, z] = state.target;
  $('#ikTiles').innerHTML = [['ρ (m)', rho.toFixed(3)], ['z (m)', z.toFixed(3)], ['regular IK solutions', state.sols.length]]
    .map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
  $('#ikChips').innerHTML = state.sols.map((s, i) => `<button class="chip ${i === state.sel ? 'on' : ''}" data-i="${i}" style="color:${IK_COLORS[i % 8]};${i === state.sel ? `background:${IK_COLORS[i % 8]}` : ''}">IK ${i + 1}</button>`).join('') || '<span class="note">No real solution: the target is outside the workspace.</span>';
  $$('#ikChips .chip').forEach((c) => (c.onclick = () => { state.sel = Number(c.dataset.i); solve(); }));
  const deg = (x) => ((x * 180) / PI).toFixed(1);
  $('#ikTable').innerHTML = state.sols.length ? `<table class="starts"><thead><tr><th>IK</th><th>q₁ (°)</th><th>q₂ (°)</th><th>q₃ (°)</th><th>det Jₚ</th><th>aspect</th></tr></thead><tbody>${
    state.sols.map((s, i) => `<tr><td><span class="dot" style="background:${IK_COLORS[i % 8]}"></span>${i + 1}</td><td>${deg(s.q1)}</td><td>${deg(s.q2)}</td><td>${deg(s.q3)}</td><td>${fmt(s.det, 3)}</td><td>D ${s.det > 0 ? '> 0' : '< 0'}</td></tr>`).join('')
  }</tbody></table>` : '';
  stick();
  paintQ(); paintW();
  legend();
}

function cusps() {
  const C = state.A.cusps, deg = (x) => ((x * 180) / PI).toFixed(2);
  $('#cuspList').innerHTML = C.length ? `<p style="margin-bottom:8px"><b>${C.length} cusps · cuspidal</b></p><table class="starts"><thead><tr><th>q₂ (°)</th><th>q₃ (°)</th><th>ρ (m)</th><th>z (m)</th></tr></thead><tbody>${
    C.map((c) => `<tr><td>${deg(c.q[0])}</td><td>${deg(c.q[1])}</td><td>${c.image[0].toFixed(4)}</td><td>${c.image[1].toFixed(4)}</td></tr>`).join('')
  }</tbody></table>` : '<p><b>No cusps found · noncuspidal</b></p>';
}

// side view of the selected IK (point placed on the +x meridian, y shown as depth)
function stick() {
  const cv = $('#stick'), dpr = Math.min(devicePixelRatio, 2), W = cv.clientWidth || 320, H = 220;
  cv.style.height = `${H}px`; cv.width = W * dpr; cv.height = H * dpr; const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H);
  const s = state.sols[state.sel];
  if (!s) return;
  const { o, p } = chain(state.P, [s.q1, s.q2, s.q3]);
  const pts = [...o, p];
  const xs = pts.map((q) => q[0]).concat(0), zs = pts.map((q) => q[2]).concat(0);
  const xm = Math.min(...xs), xM = Math.max(...xs), zm = Math.min(...zs), zM = Math.max(...zs);
  const sc = Math.min((W - 40) / Math.max(xM - xm, 1e-3), (H - 40) / Math.max(zM - zm, 1e-3));
  const cx = W / 2 - ((xm + xM) / 2) * sc, cz = H / 2 + ((zm + zM) / 2) * sc + 6;
  const X = (q) => cx + q[0] * sc, Y = (q) => cz - q[2] * sc;
  ctx.strokeStyle = '#ddd'; ctx.beginPath(); ctx.moveTo(0, cz); ctx.lineTo(W, cz); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke();
  ctx.fillStyle = '#555'; ctx.font = '10.5px Inter, Arial'; ctx.fillText(`side view (x, z) · IK ${state.sel + 1} · base at the cross`, 6, 13);
  ctx.strokeStyle = IK_COLORS[state.sel % 8]; ctx.lineWidth = 4; ctx.lineCap = 'round'; ctx.beginPath();
  pts.forEach((q, i) => (i ? ctx.lineTo(X(q), Y(q)) : ctx.moveTo(X(q), Y(q)))); ctx.stroke();
  pts.forEach((q, i) => { ctx.beginPath(); ctx.arc(X(q), Y(q), i === pts.length - 1 ? 5 : 3.5, 0, 2 * PI); ctx.fillStyle = i === pts.length - 1 ? '#141414' : '#fff'; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = '#141414'; ctx.stroke(); });
}

// ---------------------------------------------------------------- drawing helpers
function setup(canvas) {
  const r = canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio, 2);
  canvas.width = Math.max(1, r.width * dpr); canvas.height = Math.max(1, r.height * dpr);
  const ctx = canvas.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#fff'; ctx.fillRect(0, 0, r.width, r.height);
  return { ctx, W: r.width, H: r.height };
}
const diverging = (t) => { // t in [-1, 1]: blue → white → red
  const a = Math.min(1, Math.abs(t)) ** 0.8;
  return t >= 0 ? [255 - a * (255 - 200), 255 - a * (255 - 70), 255 - a * (255 - 60)] : [255 - a * (255 - 45), 255 - a * (255 - 100), 255 - a * (255 - 170)];
};
function marker(ctx, x, y, i, sel) {
  ctx.beginPath(); ctx.arc(x, y, sel ? 9 : 7.5, 0, 2 * PI); ctx.fillStyle = IK_COLORS[i % 8]; ctx.fill();
  ctx.lineWidth = sel ? 2.5 : 1.5; ctx.strokeStyle = '#fff'; ctx.stroke();
  ctx.fillStyle = '#fff'; ctx.font = '700 9.5px Inter, Arial'; ctx.textAlign = 'center'; ctx.fillText(String(i + 1), x, y + 3.3); ctx.textAlign = 'left';
}
function diamond(ctx, x, y, r = 5) { ctx.beginPath(); ctx.moveTo(x, y - r); ctx.lineTo(x + r, y); ctx.lineTo(x, y + r); ctx.lineTo(x - r, y); ctx.closePath(); ctx.fillStyle = '#ff0000'; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1; ctx.stroke(); }

// ---------------------------------------------------------------- joint space
let gq = null;
function paintQ() {
  const cv = $('#qc'), { ctx, W, H } = setup(cv), A = state.A;
  if (!A) return;
  const phone = W < 420, PAD = { l: phone ? 36 : 44, r: phone ? 34 : 52, t: 10, b: phone ? 34 : 40 };
  const S = Math.min(W - PAD.l - PAD.r, H - PAD.t - PAD.b), x0 = PAD.l + (W - PAD.l - PAD.r - S) / 2, y0 = PAD.t + (H - PAD.t - PAD.b - S) / 2;
  const X = (q2) => x0 + ((q2 + PI) / (2 * PI)) * S, Y = (q3) => y0 + S - ((q3 + PI) / (2 * PI)) * S;
  gq = { X, Y, x0, y0, S };
  const n = A.n, img = ctx.createImageData(n, n);
  for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
    const c = diverging(A.D[j * n + i] / A.dMax), k = ((n - 1 - j) * n + i) * 4;
    img.data[k] = c[0]; img.data[k + 1] = c[1]; img.data[k + 2] = c[2]; img.data[k + 3] = 255;
  }
  const off = document.createElement('canvas'); off.width = n; off.height = n; off.getContext('2d').putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = true; ctx.drawImage(off, x0, y0, S, S);
  ctx.strokeStyle = '#141414'; ctx.lineWidth = 1.4;
  for (const c of A.curves) {
    ctx.beginPath();
    c.forEach(([a, b], i) => { if (i && (Math.abs(a - c[i - 1][0]) > 1 || Math.abs(b - c[i - 1][1]) > 1)) ctx.moveTo(X(a), Y(b)); else if (i) ctx.lineTo(X(a), Y(b)); else ctx.moveTo(X(a), Y(b)); });
    ctx.stroke();
  }
  A.cusps.forEach((c) => diamond(ctx, X(c.q[0]), Y(c.q[1]), 4.5));
  state.sols.forEach((s, i) => marker(ctx, X(s.q2), Y(s.q3), i, i === state.sel));
  ctx.strokeStyle = '#141414'; ctx.lineWidth = 1; ctx.strokeRect(x0, y0, S, S);
  ctx.fillStyle = '#141414'; ctx.font = `${phone ? 9.5 : 10.5}px Inter, Arial`;
  const labels = [[-PI, '−π'], [-PI / 2, '−π/2'], [0, '0'], [PI / 2, 'π/2'], [PI, 'π']];
  ctx.textAlign = 'center'; labels.forEach(([v, t]) => ctx.fillText(t, X(v), y0 + S + 13)); ctx.fillText('q₂ [rad]', x0 + S / 2, y0 + S + 27);
  ctx.textAlign = 'right'; labels.forEach(([v, t]) => ctx.fillText(t, x0 - 4, Y(v) + 3));
  ctx.save(); ctx.translate(10, y0 + S / 2); ctx.rotate(-PI / 2); ctx.textAlign = 'center'; ctx.fillText('q₃ [rad]', 0, 0); ctx.restore();
  // colour bar
  const bx = x0 + S + (phone ? 8 : 12), bw = 8;
  for (let k = 0; k < S; k++) { const c = diverging(1 - (2 * k) / S); ctx.fillStyle = `rgb(${c.map(Math.round).join(',')})`; ctx.fillRect(bx, y0 + k, bw, 1.5); }
  ctx.textAlign = 'left'; ctx.fillStyle = '#141414'; ctx.font = '9.5px Inter, Arial';
  const m = fmt(A.dMax, 2);
  ctx.fillText(`+${m}`, bx - 2, y0 - 2 < 8 ? 8 : y0 - 2); ctx.fillText('0', bx + bw + 3, y0 + S / 2 + 3); ctx.fillText(`−${m}`, bx - 2, y0 + S + 10);
}

// ---------------------------------------------------------------- workspace section
let gw = null;
function wFrame(W, H) {
  const b = state.A.bounds, phone = W < 420, PAD = { l: phone ? 36 : 44, r: 10, t: 10, b: phone ? 34 : 40 };
  const r0 = 0, r1 = b.rhoMax * 1.06, z0 = b.zMin - 0.06 * (b.zMax - b.zMin), z1 = b.zMax + 0.06 * (b.zMax - b.zMin);
  const sc = Math.min((W - PAD.l - PAD.r) / (r1 - r0), (H - PAD.t - PAD.b) / (z1 - z0));
  const w = (r1 - r0) * sc, h = (z1 - z0) * sc, x0 = PAD.l + (W - PAD.l - PAD.r - w) / 2, y0 = PAD.t + (H - PAD.t - PAD.b - h) / 2;
  return { r0, r1, z0, z1, sc, w, h, x0, y0, X: (r) => x0 + (r - r0) * sc, Y: (z) => y0 + h - (z - z0) * sc };
}
function drawCurves(ctx, F) {
  for (const c of state.A.images) {
    ctx.beginPath();
    c.forEach(([r, z], i) => { const jump = i && Math.hypot(r - c[i - 1][0], z - c[i - 1][1]) > 0.25 * (F.r1 - F.r0); if (!i || jump) ctx.moveTo(F.X(r), F.Y(z)); else ctx.lineTo(F.X(r), F.Y(z)); });
    ctx.stroke();
  }
}
// Regions between critical curves: rasterise the curves, flood-fill, solve IK once per region
function computeRegions(F) {
  const RW = Math.max(60, Math.round(F.w / 2)), RH = Math.max(60, Math.round(F.h / 2));
  const off = document.createElement('canvas'); off.width = RW; off.height = RH;
  const c = off.getContext('2d'), k = RW / F.w;
  c.strokeStyle = '#000'; c.lineWidth = 1.6;
  drawCurves(c, { ...F, X: (r) => (r - F.r0) * F.sc * k, Y: (z) => (F.h - (z - F.z0) * F.sc) * k });
  const wall = c.getImageData(0, 0, RW, RH).data, lab = new Int32Array(RW * RH).fill(-1), regions = [];
  for (let s = 0; s < RW * RH; s++) {
    if (lab[s] >= 0 || wall[4 * s + 3] > 40) continue;
    const id = regions.length, stack = [s], px = []; lab[s] = id;
    while (stack.length) {
      const t = stack.pop(); px.push(t);
      const x = t % RW, y = (t / RW) | 0;
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const nx = x + dx, ny = y + dy; if (nx < 0 || ny < 0 || nx >= RW || ny >= RH) continue;
        const u = ny * RW + nx; if (lab[u] < 0 && wall[4 * u + 3] <= 40) { lab[u] = id; stack.push(u); }
      }
    }
    regions.push({ px });
  }
  const rz = (x, y) => [F.r0 + x / k / F.sc, F.z0 + (F.h - y / k) / F.sc];
  regions.forEach((R) => {
    if (R.px.length < 6) { R.count = null; return; }
    // representative pixel: deepest inside (largest clear square around it), near the centroid
    let cx = 0, cy = 0; R.px.forEach((t) => { cx += t % RW; cy += (t / RW) | 0; }); cx /= R.px.length; cy /= R.px.length;
    let best = R.px[0], bs = -1;
    for (const t of R.px) {
      const x = t % RW, y = (t / RW) | 0;
      let d = 0; while (d < 8) { const r = d + 1; if ([[x + r, y], [x - r, y], [x, y + r], [x, y - r], [x + r, y + r], [x - r, y - r], [x + r, y - r], [x - r, y + r]].every(([a, b]) => a >= 0 && b >= 0 && a < RW && b < RH && lab[b * RW + a] === lab[t])) d++; else break; }
      const score = d * 1000 - Math.hypot(x - cx, y - cy);
      if (score > bs) { bs = score; best = t; }
    }
    const [r, z] = rz(best % RW, (best / RW) | 0);
    R.count = solveMeridian(state.A, r, z).length;
    R.at = [r, z];
  });
  return { RW, RH, lab, regions, k };
}
function paintW() {
  const cv = $('#wc'), { ctx, W, H } = setup(cv);
  if (!state.A) return;
  const F = wFrame(W, H); gw = F;
  if (state.regions) {
    const key = `${Math.round(F.w)}x${Math.round(F.h)}`;
    if (!state.regionData || state.regionData.key !== key) state.regionData = { key, ...computeRegions(F) };
    const R = state.regionData, img = ctx.createImageData(R.RW, R.RH);
    for (let s = 0; s < R.RW * R.RH; s++) {
      const reg = R.regions[R.lab[s]], n = reg?.count;
      const col = n === null || n === undefined ? '#ffffff' : COUNT_FILL[Math.min(n, 8)];
      const v = [1, 3, 5].map((i) => parseInt(col.slice(i, i + 2), 16));
      img.data.set([...v, 255], 4 * s);
    }
    const off = document.createElement('canvas'); off.width = R.RW; off.height = R.RH; off.getContext('2d').putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false; ctx.drawImage(off, F.x0, F.y0, F.w, F.h);
  } else { ctx.fillStyle = '#f7f9fc'; ctx.fillRect(F.x0, F.y0, F.w, F.h); }
  // grid
  ctx.strokeStyle = 'rgba(0,0,0,.07)'; ctx.lineWidth = 1;
  const st = niceStep((F.r1 - F.r0) / 5);
  for (let r = 0; r <= F.r1; r += st) { ctx.beginPath(); ctx.moveTo(F.X(r), F.y0); ctx.lineTo(F.X(r), F.y0 + F.h); ctx.stroke(); }
  for (let z = Math.ceil(F.z0 / st) * st; z <= F.z1; z += st) { ctx.beginPath(); ctx.moveTo(F.x0, F.Y(z)); ctx.lineTo(F.x0 + F.w, F.Y(z)); ctx.stroke(); }
  ctx.strokeStyle = '#141414'; ctx.lineWidth = 1.4; drawCurves(ctx, F);
  if (state.regions) {
    ctx.font = '600 11px Inter, Arial'; ctx.textAlign = 'center';
    state.regionData.regions.forEach((R) => {
      if (R.count === null || R.count === undefined || R.px.length < 120) return;
      ctx.fillStyle = R.count ? '#1f4f8f' : '#9a9a9a'; ctx.fillText(String(R.count), F.X(R.at[0]), F.Y(R.at[1]) + 4);
    });
    ctx.textAlign = 'left';
  }
  state.A.cusps.forEach((c) => diamond(ctx, F.X(c.image[0]), F.Y(c.image[1])));
  const [tr, tz] = state.target;
  ctx.setLineDash([3, 3]); ctx.strokeStyle = 'rgba(20,20,20,.45)';
  ctx.beginPath(); ctx.moveTo(F.x0, F.Y(tz)); ctx.lineTo(F.x0 + F.w, F.Y(tz)); ctx.moveTo(F.X(tr), F.y0); ctx.lineTo(F.X(tr), F.y0 + F.h); ctx.stroke(); ctx.setLineDash([]);
  ctx.beginPath(); ctx.arc(F.X(tr), F.Y(tz), 7, 0, 2 * PI); ctx.lineWidth = 2; ctx.strokeStyle = '#141414'; ctx.stroke();
  ctx.beginPath(); ctx.arc(F.X(tr), F.Y(tz), 2.5, 0, 2 * PI); ctx.fillStyle = '#141414'; ctx.fill();
  ctx.lineWidth = 1; ctx.strokeRect(F.x0, F.y0, F.w, F.h);
  ctx.fillStyle = '#141414'; ctx.font = '10.5px Inter, Arial'; ctx.textAlign = 'center';
  for (let r = 0; r <= F.r1 + 1e-9; r += st) ctx.fillText(+r.toFixed(3) + '', F.X(r), F.y0 + F.h + 13);
  ctx.fillText('ρ [m]', F.x0 + F.w / 2, F.y0 + F.h + 27);
  ctx.textAlign = 'right';
  for (let z = Math.ceil(F.z0 / st) * st; z <= F.z1 + 1e-9; z += st) ctx.fillText(+z.toFixed(3) + '', F.x0 - 4, F.Y(z) + 3);
  ctx.save(); ctx.translate(10, F.y0 + F.h / 2); ctx.rotate(-PI / 2); ctx.textAlign = 'center'; ctx.fillText('z [m]', 0, 0); ctx.restore(); ctx.textAlign = 'left';
}
function niceStep(raw) { const p = 10 ** Math.floor(Math.log10(raw)), m = raw / p; return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p; }

function legend() {
  const [r, z] = state.target;
  $('#legend').innerHTML = `<span>— singular configurations → critical values</span><span><span class="dot" style="background:#ff0000"></span>cusp</span><span>◎ target · ρ = ${r.toFixed(3)} m · z = ${z.toFixed(3)} m</span><span><b>${state.sols.length} regular IK solutions</b></span>${state.regions ? '<span>numbers: IK count per region</span>' : ''}`;
}

// ---------------------------------------------------------------- interaction
$('#qc').addEventListener('pointerdown', (e) => {
  if (!gq) return;
  const r = e.target.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  // tap on an IK marker selects it
  const hit = state.sols.findIndex((s) => Math.hypot(gq.X(s.q2) - x, gq.Y(s.q3) - y) < 10);
  if (hit >= 0) { state.sel = hit; solve(); return; }
  if (x < gq.x0 || y < gq.y0 || x > gq.x0 + gq.S || y > gq.y0 + gq.S) return;
  const q2 = ((x - gq.x0) / gq.S) * 2 * PI - PI, q3 = ((gq.y0 + gq.S - y) / gq.S) * 2 * PI - PI;
  state.target = meridian(state.P, q2, q3);
  solve();
  const near = state.sols.findIndex((s) => Math.abs(wrap(s.q2 - q2)) < 1e-3 && Math.abs(wrap(s.q3 - q3)) < 1e-3);
  if (near >= 0) { state.sel = near; solve(); }
});
$('#wc').addEventListener('pointerdown', (e) => {
  if (!gw) return;
  const r = e.target.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  if (x < gw.x0 || y < gw.y0 || x > gw.x0 + gw.w || y > gw.y0 + gw.h) return;
  state.target = [gw.r0 + (x - gw.x0) / gw.sc, gw.z0 + (gw.y0 + gw.h - y) / gw.sc];
  state.sel = 0; solve();
});
new ResizeObserver(() => { if (state.A) { paintQ(); paintW(); } }).observe($('#twin'));
$$('.tabs').forEach((t) => t.addEventListener('tab', (e) => { if (e.detail === 'ik') requestAnimationFrame(stick); }));

rebuild(true);
renderMath();
