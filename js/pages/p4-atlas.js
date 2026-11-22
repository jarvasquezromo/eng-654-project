// Step 4 · Topology atlas: the Lecture 06 picture for GoFa (tool axis vertical, two wrist sheets).
import { mountChrome, mountTabs, renderMath, $, $$ } from '../ui.js';
import { RobotViewer } from '../viewer.js';
import { buildGofaAtlas, solveTarget, sheetConfig, meridianOf, wrap, Q3_FORBIDDEN, APPROACH } from '../gofa-atlas.js';
import { fmt } from '../math.js';

mountChrome();
mountTabs();

const PI = Math.PI;
const IK_COLORS = ['#c62828', '#1f5fa8', '#2e7d32', '#7b3fa0', '#e8590c', '#00838f', '#6d4c41', '#ad1457', '#5c6bc0', '#9e9d24', '#d81b60', '#00897b', '#8d6e63', '#3949ab', '#f4511e', '#546e7a'];
const SHEET_COLORS = { A: '#141414', B: '#1f5fa8', U: '#1f8a4c', D: '#e8590c' };
const COUNT_FILL = ['#ffffff', '#f6f8fc', '#eef3fb', '#e6eefa', '#dde8f8', '#d3e2f7', '#c9dbf5', '#bfd4f3', '#b5cdf1', '#abc6ef', '#a1bfed', '#97b8eb', '#8db1e9', '#83aae7', '#79a3e5', '#6f9ce3', '#6595e1'];
const state = { ap: 'down', A: null, target: null, sols: [], sel: 0, limits: false, regions: true, regionData: null };
const deg = (x) => ((x * 180) / PI).toFixed(1);
const shown = () => (state.limits ? state.sols.filter((s) => s.within) : state.sols);

// ---------------------------------------------------------------- 3D robot
const viewer = new RobotViewer($('#robot3d'));
viewer.camera.position.set(1.9, -1.9, 1.3);
let robotReady = false;
viewer.load().then(() => { robotReady = true; viewer.setOpacity(1); showRobot(); });
function showRobot() {
  const s = shown()[state.sel];
  viewer.clearOverlay();
  if (state.target) viewer.addPoint([state.target[0], 0, state.target[1]], { color: 0xff0000, radius: 0.015 });
  if (!robotReady) return;
  viewer.setRobotVisible(!!s);
  if (s) viewer.setQ(s.q);
  $('#robotCap').textContent = s ? `IK ${state.sel + 1} · sheet ${s.id} · ${s.within ? 'within limits' : 'violates q₃ limit'}` : 'no solution';
}

// ---------------------------------------------------------------- build and solve
function rebuild() {
  state.A = buildGofaAtlas(state.ap);
  state.regionData = null;
  if (!state.target) state.target = [0.55, state.ap === 'down' ? 0.35 : 0.6];
  solve();
}
function solve() {
  state.sols = solveTarget(state.ap, ...state.target).sort((a, b) => a.id.localeCompare(b.id) || a.a - b.a);
  const list = shown();
  state.sel = Math.min(state.sel, Math.max(0, list.length - 1));
  const [rho, z] = state.target;
  $('#ikTiles').innerHTML = [['ρ (m)', rho.toFixed(3)], ['z (m)', z.toFixed(3)], ['IK solutions', state.limits ? `${list.length} in limits / ${state.sols.length}` : state.sols.length]]
    .map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
  $('#ikChips').innerHTML = list.map((s, i) => `<button class="chip ${i === state.sel ? 'on' : ''}" data-i="${i}" style="color:${IK_COLORS[i % 16]};${i === state.sel ? `background:${IK_COLORS[i % 16]}` : ''}">IK ${i + 1}</button>`).join('') || '<span class="note">No solution: the target is outside the workspace for a vertical tool.</span>';
  $$('#ikChips .chip').forEach((c) => (c.onclick = () => { state.sel = Number(c.dataset.i); solve(); }));
  $('#ikTable').innerHTML = list.length ? `<table class="starts"><thead><tr><th>IK</th><th>sheet</th>${[1, 2, 3, 4, 5, 6].map((i) => `<th>q${'₁₂₃₄₅₆'[i - 1]}</th>`).join('')}<th>det J</th></tr></thead><tbody>${
    list.map((s, i) => `<tr class="${i === state.sel ? 'sel' : ''} ${s.within ? '' : 'out'}"><td><span class="dot" style="background:${IK_COLORS[i % 16]}"></span>${i + 1}</td><td><span class="dot" style="background:${SHEET_COLORS[s.id]}"></span>${s.id}</td>${s.q.map((x) => `<td>${deg(x)}</td>`).join('')}<td>${fmt(s.det, 3)}</td></tr>`).join('')
  }</tbody></table>` : '';
  paintAll(); showRobot(); legend();
}

// ---------------------------------------------------------------- drawing helpers
function setup(canvas) {
  const r = canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio, 2);
  canvas.width = Math.max(1, r.width * dpr); canvas.height = Math.max(1, r.height * dpr);
  const ctx = canvas.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#fff'; ctx.fillRect(0, 0, r.width, r.height);
  return { ctx, W: r.width, H: r.height };
}
const diverging = (t) => {
  const a = Math.min(1, Math.abs(t)) ** 0.6;
  return t >= 0 ? [255 - a * 55, 255 - a * 185, 255 - a * 195] : [255 - a * 210, 255 - a * 155, 255 - a * 85];
};
function marker(ctx, x, y, i, sel, faded) {
  ctx.globalAlpha = faded ? 0.45 : 1;
  ctx.beginPath(); ctx.arc(x, y, sel ? 9 : 7.5, 0, 2 * PI); ctx.fillStyle = IK_COLORS[i % 16]; ctx.fill();
  ctx.lineWidth = sel ? 2.5 : 1.5; ctx.strokeStyle = '#fff'; ctx.stroke();
  ctx.fillStyle = '#fff'; ctx.font = '700 9.5px Inter, Arial'; ctx.textAlign = 'center'; ctx.fillText(String(i + 1), x, y + 3.3); ctx.textAlign = 'left';
  ctx.globalAlpha = 1;
}
function diamond(ctx, x, y, r = 5) { ctx.beginPath(); ctx.moveTo(x, y - r); ctx.lineTo(x + r, y); ctx.lineTo(x, y + r); ctx.lineTo(x - r, y); ctx.closePath(); ctx.fillStyle = '#ff0000'; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1; ctx.stroke(); }
function niceStep(raw) { const p = 10 ** Math.floor(Math.log10(raw)), m = raw / p; return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p; }

// ---------------------------------------------------------------- joint space (one canvas per sheet)
const geo = {};
function paintSheet(id, sheet) {
  const cv = $(id), { ctx, W, H } = setup(cv), A = state.A;
  const PAD = { l: 34, r: 24, t: 16, b: 28 };
  const S = Math.min(W - PAD.l - PAD.r, H - PAD.t - PAD.b), x0 = PAD.l + (W - PAD.l - PAD.r - S) / 2, y0 = PAD.t + (H - PAD.t - PAD.b - S) / 2;
  const X = (q2) => x0 + ((q2 + PI) / (2 * PI)) * S, Y = (q3) => y0 + S - ((q3 + PI) / (2 * PI)) * S;
  geo[id] = { X, Y, x0, y0, S, sheet };
  const n = A.n, img = ctx.createImageData(n, n), dMax = Math.max(...A.sheets.map((s) => s.dMax));
  for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
    const d = sheet.D[j * n + i], c = diverging(d / dMax), k = ((n - 1 - j) * n + i) * 4;
    img.data[k] = c[0]; img.data[k + 1] = c[1]; img.data[k + 2] = c[2]; img.data[k + 3] = 255;
  }
  const off = document.createElement('canvas'); off.width = n; off.height = n; off.getContext('2d').putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = true; ctx.drawImage(off, x0, y0, S, S);
  if (state.limits) { // forbidden q3 band, hatched: horizontal on A/B, vertical (in q2 = s − q3) on U/D
    const rects = [];
    if (sheet.id === 'A' || sheet.id === 'B') rects.push([x0, Y(Q3_FORBIDDEN[1]), S, Y(Q3_FORBIDDEN[0]) - Y(Q3_FORBIDDEN[1])]);
    else {
      const sv = sheet.id === 'U' ? -PI / 2 : PI / 2, lo = sv - Q3_FORBIDDEN[1], hi = sv - Q3_FORBIDDEN[0];
      for (const k of [-1, 0, 1]) { const a = Math.max(-PI, lo + 2 * PI * k), b = Math.min(PI, hi + 2 * PI * k); if (b > a) rects.push([X(a), y0, X(b) - X(a), S]); }
    }
    for (const [rx, ry, rw, rh] of rects) {
      ctx.save(); ctx.beginPath(); ctx.rect(rx, ry, rw, rh); ctx.clip();
      ctx.fillStyle = 'rgba(255,255,255,.65)'; ctx.fillRect(rx, ry, rw, rh);
      ctx.strokeStyle = 'rgba(0,0,0,.35)'; ctx.lineWidth = 1;
      for (let x = rx - rh; x < rx + rw; x += 7) { ctx.beginPath(); ctx.moveTo(x, ry + rh); ctx.lineTo(x + rh, ry); ctx.stroke(); }
      ctx.restore();
    }
  }
  ctx.strokeStyle = '#141414'; ctx.lineWidth = 1.3;
  for (const c of sheet.curves) {
    ctx.beginPath();
    c.forEach(([a, b], i) => { if (!i || Math.abs(a - c[i - 1][0]) > 0.5 || Math.abs(b - c[i - 1][1]) > 0.5) ctx.moveTo(X(a), Y(b)); else ctx.lineTo(X(a), Y(b)); });
    ctx.stroke();
  }
  sheet.cusps.forEach((c) => diamond(ctx, X(c.q[0]), Y(c.q[1]), 4.5));
  shown().forEach((s, i) => { if (s.id === sheet.id) marker(ctx, X(s.a), Y(s.b), i, i === state.sel, false); });
  ctx.strokeStyle = SHEET_COLORS[sheet.id]; ctx.lineWidth = 2; ctx.strokeRect(x0, y0, S, S);
  ctx.fillStyle = SHEET_COLORS[sheet.id]; ctx.font = '600 10.5px Inter, Arial';
  ctx.fillText(`${sheet.id} · ${sheet.name}`, x0, y0 - 5);
  ctx.font = '9px Inter, Arial';
  const labels = [[-PI, '−π'], [0, '0'], [PI, 'π']];
  ctx.textAlign = 'center'; labels.forEach(([v, t]) => ctx.fillText(t, X(v), y0 + S + 12)); ctx.fillText('q₂', x0 + S / 2, y0 + S + 24);
  ctx.textAlign = 'right'; labels.forEach(([v, t]) => ctx.fillText(t, x0 - 4, Y(v) + 3));
  ctx.save(); ctx.translate(10, y0 + S / 2); ctx.rotate(-PI / 2); ctx.textAlign = 'center'; ctx.fillText(sheet.yName, 0, 0); ctx.restore();
  const bx = x0 + S + 5, bw = 5;
  for (let k = 0; k < S; k++) { const c = diverging(1 - (2 * k) / S); ctx.fillStyle = `rgb(${c.map(Math.round).join(',')})`; ctx.fillRect(bx, y0 + k, bw, 1.5); }
  ctx.textAlign = 'left'; ctx.fillStyle = '#141414'; ctx.font = '9px Inter, Arial';
  ctx.fillText('+', bx + bw + 1, y0 + 7); ctx.fillText('−', bx + bw + 1, y0 + S);
}

// ---------------------------------------------------------------- workspace section
let gw = null;
function wFrame(W, H) {
  const b = state.A.bounds, phone = W < 420, PAD = { l: phone ? 38 : 46, r: 10, t: 12, b: phone ? 34 : 40 };
  const r0 = 0, r1 = b.rhoMax * 1.06, dz = b.zMax - b.zMin, z0 = b.zMin - 0.05 * dz, z1 = b.zMax + 0.05 * dz;
  const sc = Math.min((W - PAD.l - PAD.r) / (r1 - r0), (H - PAD.t - PAD.b) / (z1 - z0));
  const w = (r1 - r0) * sc, h = (z1 - z0) * sc, x0 = PAD.l + (W - PAD.l - PAD.r - w) / 2, y0 = PAD.t + (H - PAD.t - PAD.b - h) / 2;
  return { r0, r1, z0, z1, sc, w, h, x0, y0, X: (r) => x0 + (r - r0) * sc, Y: (z) => y0 + h - (z - z0) * sc };
}
function drawCurves(ctx, F, colored = true) {
  for (const sh of state.A.sheets) {
    ctx.strokeStyle = colored ? SHEET_COLORS[sh.id] : '#000';
    sh.images.forEach((c, ci) => {
      const qc = sh.curves[ci];
      ctx.beginPath();
      c.forEach(([r, z], i) => {
        const jump = i && (Math.abs(qc[i][0] - qc[i - 1][0]) > 0.5 || Math.abs(qc[i][1] - qc[i - 1][1]) > 0.5 || Math.hypot(r - c[i - 1][0], z - c[i - 1][1]) > 0.15);
        if (!i || jump) ctx.moveTo(F.X(r), F.Y(z)); else ctx.lineTo(F.X(r), F.Y(z));
      });
      ctx.stroke();
    });
  }
}
function computeRegions(F) {
  const RW = Math.max(60, Math.round(F.w / 2)), RH = Math.max(60, Math.round(F.h / 2)), k = RW / F.w;
  const off = document.createElement('canvas'); off.width = RW; off.height = RH;
  const c = off.getContext('2d'); c.lineWidth = 1.6;
  drawCurves(c, { ...F, X: (r) => (r - F.r0) * F.sc * k, Y: (z) => (F.h - (z - F.z0) * F.sc) * k }, false);
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
  regions.forEach((R) => {
    if (R.px.length < 6) { R.all = null; return; }
    let cx = 0, cy = 0; R.px.forEach((t) => { cx += t % RW; cy += (t / RW) | 0; }); cx /= R.px.length; cy /= R.px.length;
    let best = R.px[0], bs = -Infinity;
    for (const t of R.px) {
      const x = t % RW, y = (t / RW) | 0;
      let d = 0; while (d < 8) { const r = d + 1; if ([[x + r, y], [x - r, y], [x, y + r], [x, y - r], [x + r, y + r], [x - r, y - r], [x + r, y - r], [x - r, y + r]].every(([a, b]) => a >= 0 && b >= 0 && a < RW && b < RH && lab[b * RW + a] === lab[t])) d++; else break; }
      const score = d * 1000 - Math.hypot(x - cx, y - cy);
      if (score > bs) { bs = score; best = t; }
    }
    const r = F.r0 + (best % RW) / k / F.sc, z = F.z0 + (F.h - ((best / RW) | 0) / k) / F.sc;
    const sols = solveTarget(state.ap, r, z);
    R.all = sols.length; R.within = sols.filter((s) => s.within).length; R.at = [r, z];
  });
  return { RW, RH, lab, regions };
}
function paintW() {
  const cv = $('#wc'), { ctx, W, H } = setup(cv), F = wFrame(W, H); gw = F;
  const cnt = (R) => (R.all === null || R.all === undefined ? null : state.limits ? R.within : R.all);
  if (state.regions) {
    const key = `${state.ap}:${Math.round(F.w)}x${Math.round(F.h)}`;
    if (!state.regionData || state.regionData.key !== key) state.regionData = { key, ...computeRegions(F) };
    const R = state.regionData, img = ctx.createImageData(R.RW, R.RH);
    for (let s = 0; s < R.RW * R.RH; s++) {
      const n = cnt(R.regions[R.lab[s]] || { all: null });
      const col = n === null ? '#ffffff' : COUNT_FILL[Math.min(n, 16)];
      img.data.set([parseInt(col.slice(1, 3), 16), parseInt(col.slice(3, 5), 16), parseInt(col.slice(5, 7), 16), 255], 4 * s);
    }
    const off = document.createElement('canvas'); off.width = R.RW; off.height = R.RH; off.getContext('2d').putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false; ctx.drawImage(off, F.x0, F.y0, F.w, F.h);
  } else { ctx.fillStyle = '#f7f9fc'; ctx.fillRect(F.x0, F.y0, F.w, F.h); }
  ctx.strokeStyle = 'rgba(0,0,0,.07)'; ctx.lineWidth = 1;
  const st = niceStep((F.r1 - F.r0) / 5);
  for (let r = 0; r <= F.r1; r += st) { ctx.beginPath(); ctx.moveTo(F.X(r), F.y0); ctx.lineTo(F.X(r), F.y0 + F.h); ctx.stroke(); }
  for (let z = Math.ceil(F.z0 / st) * st; z <= F.z1; z += st) { ctx.beginPath(); ctx.moveTo(F.x0, F.Y(z)); ctx.lineTo(F.x0 + F.w, F.Y(z)); ctx.stroke(); }
  ctx.lineWidth = 1.3; drawCurves(ctx, F, true);
  if (state.regions) {
    ctx.font = '600 11px Inter, Arial'; ctx.textAlign = 'center';
    state.regionData.regions.forEach((R) => {
      const n = cnt(R); if (n === null || R.px.length < 90) return;
      ctx.fillStyle = n ? '#1f3f73' : '#9a9a9a'; ctx.fillText(String(n), F.X(R.at[0]), F.Y(R.at[1]) + 4);
    });
    ctx.textAlign = 'left';
  }
  state.A.sheets.forEach((sh) => sh.cusps.forEach((c) => diamond(ctx, F.X(c.image[0]), F.Y(c.image[1]))));
  const [tr, tz] = state.target;
  ctx.setLineDash([3, 3]); ctx.strokeStyle = 'rgba(20,20,20,.45)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(F.x0, F.Y(tz)); ctx.lineTo(F.x0 + F.w, F.Y(tz)); ctx.moveTo(F.X(tr), F.y0); ctx.lineTo(F.X(tr), F.y0 + F.h); ctx.stroke(); ctx.setLineDash([]);
  ctx.beginPath(); ctx.arc(F.X(tr), F.Y(tz), 7, 0, 2 * PI); ctx.lineWidth = 2; ctx.strokeStyle = '#141414'; ctx.stroke();
  ctx.beginPath(); ctx.arc(F.X(tr), F.Y(tz), 2.5, 0, 2 * PI); ctx.fillStyle = '#141414'; ctx.fill();
  ctx.lineWidth = 1; ctx.strokeStyle = '#141414'; ctx.strokeRect(F.x0, F.y0, F.w, F.h);
  ctx.fillStyle = '#141414'; ctx.font = '10.5px Inter, Arial'; ctx.textAlign = 'center';
  for (let r = 0; r <= F.r1 + 1e-9; r += st) ctx.fillText(+r.toFixed(3) + '', F.X(r), F.y0 + F.h + 13);
  ctx.fillText('ρ [m]', F.x0 + F.w / 2, F.y0 + F.h + 27);
  ctx.textAlign = 'right';
  for (let z = Math.ceil(F.z0 / st) * st; z <= F.z1 + 1e-9; z += st) ctx.fillText(+z.toFixed(3) + '', F.x0 - 4, F.Y(z) + 3);
  ctx.save(); ctx.translate(10, F.y0 + F.h / 2); ctx.rotate(-PI / 2); ctx.textAlign = 'center'; ctx.fillText('z [m]', 0, 0); ctx.restore(); ctx.textAlign = 'left';
}

function paintAll() { if (!state.A) return; state.A.sheets.forEach((sh) => paintSheet(`#s${sh.id}`, sh)); paintW(); }
function legend() {
  const [r, z] = state.target, nc = state.A.sheets.reduce((a, s) => a + s.cusps.length, 0);
  $('#legend').innerHTML = `<span><b>${APPROACH[state.ap].name}</b></span><span>critical curves of sheet ${state.A.sheets.map((sh) => `<span class="dot" style="background:${SHEET_COLORS[sh.id]}"></span>${sh.id}`).join(' ')}</span><span><span class="dot" style="background:#ff0000"></span>cusp (${nc} found)</span><span>◎ ρ = ${r.toFixed(3)} m · z = ${z.toFixed(3)} m</span><span><b>${shown().length} IK solutions</b>${state.limits ? ' within limits' : ''}</span>${state.regions ? '<span>numbers: IK count per region</span>' : ''}`;
}

// ---------------------------------------------------------------- interaction
['#sA', '#sB', '#sU', '#sD'].forEach((id) => $(id).addEventListener('pointerdown', (e) => {
  const g = geo[id]; if (!g) return;
  const r = e.target.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  const list = shown();
  const hit = list.findIndex((s) => s.id === g.sheet.id && Math.hypot(g.X(s.a) - x, g.Y(s.b) - y) < 10);
  if (hit >= 0) { state.sel = hit; solve(); return; }
  if (x < g.x0 || y < g.y0 || x > g.x0 + g.S || y > g.y0 + g.S) return;
  const a = ((x - g.x0) / g.S) * 2 * PI - PI, b = ((g.y0 + g.S - y) / g.S) * 2 * PI - PI;
  state.target = meridianOf(sheetConfig(g.sheet, a, b, state.A.v));
  solve();
  const near = shown().findIndex((s) => s.id === g.sheet.id && Math.abs(wrap(s.a - a)) < 1e-3 && Math.abs(wrap(s.b - b)) < 1e-3);
  if (near >= 0) { state.sel = near; solve(); }
}));
$('#wc').addEventListener('pointerdown', (e) => {
  if (!gw) return;
  const r = e.target.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  if (x < gw.x0 || y < gw.y0 || x > gw.x0 + gw.w || y > gw.y0 + gw.h) return;
  state.target = [gw.r0 + (x - gw.x0) / gw.sc, gw.z0 + (gw.y0 + gw.h - y) / gw.sc];
  state.sel = 0; solve();
});
$$('#apBtns button').forEach((b) => (b.onclick = () => { state.ap = b.dataset.ap; $$('#apBtns button').forEach((x) => x.classList.toggle('on', x === b)); state.target = null; rebuild(); }));
$('#limBtn').onclick = () => { state.limits = !state.limits; $('#limBtn').classList.toggle('on', state.limits); state.sel = 0; solve(); };
$('#regBtn').onclick = () => { state.regions = !state.regions; $('#regBtn').classList.toggle('on', state.regions); paintAll(); legend(); };
new ResizeObserver(() => paintAll()).observe($('#twin'));

rebuild();
renderMath();
