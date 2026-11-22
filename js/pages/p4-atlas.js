// Step 4 · Topology atlas: the Lecture 06 picture for GoFa (tool axis vertical, four sheets),
// on the meridian section or on any xy / xz / yz plane.
import { mountChrome, mountTabs, renderMath, $, $$ } from '../ui.js';
import { RobotViewer, THREE } from '../viewer.js';
import { buildGofaAtlas, solvePoint, sheetConfig, meridianOf, wrap, Q3_FORBIDDEN, APPROACH } from '../gofa-atlas.js';
import { fmt, rpy, rad } from '../math.js';
import { solveTool, toolPose } from '../tool-atlas.js';
import { sheetOf, detJ, verticalPose } from '../gofa-atlas.js';
import { useSupplied, solveVerified } from '../supplied.js';
import { mountIkSource } from '../ikui.js';

mountChrome();
mountTabs();
mountIkSource();

const PI = Math.PI;
const IK_COLORS = ['#c62828', '#1f5fa8', '#2e7d32', '#7b3fa0', '#e8590c', '#00838f', '#6d4c41', '#ad1457', '#5c6bc0', '#9e9d24', '#d81b60', '#00897b', '#8d6e63', '#3949ab', '#f4511e', '#546e7a'];
const SHEET_COLORS = { A: '#141414', B: '#1f5fa8', U: '#1f8a4c', D: '#e8590c' };
const COUNT_FILL = ['#ffffff', '#f6f8fc', '#eef3fb', '#e6eefa', '#dde8f8', '#d3e2f7', '#c9dbf5', '#bfd4f3', '#b5cdf1', '#abc6ef', '#a1bfed', '#97b8eb', '#8db1e9', '#83aae7', '#79a3e5', '#6f9ce3', '#6595e1'];
// target is a 3-D tool point p = [x, y, z]; the workspace view is the meridian (ρ, z) or a Cartesian plane
const state = { ap: 'down', A: null, target: null, sols: [], sel: 0, limits: false, regions: true, regionData: null, plane: 'rz', offset: { xy: 0.35, xz: 0, yz: 0.3 },
  tool: { key: 'down', rpy: [0, 180, 0], len: 0 }, num: null };
// ---------------------------------------------------------------- tool
// Orientation R = Rz(yaw) Ry(pitch) Rx(roll) and length ℓ along the tool z axis (TCP = tool0 + ℓ z6).
// A vertical tool axis keeps the exact (ρ, z) atlas; any other orientation switches to the numerical map.
const TOOLS = { down: [0, 180, 0], up: [0, 0, 0], fwd: [0, 90, 0], side: [-90, 0, 0], tilt: [0, 135, 0] };
const toolR = () => rpy(state.tool.rpy.map(rad));
function toolAxis() { const R = toolR(); return [R[0][2], R[1][2], R[2][2]]; }
function vertical() { const z = toolAxis(); return Math.abs(Math.abs(z[2]) - 1) < 1e-9 ? (z[2] < 0 ? 'down' : 'up') : null; }
const exact = () => !!vertical();
const dzTool = () => state.tool.len * (state.ap === 'down' ? -1 : 1); // TCP height minus tool0 height (exact mode)
const NUM_COLORS = { '+': '#141414', '−': '#1f5fa8' };
const AXN = ['x', 'y', 'z'];
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
  drawPlane3D();
  if (state.target) viewer.addPoint(state.target, { color: 0xff0000, radius: 0.015 });
  if (!robotReady) return;
  viewer.setRobotVisible(!!s);
  if (s) viewer.setQ(s.q);
  if (s && state.tool.len > 0) { const z = toolAxis(), t = state.target; viewer.addArrow([t[0] - state.tool.len * z[0], t[1] - state.tool.len * z[1], t[2] - state.tool.len * z[2]], z, { color: 0xd4a017, scale: state.tool.len, radius: 0.006 }); }
  $('#robotCap').textContent = s ? `IK ${state.sel + 1} · ${exact() ? `sheet ${s.id}` : `wrist q₅ ${s.id === '+' ? '≥' : '<'} 0`} · ${s.within ? 'within limits' : 'violates q₃ limit'}` : 'no solution';
}

// the working plane as a translucent quad in the 3-D view
function drawPlane3D() {
  if (!state.A) return;
  const P = planeDef(), [u0, u1] = P.uR, [v0, v1] = P.vR;
  const c = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]].map(([u, v]) => P.to3D(u, v));
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute([0, 1, 2, 0, 2, 3].flatMap((i) => c[i]), 3));
  viewer.overlay.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x2f6fe4, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false })));
  viewer.addPolyline(c, { color: 0x2f6fe4, closed: true });
}

// ---------------------------------------------------------------- planes
// u, v: plot axes; to3D maps them to the tool point; rz gives the meridian coordinates (ρ, z)
function planeDef() {
  const b = state.A.bounds, L = exact() ? 0 : state.tool.len, R = b.rhoMax * 1.06 + L, dz = b.zMax - b.zMin, sh = exact() ? dzTool() : 0;
  const Z = [b.zMin - 0.05 * dz + sh - L, b.zMax + 0.05 * dz + sh + L];
  const h = state.offset[state.plane];
  switch (state.plane) {
    case 'xy': return { uName: 'x', vName: 'y', uR: [-R, R], vR: [-R, R], to3D: (u, v) => [u, v, h], rz: (u, v) => [Math.hypot(u, v), h], from3D: (p) => [p[0], p[1]], n: 2, h };
    case 'xz': return { uName: 'x', vName: 'z', uR: [-R, R], vR: Z, to3D: (u, v) => [u, h, v], rz: (u, v) => [Math.hypot(u, h), v], from3D: (p) => [p[0], p[2]], n: 1, h };
    case 'yz': return { uName: 'y', vName: 'z', uR: [-R, R], vR: Z, to3D: (u, v) => [h, u, v], rz: (u, v) => [Math.hypot(h, u), v], from3D: (p) => [p[1], p[2]], n: 0, h };
    default: return { uName: 'ρ', vName: 'z', uR: [0, R], vR: Z, to3D: (u, v) => [u, 0, v], rz: (u, v) => [u, v], from3D: (p) => [Math.hypot(p[0], p[1]), p[2]], n: -1, h: 0 };
  }
}
// Critical curves of one sheet in plane coordinates (exact: the critical set is a surface of revolution)
function planeCurves(sh, P) {
  const out = [];
  const pieces = sh.images.map((c, ci) => { // split where the joint-space polyline wraps around
    const qc = sh.curves[ci], segs = [[]];
    const dz = dzTool();
    c.forEach((pt, i) => { if (i && (Math.abs(qc[i][0] - qc[i - 1][0]) > 0.5 || Math.abs(qc[i][1] - qc[i - 1][1]) > 0.5)) segs.push([]); segs[segs.length - 1].push([pt[0], pt[1] + dz]); });
    return segs;
  }).flat().filter((x) => x.length > 1);
  if (state.plane === 'rz') return pieces;
  if (state.plane === 'xy') {
    const h = P.h;
    for (const c of pieces) for (let i = 1; i < c.length; i++) {
      const [r0, z0] = c[i - 1], [r1, z1] = c[i];
      if ((z0 - h) * (z1 - h) > 0 || z0 === z1) continue;
      const r = r0 + ((h - z0) / (z1 - z0)) * (r1 - r0);
      out.push(Array.from({ length: 97 }, (_, k) => [r * Math.cos((2 * PI * k) / 96), r * Math.sin((2 * PI * k) / 96)]));
    }
    return out;
  }
  // xz / yz at offset c: u = ±√(ρ² − c²). Where a curve dips below ρ = |c| it is cut at u = 0,
  // with the exact crossing added so the ± branches join and regions stay closed.
  const c0 = Math.abs(P.h);
  for (const c of pieces) for (const sg of [1, -1]) {
    let cur = [];
    for (let i = 0; i < c.length; i++) {
      const [r, z] = c[i], ok = r >= c0;
      if (i) {
        const [rp, zp] = c[i - 1], okp = rp >= c0;
        if (ok !== okp) { const t = (c0 - rp) / (r - rp); cur.push([0, zp + t * (z - zp)]); if (!ok) { if (cur.length > 1) out.push(cur); cur = []; } }
      }
      if (ok) cur.push([sg * Math.sqrt(r * r - c0 * c0), z]);
    }
    if (cur.length > 1) out.push(cur);
  }
  return out;
}

// ---------------------------------------------------------------- build and solve
function rebuild() {
  state.A = buildGofaAtlas(state.ap);
  state.regionData = null;
  if (!state.target) state.target = [0.55, 0, state.ap === 'down' ? 0.35 : 0.6];
  syncPlaneUI();
  solve();
}
function solve() {
  state.src = 'browser IK';
  if (exact()) {
    const t = state.target;
    state.sols = solvePoint(state.ap, [t[0], t[1], t[2] - dzTool()]).sort((a, b) => a.id.localeCompare(b.id) || a.a - b.a);
  } else {
    state.sols = solveTool(toolR(), state.tool.len, state.target).map((x) => ({ ...x, id: x.w > 0 ? '+' : '−', a: wrap(x.q[1]), b: wrap(x.q[2]) }))
      .sort((a, b) => b.w - a.w || a.a - b.a);
  }
  render();
  fetchSupplied(); // replace with the supplied solver's verified answers when the helper is on
}
function render() {
  const list = shown();
  state.sel = Math.min(state.sel, Math.max(0, list.length - 1));
  const [x, y, z] = state.target, rho = Math.hypot(x, y);
  $('#ikTiles').innerHTML = [['tool point (m)', `${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}`], ['ρ (m)', rho.toFixed(3)], ['IK solutions', state.sols.continuum ? `∞ (${state.sols.filter((s) => s.family).length} families)` : state.limits ? `${list.length} in limits / ${state.sols.length}` : state.sols.length]]
    .map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
  $('#ikChips').innerHTML = list.map((s, i) => `<button class="chip ${i === state.sel ? 'on' : ''}" data-i="${i}" style="color:${IK_COLORS[i % 16]};${i === state.sel ? `background:${IK_COLORS[i % 16]}` : ''}">IK ${i + 1}</button>`).join('') || '<span class="note">No solution: the target is outside the workspace for this tool.</span>';
  $$('#ikChips .chip').forEach((c) => (c.onclick = () => { state.sel = Number(c.dataset.i); render(); }));
  $('#ikTable').innerHTML = list.length ? `<table class="starts"><thead><tr><th>IK</th><th>sheet</th>${[1, 2, 3, 4, 5, 6].map((i) => `<th>q${'₁₂₃₄₅₆'[i - 1]}</th>`).join('')}<th>det J</th></tr></thead><tbody>${
    list.map((s, i) => `<tr class="${i === state.sel ? 'sel' : ''} ${s.within ? '' : 'out'}"><td><span class="dot" style="background:${IK_COLORS[i % 16]}"></span>${i + 1}</td><td><span class="dot" style="background:${SHEET_COLORS[s.id] || NUM_COLORS[s.id]}"></span>${exact() ? s.id : `q₅ ${s.id === '+' ? '≥' : '<'} 0`}</td>${s.q.map((x) => `<td>${deg(x)}</td>`).join('')}<td>${fmt(s.det, 3)}</td></tr>`).join('')
  }</tbody></table>${state.sols.continuum ? '<p class="note" style="margin-top:8px">Singular pose: the IK set contains continuous families (q₁ and q₆ can turn together without moving the tool), so there are infinitely many solutions. Each family is shown by one member.</p>' : ''}` : '';
  $('#srcNote').textContent = state.suppliedNote && state.fromSupplied ? state.suppliedNote : 'IK from the browser solver (switch “IK” at the top right to use the supplied solver).';
  paintAll(); showRobot(); legend();
}
// IK panel from the supplied solver (the maps keep the browser IK for speed)
let reqId = 0;
async function fetchSupplied() {
  const id = ++reqId, target = state.target.slice();
  state.fromSupplied = false;
  if (!(await useSupplied())) return;
  const T = exact() ? verticalPose(state.ap, [target[0], target[1], target[2] - dzTool()]) : toolPose(toolR(), state.tool.len, target);
  try {
    const { sols, stats } = await solveVerified([T]);
    if (sols[0].error) sols[0].forEach((x) => { x.fallback = true; });
    if (id !== reqId || target.some((x, i) => x !== state.target[i])) return;
    const list = sols[0].map((s) => (exact() ? { ...s, ...sheetOf(s.q), det: detJ(s.q) } : { ...s, w: s.q[4] >= 0 ? 1 : -1, id: s.q[4] >= 0 ? '+' : '−', a: wrap(s.q[1]), b: wrap(s.q[2]), det: detJ(s.q) }));
    const n0 = state.sols.length;
    state.suppliedNote = sols[0].error ? `supplied solver gave up (${sols[0].error}); showing the browser IK for this pose` : `supplied solver · ${stats.accepted}/${stats.returned} verified by our FK (max ${stats.maxPos.toExponential(1)} m) · browser IK found ${n0}`;
    if (sols[0].error) { state.fromSupplied = true; render(); return; }
    state.sols = list.sort((a, b) => String(a.id).localeCompare(String(b.id)) || a.a - b.a);
    state.fromSupplied = true;
    render();
  } catch (e) { $('#srcNote').textContent = `supplied solver failed: ${e.message}`; }
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

// ---------------------------------------------------------------- workspace section (meridian or plane)
let gw = null;
function wFrame(W, H) {
  const P = planeDef(), phone = W < 420, PAD = { l: phone ? 38 : 46, r: 10, t: 12, b: phone ? 34 : 40 };
  const [r0, r1] = P.uR, [z0, z1] = P.vR;
  const sc = Math.min((W - PAD.l - PAD.r) / (r1 - r0), (H - PAD.t - PAD.b) / (z1 - z0));
  const w = (r1 - r0) * sc, h = (z1 - z0) * sc, x0 = PAD.l + (W - PAD.l - PAD.r - w) / 2, y0 = PAD.t + (H - PAD.t - PAD.b - h) / 2;
  return { P, r0, r1, z0, z1, sc, w, h, x0, y0, X: (r) => x0 + (r - r0) * sc, Y: (z) => y0 + h - (z - z0) * sc };
}
function drawCurves(ctx, F, colored = true) {
  for (const sh of state.A.sheets) {
    ctx.strokeStyle = colored ? SHEET_COLORS[sh.id] : '#000';
    for (const c of planeCurves(sh, F.P)) {
      ctx.beginPath();
      c.forEach(([u, v], i) => {
        const jump = i && Math.hypot(u - c[i - 1][0], v - c[i - 1][1]) > 0.15;
        if (!i || jump) ctx.moveTo(F.X(u), F.Y(v)); else ctx.lineTo(F.X(u), F.Y(v));
      });
      ctx.stroke();
    }
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
    const u = F.r0 + (best % RW) / k / F.sc, v = F.z0 + (F.h - ((best / RW) | 0) / k) / F.sc;
    const p3 = F.P.to3D(u, v), sols = solvePoint(state.ap, [p3[0], p3[1], p3[2] - dzTool()]);
    R.all = sols.length; R.within = sols.filter((s) => s.within).length; R.at = [u, v];
  });
  return { RW, RH, lab, regions };
}
function paintW() {
  const cv = $('#wc'), { ctx, W, H } = setup(cv), F = wFrame(W, H); gw = F;
  const cnt = (R) => (R.all === null || R.all === undefined ? null : state.limits ? R.within : R.all);
  if (state.regions) {
    const key = `${state.ap}:${state.tool.len}:${state.plane}:${F.P.h}:${Math.round(F.w)}x${Math.round(F.h)}`;
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
  const st = niceStep((F.r1 - F.r0) / 5), ticks = (a, b) => { const o = []; for (let t = Math.ceil(a / st) * st; t <= b + 1e-9; t += st) o.push(+t.toFixed(6)); return o; };
  ticks(F.r0, F.r1).forEach((r) => { ctx.beginPath(); ctx.moveTo(F.X(r), F.y0); ctx.lineTo(F.X(r), F.y0 + F.h); ctx.stroke(); });
  ticks(F.z0, F.z1).forEach((z) => { ctx.beginPath(); ctx.moveTo(F.x0, F.Y(z)); ctx.lineTo(F.x0 + F.w, F.Y(z)); ctx.stroke(); });
  ctx.lineWidth = 1.3; drawCurves(ctx, F, true);
  if (state.regions) {
    ctx.font = '600 11px Inter, Arial'; ctx.textAlign = 'center';
    state.regionData.regions.forEach((R) => {
      const n = cnt(R); if (n === null || R.px.length < 90) return;
      ctx.fillStyle = n ? '#1f3f73' : '#9a9a9a'; ctx.fillText(String(n), F.X(R.at[0]), F.Y(R.at[1]) + 4);
    });
    ctx.textAlign = 'left';
  }
  if (state.plane === 'rz') state.A.sheets.forEach((sh) => sh.cusps.forEach((c) => diamond(ctx, F.X(c.image[0]), F.Y(c.image[1] + dzTool()))));
  // base axis
  ctx.setLineDash([5, 4]); ctx.strokeStyle = 'rgba(20,20,20,.5)'; ctx.lineWidth = 1;
  if (state.plane === 'xy') { ctx.beginPath(); ctx.arc(F.X(0), F.Y(0), 3, 0, 2 * PI); ctx.stroke(); }
  else if (state.plane === 'rz' || Math.abs(F.P.h) < 1e-9) { ctx.beginPath(); ctx.moveTo(F.X(0), F.y0); ctx.lineTo(F.X(0), F.y0 + F.h); ctx.stroke(); }
  ctx.setLineDash([]);
  // target (only if it lies on the plane)
  const [tu, tv] = F.P.from3D(state.target), onPlane = F.P.n < 0 || Math.abs(state.target[F.P.n] - F.P.h) < 1e-6;
  if (onPlane) {
    ctx.setLineDash([3, 3]); ctx.strokeStyle = 'rgba(20,20,20,.45)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(F.x0, F.Y(tv)); ctx.lineTo(F.x0 + F.w, F.Y(tv)); ctx.moveTo(F.X(tu), F.y0); ctx.lineTo(F.X(tu), F.y0 + F.h); ctx.stroke(); ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(F.X(tu), F.Y(tv), 7, 0, 2 * PI); ctx.lineWidth = 2; ctx.strokeStyle = '#141414'; ctx.stroke();
    ctx.beginPath(); ctx.arc(F.X(tu), F.Y(tv), 2.5, 0, 2 * PI); ctx.fillStyle = '#141414'; ctx.fill();
  }
  ctx.lineWidth = 1; ctx.strokeStyle = '#141414'; ctx.strokeRect(F.x0, F.y0, F.w, F.h);
  ctx.fillStyle = '#141414'; ctx.font = '10.5px Inter, Arial'; ctx.textAlign = 'center';
  ticks(F.r0, F.r1).forEach((r) => ctx.fillText(+r.toFixed(3) + '', F.X(r), F.y0 + F.h + 13));
  ctx.fillText(`${F.P.uName} [m]`, F.x0 + F.w / 2, F.y0 + F.h + 27);
  ctx.textAlign = 'right';
  ticks(F.z0, F.z1).forEach((z) => ctx.fillText(+z.toFixed(3) + '', F.x0 - 4, F.Y(z) + 3));
  ctx.save(); ctx.translate(10, F.y0 + F.h / 2); ctx.rotate(-PI / 2); ctx.textAlign = 'center'; ctx.fillText(`${F.P.vName} [m]`, 0, 0); ctx.restore(); ctx.textAlign = 'left';
  $('#wTitle').textContent = state.plane === 'rz' ? 'Workspace section (ρ, z)' : `Plane ${state.plane} at ${AXN[F.P.n]} = ${F.P.h.toFixed(2)} m`;
  $('#wSub').textContent = state.plane === 'rz' ? 'ρ ≥ 0 · rotate about z for the full workspace' : 'exact slice of the rotated critical surfaces';
}

// ---------------------------------------------------------------- plane controls
function syncPlaneUI() {
  $$('#planeBtns button').forEach((b) => b.classList.toggle('on', b.dataset.pl === state.plane));
  const box = $('#offBox');
  box.hidden = state.plane === 'rz';
  if (state.plane === 'rz' || !state.A) return;
  const P = planeDef(), b = state.A.bounds, R = b.rhoMax;
  const L = state.tool.len, range = state.plane === 'xy' ? [b.zMin - L, b.zMax + L] : [-R - L, R + L];
  const inp = $('#off'); inp.min = range[0].toFixed(2); inp.max = range[1].toFixed(2); inp.value = state.offset[state.plane];
  $('#offName').textContent = `${AXN[P.n]} =`; $('#offOut').textContent = `${Number(inp.value).toFixed(2)} m`;
}
// Keep the target on the current plane when switching or moving it
function moveTargetOntoPlane() {
  if (state.plane === 'rz') { state.target = [Math.hypot(state.target[0], state.target[1]), 0, state.target[2]]; return; }
  const P = planeDef(); state.target = state.target.slice(); state.target[P.n] = P.h;
}
$$('#planeBtns button').forEach((b) => (b.onclick = () => { if (b.disabled) return; state.plane = b.dataset.pl; state.regionData = null; syncPlaneUI(); moveTargetOntoPlane(); solve(); }));
$('#off').oninput = (e) => { state.offset[state.plane] = Number(e.target.value); $('#offOut').textContent = `${state.offset[state.plane].toFixed(2)} m`; showRobot(); };
$('#off').onchange = () => { state.regionData = null; moveTargetOntoPlane(); solve(); };

function paintAll() {
  if (!state.A) return;
  $('.quad').classList.toggle('two', !exact());
  $('#qTitle').textContent = exact() ? 'Joint space · four sheets' : 'Joint space · footprint of the plane';
  $('#qSub').textContent = exact() ? 'det J on each sheet, zero set in black' : 'every IK solution of the plane at (q₂, q₃), coloured by det J';
  if (exact()) { state.A.sheets.forEach((sh) => paintSheet(`#s${sh.id}`, sh)); paintW(); }
  else { paintWristNum('#sA', 1); paintWristNum('#sB', -1); paintWNum(); }
}
function legend() {
  if (!exact()) return legendNum();
  const [x, y, z] = state.target, r = Math.hypot(x, y), nc = state.A.sheets.reduce((a, s) => a + s.cusps.length, 0);
  $('#legend').innerHTML = `<span><b>${APPROACH[state.ap].name}${state.tool.len ? ` · ℓ = ${state.tool.len.toFixed(2)} m` : ''}</b></span><span>critical curves of sheet ${state.A.sheets.map((sh) => `<span class="dot" style="background:${SHEET_COLORS[sh.id]}"></span>${sh.id}`).join(' ')}</span><span><span class="dot" style="background:#ff0000"></span>cusp (${nc} found)</span><span>◎ (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}) m · ρ = ${r.toFixed(3)} m</span><span><b>${shown().length} IK solutions</b>${state.limits ? ' within limits' : ''}</span>${state.regions ? '<span>numbers: IK count per region</span>' : ''}`;
}

// ---------------------------------------------------------------- numerical mode (tool axis not vertical)
// Plane grid solved point by point in web workers: IK count per pixel, and every solution's (q2, q3),
// wrist sign and det J for the joint-space footprint.
let workers = [];
function planeGrid() {
  const P = planeDef(), res = window.matchMedia('(max-width: 980px)').matches ? 70 : 90;
  const step = (P.uR[1] - P.uR[0]) / res, nv = Math.round((P.vR[1] - P.vR[0]) / step);
  const ui = { xy: 0, xz: 0, yz: 1 }[state.plane], vi = { xy: 1, xz: 2, yz: 2 }[state.plane];
  return { u0: P.uR[0], v0: P.vR[0], step, nu: res, nv, ui, vi, n: P.n, h: P.h };
}
function computeNum() {
  const G = planeGrid(), key = JSON.stringify([state.tool, state.plane, G.h, G.nu]);
  if (state.num?.key === key) return;
  workers.forEach((w) => w.terminate());
  const n = Math.max(1, Math.min(8, (navigator.hardwareConcurrency || 4) - 1));
  const num = { key, G, all: new Uint8Array(G.nu * G.nv), inl: new Uint8Array(G.nu * G.nv), sols: [], rows: 0, dMax: 1e-9, done: false };
  state.num = num;
  $('#progress').hidden = false; $('#progress').textContent = 'Solving IK on the plane…';
  let last = 0;
  const onRow = (r) => {
    if (state.num !== num) return;
    num.all.set(r.all, r.j * G.nu); num.inl.set(r.inl, r.j * G.nu); num.sols.push(r.sols);
    for (let k = 4; k < r.sols.length; k += 6) num.dMax = Math.max(num.dMax, Math.abs(r.sols[k]));
    num.rows++;
    $('#progress').textContent = `Solving IK on the plane ${Math.round((100 * num.rows) / G.nv)}%`;
    if (num.rows === G.nv) { num.done = true; $('#progress').hidden = true; paintAll(); }
    else if (performance.now() - last > 300) { last = performance.now(); paintAll(); }
  };
  try {
    workers = Array.from({ length: n }, () => new Worker(new URL('../atlas-worker.js', import.meta.url), { type: 'module' }));
    const R = toolR();
    workers.forEach((w, wi) => {
      w.onmessage = (e) => { if (!e.data.done) onRow(e.data); };
      const rows = []; for (let j = wi; j < G.nv; j += n) rows.push(j);
      w.postMessage({ R, len: state.tool.len, G, rows });
    });
  } catch (err) { console.error(err); }
}
function paintWNum() {
  computeNum();
  const cv = $('#wc'), { ctx, W, H } = setup(cv), F = wFrame(W, H); gw = F;
  const N = state.num, G = N.G, cnt = state.limits ? N.inl : N.all, px = G.step * F.sc;
  const X = (i) => F.x0 + i * px, Y = (j) => F.y0 + F.h - (j + 1) * px;
  const filled = (k) => Math.floor(k / G.nu) < N.rows || N.done;
  for (let j = 0; j < G.nv; j++) for (let i = 0; i < G.nu; i++) {
    const k = j * G.nu + i;
    ctx.fillStyle = state.regions ? (cnt[k] === 255 ? '#141414' : COUNT_FILL[Math.min(cnt[k], 16)]) : '#f7f9fc';
    ctx.fillRect(X(i), Y(j), px + 0.6, px + 0.6);
  }
  // critical curves ≈ where the IK count changes between neighbouring points
  ctx.strokeStyle = '#141414'; ctx.lineWidth = 1.3; ctx.beginPath();
  for (let j = 0; j < G.nv; j++) for (let i = 0; i < G.nu; i++) {
    const k = j * G.nu + i;
    if (i + 1 < G.nu && N.all[k] !== N.all[k + 1]) { ctx.moveTo(X(i + 1), Y(j)); ctx.lineTo(X(i + 1), Y(j) + px); }
    if (j + 1 < G.nv && N.all[k] !== N.all[k + G.nu]) { ctx.moveTo(X(i), Y(j)); ctx.lineTo(X(i) + px, Y(j)); }
  }
  ctx.stroke();
  // labels: one per connected region of equal count, at its deepest pixel
  if (state.regions && N.done) {
    const lab = new Int32Array(G.nu * G.nv).fill(-1);
    ctx.font = '600 11px Inter, Arial'; ctx.textAlign = 'center';
    for (let s0 = 0; s0 < lab.length; s0++) {
      if (lab[s0] >= 0) continue;
      const stack = [s0], pix = []; lab[s0] = s0;
      while (stack.length) {
        const t = stack.pop(); pix.push(t);
        const x = t % G.nu, y = (t / G.nu) | 0;
        for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
          const nx = x + dx, ny = y + dy; if (nx < 0 || ny < 0 || nx >= G.nu || ny >= G.nv) continue;
          const u = ny * G.nu + nx; if (lab[u] < 0 && cnt[u] === cnt[s0]) { lab[u] = s0; stack.push(u); }
        }
      }
      if (pix.length < 40) continue;
      let best = pix[0], bd = -1;
      for (const t of pix) {
        const x = t % G.nu, y = (t / G.nu) | 0; let d = 0;
        while (d < 10 && [[x + d + 1, y], [x - d - 1, y], [x, y + d + 1], [x, y - d - 1]].every(([a, b]) => a >= 0 && b >= 0 && a < G.nu && b < G.nv && cnt[b * G.nu + a] === cnt[s0])) d++;
        if (d > bd) { bd = d; best = t; }
      }
      ctx.fillStyle = cnt[s0] ? '#1f3f73' : '#9a9a9a';
      ctx.fillText(cnt[s0] === 255 ? '∞' : String(cnt[s0]), X(best % G.nu) + px / 2, Y((best / G.nu) | 0) + px / 2 + 4);
    }
    ctx.textAlign = 'left';
  }
  const st = niceStep((F.r1 - F.r0) / 5), ticks = (a, b) => { const o = []; for (let t = Math.ceil(a / st) * st; t <= b + 1e-9; t += st) o.push(+t.toFixed(6)); return o; };
  ctx.setLineDash([5, 4]); ctx.strokeStyle = 'rgba(20,20,20,.5)'; ctx.lineWidth = 1;
  if (state.plane === 'xy') { ctx.beginPath(); ctx.arc(F.X(0), F.Y(0), 3, 0, 2 * PI); ctx.stroke(); }
  else if (Math.abs(F.P.h) < 1e-9) { ctx.beginPath(); ctx.moveTo(F.X(0), F.y0); ctx.lineTo(F.X(0), F.y0 + F.h); ctx.stroke(); }
  ctx.setLineDash([]);
  const [tu, tv] = F.P.from3D(state.target);
  ctx.beginPath(); ctx.arc(F.X(tu), F.Y(tv), 7, 0, 2 * PI); ctx.lineWidth = 2; ctx.strokeStyle = '#141414'; ctx.stroke();
  ctx.beginPath(); ctx.arc(F.X(tu), F.Y(tv), 2.5, 0, 2 * PI); ctx.fillStyle = '#141414'; ctx.fill();
  ctx.lineWidth = 1; ctx.strokeRect(F.x0, F.y0, F.w, F.h);
  ctx.fillStyle = '#141414'; ctx.font = '10.5px Inter, Arial'; ctx.textAlign = 'center';
  ticks(F.r0, F.r1).forEach((r) => ctx.fillText(+r.toFixed(3) + '', F.X(r), F.y0 + F.h + 13));
  ctx.fillText(`${F.P.uName} [m]`, F.x0 + F.w / 2, F.y0 + F.h + 27);
  ctx.textAlign = 'right';
  ticks(F.z0, F.z1).forEach((z) => ctx.fillText(+z.toFixed(3) + '', F.x0 - 4, F.Y(z) + 3));
  ctx.save(); ctx.translate(10, F.y0 + F.h / 2); ctx.rotate(-PI / 2); ctx.textAlign = 'center'; ctx.fillText(`${F.P.vName} [m]`, 0, 0); ctx.restore(); ctx.textAlign = 'left';
  $('#wTitle').textContent = `Plane ${state.plane} at ${AXN[F.P.n]} = ${F.P.h.toFixed(2)} m`;
  $('#wSub').textContent = `numerical · ${G.nu} × ${G.nv} points · boundaries = count changes`;
}
// Joint-space footprint of the plane: every IK solution of every plane point, at its (q2, q3),
// coloured by det J; near-singular points are dark, so the singular curves show as dark lines.
function paintWristNum(id, w) {
  const cv = $(id), { ctx, W, H } = setup(cv);
  const PAD = { l: 34, r: 24, t: 16, b: 28 };
  const S = Math.min(W - PAD.l - PAD.r, H - PAD.t - PAD.b), x0 = PAD.l + (W - PAD.l - PAD.r - S) / 2, y0 = PAD.t + (H - PAD.t - PAD.b - S) / 2;
  const X = (q2) => x0 + ((wrap(q2) + PI) / (2 * PI)) * S, Y = (q3) => y0 + S - ((wrap(q3) + PI) / (2 * PI)) * S;
  geo[id] = { X, Y, x0, y0, S, w };
  ctx.fillStyle = '#fafafa'; ctx.fillRect(x0, y0, S, S);
  const N = state.num;
  if (N) {
    const sz = Math.max(1.6, (S / 2 / PI) * N.G.step * 2.2);
    for (const arr of N.sols) for (let k = 0; k < arr.length; k += 6) {
      if (arr[k + 3] !== w || (state.limits && !arr[k + 5])) continue;
      const t = arr[k + 4] / N.dMax, near = Math.abs(t) < 0.03;
      const c = near ? [20, 20, 20] : diverging(t * 1.6);
      ctx.fillStyle = `rgb(${c.map(Math.round).join(',')})`;
      ctx.fillRect(X(arr[k + 1]) - sz / 2, Y(arr[k + 2]) - sz / 2, sz, sz);
    }
  }
  if (state.limits) {
    const ya = Y(Q3_FORBIDDEN[1]), yb = Y(Q3_FORBIDDEN[0]);
    ctx.fillStyle = 'rgba(255,255,255,.6)'; ctx.fillRect(x0, ya, S, yb - ya);
  }
  shown().forEach((s, i) => { if (s.w === w) marker(ctx, X(s.q[1]), Y(s.q[2]), i, i === state.sel, false); });
  ctx.strokeStyle = NUM_COLORS[w > 0 ? '+' : '−']; ctx.lineWidth = 2; ctx.strokeRect(x0, y0, S, S);
  ctx.fillStyle = NUM_COLORS[w > 0 ? '+' : '−']; ctx.font = '600 10.5px Inter, Arial';
  ctx.fillText(`Wrist q₅ ${w > 0 ? '≥' : '<'} 0 · footprint of the plane`, x0, y0 - 5);
  ctx.font = '9px Inter, Arial'; ctx.fillStyle = '#141414';
  const labels = [[-PI, '−π'], [0, '0'], [PI, 'π']];
  ctx.textAlign = 'center'; labels.forEach(([v, t]) => ctx.fillText(t, X(v === PI ? PI - 1e-9 : v), y0 + S + 12)); ctx.fillText('q₂', x0 + S / 2, y0 + S + 24);
  ctx.textAlign = 'right'; labels.forEach(([v, t]) => ctx.fillText(t, x0 - 4, Y(v === PI ? PI - 1e-9 : v) + 3));
  ctx.save(); ctx.translate(10, y0 + S / 2); ctx.rotate(-PI / 2); ctx.textAlign = 'center'; ctx.fillText('q₃', 0, 0); ctx.restore(); ctx.textAlign = 'left';
}
// Tap on a footprint: jump to the plane point whose solution is nearest
function tapWristNum(g, x, y) {
  const list = shown();
  const hit = list.findIndex((s) => s.w === g.w && Math.hypot(g.X(s.q[1]) - x, g.Y(s.q[2]) - y) < 10);
  if (hit >= 0) { state.sel = hit; render(); return; }
  const N = state.num; if (!N) return;
  let best = null, bd = 64;
  for (const arr of N.sols) for (let k = 0; k < arr.length; k += 6) {
    if (arr[k + 3] !== g.w || (state.limits && !arr[k + 5])) continue;
    const d = (g.X(arr[k + 1]) - x) ** 2 + (g.Y(arr[k + 2]) - y) ** 2;
    if (d < bd) { bd = d; best = [arr[k], arr[k + 1], arr[k + 2]]; }
  }
  if (!best) return;
  const G = N.G, i = best[0] % G.nu, j = Math.floor(best[0] / G.nu), p = [0, 0, 0];
  p[G.ui] = G.u0 + (i + 0.5) * G.step; p[G.vi] = G.v0 + (j + 0.5) * G.step; p[G.n] = G.h;
  state.target = p; solve();
  const near = shown().findIndex((s) => s.w === g.w && Math.abs(wrap(s.q[1] - best[1])) < 1e-6 && Math.abs(wrap(s.q[2] - best[2])) < 1e-6);
  if (near >= 0) { state.sel = near; render(); }
}
function legendNum() {
  const [x, y, z] = state.target;
  $('#legend').innerHTML = `<span><b>Tool rpy (${state.tool.rpy.join('°, ')}°) · ℓ = ${state.tool.len.toFixed(2)} m</b></span><span>numbers: IK count per region · black lines: count changes</span><span>left: plane’s IK solutions at (q₂, q₃), dark = det J ≈ 0</span><span>◎ (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}) m · <b>${shown().length} IK solutions</b>${state.limits ? ' within limits' : ''}</span>`;
}

// ---------------------------------------------------------------- interaction
// A tool point with the given (ρ, z): on the meridian, or on the current plane if possible
function pointWithRhoZ(r, z) {
  if (state.plane === 'rz') return [r, 0, z];
  const h = state.offset[state.plane];
  if (state.plane === 'xy') {
    if (Math.abs(z - h) > 1e-9) { state.plane = 'rz'; syncPlaneUI(); state.regionData = null; return [r, 0, z]; }
    return [r, 0, h];
  }
  if (r < Math.abs(h)) { state.plane = 'rz'; syncPlaneUI(); state.regionData = null; return [r, 0, z]; }
  const u = Math.sqrt(r * r - h * h);
  return state.plane === 'xz' ? [u, h, z] : [h, u, z];
}
['#sA', '#sB', '#sU', '#sD'].forEach((id) => $(id).addEventListener('pointerdown', (e) => {
  const g = geo[id]; if (!g) return;
  const r = e.target.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  if (!exact()) return tapWristNum(g, x, y);
  const list = shown();
  const hit = list.findIndex((s) => s.id === g.sheet.id && Math.hypot(g.X(s.a) - x, g.Y(s.b) - y) < 10);
  if (hit >= 0) { state.sel = hit; render(); return; }
  if (x < g.x0 || y < g.y0 || x > g.x0 + g.S || y > g.y0 + g.S) return;
  const a = ((x - g.x0) / g.S) * 2 * PI - PI, b = ((g.y0 + g.S - y) / g.S) * 2 * PI - PI;
  const [rr, zz] = meridianOf(sheetConfig(g.sheet, a, b, state.A.v));
  state.target = pointWithRhoZ(rr, zz + dzTool());
  solve();
  const near = shown().findIndex((s) => s.id === g.sheet.id && Math.abs(wrap(s.a - a)) < 1e-3 && Math.abs(wrap(s.b - b)) < 1e-3);
  if (near >= 0) { state.sel = near; render(); }
}));
$('#wc').addEventListener('pointerdown', (e) => {
  if (!gw) return;
  const r = e.target.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  if (x < gw.x0 || y < gw.y0 || x > gw.x0 + gw.w || y > gw.y0 + gw.h) return;
  state.target = gw.P.to3D(gw.r0 + (x - gw.x0) / gw.sc, gw.z0 + (gw.y0 + gw.h - y) / gw.sc);
  state.sel = 0; solve();
});
// tool controls (toolbar presets and the Tool tab)
function setTool(changedOrientation) {
  const v = vertical();
  $$('#apBtns button, #toolSeg button').forEach((b) => b.classList.toggle('on', b.dataset.t === state.tool.key));
  syncToolUI();
  $('[data-pl="rz"]').disabled = !v;
  $('#toolNote').innerHTML = v ? `Tool axis vertical (${v}): exact atlas, all four sheets.` : 'Tool axis not vertical: the (ρ, z) symmetry is lost, so the plane is solved point by point (numerical map) and joint space shows where the plane’s IK solutions land, split by wrist sign.';
  state.regionData = null; state.num = null;
  if (v && (v !== state.ap || !state.A)) { state.ap = v; state.A = buildGofaAtlas(v); }
  if (!v && state.plane === 'rz') { state.plane = 'xz'; state.target = [state.target[0], state.offset.xz, state.target[2]]; }
  syncPlaneUI(); moveTargetOntoPlane();
  if (changedOrientation) state.sel = 0;
  solve();
}
$$('#apBtns button, #toolSeg button').forEach((b) => (b.onclick = () => {
  state.tool.key = b.dataset.t; if (TOOLS[b.dataset.t]) state.tool.rpy = TOOLS[b.dataset.t].slice(); setTool(true);
}));
// sliders and number boxes show the same values; a change in either recomputes
function syncToolUI() {
  $$('#apBtns button, #toolSeg button').forEach((b) => b.classList.toggle('on', b.dataset.t === state.tool.key));
  $$('#rpy [data-k]').forEach((inp) => { inp.value = state.tool.rpy[Number(inp.dataset.k)]; });
  $$('#rpy [data-kn]').forEach((inp) => { inp.value = state.tool.rpy[Number(inp.dataset.kn)]; });
  $('#len').value = state.tool.len; $('#lenNum').value = state.tool.len.toFixed(2);
}
function commitRpy(k, v) {
  if (!Number.isFinite(v)) return syncToolUI();
  state.tool.rpy[k] = Math.max(-180, Math.min(180, Math.round(v)));
  state.tool.key = Object.keys(TOOLS).find((t) => TOOLS[t].every((x, i) => x === state.tool.rpy[i])) || 'custom';
  setTool(true);
}
$$('#rpy [data-k]').forEach((inp) => {
  const k = Number(inp.dataset.k);
  inp.oninput = () => { $(`#rpy [data-kn="${k}"]`).value = inp.value; };
  inp.onchange = () => commitRpy(k, Number(inp.value));
});
$$('#rpy [data-kn]').forEach((inp) => { inp.onchange = () => commitRpy(Number(inp.dataset.kn), Number(inp.value)); });
const commitLen = (v) => { if (!Number.isFinite(v)) return syncToolUI(); state.tool.len = Math.max(0, Math.min(0.3, Math.round(v * 100) / 100)); setTool(false); };
$('#len').oninput = (e) => { $('#lenNum').value = Number(e.target.value).toFixed(2); };
$('#len').onchange = (e) => commitLen(Number(e.target.value));
$('#lenNum').onchange = (e) => commitLen(Number(e.target.value));
syncToolUI();
$('#toolNote').textContent = 'Tool axis vertical (down): exact atlas, all four sheets.';
window.addEventListener('ik-source', () => solve());
rebuild();
renderMath();
