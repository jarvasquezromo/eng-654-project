// Step 4 · Topology atlas: the Lecture 06 picture for GoFa (tool axis vertical, four sheets),
// on the meridian section or on any xy / xz / yz plane.
import { mountChrome, mountTabs, renderMath, $, $$ } from '../ui.js';
import { RobotViewer, THREE } from '../viewer.js';
import { buildGofaAtlas, solvePoint, sheetConfig, meridianOf, wrap, Q3_FORBIDDEN, APPROACH } from '../gofa-atlas.js';
import { fmt } from '../math.js';

mountChrome();
mountTabs();

const PI = Math.PI;
const IK_COLORS = ['#c62828', '#1f5fa8', '#2e7d32', '#7b3fa0', '#e8590c', '#00838f', '#6d4c41', '#ad1457', '#5c6bc0', '#9e9d24', '#d81b60', '#00897b', '#8d6e63', '#3949ab', '#f4511e', '#546e7a'];
const SHEET_COLORS = { A: '#141414', B: '#1f5fa8', U: '#1f8a4c', D: '#e8590c' };
const COUNT_FILL = ['#ffffff', '#f6f8fc', '#eef3fb', '#e6eefa', '#dde8f8', '#d3e2f7', '#c9dbf5', '#bfd4f3', '#b5cdf1', '#abc6ef', '#a1bfed', '#97b8eb', '#8db1e9', '#83aae7', '#79a3e5', '#6f9ce3', '#6595e1'];
// target is a 3-D tool point p = [x, y, z]; the workspace view is the meridian (ρ, z) or a Cartesian plane
const state = { ap: 'down', A: null, target: null, sols: [], sel: 0, limits: false, regions: true, regionData: null, plane: 'rz', offset: { xy: 0.35, xz: 0, yz: 0.3 } };
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
  $('#robotCap').textContent = s ? `IK ${state.sel + 1} · sheet ${s.id} · ${s.within ? 'within limits' : 'violates q₃ limit'}` : 'no solution';
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
  const b = state.A.bounds, R = b.rhoMax * 1.06, dz = b.zMax - b.zMin, Z = [b.zMin - 0.05 * dz, b.zMax + 0.05 * dz];
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
    c.forEach((pt, i) => { if (i && (Math.abs(qc[i][0] - qc[i - 1][0]) > 0.5 || Math.abs(qc[i][1] - qc[i - 1][1]) > 0.5)) segs.push([]); segs[segs.length - 1].push(pt); });
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
  state.sols = solvePoint(state.ap, state.target).sort((a, b) => a.id.localeCompare(b.id) || a.a - b.a);
  const list = shown();
  state.sel = Math.min(state.sel, Math.max(0, list.length - 1));
  const [x, y, z] = state.target, rho = Math.hypot(x, y);
  $('#ikTiles').innerHTML = [['tool point (m)', `${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}`], ['ρ (m)', rho.toFixed(3)], ['IK solutions', state.limits ? `${list.length} in limits / ${state.sols.length}` : state.sols.length]]
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
    const sols = solvePoint(state.ap, F.P.to3D(u, v));
    R.all = sols.length; R.within = sols.filter((s) => s.within).length; R.at = [u, v];
  });
  return { RW, RH, lab, regions };
}
function paintW() {
  const cv = $('#wc'), { ctx, W, H } = setup(cv), F = wFrame(W, H); gw = F;
  const cnt = (R) => (R.all === null || R.all === undefined ? null : state.limits ? R.within : R.all);
  if (state.regions) {
    const key = `${state.ap}:${state.plane}:${F.P.h}:${Math.round(F.w)}x${Math.round(F.h)}`;
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
  if (state.plane === 'rz') state.A.sheets.forEach((sh) => sh.cusps.forEach((c) => diamond(ctx, F.X(c.image[0]), F.Y(c.image[1]))));
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
  const range = state.plane === 'xy' ? [b.zMin, b.zMax] : [-R, R];
  const inp = $('#off'); inp.min = range[0].toFixed(2); inp.max = range[1].toFixed(2); inp.value = state.offset[state.plane];
  $('#offName').textContent = `${AXN[P.n]} =`; $('#offOut').textContent = `${Number(inp.value).toFixed(2)} m`;
}
// Keep the target on the current plane when switching or moving it
function moveTargetOntoPlane() {
  if (state.plane === 'rz') { state.target = [Math.hypot(state.target[0], state.target[1]), 0, state.target[2]]; return; }
  const P = planeDef(); state.target = state.target.slice(); state.target[P.n] = P.h;
}
$$('#planeBtns button').forEach((b) => (b.onclick = () => { state.plane = b.dataset.pl; state.regionData = null; syncPlaneUI(); moveTargetOntoPlane(); solve(); }));
$('#off').oninput = (e) => { state.offset[state.plane] = Number(e.target.value); $('#offOut').textContent = `${state.offset[state.plane].toFixed(2)} m`; showRobot(); };
$('#off').onchange = () => { state.regionData = null; moveTargetOntoPlane(); solve(); };

function paintAll() { if (!state.A) return; state.A.sheets.forEach((sh) => paintSheet(`#s${sh.id}`, sh)); paintW(); }
function legend() {
  const [x, y, z] = state.target, r = Math.hypot(x, y), nc = state.A.sheets.reduce((a, s) => a + s.cusps.length, 0);
  $('#legend').innerHTML = `<span><b>${APPROACH[state.ap].name}</b></span><span>critical curves of sheet ${state.A.sheets.map((sh) => `<span class="dot" style="background:${SHEET_COLORS[sh.id]}"></span>${sh.id}`).join(' ')}</span><span><span class="dot" style="background:#ff0000"></span>cusp (${nc} found)</span><span>◎ (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}) m · ρ = ${r.toFixed(3)} m</span><span><b>${shown().length} IK solutions</b>${state.limits ? ' within limits' : ''}</span>${state.regions ? '<span>numbers: IK count per region</span>' : ''}`;
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
  const list = shown();
  const hit = list.findIndex((s) => s.id === g.sheet.id && Math.hypot(g.X(s.a) - x, g.Y(s.b) - y) < 10);
  if (hit >= 0) { state.sel = hit; solve(); return; }
  if (x < g.x0 || y < g.y0 || x > g.x0 + g.S || y > g.y0 + g.S) return;
  const a = ((x - g.x0) / g.S) * 2 * PI - PI, b = ((g.y0 + g.S - y) / g.S) * 2 * PI - PI;
  const [rr, zz] = meridianOf(sheetConfig(g.sheet, a, b, state.A.v));
  state.target = pointWithRhoZ(rr, zz);
  solve();
  const near = shown().findIndex((s) => s.id === g.sheet.id && Math.abs(wrap(s.a - a)) < 1e-3 && Math.abs(wrap(s.b - b)) < 1e-3);
  if (near >= 0) { state.sel = near; solve(); }
}));
$('#wc').addEventListener('pointerdown', (e) => {
  if (!gw) return;
  const r = e.target.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  if (x < gw.x0 || y < gw.y0 || x > gw.x0 + gw.w || y > gw.y0 + gw.h) return;
  state.target = gw.P.to3D(gw.r0 + (x - gw.x0) / gw.sc, gw.z0 + (gw.y0 + gw.h - y) / gw.sc);
  state.sel = 0; solve();
});
$$('#apBtns button').forEach((b) => (b.onclick = () => { state.ap = b.dataset.ap; $$('#apBtns button').forEach((x) => x.classList.toggle('on', x === b)); state.target = null; rebuild(); if (state.plane !== 'rz') { moveTargetOntoPlane(); solve(); } }));
$('#limBtn').onclick = () => { state.limits = !state.limits; $('#limBtn').classList.toggle('on', state.limits); state.sel = 0; solve(); };
$('#regBtn').onclick = () => { state.regions = !state.regions; $('#regBtn').classList.toggle('on', state.regions); paintAll(); legend(); };
new ResizeObserver(() => paintAll()).observe($('#twin'));

rebuild();
renderMath();
