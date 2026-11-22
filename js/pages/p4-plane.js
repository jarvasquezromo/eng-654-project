// Step 4b: conditioning over a Cartesian plane (xy, xz or yz) at a chosen offset, for a fixed tool.
import { RobotViewer, THREE } from '../viewer.js';
import { mountChrome, mountTabs, renderMath, $, $$ } from '../ui.js';
import { pointJacobian, conditioning, poeFK } from '../gofa.js';
import { solveIK, branchIndex, branchLabel, BRANCHES } from '../ik.js';
import { rpy, homog, deg, rad, fmt, posOf } from '../math.js';

mountChrome();
mountTabs();

const AX = ['x', 'y', 'z'];
const PLANES = {
  xy: { u: 0, v: 1, n: 2, uR: [-1.2, 1.2], vR: [-1.2, 1.2], nR: [-0.6, 1.5], n0: 0.40, note: 'Height above the base frame (the base mounting plane is z = 0).' },
  xz: { u: 0, v: 2, n: 1, uR: [-1.2, 1.2], vR: [-0.6, 1.5], nR: [-1.1, 1.1], n0: 0.00, note: 'y = 0 is the vertical plane through the base axis, facing forward.' },
  yz: { u: 1, v: 2, n: 0, uR: [-1.2, 1.2], vR: [-0.6, 1.5], nR: [-1.1, 1.1], n0: 0.40, note: 'Distance in front of the base axis along x.' },
};
const TOOLS = { down: [0, 180, 0], fwd: [0, 90, 0], side: [-90, 0, 0], up: [0, 0, 0] };
const BR_COLORS = ['#2f6fe4', '#7aa7f5', '#ff0000', '#ff8a80', '#1f8a4c', '#8fd19e', '#d4a017', '#f2d27a'];
const isPhone = () => window.matchMedia('(max-width: 980px)').matches;

const state = {
  plane: 'xy', offset: { xy: 0.40, xz: 0.00, yz: 0.40 }, tool: 'down', rpy: TOOLS.down.slice(), tcp: 0,
  res: isPhone() ? 70 : 90, L: 0.5, limits: true, show: 'best', probe: null, pick: null,
};
let grid = null, job = 0;

// ---------------- controls
const showSel = $('#show');
showSel.innerHTML = [
  ['best', 'Best 1/κ over all branches'], ['count', 'Number of IK solutions'], ['which', 'Which branch is best'],
  ...BRANCHES.map((b, i) => [`b${i}`, `1/κ on ${branchLabel(b)}`]),
].map(([v, t]) => `<option value="${v}">${t}</option>`).join('');
showSel.onchange = () => { state.show = showSel.value; state.pick = null; pose(); };
$('#res').value = state.res; $('#resOut').textContent = state.res;

const offIn = $('#off');
function syncPlaneUI() {
  const P = PLANES[state.plane];
  $$('[data-p]').forEach((b) => b.classList.toggle('on', b.dataset.p === state.plane));
  $('#offName').textContent = `${AX[P.n]} (m)`;
  offIn.min = P.nR[0]; offIn.max = P.nR[1]; offIn.value = state.offset[state.plane];
  $('#offOut').textContent = Number(offIn.value).toFixed(2);
  $('#offNote').textContent = `Plane ${AX[P.u]}${AX[P.v]} at ${AX[P.n]} = ${state.offset[state.plane].toFixed(2)} m. ${P.note}`;
}
$$('[data-p]').forEach((b) => (b.onclick = () => { state.plane = b.dataset.p; state.probe = null; syncPlaneUI(); compute(); }));
offIn.oninput = () => { state.offset[state.plane] = Number(offIn.value); syncPlaneUI(); drawPlane(); };
offIn.onchange = () => { state.probe = null; compute(); };

const rpyInputs = $$('#rpy input');
function syncTool() {
  $$('#toolSeg button').forEach((b) => b.classList.toggle('on', b.dataset.t === state.tool));
  rpyInputs.forEach((inp, k) => { inp.value = state.rpy[k]; inp.nextElementSibling.textContent = `${state.rpy[k]}°`; });
}
$$('#toolSeg button').forEach((b) => (b.onclick = () => {
  state.tool = b.dataset.t;
  if (TOOLS[state.tool]) state.rpy = TOOLS[state.tool].slice();
  syncTool(); compute();
}));
rpyInputs.forEach((inp) => {
  inp.oninput = () => {
    state.rpy[Number(inp.dataset.k)] = Number(inp.value);
    const match = Object.entries(TOOLS).find(([, v]) => v.every((x, i) => x === state.rpy[i]));
    state.tool = match ? match[0] : 'custom';
    syncTool(); pose();
  };
  inp.onchange = () => compute();
});
$('#tcp').oninput = (e) => { state.tcp = Number(e.target.value); $('#tcpOut').textContent = state.tcp.toFixed(2); pose(); };
$('#tcp').onchange = () => compute();
$('#res').oninput = (e) => { $('#resOut').textContent = e.target.value; };
$('#res').onchange = (e) => { state.res = Number(e.target.value); compute(); };
$('#L').oninput = (e) => { $('#Lout').textContent = Number(e.target.value).toFixed(2); };
$('#L').onchange = (e) => { state.L = Number(e.target.value); compute(); };
$('#limits').onchange = (e) => { state.limits = e.target.checked; compute(); };

// ---------------- geometry helpers
const toolR = () => rpy(state.rpy.map(rad));
function pointOf(u, v) {
  const P = PLANES[state.plane], p = [0, 0, 0];
  p[P.u] = u; p[P.v] = v; p[P.n] = state.offset[state.plane];
  return p;
}
// All IK solutions for a TCP position with the current tool, with conditioning
function solveAt(p, R = toolR(), samples = 90) {
  const z = [R[0][2], R[1][2], R[2][2]], l = state.tcp;
  const T = homog(R, [p[0] - l * z[0], p[1] - l * z[1], p[2] - l * z[2]]);
  return solveIK(T, { samples }).map((s) => {
    const c = conditioning(pointJacobian(s.q, p), state.L);
    return { ...s, b: branchIndex(s.branch), invKappa: c.invKappa, sigmaMin: c.sigmaMin, kappa: c.kappa };
  });
}

// ---------------- computation (chunked so the page stays responsive, also on phones)
function compute() {
  const id = ++job, P = PLANES[state.plane];
  const span = P.uR[1] - P.uR[0], step = span / state.res;
  const nu = state.res, nv = Math.round((P.vR[1] - P.vR[0]) / step), cells = nu * nv;
  const g = {
    plane: state.plane, nu, nv, step, P,
    best: new Float32Array(cells).fill(NaN), count: new Uint8Array(cells), which: new Int8Array(cells).fill(-1),
    branch: new Float32Array(cells * 8).fill(NaN),
  };
  const R = toolR();
  $('#progress').hidden = false;
  let b = 0;
  const run = () => {
    if (id !== job) return;
    const t0 = performance.now();
    for (; b < nv && performance.now() - t0 < 28; b++) {
      const v = P.vR[0] + (b + 0.5) * step;
      for (let a = 0; a < nu; a++) {
        const u = P.uR[0] + (a + 0.5) * step, k = b * nu + a;
        const sols = solveAt(pointOf(u, v), R, 64).filter((s) => !state.limits || s.within);
        g.count[k] = sols.length;
        for (const s of sols) {
          if (!(s.invKappa <= g.best[k])) { g.best[k] = s.invKappa; g.which[k] = s.b; }
          const j = s.b * cells + k;
          if (!(s.invKappa <= g.branch[j])) g.branch[j] = s.invKappa;
        }
      }
    }
    $('#progress').textContent = `Solving IK ${Math.round((100 * b) / nv)}%`;
    if (b < nv) return requestAnimationFrame(run);
    grid = g;
    $('#progress').hidden = true;
    summarize();
    if (!state.probe) {
      let bi = -1, bv = -1;
      g.best.forEach((x, i) => { if (x > bv) { bv = x; bi = i; } });
      state.probe = bi >= 0 ? [P.uR[0] + ((bi % nu) + 0.5) * step, P.vR[0] + (Math.floor(bi / nu) + 0.5) * step] : [(P.uR[0] + P.uR[1]) / 2, (P.vR[0] + P.vR[1]) / 2];
      state.pick = null;
    }
    pose();
  };
  requestAnimationFrame(run);
}

function summarize() {
  const g = grid, cells = g.nu * g.nv;
  let reach = 0, bv = 0, bi = 0, tot = 0;
  for (let k = 0; k < cells; k++) { if (g.count[k]) { reach++; tot += g.count[k]; } if (g.best[k] > bv) { bv = g.best[k]; bi = k; } }
  const bu = g.P.uR[0] + ((bi % g.nu) + 0.5) * g.step, bvv = g.P.vR[0] + (Math.floor(bi / g.nu) + 0.5) * g.step;
  const area = reach * g.step * g.step;
  $('#summary').innerHTML = [
    ['reachable area', `${area.toFixed(2)} m²`], ['avg. solutions', reach ? (tot / reach).toFixed(1) : '0'],
    ['best 1/κ', fmt(bv, 3)], [`at (${AX[g.P.u]}, ${AX[g.P.v]})`, `${bu.toFixed(2)}, ${bvv.toFixed(2)}`],
  ].map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
}

// ---------------- drawing
const MAGMA = [[0, [252, 253, 191]], [0.25, [252, 137, 97]], [0.5, [183, 55, 121]], [0.75, [81, 18, 124]], [1, [0, 0, 4]]];
function magma(t) {
  t = Math.max(0, Math.min(1, t));
  for (let i = 1; i < MAGMA.length; i++) if (t <= MAGMA[i][0]) {
    const [t0, c0] = MAGMA[i - 1], [t1, c1] = MAGMA[i], u = (t - t0) / (t1 - t0);
    return c0.map((x, k) => x + u * (c1[k] - x));
  }
  return MAGMA[MAGMA.length - 1][1];
}
const hex = (h) => [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16));
const NONE = [232, 232, 232];
const COUNT_COL = (n) => magma(0.15 + 0.85 * (1 - Math.min(n, 16) / 16));

const canvas = $('#canvas'), ctx = canvas.getContext('2d');
function paint() {
  const r = canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio, 2);
  if (!r.width) return;
  canvas.width = r.width * dpr; canvas.height = r.height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#fff'; ctx.fillRect(0, 0, r.width, r.height);
  if (!grid) return;
  const g = grid, P = g.P, phone = r.width < 600;
  const PAD = { l: phone ? 44 : 58, r: (r.width > 980 ? 290 : 12), t: phone ? 12 : 54, b: phone ? 40 : 96 };
  const uS = P.uR[1] - P.uR[0], vS = P.vR[1] - P.vR[0];
  const W0 = r.width - PAD.l - PAD.r, H0 = r.height - PAD.t - PAD.b, s = Math.min(W0 / uS, H0 / vS);
  const W = uS * s, H = vS * s, x0 = PAD.l + (W0 - W) / 2, y0 = PAD.t + (H0 - H) / 2;
  const X = (u) => x0 + (u - P.uR[0]) * s, Y = (v) => y0 + H - (v - P.vR[0]) * s;
  state.plot = { x0, y0, W, H, s };

  const cells = g.nu * g.nv, img = ctx.createImageData(g.nu, g.nv), show = state.show;
  const bsel = show[0] === 'b' ? Number(show.slice(1)) : -1;
  let maxv = 0;
  const val = (k) => (show === 'best' ? g.best[k] : bsel >= 0 ? g.branch[bsel * cells + k] : NaN);
  for (let k = 0; k < cells; k++) { const x = val(k); if (x > maxv) maxv = x; }
  for (let b = 0; b < g.nv; b++) for (let a = 0; a < g.nu; a++) {
    const k = b * g.nu + a, p = ((g.nv - 1 - b) * g.nu + a) * 4;
    let c;
    if (show === 'count') c = g.count[k] ? COUNT_COL(g.count[k]) : NONE;
    else if (show === 'which') c = g.which[k] >= 0 ? hex(BR_COLORS[g.which[k]]) : NONE;
    else { const x = val(k); c = Number.isFinite(x) ? magma(1 - x / (maxv || 1)) : NONE; }
    img.data[p] = c[0]; img.data[p + 1] = c[1]; img.data[p + 2] = c[2]; img.data[p + 3] = 255;
  }
  const off = document.createElement('canvas'); off.width = g.nu; off.height = g.nv; off.getContext('2d').putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = !(show === 'count' || show === 'which');
  ctx.drawImage(off, x0, y0, W, H);

  // base axis / floor references
  ctx.strokeStyle = 'rgba(20,20,20,.55)'; ctx.lineWidth = 1; ctx.setLineDash([5, 4]);
  ctx.beginPath(); ctx.moveTo(X(0), y0); ctx.lineTo(X(0), y0 + H); ctx.moveTo(x0, Y(0)); ctx.lineTo(x0 + W, Y(0)); ctx.stroke();
  ctx.setLineDash([]);
  ctx.font = '600 10.5px Inter, Arial'; ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,255,.9)'; ctx.fillStyle = '#141414';
  const tag = (t, x, y) => { ctx.strokeText(t, x, y); ctx.fillText(t, x, y); };
  if (state.plane === 'xy') tag('base axis', X(0) + 6, Y(0) - 6);
  else { tag('base axis', X(0) + 5, y0 + 12); tag('z = 0 (base plane)', x0 + 6, Y(0) - 5); }

  // probe
  if (state.probe) {
    ctx.beginPath(); ctx.arc(X(state.probe[0]), Y(state.probe[1]), 6.5, 0, 2 * Math.PI);
    ctx.strokeStyle = '#00e0ff'; ctx.lineWidth = 2.4; ctx.stroke();
  }
  // axes and ticks every 0.25 m
  ctx.strokeStyle = '#141414'; ctx.lineWidth = 1; ctx.strokeRect(x0, y0, W, H);
  ctx.fillStyle = '#141414'; ctx.font = `${phone ? 10 : 11}px Inter, Arial`; ctx.textAlign = 'center';
  const tick = phone ? 0.5 : 0.25;
  for (let u = Math.ceil(P.uR[0] / tick) * tick; u <= P.uR[1] + 1e-9; u += tick) ctx.fillText(u.toFixed(2).replace('-0.00', '0.00'), X(u), y0 + H + 14);
  ctx.fillText(`${AX[P.u]} (m)`, x0 + W / 2, y0 + H + (phone ? 30 : 32));
  ctx.textAlign = 'right';
  for (let v = Math.ceil(P.vR[0] / tick) * tick; v <= P.vR[1] + 1e-9; v += tick) ctx.fillText(v.toFixed(2).replace('-0.00', '0.00'), x0 - 5, Y(v) + 4);
  ctx.save(); ctx.translate(12, y0 + H / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText(`${AX[P.v]} (m)`, 0, 0); ctx.restore();
  ctx.textAlign = 'left';
  legend(maxv);
}

function legend(maxv) {
  const P = PLANES[state.plane], where = `${AX[P.u]}${AX[P.v]} plane, ${AX[P.n]} = ${state.offset[state.plane].toFixed(2)} m · tool ${state.tool === 'custom' ? `rpy (${state.rpy.join('°, ')}°)` : $(`#toolSeg [data-t="${state.tool}"]`).textContent} · ℓ = ${state.tcp.toFixed(2)} m`;
  let body;
  if (state.show === 'count') body = '<span>colour: number of valid IK solutions (darker = more)</span>';
  else if (state.show === 'which') body = BRANCHES.map((b, i) => `<span><span class="dot" style="background:${BR_COLORS[i]}"></span>${branchLabel(b)}</span>`).join('');
  else body = `<span>1/κ: dark = near-singular, light = well conditioned (max ${fmt(maxv, 3)})</span>`;
  $('#legend').innerHTML = `<span><b>${where}</b></span>${body}<span><span class="dot" style="background:#e8e8e8;border:1px solid #bbb"></span>no valid IK</span><span>○ probe</span>`;
}

// ---------------- 3D inset: robot, the working plane, probe and tool
const inset = new RobotViewer($('#inset'));
inset.camera.position.set(2.2, -2.2, 1.6);
inset.load().then(() => { inset.setOpacity(1); pose(); });

function drawPlane() {
  inset.clearOverlay();
  const P = PLANES[state.plane];
  const corners = [[P.uR[0], P.vR[0]], [P.uR[1], P.vR[0]], [P.uR[1], P.vR[1]], [P.uR[0], P.vR[1]]].map(([u, v]) => pointOf(u, v));
  const geo = new THREE.BufferGeometry();
  const tri = [0, 1, 2, 0, 2, 3].flatMap((i) => corners[i]);
  geo.setAttribute('position', new THREE.Float32BufferAttribute(tri, 3));
  inset.overlay.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.1, side: THREE.DoubleSide, depthWrite: false })));
  inset.addPolyline(corners, { color: 0xff0000, closed: true });
  if (state.probe) {
    const p = pointOf(...state.probe);
    inset.addPoint(p, { color: 0x00c8e0, radius: 0.018 });
    const R = toolR(), z = [R[0][2], R[1][2], R[2][2]];
    inset.addArrow(p, z, { color: 0x141414, scale: 0.15, radius: 0.004 });
  }
  inset.render();
}

let current = [];
function pose() {
  drawPlane();
  if (!state.probe) return paint();
  const p = pointOf(...state.probe);
  const all = solveAt(p).sort((a, b) => a.b - b.b || b.invKappa - a.invKappa);
  current = all;
  const valid = all.filter((s) => s.within);
  const bsel = state.show[0] === 'b' ? Number(state.show.slice(1)) : -1;
  if (state.pick === null || !all[state.pick]) {
    const pool = (bsel >= 0 ? valid.filter((s) => s.b === bsel) : valid);
    const best = (pool.length ? pool : valid.length ? valid : all).reduce((m, s) => (!m || s.invKappa > m.invKappa ? s : m), null);
    state.pick = best ? all.indexOf(best) : null;
  }
  const sel = state.pick !== null ? all[state.pick] : null;
  if (sel) inset.setQ(sel.q);
  inset.setRobotVisible(!!sel);
  if (sel && state.tcp > 0) { const F = poeFK(sel.q); inset.addArrow(posOf(F), [F[0][2], F[1][2], F[2][2]], { color: 0xd4a017, scale: state.tcp, radius: 0.006 }); }
  const P = PLANES[state.plane];
  $('#insetCap').textContent = sel ? `${AX[P.u]} = ${state.probe[0].toFixed(2)}, ${AX[P.v]} = ${state.probe[1].toFixed(2)} · ${sel.label}` : 'No IK solution at this point';
  $('#probeTiles').innerHTML = [
    ['TCP (m)', p.map((x) => x.toFixed(2)).join(', ')], ['solutions', `${valid.length} valid / ${all.length}`],
    ['best 1/κ', valid.length ? fmt(Math.max(...valid.map((s) => s.invKappa)), 3) : '—'], ['shown', sel ? fmt(sel.invKappa, 3) : '—'],
  ].map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
  $('#solTable').innerHTML = all.length ? `<table class="sols"><thead><tr><th>Branch</th>${[1, 2, 3, 4, 5, 6].map((i) => `<th>q${'₁₂₃₄₅₆'[i - 1]}</th>`).join('')}<th>1/κ</th></tr></thead><tbody>${
    all.map((s, i) => `<tr data-i="${i}" class="${i === state.pick ? 'sel' : ''} ${s.within ? '' : 'out'}"><td><span class="dot" style="background:${BR_COLORS[s.b]}"></span>${s.label}</td>${s.q.map((x) => `<td>${deg(x).toFixed(1)}</td>`).join('')}<td>${fmt(s.invKappa, 3)}</td></tr>`).join('')
  }</tbody></table>` : '<p class="note">No IK solution: the point is out of reach for this tool orientation and length.</p>';
  $$('#solTable tr[data-i]').forEach((tr) => (tr.onclick = () => { state.pick = Number(tr.dataset.i); pose(); }));
  $('#probeNote').innerHTML = all.some((s) => !s.within) ? 'Struck-through rows violate a joint limit (they are excluded from the map when the limit filter is on).' : '';
  paint();
}

// ---------------- probing: tap anywhere, drag with a mouse (touch drags scroll the page)
let dragging = false;
const probeAt = (ev) => {
  const P = PLANES[state.plane], S = state.plot;
  if (!S) return;
  const r = canvas.getBoundingClientRect(), x = ev.clientX - r.left, y = ev.clientY - r.top;
  if (x < S.x0 || x > S.x0 + S.W || y < S.y0 || y > S.y0 + S.H) return;
  state.probe = [P.uR[0] + (x - S.x0) / S.s, P.vR[0] + (S.y0 + S.H - y) / S.s];
  state.pick = null;
  pose();
};
canvas.addEventListener('pointerdown', (e) => { dragging = e.pointerType === 'mouse'; probeAt(e); });
canvas.addEventListener('pointermove', (e) => dragging && probeAt(e));
window.addEventListener('pointerup', () => { dragging = false; });
new ResizeObserver(() => paint()).observe($('#heat'));

syncPlaneUI();
syncTool();
compute();
renderMath();
