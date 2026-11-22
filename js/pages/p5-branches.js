// Step 5: validate all IK candidates along the contour and organise them into persistent branches.
import { RobotViewer } from '../viewer.js';
import { mountChrome, mountTabs, renderMath, $, $$ } from '../ui.js';
import { loadSettings, contourPoses, candidatesAlong, centerOf, trackBranches, representatives, wrap } from '../tasks.js';
import { drawTask, branchName, TRACK_COLORS, orientName } from '../taskui.js';
import { poeFK, poseError, LIMITS } from '../gofa.js';
import { homog, quatToR, deg, rad, fmt, fmtSci } from '../math.js';
import { linePlot } from '../plot.js';

mountChrome();
mountTabs();

const S = loadSettings();
const state = { which: 'A', k: 0, joint: 0, sel: 0, cands: [], branches: [], playing: false };
const orient = () => (state.which === 'A' ? S.orientA : S.orientB);
const viewer = new RobotViewer($('#view'));
viewer.view('iso');

$('#jointSeg').innerHTML = [1, 2, 3, 4, 5, 6].map((i) => `<button data-j="${i - 1}" class="${i === 1 ? 'on' : ''}">q${'₁₂₃₄₅₆'[i - 1]}</button>`).join('') + '<button data-j="k">1/κ</button>';
$$('#jointSeg button').forEach((b) => (b.onclick = () => { state.joint = b.dataset.j === 'k' ? 'k' : Number(b.dataset.j); $$('#jointSeg button').forEach((x) => x.classList.toggle('on', x === b)); plots(); }));

// unwrap a branch so its curve is continuous (degrees)
const unwrapped = (br) => { const out = [br.q[0].slice()]; for (let i = 1; i < br.q.length; i++) out.push(out[i - 1].map((x, j) => x + wrap(br.q[i][j] - x))); return out; };

function compute() {
  const poses = contourPoses(S, centerOf(S, S.center), orient());
  state.poses = poses;
  state.cands = candidatesAlong(S, poses);
  state.branches = trackBranches(state.cands, { gate: 1.5 * rad(S.dqMax), closed: true });
  state.branches.forEach((br) => {
    br.uq = unwrapped(br);
    br.cs = br.idx.map((j, n) => state.cands[br.start + n][j]);
    br.within = br.cs.filter((c) => c.within).length / br.cs.length;
    br.minK = Math.min(...br.cs.map((c) => (c.within ? c.invKappa : Infinity)));
    br.labels = [...new Set(br.cs.map((c) => c.b))];
    br.full = br.start === 0 && br.end === S.N - 1;
  });
  state.sel = Math.max(0, state.branches.findIndex((b) => b.full));
  $('#kSlider').max = S.N - 1;
  $('#legend').innerHTML = `<span><b>${orientName(orient())}</b> · centre (${S.center.map((x) => x.toFixed(2)).join(', ')}, ${S.z.toFixed(2)}) m</span><span><span class="dot" style="background:#ff0000"></span>contour</span><span><span class="dot" style="background:#00c8e0"></span>sample k</span>`;
  tiles(); plots(); table(); validate(); draw();
}

function tiles() {
  const B = state.branches, n = B.length;
  const full = B.filter((b) => b.full).length;
  const perm = B.filter((b) => b.full && b.closesTo !== undefined && b.closesTo !== null && b.closesTo !== b.id).length;
  $('#tiles').innerHTML = [
    ['branches', n], ['span the loop', full], ['end early', B.filter((b) => b.start === 0 && !b.full).length], ['born mid-loop', B.filter((b) => b.start > 0).length], ['return on another branch', perm],
  ].map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
}

function plots() {
  const B = state.branches, j = state.joint;
  const series = B.map((br, i) => {
    const y = new Array(S.N).fill(NaN);
    br.uq.forEach((q, n) => { y[br.start + n] = j === 'k' ? br.cs[n].invKappa : deg(q[j]); });
    return { y, color: TRACK_COLORS[i % TRACK_COLORS.length], width: i === state.sel ? 3.2 : 1.3, alpha: i === state.sel ? 1 : 0.65 };
  });
  series.push(series.splice(state.sel, 1)[0]);
  const lim = j === 'k' ? [] : [{ y: deg(LIMITS[j][0]), color: '#999' }, { y: deg(LIMITS[j][1]), color: '#999' }];
  const vals = series.flatMap((x) => x.y.filter(Number.isFinite));
  const yRange = j === 'k' ? [0, Math.max(0.3, ...vals)] : [Math.min(deg(LIMITS[j][0]), ...vals) - 10, Math.max(deg(LIMITS[j][1]), ...vals) + 10];
  linePlot($('#jPlot'), {
    series, xRange: [0, S.N - 1], yRange, vlines: [{ x: state.k, color: '#00a3c4' }], hlines: j === 'k' ? [{ y: S.margin, color: '#9b51e0' }] : lim,
    xLabel: 'sample k', yLabel: j === 'k' ? '1/κ' : `q${j + 1} (°)`, height: 190,
  });
  linePlot($('#cPlot'), {
    series: [{ y: state.cands.map((c) => c.length), color: '#141414', points: true }, { y: state.cands.map((c) => c.filter((x) => x.within).length), color: '#ff0000', points: true }],
    xRange: [0, S.N - 1], yRange: [0, Math.max(...state.cands.map((c) => c.length), 1) + 1], vlines: [{ x: state.k, color: '#00a3c4' }],
    xLabel: 'sample k · black: all candidates · red: within limits', yLabel: '# IK', height: 130,
  });
}

function table() {
  $('#brTable').innerHTML = `<table class="starts"><thead><tr><th>#</th><th>alive k</th><th>label(s)</th><th>in limits</th><th>min 1/κ</th><th>after one loop</th></tr></thead><tbody>${
    state.branches.map((br, i) => {
      const end = br.full ? (br.closesTo === br.id ? 'itself' : br.closesTo === null || br.closesTo === undefined ? 'lost' : `#${br.closesTo + 1}`) : br.end < S.N - 1 ? `ends at ${br.end}` : '—';
      return `<tr data-i="${i}" class="${i === state.sel ? 'sel' : ''}"><td><span class="dot" style="background:${TRACK_COLORS[i % TRACK_COLORS.length]}"></span>${i + 1}</td><td>${br.start}–${br.end}</td><td>${br.labels.map(branchName).join('<br>')}</td><td>${Math.round(100 * br.within)}%</td><td>${Number.isFinite(br.minK) ? fmt(br.minK, 3) : '—'}</td><td>${end}</td></tr>`;
    }).join('')
  }</tbody></table>`;
  $$('#brTable tr[data-i]').forEach((tr) => (tr.onclick = () => { state.sel = Number(tr.dataset.i); table(); plots(); draw(); }));
}

function validate() {
  const all = state.cands.flat();
  const q6two = all.filter((c) => representatives(c.q).length === 2).length;
  const out = all.filter((c) => !c.within).length;
  $('#valTiles').innerHTML = [
    ['candidates', all.length], ['max position residual', `${fmtSci(Math.max(...all.map((c) => c.position_error)))} m`],
    ['max rotation residual', fmtSci(Math.max(...all.map((c) => c.rotation_error)))], ['outside limits', out], ['two q₆ representatives', q6two],
  ].map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
  $('#valTable').innerHTML = `<table class="starts"><thead><tr><th>k</th><th>#IK</th><th>in limits</th><th>max residual</th><th>min 1/κ (in limits)</th></tr></thead><tbody>${
    state.cands.map((C, k) => `<tr><td>${k}</td><td>${C.length}</td><td>${C.filter((c) => c.within).length}</td><td>${C.length ? fmtSci(Math.max(...C.map((c) => Math.max(c.position_error, c.rotation_error)))) : '—'}</td><td>${C.some((c) => c.within) ? fmt(Math.min(...C.filter((c) => c.within).map((c) => c.invKappa)), 3) : '—'}</td></tr>`).join('')
  }</tbody></table>`;
}

function draw() {
  viewer.clearOverlay();
  drawTask(viewer, S, S.center, orient(), { grid: false, current: state.k });
  const br = state.branches[state.sel];
  const n = br ? state.k - br.start : -1;
  if (br && n >= 0 && n < br.uq.length) { viewer.setRobotVisible(true); viewer.setQ(br.uq[n]); } else viewer.setRobotVisible(false);
}

// supplied solver example, verified with our FK
fetch('../data/ik-example.json').then((r) => r.json()).then((ex) => {
  const target = homog(quatToR([0.7071067811865476, 0, 0.7071067811865476, 0]), [0.571, 0, 0.899]);
  const rows = ex.solutions.map((s, i) => {
    const e = poseError(poeFK(s.q), target);
    const within = s.q.every((x, j) => x >= LIMITS[j][0] - 1e-9 && x <= LIMITS[j][1] + 1e-9);
    return `<tr><td>${i + 1}</td>${s.q.map((x) => `<td>${deg(x).toFixed(1)}</td>`).join('')}<td>${fmtSci(Math.max(e.position, e.rotation))}</td><td>${within === s.within_limits ? '✓' : '✗'} ${within ? 'in' : 'out'}</td><td>${s.radial_branch}/${s.j4_branch}</td></tr>`;
  });
  $('#supTable').innerHTML = `<table class="starts"><thead><tr><th>#</th><th>q₁</th><th>q₂</th><th>q₃</th><th>q₄</th><th>q₅</th><th>q₆</th><th>our FK err</th><th>limits (agrees)</th><th>radial/j4</th></tr></thead><tbody>${rows.join('')}</tbody></table>`;
  $('#supNote').textContent = `${ex.solutions.length} solutions, ${ex.diagnostics.real_roots} real polynomial roots of ${ex.diagnostics.polynomial_roots}. Our FK error is the max of position (m) and rotation residual; "agrees" compares our limit check with the solver's within_limits flag.`;
}).catch(() => { $('#supNote').textContent = 'Could not load data/ik-example.json.'; });

$('#kSlider').oninput = (e) => { state.k = Number(e.target.value); $('#kOut').textContent = state.k; plots(); draw(); };
$('#play').onclick = () => {
  state.playing = !state.playing; $('#play').textContent = state.playing ? '❚❚ Pause' : '▶ Play';
  const tick = () => {
    if (!state.playing) return;
    state.k = (state.k + 1) % S.N; $('#kSlider').value = state.k; $('#kOut').textContent = state.k; plots(); draw();
    setTimeout(() => requestAnimationFrame(tick), 90);
  };
  tick();
};
$$('#orientBtns button').forEach((b) => (b.onclick = () => { state.which = b.dataset.o; $$('#orientBtns button').forEach((x) => x.classList.toggle('on', x === b)); compute(); }));
$$('#views button').forEach((b) => (b.onclick = () => viewer.view(b.dataset.v)));

viewer.load().then(() => { $('#status').hidden = true; viewer.setOpacity(1); compute(); });
renderMath();
