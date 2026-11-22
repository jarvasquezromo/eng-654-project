// Step 6: the closed full-pose contour and its 2-D placement family.
import { RobotViewer } from '../viewer.js';
import { mountChrome, mountTabs, renderMath, $, $$ } from '../ui.js';
import { loadSettings, saveSettings, contourPoses, candidatesAlong, centerOf, representatives, nearestContinuation, branchAwareFeasible, REASON_TEXT } from '../tasks.js';
import { drawTask, orientOptions, bindSettings, branchName, BR_COLORS, REASON_COLORS } from '../taskui.js';
import { linePlot } from '../plot.js';
import { fmt, fmtSci, deg } from '../math.js';

mountChrome();
mountTabs();

const S = loadSettings();
const state = { which: 'A', pick: 0, k: 0, poses: [], cands: [], starts: [], path: null, playing: false };
const orient = () => (state.which === 'A' ? S.orientA : S.orientB);

const viewer = new RobotViewer($('#view'));
viewer.view('iso');
orientOptions($('#oA'), S.orientA);
orientOptions($('#oB'), S.orientB);

function draw() {
  viewer.clearOverlay();
  drawTask(viewer, S, S.center, orient(), { current: state.k % S.N });
  const p = state.path?.[state.k];
  if (p) { viewer.setRobotVisible(true); viewer.setQ(p.q); } else viewer.setRobotVisible(!!state.path?.length);
}

// Evaluate the current placement: candidates, start configurations and their outcome
function evaluate() {
  saveSettings(S);
  state.poses = contourPoses(S, centerOf(S, S.center), orient());
  state.cands = candidatesAlong(S, state.poses);
  const counts = state.cands.map((c) => c.length);
  const aware = counts.every((n) => n > 0) ? branchAwareFeasible(S, state.cands) : [];
  state.starts = [];
  for (const c of state.cands[0]) {
    const reps = representatives(c.q);
    for (const q0 of (reps.length ? reps : [c.q])) {
      let r;
      if (!reps.length) r = { ok: false, reason: 'limit', at: 0, path: [{ q: q0, invKappa: c.invKappa, step: 0 }] };
      else r = nearestContinuation(S, state.cands, state.poses, q0, { keepPath: true });
      if (reps.length && !(c.invKappa >= S.margin) && r.ok) r = { ...r, ok: false, reason: 'margin', at: 0 };
      const aw = aware.find((a) => Math.max(...a.q.map((x, i) => Math.abs(x - q0[i]))) < 1e-9);
      state.starts.push({ c, q0, r, aware: !!aw?.ok });
    }
  }
  if (state.pick >= state.starts.length) state.pick = 0;
  const best = state.starts.findIndex((s) => s.r.ok);
  if (!state.starts[state.pick]?.r.ok && best >= 0) state.pick = best;
  report(counts);
  selectStart(state.pick);
}

function report(counts) {
  const nOk = state.starts.filter((s) => s.r.ok).length, nAw = state.starts.filter((s) => s.aware).length;
  $('#checkTiles').innerHTML = [
    ['orientation', state.which], ['min #IK on contour', Math.min(...counts)],
    ['starts at s = 0', state.starts.length], ['nearest feasible', `${nOk}`], ['branch-aware feasible', `${nAw}`],
    ['max FK residual', fmtSci(Math.max(0, ...state.cands.flat().map((c) => Math.max(c.position_error, c.rotation_error))))],
  ].map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
  linePlot($('#countPlot'), {
    series: [{ y: counts, color: '#141414', points: true }, { y: state.cands.map((c) => c.filter((x) => x.within).length), color: '#ff0000', points: true }],
    xLabel: 'sample k (black: all solutions · red: within limits)', yLabel: '# IK', yRange: [0, Math.max(...counts, 1) + 1], height: 150,
  });
  $('#starts').innerHTML = state.starts.length ? `<table class="starts"><thead><tr><th>#</th><th>Branch at s = 0</th><th>q₆ (°)</th><th>1/κ</th><th>Nearest</th><th>Aware</th></tr></thead><tbody>${
    state.starts.map((s, i) => `<tr data-i="${i}" class="${i === state.pick ? 'sel' : ''}"><td>${i + 1}</td><td><span class="dot" style="background:${BR_COLORS[s.c.b]}"></span>${branchName(s.c.b)}</td><td>${deg(s.q0[5]).toFixed(0)}</td><td>${fmt(s.c.invKappa, 3)}</td><td><span class="tag" style="background:${REASON_COLORS[s.r.reason]}" title="${REASON_TEXT[s.r.reason]}">${s.r.ok ? 'ok' : `${s.r.reason} @${s.r.at}`}</span></td><td>${s.aware ? '✓' : '—'}</td></tr>`).join('')
  }</tbody></table>` : '<p class="note">No IK solution at s = 0 for this placement.</p>';
  $$('#starts tr[data-i]').forEach((tr) => (tr.onclick = () => selectStart(Number(tr.dataset.i))));
}

function selectStart(i) {
  state.pick = i;
  $$('#starts tr[data-i]').forEach((tr) => tr.classList.toggle('sel', Number(tr.dataset.i) === i));
  const s = state.starts[i];
  state.path = s?.r.path || null;
  const n = state.path ? state.path.length - 1 : 0;
  $('#kSlider').max = n; state.k = Math.min(state.k, n); $('#kSlider').value = state.k; $('#kOut').textContent = state.k;
  if (state.path) {
    linePlot($('#kPlot'), {
      series: [{ y: state.path.map((p) => p.invKappa), color: '#9b51e0' }, { y: state.path.map((p) => p.step / ((S.dqMax * Math.PI) / 180) * 0.25), color: '#2f6fe4', dash: [4, 3] }],
      xRange: [0, S.N], yRange: [0, 0.3], hlines: [{ y: S.margin, color: '#9b51e0' }, { y: 0.25, color: '#2f6fe4' }],
      xLabel: 'k · purple 1/κ · blue dashed step / Δq_max × ¼', yLabel: '', height: 150,
    });
  }
  draw();
}

$('#kSlider').oninput = (e) => { state.k = Number(e.target.value); $('#kOut').textContent = state.k; draw(); };
$('#play').onclick = () => {
  state.playing = !state.playing; $('#play').textContent = state.playing ? '❚❚ Pause' : '▶ Play';
  const tick = () => {
    if (!state.playing) return;
    const n = state.path ? state.path.length : 1;
    state.k = (state.k + 1) % n; $('#kSlider').value = state.k; $('#kOut').textContent = state.k; draw();
    setTimeout(() => requestAnimationFrame(tick), 70);
  };
  tick();
};
$$('#orientBtns button').forEach((b) => (b.onclick = () => {
  state.which = b.dataset.o; $$('#orientBtns button').forEach((x) => x.classList.toggle('on', x === b)); evaluate();
}));
$$('#views button').forEach((b) => (b.onclick = () => viewer.view(b.dataset.v)));

let timer = null;
const later = () => { clearTimeout(timer); timer = setTimeout(evaluate, 120); };
bindSettings(document.body, S, { onInput: () => { saveSettings(S); draw(); }, onChange: later });

viewer.load().then(() => { $('#status').hidden = true; viewer.setOpacity(1); evaluate(); });
renderMath();
