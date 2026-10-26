import { RobotViewer, loadModel } from '../viewer.js';
import { mountChrome, mountTabs, jointSliders, matrixTable, renderMath, $, $$ } from '../ui.js';
import { DH, SCREW_AXES, M_HOME, dhFK, urdfFK, currentAxes, REFERENCE_Q, LIMITS, chainTo } from '../gofa.js';
import { deg, fmt, posOf, rotOf, rToQuat } from '../math.js';

mountChrome();
mountTabs();

const viewer = new RobotViewer($('#view'));
const layers = { urdf: false, dh: true, screw: true, ghost: false };
let sliders;

function draw(q) {
  viewer.setQ(q);
  viewer.clearOverlay();
  if (layers.dh) dhFK(q).forEach((T, i) => viewer.addFrame(T, { size: 0.11, label: `{${i}}`, lineWidth: 0.0045 }));
  if (layers.urdf && viewer.model) {
    urdfFK(viewer.model, q).jointFrames.forEach((T, i) => viewer.addFrame(T, { size: 0.06, label: `J${i + 1}`, lineWidth: 0.0025, opacity: 0.8 }));
  }
  if (layers.screw) currentAxes(q).forEach(({ w, r }) => viewer.addAxisLine(r, w, { length: 0.55 }));
  const T = dhFK(q)[6], p = posOf(T), quat = rToQuat(rotOf(T));
  $('#toolpose').innerHTML = `<table><tr><th>p (m)</th><td>${p.map((x) => fmt(x, 4)).join(', ')}</td></tr>
    <tr><th>quat [w x y z]</th><td>${quat.map((x) => fmt(x, 4)).join(', ')}</td></tr></table>`;
}

(async () => {
  await viewer.load();
  $('#status').hidden = true;
  viewer.setOpacity(1);
  sliders = jointSliders($('#sliders'), REFERENCE_Q, draw);
  draw(REFERENCE_Q);
  $('#home').onclick = () => sliders.set([0, 0, 0, 0, 0, 0]);
  $('#ref').onclick = () => sliders.set(REFERENCE_Q);
  $('#rand').onclick = () => sliders.set(LIMITS.map(([a, b]) => a + Math.random() * (b - a)));

  // URDF table straight from the parsed file
  const { model } = await loadModel();
  const rev = chainTo(model).filter((j) => j.type === 'revolute');
  $('#urdfTable').innerHTML = `<table><thead><tr><th>joint</th><th>parent → child</th><th>origin xyz (m)</th><th>rpy</th><th>axis</th><th>limits (°)</th></tr></thead><tbody>${
    rev.map((j) => `<tr><td>${j.name}</td><td>${j.parent.replace('link_', 'L')} → ${j.child.replace('link_', 'L')}</td><td>${j.xyz.map((x) => x.toFixed(3)).join(' ')}</td><td>${j.rpy.join(' ')}</td><td>${j.axis.join(' ')}</td><td>${deg(j.lower).toFixed(0)} … ${deg(j.upper).toFixed(0)}</td></tr>`).join('')
  }</tbody></table>`;
})();

$$('#views button').forEach((b) => (b.onclick = () => viewer.view(b.dataset.v)));
$$('#layers button').forEach((b) => (b.onclick = () => {
  const k = b.dataset.l; layers[k] = !layers[k]; b.classList.toggle('on', layers[k]);
  if (k === 'ghost') viewer.setOpacity(layers.ghost ? 0.25 : 1);
  draw(sliders?.get() ?? REFERENCE_Q);
}));

// D–H table
const piStr = (x) => {
  const r = x / Math.PI;
  if (Math.abs(x) < 1e-12) return '0';
  const map = { 1: 'π', '-1': '−π', 0.5: 'π/2', '-0.5': '−π/2' };
  return map[String(Math.round(r * 1e6) / 1e6)] ?? fmt(x, 4);
};
$('#dhTable').innerHTML = `<table><thead><tr><th>i</th><th>θ<sub>i,0</sub></th><th>d<sub>i</sub> (m)</th><th>a<sub>i</sub> (m)</th><th>α<sub>i</sub></th></tr></thead><tbody>${
  DH.map((r, i) => `<tr><td>${i + 1}</td><td>${piStr(r.theta0)}</td><td>${r.d.toFixed(3)}</td><td>${r.a.toFixed(3)}</td><td>${piStr(r.alpha)}</td></tr>`).join('')
}</tbody></table>`;
$('#dhWhy').innerHTML = `<ol style="padding-left:18px;margin:0">
  <li>Axes 1 (z) and 2 (y) intersect 0.265 m up: \\(d_1\\), and \\(\\alpha_1=-\\pi/2\\) turns z into y.</li>
  <li>Axes 2 and 3 are parallel, 0.444 m apart along z: \\(a_2\\). \\(x_2\\) points up, so \\(\\theta_{2,0}=-\\pi/2\\).</li>
  <li>Axes 3 (y) and 4 (x) are skew, 0.110 m apart: \\(a_3\\), \\(\\alpha_3=-\\pi/2\\).</li>
  <li>Axes 4 and 5 intersect 0.470 m along axis 4: \\(d_4\\).</li>
  <li>Axes 5 (y) and 6 (x) are skew, 0.080 m apart: \\(a_5\\). This is the wrist offset.</li>
  <li>0.101 m along axis 6 reaches tool0: \\(d_6\\); \\(\\theta_{6,0}=\\pi\\) aligns \\(x_6\\) with tool0's x.</li></ol>`;

// Screw table and M
$('#screwTable').innerHTML = `<table><thead><tr><th>i</th><th>ω<sub>i</sub></th><th>r<sub>i</sub> (m)</th><th>v<sub>i</sub> = −ω×r (m)</th></tr></thead><tbody>${
  SCREW_AXES.map((s, i) => `<tr><td>${i + 1}</td><td>${s.w.join(' ')}</td><td>${s.r.map((x) => x.toFixed(3)).join(' ')}</td><td>${s.v.map((x) => fmt(x, 3)).join(' ')}</td></tr>`).join('')
}</tbody></table>`;
$('#mTable').innerHTML = matrixTable(M_HOME, { digits: 3 });

renderMath();
