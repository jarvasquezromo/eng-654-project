// UI helpers shared by steps 5–7: drawing the contour and placement grid, branch colours, start lists.
import { THREE } from './viewer.js';
import { contourPoses, centerOf, gridCenters, ORIENTATIONS } from './tasks.js';
import { branchLabel, BRANCHES } from './ik.js';

export const BR_COLORS = ['#2f6fe4', '#7aa7f5', '#ff0000', '#ff8a80', '#1f8a4c', '#8fd19e', '#d4a017', '#f2d27a'];
export const TRACK_COLORS = ['#2f6fe4', '#ff0000', '#1f8a4c', '#d4a017', '#9b51e0', '#00a3c4', '#e8590c', '#7a7a7a', '#c2185b', '#5c6bc0', '#43a047', '#8d6e63', '#26a69a', '#f06292', '#455a64', '#b8860b'];
export const REASON_COLORS = { ok: '#1f8a4c', noik: '#bdbdbd', limit: '#e8590c', jump: '#2f6fe4', margin: '#9b51e0' };
export const branchName = (b) => branchLabel(BRANCHES[b]);
export const orientName = (k) => ORIENTATIONS[k].name;

// Contour polyline, approach arrows, and optionally the grid of placement centres
export function drawTask(viewer, S, xy, orient, { grid = true, arrows = 8, current = null } = {}) {
  const poses = contourPoses(S, centerOf(S, xy), orient);
  const pts = poses.map((T) => [T[0][3], T[1][3], T[2][3]]);
  viewer.addPolyline(pts, { color: 0xff0000, closed: true });
  const every = Math.max(1, Math.round(S.N / arrows));
  poses.forEach((T, k) => {
    if (k % every) return;
    viewer.addArrow(pts[k], [T[0][2], T[1][2], T[2][2]], { color: 0x141414, scale: 0.07, radius: 0.0035 });
  });
  viewer.addPoint(pts[0], { color: 0xff0000, radius: 0.011, label: 's = 0' });
  if (current !== null && pts[current]) viewer.addPoint(pts[current], { color: 0x00c8e0, radius: 0.014 });
  if (grid) {
    const g = gridCenters(S), arr = new Float32Array(g.length * 3);
    g.forEach((c, i) => { arr[3 * i] = c[0]; arr[3 * i + 1] = c[1]; arr[3 * i + 2] = S.z; });
    const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.BufferAttribute(arr, 3));
    viewer.overlay.add(new THREE.Points(geo, new THREE.PointsMaterial({ color: 0x8a8a8a, size: 4, sizeAttenuation: false })));
    const [x0, x1] = S.xRange, [y0, y1] = S.yRange;
    viewer.addPolyline([[x0, y0, S.z], [x1, y0, S.z], [x1, y1, S.z], [x0, y1, S.z]], { color: 0x8a8a8a, closed: true });
  }
  viewer.render();
  return poses;
}

// <select> of orientation presets
export function orientOptions(sel, value) {
  sel.innerHTML = Object.entries(ORIENTATIONS).map(([k, o]) => `<option value="${k}" ${k === value ? 'selected' : ''}>${o.name} · rpy (${o.rpy.join('°, ')}°)</option>`).join('');
}

// Bind range inputs with data-s="key" (or "key.0") to a settings object; onInput live, onChange committed.
export function bindSettings(root, S, { onInput = () => {}, onChange = () => {} } = {}) {
  root.querySelectorAll('[data-s]').forEach((inp) => {
    const [key, idx] = inp.dataset.s.split('.');
    const get = () => (idx === undefined ? S[key] : S[key][Number(idx)]);
    const set = (v) => { if (idx === undefined) S[key] = v; else { S[key] = S[key].slice(); S[key][Number(idx)] = v; } };
    const out = inp.parentElement.querySelector('output');
    const digits = Number(inp.dataset.d ?? 2);
    const show = () => { if (out) out.textContent = Number(get()).toFixed(digits) + (inp.dataset.u || ''); };
    if (inp.tagName === 'SELECT') { inp.value = get(); inp.onchange = () => { set(inp.value); onChange(key); }; return; }
    inp.value = get(); show();
    inp.oninput = () => { set(Number(inp.value)); show(); onInput(key); };
    inp.onchange = () => onChange(key);
  });
}
