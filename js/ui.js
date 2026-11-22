// Shared page chrome: header, step navigation, tabs, joint sliders, tables, KaTeX.
import { LIMITS, JOINT_NAMES } from './gofa.js';
import { deg, fmt } from './math.js';

export const STEPS = [
  { n: 1, file: '01-model.html', title: 'Kinematic model', short: 'URDF → D–H & screws' },
  { n: 2, file: '02-fk.html', title: 'FK validation', short: 'URDF vs D–H vs PoE' },
  { n: 3, file: '03-jacobian.html', title: 'Jacobian', short: 'Screw J and ³J₅' },
  { n: 4, file: '04-slice.html', title: 'Conditioning slices', short: '(q₂, q₃) and xy · xz · yz planes' },
  { n: 5, file: '05-ik-branches.html', title: 'IK branches', short: 'Validate & track' },
  { n: 6, file: '06-placement.html', title: 'Contour & placement', short: 'Task family' },
  { n: 7, file: '07-feasibility.html', title: 'Feasibility maps', short: 'Per branch & failure' },
  { n: 8, file: '08-continuation.html', title: 'Continuation', short: 'Nearest vs branch-aware', soon: true },
];

export function mountChrome() {
  const step = Number(document.body.dataset.step || 0);
  const header = document.createElement('header');
  header.className = 'top';
  header.innerHTML = `
    <a class="brand" href="../index.html"><span class="brand-mark">1A</span>
      <span><strong class="brand-name">GoFa kinematics lab</strong><span class="brand-subtitle">EPFL · ENG-654 · Project 1A</span></span></a>
    <nav class="top-links"><a href="../docs/project_1A.pdf" target="_blank" rel="noopener">Assignment PDF</a>
      <a href="https://epfl-lasa.github.io/eng-654/" target="_blank" rel="noopener">Course ↗</a></nav>`;
  const steps = document.createElement('nav');
  steps.className = 'steps';
  steps.innerHTML = STEPS.map((s) => {
    const cls = ['step', s.n === step ? 'current' : '', s.soon ? 'soon' : ''].join(' ');
    const inner = `<span class="step-n">${s.n}</span><span class="step-t"><strong>${s.title}</strong><small>${s.soon ? 'Coming next' : s.short}</small></span>`;
    return s.soon ? `<span class="${cls}" title="Needs the IK pipeline — next stage">${inner}</span>` : `<a class="${cls}" href="${s.file}">${inner}</a>`;
  }).join('');
  document.body.prepend(header, steps);
  if (window.matchMedia('(max-width: 980px)').matches) document.querySelectorAll('details.dock').forEach((d) => { d.open = false; });
  steps.querySelector('.current')?.scrollIntoView({ inline: 'center', block: 'nearest' });

  const prev = STEPS.find((s) => s.n === step - 1), next = STEPS.find((s) => s.n === step + 1 && !s.soon);
  const foot = document.querySelector('.panel-foot');
  if (foot) foot.innerHTML = `${prev ? `<a class="button" href="${prev.file}">← ${prev.title}</a>` : '<span></span>'}${next ? `<a class="button primary" href="${next.file}">${next.title} →</a>` : ''}`;
}

export function mountTabs(root = document) {
  root.querySelectorAll('.tabs').forEach((tabs) => {
    const buttons = [...tabs.querySelectorAll('[data-tab]')];
    const show = (id) => {
      buttons.forEach((b) => b.classList.toggle('active', b.dataset.tab === id));
      document.querySelectorAll(`[data-pane]`).forEach((p) => { if (buttons.some((b) => b.dataset.tab === p.dataset.pane)) p.hidden = p.dataset.pane !== id; });
      tabs.dispatchEvent(new CustomEvent('tab', { detail: id }));
    };
    buttons.forEach((b) => b.addEventListener('click', () => show(b.dataset.tab)));
    show(buttons.find((b) => b.classList.contains('active'))?.dataset.tab ?? buttons[0].dataset.tab);
  });
}

// Joint sliders; onChange(q) on every input. Returns { get, set }.
export function jointSliders(container, q0, onChange, { only = null } = {}) {
  let q = q0.slice();
  const idx = only ?? [0, 1, 2, 3, 4, 5];
  container.classList.add('sliders');
  container.innerHTML = idx.map((i) => `
    <label class="slider"><span class="slider-name">${JOINT_NAMES[i].replace(/(\d)/, '<sub>$1</sub>')}</span>
      <input type="range" min="${LIMITS[i][0]}" max="${LIMITS[i][1]}" step="0.001" value="${q[i]}" data-i="${i}">
      <output data-o="${i}"></output></label>`).join('');
  const out = (i) => { container.querySelector(`[data-o="${i}"]`).textContent = `${deg(q[i]).toFixed(1)}°`; };
  idx.forEach(out);
  container.querySelectorAll('input').forEach((inp) => inp.addEventListener('input', () => {
    const i = Number(inp.dataset.i); q[i] = Number(inp.value); out(i); onChange(q.slice());
  }));
  return {
    get: () => q.slice(),
    set(nq) { q = nq.slice(); idx.forEach((i) => { container.querySelector(`[data-i="${i}"]`).value = q[i]; out(i); }); onChange(q.slice()); },
  };
}

export function matrixTable(M, { digits = 3, rowLabels = null, colLabels = null, highlightCol = -1, cellClass = null } = {}) {
  const head = colLabels ? `<thead><tr>${rowLabels ? '<th></th>' : ''}${colLabels.map((c, j) => `<th class="${j === highlightCol ? 'hl' : ''}">${c}</th>`).join('')}</tr></thead>` : '';
  const body = M.map((row, i) => `<tr>${rowLabels ? `<th>${rowLabels[i]}</th>` : ''}${row.map((x, j) => {
    const extra = cellClass ? cellClass(i, j, x) : '';
    return `<td class="${j === highlightCol ? 'hl' : ''} ${extra}">${fmt(x, digits)}</td>`;
  }).join('')}</tr>`).join('');
  return `<table class="mat">${head}<tbody>${body}</tbody></table>`;
}

export function renderMath(root = document.body) {
  const go = () => window.renderMathInElement?.(root, {
    delimiters: [{ left: '\\[', right: '\\]', display: true }, { left: '\\(', right: '\\)', display: false }],
    throwOnError: false,
  });
  if (window.renderMathInElement) go(); else window.addEventListener('load', go, { once: true });
}

export const $ = (s, r = document) => r.querySelector(s);
export const $$ = (s, r = document) => [...r.querySelectorAll(s)];
