// IK source switch: the browser solver (js/ik.js) or the supplied crb15000-ik, reached through the
// local helper (tools/ik-helper/ik_helper.py). Every configuration from the supplied solver is
// verified here with our own PoE forward kinematics, its periodic representation is resolved
// (q3 shifted into its range, q6 kept in (−π, π]), our joint-limit check is compared with the
// solver's flag, and it gets the same branch labels as the browser solver.
import { poeFK } from './gofa.js';
import { fitLimits, classifyBranch, branchLabel, solveIK } from './ik.js';

const KEY_SRC = 'gofa-ik-source', KEY_URL = 'gofa-ik-helper';
export const helperURL = () => { try { return localStorage.getItem(KEY_URL) || 'http://127.0.0.1:8654'; } catch { return 'http://127.0.0.1:8654'; } };
export function getSource() { try { return localStorage.getItem(KEY_SRC) === 'supplied' ? 'supplied' : 'browser'; } catch { return 'browser'; } }
export function setSource(src) {
  try { localStorage.setItem(KEY_SRC, src); } catch { /* ignore */ }
  window.dispatchEvent(new CustomEvent('ik-source', { detail: src }));
}

let status = null;
export async function helperStatus(force = false) {
  if (status && !force && Date.now() - status.t < 5000) return status;
  try {
    const ctl = new AbortController(), timer = setTimeout(() => ctl.abort(), 1200);
    const r = await fetch(`${helperURL()}/health`, { signal: ctl.signal });
    clearTimeout(timer);
    const j = await r.json();
    status = { ok: !!j.ok, solver: j.solver, threads: j.threads, t: Date.now() };
  } catch (e) { status = { ok: false, error: String(e.message || e), t: Date.now() }; }
  return status;
}
// Use the supplied solver only if chosen and the helper answers
export async function useSupplied() { return getSource() === 'supplied' && (await helperStatus()).ok; }

// Raw call: poses are 4×4 tool0 matrices
export async function solveRaw(Ts, { chunk = 20000, onProgress } = {}) {
  const out = [];
  for (let i = 0; i < Ts.length; i += chunk) {
    const poses = Ts.slice(i, i + chunk).map((T) => [...T[0], ...T[1], ...T[2], 0, 0, 0, 1]);
    const r = await fetch(`${helperURL()}/solve`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ frame: 'tool0', poses }) });
    const j = await r.json();
    if (!r.ok || j.error) throw new Error(j.error || `helper HTTP ${r.status}`);
    out.push(...j.results);
    onProgress?.(Math.min(Ts.length, i + chunk), Ts.length);
  }
  return out;
}

export const newStats = () => ({ poses: 0, errors: 0, fallback: 0, fallbackFound: 0, messages: {}, returned: 0, accepted: 0, rejected: 0, maxPos: 0, maxRot: 0, limitAgree: 0, limitDisagree: 0, q3Shifted: 0 });

// Verify one pose's answer. Returns solutions shaped like js/ik.js solveIK output.
export function verify(T, res, stats = newStats(), tol = 1e-7) {
  stats.poses++;
  if (res.error) {
    // The supplied solver gave up on this pose (e.g. "polynomial roots unresolved", or an identically zero
    // polynomial for unreachable / continuous-family poses). That is not evidence of "no IK": fall back to
    // the browser solver for this pose (its answers are FK-checked too) and count it.
    stats.errors++; stats.messages[res.error] = (stats.messages[res.error] || 0) + 1;
    const out = solveIK(T).map((x) => ({ ...x, source: 'browser-fallback' }));
    stats.fallback++; if (out.length) stats.fallbackFound++;
    out.error = res.error;
    return out;
  }
  const out = [];
  for (const s of res.solutions) {
    stats.returned++;
    const F = poeFK(s.q);
    let ep = 0, er = 0;
    for (let i = 0; i < 3; i++) { ep += (F[i][3] - T[i][3]) ** 2; for (let j = 0; j < 3; j++) er = Math.max(er, Math.abs(F[i][j] - T[i][j])); }
    ep = Math.sqrt(ep);
    stats.maxPos = Math.max(stats.maxPos, ep); stats.maxRot = Math.max(stats.maxRot, er);
    if (ep > tol || er > tol) { stats.rejected++; continue; }
    stats.accepted++;
    const { q, within } = fitLimits(s.q);
    if (Math.abs(q[2] - s.q[2]) > 1e-9) stats.q3Shifted++;
    if (within === !!s.within_limits) stats.limitAgree++; else stats.limitDisagree++;
    const branch = classifyBranch(q);
    out.push({ q, within, branch, label: branchLabel(branch), position_error: ep, rotation_error: er, family: false, source: 'supplied', solverWithin: !!s.within_limits, t6: s.t6, radial_branch: s.radial_branch, j4_branch: s.j4_branch });
  }
  return out;
}

export async function solveVerified(Ts, opts = {}) {
  const stats = newStats(), raw = await solveRaw(Ts, opts);
  return { sols: raw.map((r, k) => verify(Ts[k], r, stats)), stats };
}

export function statsHTML(st, seconds) {
  const sci = (x) => (x ? x.toExponential(1) : '0');
  return [
    ['source', 'supplied crb15000-ik'], ['poses', st.poses], ['returned configs', st.returned],
    ['accepted by our FK', `${st.accepted} (${st.rejected} rejected)`], ['max residual', `${sci(st.maxPos)} m · ${sci(st.maxRot)}`],
    ['limit flag agrees', `${st.limitAgree}/${st.limitAgree + st.limitDisagree}`],
    ['solver gave up', st.errors ? `${st.errors} poses → browser IK (${st.fallbackFound} had solutions)` : '0'],
    ...(seconds !== undefined ? [['time', `${seconds.toFixed(1)} s`]] : []),
  ].map(([k, x]) => `<div class="tile"><small>${k}</small><b>${x}</b></div>`).join('');
}
export const errorList = (st) => Object.entries(st.messages).map(([m, n]) => `${n}× ${m}`).join('; ');
