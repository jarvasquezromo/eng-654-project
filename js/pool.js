// Small pool of module workers. run(jobs, onResult) splits jobs round-robin and resolves when all are done.
// Falls back to the main thread (in chunks) if module workers are unavailable.
import { evaluatePlacement, centerOf } from './tasks.js';

export function makePool() {
  const n = Math.max(1, Math.min(8, (navigator.hardwareConcurrency || 4) - 1));
  let workers = [];
  try { workers = Array.from({ length: n }, () => new Worker(new URL('./feas-worker.js', import.meta.url), { type: 'module' })); } catch { workers = []; }
  let gen = 0;
  return {
    size: workers.length || 1,
    run(S, orient, jobs, onResult) {
      const id = ++gen;
      if (!workers.length) {
        return new Promise((resolve) => {
          let k = 0;
          const step = () => {
            if (id !== gen) return resolve(false);
            const t0 = performance.now();
            while (k < jobs.length && performance.now() - t0 < 30) { const j = jobs[k++]; onResult(j.i, evaluatePlacement(S, centerOf(S, j.xy), orient)); }
            if (k < jobs.length) requestAnimationFrame(step); else resolve(true);
          };
          step();
        });
      }
      // restart workers so a stale batch cannot deliver results
      workers.forEach((w, i) => { w.terminate(); workers[i] = new Worker(new URL('./feas-worker.js', import.meta.url), { type: 'module' }); });
      return new Promise((resolve) => {
        let pending = workers.length;
        workers.forEach((w, wi) => {
          w.onmessage = (e) => {
            if (id !== gen) return;
            if (e.data.done) { if (--pending === 0) resolve(true); return; }
            onResult(e.data.i, e.data.r);
          };
          w.onerror = (err) => { console.error(err); if (--pending === 0) resolve(true); };
          w.postMessage({ S, orient, jobs: jobs.filter((_, k) => k % workers.length === wi) });
        });
      });
    },
    cancel() { gen++; },
  };
}
