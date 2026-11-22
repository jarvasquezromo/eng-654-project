// Worker: evaluates a batch of placements (Step 7) off the main thread.
import { evaluatePlacement, centerOf } from './tasks.js';

self.onmessage = (e) => {
  const { S, orient, jobs } = e.data;
  for (const { i, xy, sols } of jobs) {
    // sols: verified supplied-solver answers for each contour sample (or undefined → browser IK)
    const r = evaluatePlacement(S, centerOf(S, xy), orient, sols ? (T0, k) => sols[k] : null);
    r.starts.forEach((s) => { delete s.q0; });
    self.postMessage({ i, r });
  }
  self.postMessage({ done: true });
};
