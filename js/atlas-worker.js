// Worker for the numerical (any-tool) topology atlas: one row of the plane grid per message.
import { computeRows } from './tool-atlas.js';

self.onmessage = (e) => {
  const { R, len, G, rows } = e.data;
  for (const j of rows) {
    const [r] = computeRows(R, len, G, [j]);
    self.postMessage(r, [r.all.buffer, r.inl.buffer, r.sols.buffer]);
  }
  self.postMessage({ done: true });
};
