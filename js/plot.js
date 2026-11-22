// Minimal canvas line plots for the step pages (crisp on phones, no dependencies).
export function linePlot(canvas, { series, xLabel = '', yLabel = '', xRange, yRange, bands = [], vlines = [], hlines = [], height } = {}) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const W = canvas.clientWidth || 320, H = height || canvas.clientHeight || 180;
  canvas.width = W * dpr; canvas.height = H * dpr; canvas.style.height = `${H}px`;
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const P = { l: 46, r: 8, t: 8, b: 30 };
  const all = series.flatMap((s) => s.y.filter(Number.isFinite));
  const [x0, x1] = xRange || [0, Math.max(...series.map((s) => s.y.length - 1), 1)];
  let [y0, y1] = yRange || [Math.min(...all, 0), Math.max(...all, 1)];
  if (y1 - y0 < 1e-9) { y0 -= 1; y1 += 1; }
  const X = (x) => P.l + ((x - x0) / (x1 - x0)) * (W - P.l - P.r), Y = (y) => H - P.b - ((y - y0) / (y1 - y0)) * (H - P.t - P.b);
  bands.forEach(({ from, to, color }) => { ctx.fillStyle = color; ctx.fillRect(P.l, Y(to), W - P.l - P.r, Y(from) - Y(to)); });
  ctx.strokeStyle = '#e6e6e6'; ctx.lineWidth = 1; ctx.font = '10.5px Inter, Arial'; ctx.fillStyle = '#555';
  const ticks = (a, b, n) => { const st = niceStep((b - a) / n); const out = []; for (let v = Math.ceil(a / st) * st; v <= b + 1e-9; v += st) out.push(+v.toFixed(10)); return out; };
  ctx.textAlign = 'right';
  ticks(y0, y1, 4).forEach((v) => { ctx.beginPath(); ctx.moveTo(P.l, Y(v)); ctx.lineTo(W - P.r, Y(v)); ctx.stroke(); ctx.fillText(fmtTick(v), P.l - 4, Y(v) + 3); });
  ctx.textAlign = 'center';
  ticks(x0, x1, 5).forEach((v) => ctx.fillText(fmtTick(v), X(v), H - P.b + 13));
  ctx.fillText(xLabel, P.l + (W - P.l - P.r) / 2, H - 4);
  ctx.save(); ctx.translate(10, P.t + (H - P.t - P.b) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(yLabel, 0, 0); ctx.restore();
  ctx.save(); ctx.beginPath(); ctx.rect(P.l, P.t, W - P.l - P.r, H - P.t - P.b); ctx.clip();
  hlines.forEach(({ y, color = '#ff0000', dash = [4, 3] }) => { ctx.strokeStyle = color; ctx.setLineDash(dash); ctx.beginPath(); ctx.moveTo(P.l, Y(y)); ctx.lineTo(W - P.r, Y(y)); ctx.stroke(); ctx.setLineDash([]); });
  vlines.forEach(({ x, color = '#141414', dash = [] }) => { ctx.strokeStyle = color; ctx.setLineDash(dash); ctx.beginPath(); ctx.moveTo(X(x), P.t); ctx.lineTo(X(x), H - P.b); ctx.stroke(); ctx.setLineDash([]); });
  for (const s of series) {
    ctx.strokeStyle = s.color; ctx.lineWidth = s.width || 1.6; ctx.globalAlpha = s.alpha ?? 1; ctx.setLineDash(s.dash || []);
    ctx.beginPath(); let pen = false;
    s.y.forEach((y, i) => {
      const x = s.x ? s.x[i] : i;
      if (!Number.isFinite(y)) { pen = false; return; }
      if (pen) ctx.lineTo(X(x), Y(y)); else ctx.moveTo(X(x), Y(y));
      pen = true;
      if (s.points) { ctx.stroke(); ctx.beginPath(); ctx.arc(X(x), Y(y), 1.8, 0, 2 * Math.PI); ctx.fillStyle = s.color; ctx.fill(); ctx.beginPath(); ctx.moveTo(X(x), Y(y)); }
    });
    ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
  }
  ctx.restore();
  ctx.strokeStyle = '#141414'; ctx.lineWidth = 1; ctx.strokeRect(P.l, P.t, W - P.l - P.r, H - P.t - P.b);
  return { X, Y, P, W, H, x0, x1 };
}
function niceStep(raw) { const p = 10 ** Math.floor(Math.log10(raw)), m = raw / p; return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p; }
const fmtTick = (v) => (Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 1 ? +v.toFixed(1) + '' : +v.toFixed(3) + '');
