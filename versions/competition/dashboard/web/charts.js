/* Tiny canvas charts (no libraries, works offline).
   LineChart : several series against time, optional horizontal limit lines (warn / alarm), auto Y range, hover read-out.
   Timeline  : coloured bars for "which state was active when".
   Both redraw from the data you hand them; no hidden state except the last data (for the hover). */
(function () {
  const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  const fmtT = t => { const d = new Date(t * 1000); return d.toTimeString().slice(0, 8); };

  function setup(cv) {
    const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    const w = Math.max(10, Math.round(r.width)), h = Math.max(10, Math.round(r.height));
    if (cv.width !== w * dpr || cv.height !== h * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
    const g = cv.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, h);
    return { g, w, h };
  }
  function niceStep(span, n) {
    const raw = span / n, p = Math.pow(10, Math.floor(Math.log10(raw))), m = raw / p;
    return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p;
  }

  class LineChart {
    /* opts: {series:[{name,color}], unit:'mA', yMin:null, yMax:null, limits:[{y,color,label}], seconds:120} */
    constructor(cv, opts) {
      this.cv = cv; this.o = Object.assign({ unit: '', yMin: null, yMax: null, limits: [], series: [] }, opts);
      this.t = []; this.ys = []; this.hover = null;
      cv.addEventListener('mousemove', e => { const r = cv.getBoundingClientRect(); this.hover = e.clientX - r.left; this.draw(); });
      cv.addEventListener('mouseleave', () => { this.hover = null; this.draw(); });
    }
    set(t, ys, limits) { this.t = t; this.ys = ys; if (limits) this.o.limits = limits; this.draw(); }
    draw() {
      const { g, w, h } = setup(this.cv), o = this.o, t = this.t, L = 46, R = 10, T = 8, B = 20;
      const pw = w - L - R, ph = h - T - B;
      let lo = Infinity, hi = -Infinity;
      this.ys.forEach(s => s.forEach(v => { if (v != null && isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); } }));
      (o.limits || []).forEach(l => { if (l.y != null) { hi = Math.max(hi, l.y * 1.05); lo = Math.min(lo, l.y); } });
      if (!isFinite(lo)) { lo = 0; hi = 1; }
      if (o.yMin != null) lo = o.yMin; else lo = Math.min(lo, 0) === 0 && lo >= 0 ? 0 : lo;
      if (o.yMax != null) hi = o.yMax;
      if (hi - lo < 1e-9) hi = lo + 1;
      const step = niceStep(hi - lo, 4); hi = Math.ceil(hi / step) * step; if (o.yMin == null && lo < 0) lo = Math.floor(lo / step) * step;
      const t0 = t.length ? t[0] : 0, t1 = t.length ? t[t.length - 1] : 1, ts = Math.max(1e-6, t1 - t0);
      const X = v => L + (v - t0) / ts * pw, Y = v => T + (1 - (v - lo) / (hi - lo)) * ph;
      g.font = '11px ui-monospace,Menlo,monospace'; g.textBaseline = 'middle';
      g.strokeStyle = css('--line'); g.fillStyle = css('--dim'); g.lineWidth = 1;
      for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) {      // grid + Y labels
        const y = Math.round(Y(v)) + .5; g.beginPath(); g.moveTo(L, y); g.lineTo(w - R, y); g.stroke();
        g.textAlign = 'right'; g.fillText(Math.abs(v) >= 1000 ? (v / 1000).toFixed(1) + 'k' : (+v.toPrecision(4)).toString(), L - 5, y);
      }
      g.textAlign = 'center'; g.textBaseline = 'top';
      for (let i = 0; i <= 4 && t.length; i++) { const tt = t0 + ts * i / 4; g.textAlign = i === 0 ? 'left' : i === 4 ? 'right' : 'center'; g.fillText(fmtT(tt), i === 0 ? L : i === 4 ? w - R : X(tt), h - B + 5); }
      (o.limits || []).forEach(l => {                                                  // warn / alarm lines
        if (l.y == null) return; const y = Y(l.y); g.strokeStyle = l.color; g.setLineDash([5, 4]); g.beginPath(); g.moveTo(L, y); g.lineTo(w - R, y); g.stroke(); g.setLineDash([]);
        g.fillStyle = l.color; g.textAlign = 'right'; g.textBaseline = 'bottom'; if (l.label) g.fillText(l.label + ' ' + l.y, w - R - 2, y - 1);
      });
      this.ys.forEach((s, si) => {                                                     // series (gaps where the value is null)
        g.strokeStyle = (o.series[si] || {}).color || css('--info'); g.lineWidth = 1.6; g.beginPath(); let pen = false;
        for (let i = 0; i < t.length; i++) { const v = s[i]; if (v == null || !isFinite(v)) { pen = false; continue; } const x = X(t[i]), y = Y(v); if (pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; } }
        g.stroke();
      });
      if (!t.length) { g.fillStyle = css('--dim'); g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText('no data yet', w / 2, h / 2); }
      if (this.hover != null && t.length) {                                            // hover read-out
        const hv = t0 + (this.hover - L) / pw * ts; let k = 0, best = 1e18;
        for (let i = 0; i < t.length; i++) { const d = Math.abs(t[i] - hv); if (d < best) { best = d; k = i; } }
        const x = X(t[k]); g.strokeStyle = css('--dim2'); g.lineWidth = 1; g.beginPath(); g.moveTo(x, T); g.lineTo(x, T + ph); g.stroke();
        const lines = [fmtT(t[k])].concat(this.ys.map((s, si) => ((o.series[si] || {}).name || '') + ' ' + (s[k] == null ? '-' : (+s[k]).toFixed(1)) + (o.unit ? ' ' + o.unit : '')));
        const bw = Math.max(...lines.map(s => g.measureText(s).width)) + 12, bh = lines.length * 14 + 6, bx = x + 8 + bw > w ? x - 8 - bw : x + 8;
        g.fillStyle = '#0d1116ee'; g.strokeStyle = css('--line'); g.fillRect(bx, T + 2, bw, bh); g.strokeRect(bx + .5, T + 2.5, bw, bh);
        g.fillStyle = css('--fg'); g.textAlign = 'left'; g.textBaseline = 'top'; lines.forEach((s, i) => g.fillText(s, bx + 6, T + 5 + i * 14));
      }
    }
  }

  class Timeline {
    /* segs: [{t0,t1,label,color}] ; draws one row of bars with the label inside when it fits */
    constructor(cv, opts) { this.cv = cv; this.o = Object.assign({ colors: {} }, opts); this.segs = []; this.range = [0, 1]; }
    set(segs, t0, t1) { this.segs = segs; this.range = [t0, t1]; this.draw(); }
    draw() {
      const { g, w, h } = setup(this.cv), [t0, t1] = this.range, span = Math.max(1e-6, t1 - t0), L = 4, pw = w - 8, bh = h - 20;
      g.font = '11px ui-monospace,Menlo,monospace'; g.textBaseline = 'middle';
      this.segs.forEach(s => {
        const a = Math.max(0, (s.t0 - t0) / span), b = Math.min(1, (s.t1 - t0) / span); if (b <= 0 || a >= 1) return;
        const x = L + a * pw, ww = Math.max(1, (b - a) * pw);
        g.fillStyle = s.color || this.o.colors[s.label] || '#445'; g.fillRect(x, 2, ww - 1, bh);
        if (ww > 52) { g.fillStyle = '#10151a'; g.textAlign = 'left'; g.fillText(s.label, x + 4, 2 + bh / 2); }
      });
      g.fillStyle = css('--dim'); g.textBaseline = 'top'; g.textAlign = 'left'; g.fillText(fmtT(t0), L, h - 14); g.textAlign = 'right'; g.fillText(fmtT(t1), w - 4, h - 14);
    }
  }
  window.Charts = { LineChart, Timeline, fmtT };
})();
