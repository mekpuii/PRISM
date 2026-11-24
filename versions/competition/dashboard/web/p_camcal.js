// p_camcal.js - Control -> Calibrate: the camera's calibration on the dashboard (2026-11-12, the operator: "make the
// calibrate mode in the dashboard: label this colour, adjust the light and else, and save").
//
//   Picture   the plain camera picture (vision sends it only while this page is open, ~2 per second).
//             Click a cube -> its colour (H S V) -> "white cube" / "orange cube" / "light, not a cube".
//             Views: Cubes test (what would count, what is dropped as light and why) · masks per colour · plain picture.
//   Colours   H / S / V ranges per colour; "Fit from my labels" makes them from the clicks and keeps the light out.
//   Shape     the cube-shape filter (reflections are smaller, thinner, patchier than a cube).
//   Camera    the camera's own settings - they change the picture AT ONCE (it is the camera itself).
//   Save      shows every change first; the old file goes to backup/vision/; vision takes the new one in ~2 s.
// Colours and shape are only PREVIEWED here until Save; the running cell keeps its values until then.
(function () {
  const P = window.PRISM, $ = P.$, f = P.f, esc = P.esc;
  const W = 854, H = 480;
  const SHAPE0 = { min_h: 38, min_area: 1700, exit_zone_u: 160, exit_min_area: 900, aspect_lo: 0.4, aspect_hi: 3.0, fill: 0.55 };
  const SHAPE_UI = [
    ['min_h', 'Min height', 5, 120, 1, 'px - a cube is ~41-55 px tall'],
    ['min_area', 'Min area', 300, 6000, 50, 'px² - a cube is ~3000-4000'],
    ['fill', 'Min solid fill', 0.2, 0.95, 0.01, 'blob / its box - glare is patchy'],
    ['aspect_lo', 'Width/height from', 0.1, 1, 0.05, ''],
    ['aspect_hi', 'Width/height to', 1, 6, 0.1, ''],
    ['exit_zone_u', 'Feeder exit zone', 0, 400, 5, 'px from the left - half cubes allowed there'],
    ['exit_min_area', 'Min area at the exit', 100, 4000, 50, 'px²'],
  ];
  const DRAW = { white: '#5ce1ff', orange: '#ff9a2e', red: '#ff5a4f', yellow: '#f5e342', green: '#3ecf8e', blue: '#5aa9ff' };
  const CAM_UI = {       // the camera controls in plain words (order = on the page)
    power_line_frequency: ['Flicker filter', 'match the mains: 50 Hz in Thailand - stops the lamps from pulsing the picture'],
    auto_exposure: ['Exposure mode', 'Manual = the picture stops changing by itself (recommended)'],
    exposure_time_absolute: ['Exposure time', 'brighter <-> darker (only in Manual)'],
    exposure_dynamic_framerate: ['Slow down when dark', 'off keeps the frame rate steady'],
    white_balance_automatic: ['Auto white balance', 'off keeps the colours steady'],
    white_balance_temperature: ['White balance', 'K - only when auto is off'],
    focus_automatic_continuous: ['Auto focus', 'off stops focus hunting'],
    focus_absolute: ['Focus', 'only when auto focus is off'],
    brightness: ['Brightness', ''], contrast: ['Contrast', ''], saturation: ['Saturation', 'more = colours easier to tell apart'],
    gamma: ['Gamma', ''], gain: ['Gain', ''], sharpness: ['Sharpness', ''], hue: ['Hue', ''], backlight_compensation: ['Backlight', ''],
  };

  let cal = null, names = [], ranges = {}, shape = { ...SHAPE0 }, samples = {}, beltV = null, beltVChanged = false;
  let ctrls = [], camDirty = false, view = 'test', frame = null, roiMask = null, lastPick = null, timers = [], busy = false;
  let blobsNow = [], labelsDirty = false;

  P.page('/control/calib', {
    title: 'Calibrate',
    mount(el) {
      el.innerHTML = `
      <div class="cc-grid">
        <div class="card cc-pic"><h2>Picture <span class="r dim" id="cc-age"></span></h2>
          <div class="grp cc-views" id="cc-views">
            <button data-v="test" class="on">Cubes test</button><button data-v="mask">Colour masks</button><button data-v="plain">Plain</button></div>
          <div class="cc-canwrap"><canvas id="cc-can" width="${W}" height="${H}"></canvas><div class="ov-noimg" id="cc-noimg">waiting for the picture from vision…</div></div>
          <div class="dim cc-legend" id="cc-legend"></div>
          <div class="cc-pick" id="cc-pick"><span class="dim">Click a cube (or a shiny spot) on the picture.</span></div>
        </div>
        <div class="cc-side">
          <div class="card cc-easy"><h2>Teach the camera</h2>
            <ol class="cc-steps"><li><b>Click</b> a cube on the picture and press what it is (white / orange).<br><span class="dim">Shiny spots: press "light / not a cube".</span></li>
              <li>Do it <b>5 or more times</b> per colour, all along the belt.</li>
              <li>Press <b>Save</b>. Vision uses it straight away.</li></ol>
            <div id="cc-labels"></div>
            <div class="row cc-bigrow"><button class="go cc-big" id="cc-savenow">Save</button><button id="cc-undo">Undo</button></div>
            <div class="cc-note" id="cc-msg2"></div></div>
          <div class="card"><h2>Working area <span class="dim" id="cc-areastate"></span></h2>
            <div class="dim">The part of the picture where vision looks for cubes (green line). <b>Drag a box</b> over the belt, or <b>click the corners</b>, then Save. Keep the feeder exit and the pick point inside it.</div>
            <div class="row"><button id="cc-abox">Drag a box</button><button id="cc-acor">Click corners</button><button id="cc-aundo">Undo point</button><button id="cc-aclear">Clear</button></div>
            <div class="row cc-bigrow"><button class="go cc-big" id="cc-asave">Save area</button></div>
            <div class="cc-note" id="cc-amsg"></div></div>
        </div>
      </div>
      <details class="card cc-more" style="margin-top:12px"><summary><b>More settings</b> <span class="dim">camera light, colour ranges, cube shape - only if you need them</span></summary>
        <div class="grid g2" style="margin-top:12px">
          <div class="card"><h2>Camera <span class="r dim">changes the picture at once</span></h2>
            <div class="row cc-quick"><button id="cc-lock">Lock the picture as it is now</button><button id="cc-50">Flicker 50 Hz</button><button id="cc-auto">Back to automatic</button></div>
            <div id="cc-cam"></div>
            <div class="row" style="margin-top:8px"><button id="cc-beltv">Measure the empty belt</button><span class="dim" id="cc-beltvtxt"></span></div></div>
          <div>
            <div class="card"><h2>Colours <span class="r dim">H 0-179 · S, V 0-255</span></h2>
              <div class="row" style="margin-bottom:8px"><button id="cc-fit">Fit from my labels now</button><button id="cc-clear">Clear labels</button></div>
              <div class="dim cc-note" id="cc-fitmsg"></div><div id="cc-colors"></div></div>
            <div class="card" style="margin-top:12px"><h2>Cube shape <span class="r dim">what is a cube, what is light</span></h2><div id="cc-shape"></div>
              <div class="dim cc-note">Vision also ignores things that never move with the belt - the test here checks colour + shape only.</div></div>
          </div>
        </div>
        <div class="row cc-save" style="margin-top:12px"><button id="cc-check">Show what would change</button><button id="cc-reload">Throw away my edits</button><span class="msg" id="cc-msg"></span></div>
        <div id="cc-diff"></div>
      </details>`;
      $('#cc-views').onclick = e => { const v = e.target.dataset.v; if (!v) return; view = v; [...$('#cc-views').children].forEach(b => b.classList.toggle('on', b.dataset.v === v)); render(); };
      $('#cc-can').onclick = canClick;
      $('#cc-can').onmousedown = canDown; $('#cc-can').onmousemove = canMove; $('#cc-can').onmouseup = canUp;
      $('#cc-abox').onclick = () => setArea('box'); $('#cc-acor').onclick = () => setArea('corners');
      $('#cc-aundo').onclick = () => { areaPts.pop(); render(); areaShow(); }; $('#cc-aclear').onclick = () => { areaPts = []; render(); areaShow(); };
      $('#cc-asave').onclick = saveArea;
      $('#cc-fit').onclick = fit;
      $('#cc-clear').onclick = () => { if (confirm('Forget all labelled clicks?')) { samples = {}; paintLabels(); } };
      $('#cc-lock').onclick = lockNow;
      $('#cc-50').onclick = () => setCtl('power_line_frequency', 1);
      $('#cc-auto').onclick = autoAll;
      $('#cc-beltv').onclick = measureBelt;
      $('#cc-check').onclick = check;
      $('#cc-savenow').onclick = saveNow;
      $('#cc-undo').onclick = undo;
      $('#cc-reload').onclick = () => load(true);
      load(true);
      keepRaw(); timers.push(setInterval(keepRaw, 20000));
      timers.push(setInterval(grab, 650));
    },
    update() {},
    unmount() { timers.forEach(clearInterval); timers = []; frame = null; },
  });

  // ---------------------------------------------------------------- data
  async function load(all) {
    try {
      const d = await P.api('/api/cam/calib');
      cal = d.calib || {}; ctrls = d.controls || [];
      if (all) {
        names = cal.enabled || Object.keys(cal.colors || {});
        ranges = {};
        for (const n of names) ranges[n] = toSliders((cal.colors || {})[n] || [[[0, 0, 0], [179, 255, 255]]]);
        shape = { ...SHAPE0, ...(cal.shape || {}) };
        samples = JSON.parse(JSON.stringify(cal.samples || {}));
        beltV = cal.belt_v ?? null; beltVChanged = false; camDirty = false;
        roiMask = null; $('#cc-diff').innerHTML = '';
      }
      paintColors(); paintShape(); paintCam(); paintLabels(); render();
    } catch (e) { $('#cc-msg2').className = 'cc-note msg bad'; $('#cc-msg2').textContent = 'Cannot read the calibration: ' + e.message; }
  }
  // one H/S/V box per colour; h0 > h1 = the hue wraps around 0 (red) = two ranges in the file
  function toSliders(r) {
    const [lo, hi] = r[0];
    if (r.length === 1) return [lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]];
    return [r[1][0][0], hi[0], lo[1], hi[1], lo[2], hi[2]];
  }
  function toRanges(s) {
    const [h0, h1, s0, s1, v0, v1] = s;
    return h0 <= h1 ? [[[h0, s0, v0], [h1, s1, v1]]] : [[[0, s0, v0], [h1, s1, v1]], [[h0, s0, v0], [179, s1, v1]]];
  }
  const inRange = (s, h, sat, v) => sat >= s[2] && sat <= s[3] && v >= s[4] && v <= s[5] && (s[0] <= s[1] ? h >= s[0] && h <= s[1] : h >= s[0] || h <= s[1]);
  async function keepRaw() { try { await P.post('/api/cam/raw', { seconds: 35 }); } catch (e) { /* the picture just stays old */ } }

  // ---------------------------------------------------------------- picture
  async function grab() {
    if (busy || !$('#cc-can')) return;
    busy = true;
    try {
      const r = await fetch('/api/camera_raw.jpg?t=' + Date.now(), { cache: 'no-store' });
      if (!r.ok) { $('#cc-noimg').hidden = false; return; }
      const bmp = await createImageBitmap(await r.blob());
      const c = document.createElement('canvas'); c.width = W; c.height = H;
      const g = c.getContext('2d'); g.drawImage(bmp, 0, 0, W, H);
      const img = g.getImageData(0, 0, W, H);
      frame = { img, hsv: toHsv(img.data), t: Date.now() };
      $('#cc-noimg').hidden = true;
      render();
    } catch (e) { /* next time */ } finally { busy = false; }
  }
  function toHsv(d) {            // OpenCV's 8-bit HSV: H 0..179, S, V 0..255
    const n = d.length / 4, out = new Uint8Array(n * 3);
    for (let i = 0, j = 0; i < d.length; i += 4, j += 3) {
      const r = d[i], g = d[i + 1], b = d[i + 2];
      const v = r > g ? (r > b ? r : b) : (g > b ? g : b), mn = r < g ? (r < b ? r : b) : (g < b ? g : b), df = v - mn;
      let h = 0;
      if (df) { h = v === r ? 60 * (g - b) / df : v === g ? 120 + 60 * (b - r) / df : 240 + 60 * (r - g) / df; if (h < 0) h += 360; }
      out[j] = Math.round(h / 2) % 180; out[j + 1] = v ? Math.round(255 * df / v) : 0; out[j + 2] = v;
    }
    return out;
  }
  function roi() {
    if (roiMask || !cal || !cal.roi) return roiMask;
    const c = document.createElement('canvas'); c.width = W; c.height = H;
    const g = c.getContext('2d'); g.fillStyle = '#fff'; g.beginPath();
    cal.roi.forEach(([x, y], i) => i ? g.lineTo(x, y) : g.moveTo(x, y)); g.closePath(); g.fill();
    const a = g.getImageData(0, 0, W, H).data; roiMask = new Uint8Array(W * H);
    for (let i = 0; i < W * H; i++) roiMask[i] = a[i * 4 + 3] > 127 ? 1 : 0;
    return roiMask;
  }
  // 5x5 open + close like vision (separable min / max)
  function morph(m, op) {
    const t = new Uint8Array(m.length), o = new Uint8Array(m.length), k = 2;
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      let r = op ? 0 : 1; for (let d = -k; d <= k; d++) { const xx = x + d; const v = xx < 0 || xx >= W ? 0 : m[y * W + xx]; r = op ? (r | v) : (r & v); } t[y * W + x] = r;
    }
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      let r = op ? 0 : 1; for (let d = -k; d <= k; d++) { const yy = y + d; const v = yy < 0 || yy >= H ? 0 : t[yy * W + x]; r = op ? (r | v) : (r & v); } o[y * W + x] = r;
    }
    return o;
  }
  const erode = m => morph(m, 0), dilate = m => morph(m, 1);
  function masks() {
    const hsv = frame.hsv, R = roi(), out = {}, taken = new Uint8Array(W * H);
    for (const n of names) {                         // priority: the first colour wins a pixel
      const s = ranges[n], m = new Uint8Array(W * H);
      for (let i = 0, j = 0; i < W * H; i++, j += 3) {
        if ((R && !R[i]) || taken[i]) continue;
        if (inRange(s, hsv[j], hsv[j + 1], hsv[j + 2])) { m[i] = 1; taken[i] = 1; }
      }
      out[n] = erode(dilate(dilate(erode(m))));      // open, then close
    }
    return out;
  }
  function blobs(m, name) {                          // connected parts -> box, area, the shape verdict
    const seen = new Uint8Array(W * H), res = [], st = [];
    for (let i = 0; i < W * H; i++) {
      if (!m[i] || seen[i]) continue;
      let x0 = W, x1 = 0, y0 = H, y1 = 0, a = 0; st.push(i); seen[i] = 1;
      while (st.length) {
        const p = st.pop(), x = p % W, y = (p - x) / W; a++;
        if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y;
        for (const q of [p - 1, p + 1, p - W, p + W]) {
          if (q < 0 || q >= W * H || seen[q] || !m[q] || (q === p - 1 && x === 0) || (q === p + 1 && x === W - 1)) continue;
          seen[q] = 1; st.push(q);
        }
      }
      if (a < 500) continue;                         // vision's MIN_AREA: below is noise, not even a candidate
      const w = x1 - x0 + 1, h = y1 - y0 + 1, atExit = x0 + w / 2 < shape.exit_zone_u, fill = a / (w * h), asp = w / h;
      let why = '';
      if (a < (atExit ? shape.exit_min_area : shape.min_area)) why = 'too small';
      else if (h < shape.min_h && !atExit) why = 'too flat';
      else if (asp < shape.aspect_lo || asp > shape.aspect_hi) why = 'wrong shape';
      else if (fill < shape.fill) why = 'patchy';
      res.push({ name, x: x0, y: y0, w, h, a, fill, ok: !why, why });
    }
    return res;
  }
  function render() {
    const can = $('#cc-can'); if (!can) return;
    const g = can.getContext('2d');
    if (!frame) { g.fillStyle = '#0d1116'; g.fillRect(0, 0, W, H); return; }
    $('#cc-age').textContent = 'live · ' + new Date(frame.t).toTimeString().slice(0, 8);
    if (view === 'plain') { g.putImageData(frame.img, 0, 0); blobsNow = []; drawRoi(g); drawPick(g); legend(''); return; }
    const ms = masks();
    if (view === 'mask') {
      const out = new ImageData(new Uint8ClampedArray(frame.img.data), W, H), d = out.data;
      for (let i = 0; i < W * H; i++) { d[i * 4] *= 0.35; d[i * 4 + 1] *= 0.35; d[i * 4 + 2] *= 0.35; }
      names.forEach(n => {
        const c = hex(DRAW[n] || '#ffffff');
        for (let i = 0; i < W * H; i++) if (ms[n][i]) { d[i * 4] = c[0]; d[i * 4 + 1] = c[1]; d[i * 4 + 2] = c[2]; }
      });
      g.putImageData(out, 0, 0); drawRoi(g); drawPick(g);
      legend(names.map(n => `<span style="color:${DRAW[n] || '#fff'}">■</span> ${esc(n)}`).join(' &nbsp; ') + ' &nbsp; - what each colour range catches (after vision\'s smoothing)');
      return;
    }
    g.putImageData(frame.img, 0, 0); drawRoi(g);
    blobsNow = names.flatMap(n => blobs(ms[n], n));
    g.font = '12px system-ui'; g.lineWidth = 2;
    for (const b of blobsNow) {
      g.setLineDash(b.ok ? [] : [5, 4]);
      g.strokeStyle = b.ok ? (DRAW[b.name] || '#fff') : '#9aa4af';
      g.strokeRect(b.x, b.y, b.w, b.h);
      const t = b.ok ? `${b.name} cube` : `light? ${b.why}`;
      g.fillStyle = 'rgba(0,0,0,.65)'; g.fillRect(b.x, b.y - 16, g.measureText(t).width + 8, 15);
      g.fillStyle = b.ok ? (DRAW[b.name] || '#fff') : '#c9d1d9'; g.fillText(t, b.x + 4, b.y - 4);
    }
    g.setLineDash([]);
    g.strokeStyle = 'rgba(255,255,255,.35)'; g.setLineDash([3, 5]); g.beginPath(); g.moveTo(shape.exit_zone_u, 0); g.lineTo(shape.exit_zone_u, H); g.stroke(); g.setLineDash([]);
    drawPick(g);
    const ok = blobsNow.filter(b => b.ok), bad = blobsNow.filter(b => !b.ok);
    legend(`<b>${ok.length}</b> cube${ok.length === 1 ? '' : 's'} (${names.map(n => `${ok.filter(b => b.name === n).length} ${esc(n)}`).join(', ')}) · `
      + `<b>${bad.length}</b> dropped as light (grey dashed) · dotted line = feeder exit zone · click a box for its numbers`);
  }
  const hex = h => [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16));
  // ---- the working area (2026-11-13): drag a box or click corners on the picture, then Save
  let areaMode = null, areaPts = [], dragFrom = null;
  const canXY = e => { const r = $('#cc-can').getBoundingClientRect(); return [Math.round((e.clientX - r.left) * W / r.width), Math.round((e.clientY - r.top) * H / r.height)]; };
  function setArea(m) {
    areaMode = areaMode === m ? null : m; areaPts = []; dragFrom = null;
    $('#cc-abox').classList.toggle('on', areaMode === 'box'); $('#cc-acor').classList.toggle('on', areaMode === 'corners');
    $('#cc-can').style.cursor = areaMode ? 'crosshair' : ''; render(); areaShow();
  }
  function areaShow() {
    const m = $('#cc-amsg'); if (!m) return;
    m.className = 'cc-note msg';
    m.textContent = !areaMode ? '' : areaMode === 'box' ? 'Press on the picture and drag a box, let go, then Save area.' :
      `Click the corners of the belt (${areaPts.length} so far, at least 3), then Save area.`;
    $('#cc-areastate').textContent = areaPts.length ? `· new area: ${areaPts.length} corners (not saved yet)` : '';
  }
  function canDown(e) { if (areaMode === 'box' && frame) { dragFrom = canXY(e); areaPts = []; } }
  function canMove(e) {
    if (areaMode !== 'box' || !dragFrom) return;
    const [x, y] = canXY(e), [x0, y0] = dragFrom; areaPts = [[x0, y0], [x, y0], [x, y], [x0, y]]; render();
  }
  function canUp(e) { if (areaMode === 'box' && dragFrom) { canMove(e); dragFrom = null; areaShow(); } }
  function canClick(e) {
    if (areaMode === 'corners' && frame) { areaPts.push(canXY(e)); render(); areaShow(); return; }
    if (areaMode) return;
    pick(e);
  }
  async function saveArea() {
    const m = $('#cc-amsg');
    if (areaPts.length < 3) { m.className = 'cc-note msg bad'; m.textContent = 'Draw the area first (a box or at least 3 corners).'; return; }
    try {
      await P.post('/api/cam/save', { roi: areaPts });
      m.className = 'cc-note msg ok'; m.textContent = '✓ Saved ' + new Date().toTimeString().slice(0, 5) + ' - vision uses the new area in ~2 s (it relearns the empty belt for a moment).';
      areaPts = []; areaMode = null; $('#cc-abox').classList.remove('on'); $('#cc-acor').classList.remove('on'); $('#cc-can').style.cursor = '';
      await load(false); render(); $('#cc-areastate').textContent = '';
    } catch (e) { m.className = 'cc-note msg bad'; m.textContent = 'Not saved: ' + e.message; }
  }
  function drawArea(g) {
    if (!areaPts.length) return;
    g.strokeStyle = '#ffd24a'; g.fillStyle = 'rgba(255,210,74,.14)'; g.lineWidth = 2; g.setLineDash([6, 4]); g.beginPath();
    areaPts.forEach(([x, y], i) => i ? g.lineTo(x, y) : g.moveTo(x, y)); if (areaPts.length > 2) { g.closePath(); g.fill(); } g.stroke(); g.setLineDash([]);
    areaPts.forEach(([x, y]) => { g.fillStyle = '#ffd24a'; g.fillRect(x - 3, y - 3, 7, 7); });
  }
  function drawRoi(g) {
    drawArea(g);
    if (!cal || !cal.roi) return;
    g.strokeStyle = 'rgba(62,207,142,.8)'; g.lineWidth = 1.5; g.beginPath();
    cal.roi.forEach(([x, y], i) => i ? g.lineTo(x, y) : g.moveTo(x, y)); g.closePath(); g.stroke();
  }
  function drawPick(g) {
    if (!lastPick) return;
    g.strokeStyle = '#fff'; g.lineWidth = 2; g.strokeRect(lastPick.x - 4, lastPick.y - 4, 9, 9);
  }
  function legend(html) { $('#cc-legend').innerHTML = html; }

  // ---------------------------------------------------------------- labelling (the "experience")
  function pick(e) {
    if (!frame) return;
    const c = $('#cc-can'), r = c.getBoundingClientRect();
    const x = Math.round((e.clientX - r.left) * W / r.width), y = Math.round((e.clientY - r.top) * H / r.height);
    const hs = [], ss = [], vs = [];
    for (let dy = -3; dy <= 3; dy++) for (let dx = -3; dx <= 3; dx++) {
      const xx = x + dx, yy = y + dy; if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
      const j = (yy * W + xx) * 3; hs.push(frame.hsv[j]); ss.push(frame.hsv[j + 1]); vs.push(frame.hsv[j + 2]);
    }
    const med = a => a.sort((p, q) => p - q)[a.length >> 1];
    lastPick = { x, y, hsv: [med(hs), med(ss), med(vs)] };
    const [h, s, v] = lastPick.hsv;
    const now = names.filter(n => inRange(ranges[n], h, s, v));
    const b = blobsNow.find(q => x >= q.x && x < q.x + q.w && y >= q.y && y < q.y + q.h);
    $('#cc-pick').innerHTML = `<div><span class="cc-sw" style="background:rgb(${[0, 1, 2].map(i => frame.img.data[(y * W + x) * 4 + i]).join(',')})"></span>
      at (${x}, ${y}) · <b class="mono">H ${h} · S ${s} · V ${v}</b> · today counts as: <b>${now.length ? esc(now.join(', ')) : 'nothing'}</b>
      ${b ? `<div class="dim">box ${b.w}×${b.h} px · area ${b.a} · fill ${f(b.fill, 2)} → ${b.ok ? 'a cube' : 'dropped: ' + b.why}</div>` : ''}</div>
      <div class="row" style="margin-top:6px">This is: ${names.map(n => `<button data-l="${esc(n)}">${esc(n)} cube</button>`).join('')}
      <button data-l="light">light / not a cube</button></div>`;
    $('#cc-pick').querySelectorAll('button[data-l]').forEach(btn => btn.onclick = () => {
      const l = btn.dataset.l; (samples[l] = samples[l] || []).push(lastPick.hsv.slice());
      if (samples[l].length > 200) samples[l].shift();
      labelsDirty = true;
      $('#cc-pick').querySelector('.row').innerHTML = `<span class="msg ok">labelled ${esc(l)} - ${samples[l].length} so far</span>`;
      paintLabels();
    });
    render();
  }
  function paintLabels() {
    const ls = [...names, 'light'];
    $('#cc-labels').innerHTML = `<table class="ov-tab">${ls.map(l => {
      const a = samples[l] || [];
      const span = i => a.length ? `${Math.min(...a.map(x => x[i]))}-${Math.max(...a.map(x => x[i]))}` : '-';
      return `<tr><td><span class="dot" style="background:${l === 'light' ? '#9aa4af' : DRAW[l] || '#fff'}"></span>${esc(l)}</td><td class="mono">${a.length}</td>
        <td class="dim mono">H ${span(0)} · S ${span(1)} · V ${span(2)}</td></tr>`;
    }).join('')}</table>`;
  }
  // ranges from the labelled clicks, with a margin; then push each bound past any "light" click that still fits,
  // if a bound can do that without losing a labelled cube.
  function fit() {
    const msg = [];
    const lights = samples.light || [];
    for (const n of names) {
      const a = samples[n] || [];
      if (a.length < 2) { msg.push(`${n}: needs at least 2 clicks (has ${a.length})`); continue; }
      const mn = i => Math.min(...a.map(x => x[i])), mx = i => Math.max(...a.map(x => x[i]));
      let s;
      if (n === 'white' || n === 'black') s = [0, 179, n === 'black' ? 0 : 0, n === 'white' ? Math.min(255, mx(1) + 12) : 255, n === 'white' ? Math.max(0, mn(2) - 18) : 0, n === 'black' ? Math.min(255, mx(2) + 18) : 255];
      else s = [Math.max(0, mn(0) - 4), Math.min(179, mx(0) + 4), Math.max(0, mn(1) - 30), 255, Math.max(0, mn(2) - 40), 255];
      // keeping the light out needs enough cube clicks (with 2 clicks a cut would also lose real cubes, 2026-11-12:
      // white ended at V 226 and bright cubes went missing) and a margin of >= 10 around every clicked cube
      let caught = lights.filter(l => inRange(s, ...l)), fixed = 0, stuck = 0, early = 0;
      if (a.length < 5) { early = caught.length; caught = []; }
      for (const l of caught) {
        // try every bound: lower bounds up to just above the light, upper bounds down to just below it
        const tries = [];
        for (const [k, i] of [[2, 1], [4, 2], [0, 0]]) if (l[i] < mn(i) - 20) tries.push([k, Math.min(mn(i) - 10, l[i] + 1 + Math.floor((mn(i) - l[i]) / 2)), mn(i) - l[i]]);
        for (const [k, i] of [[3, 1], [5, 2], [1, 0]]) if (l[i] > mx(i) + 20) tries.push([k, Math.max(mx(i) + 10, l[i] - 1 - Math.floor((l[i] - mx(i)) / 2)), l[i] - mx(i)]);
        tries.sort((p, q) => q[2] - p[2]);
        if (tries.length) { s[tries[0][0]] = tries[0][1]; fixed++; } else stuck++;
      }
      ranges[n] = s;
      msg.push(`${n}: H ${s[0]}-${s[1]}, S ${s[2]}-${s[3]}, V ${s[4]}-${s[5]} from ${a.length} clicks` +
        (fixed ? ` · kept ${fixed} light click(s) out` : '') +
        (early ? ` · ${early} light click(s) still fit - label ${5 - a.length} more ${n} cube(s) to cut the light out safely` : '') +
        (stuck ? ` · ${stuck} light click(s) have the SAME colour as a ${n} cube - only the shape filter / the moving-belt check can tell them apart` : ''));
    }
    labelsDirty = false;
    $('#cc-fitmsg').innerHTML = msg.map(esc).join('<br>') + '<br>Look at "Cubes test", then Save when it is right.';
    return msg;
    paintColors(); render();
  }

  // ---------------------------------------------------------------- colour + shape sliders
  function slider(id, label, val, min, max, step, note) {
    return `<div class="cc-sl"><label for="${id}">${esc(label)}</label><input type="range" id="${id}" min="${min}" max="${max}" step="${step}" value="${val}">
      <input type="number" class="cc-num" data-for="${id}" min="${min}" max="${max}" step="${step}" value="${val}">${note ? `<span class="dim cc-n">${esc(note)}</span>` : ''}</div>`;
  }
  function bindSliders(root, onval) {
    root.querySelectorAll('input[type=range]').forEach(r => {
      const num = root.querySelector(`input[data-for="${r.id}"]`);
      r.oninput = () => { num.value = r.value; onval(r.id, +r.value); };
      num.onchange = () => { r.value = num.value; onval(r.id, +num.value); };
    });
  }
  function paintColors() {
    const L = ['H from', 'H to', 'S from', 'S to', 'V from', 'V to'], MX = [179, 179, 255, 255, 255, 255];
    $('#cc-colors').innerHTML = names.map(n => `<div class="cc-col"><b style="color:${DRAW[n] || '#fff'}">${esc(n)}</b>
      ${ranges[n].map((v, i) => slider(`cc-c-${n}-${i}`, L[i], v, 0, MX[i], 1, '')).join('')}</div>`).join('')
      + '<div class="dim cc-note">H from > H to = the hue wraps around red. Preview only until Save.</div>';
    bindSliders($('#cc-colors'), (id, v) => { const [, , n, i] = id.split('-'); ranges[n][+i] = v; schedule(); });
  }
  function paintShape() {
    $('#cc-shape').innerHTML = SHAPE_UI.map(([k, l, a, b, st, note]) => slider('cc-s-' + k, l, shape[k], a, b, st, note)).join('')
      + '<div class="row" style="margin-top:6px"><button id="cc-shape0">Back to the tested values</button></div>';
    bindSliders($('#cc-shape'), (id, v) => { shape[id.slice(5)] = v; schedule(); });
    $('#cc-shape0').onclick = () => { shape = { ...SHAPE0 }; paintShape(); render(); };
  }
  let rt = null;
  function schedule() { clearTimeout(rt); rt = setTimeout(render, 120); }

  // ---------------------------------------------------------------- camera controls
  function paintCam() {
    const order = Object.keys(CAM_UI).filter(k => ctrls.find(c => c.name === k));
    if (!order.length) { $('#cc-cam').innerHTML = '<div class="dim">no camera controls (is the camera plugged in?)</div>'; return; }
    $('#cc-cam').innerHTML = order.map(k => {
      const c = ctrls.find(x => x.name === k), [label, note] = CAM_UI[k];
      let inp;
      if (c.type === 'menu') inp = `<select data-c="${k}">${Object.entries(c.menu).map(([v, t]) => `<option value="${v}"${+v === c.value ? ' selected' : ''}>${esc(t)}</option>`).join('')}</select>`;
      else if (c.type === 'bool') inp = `<label class="cc-sw2"><input type="checkbox" data-c="${k}"${c.value ? ' checked' : ''}> ${c.value ? 'on' : 'off'}</label>`;
      else inp = `<input type="range" data-c="${k}" min="${c.min}" max="${c.max}" step="${c.step || 1}" value="${c.value}"${c.inactive ? ' disabled' : ''}><span class="mono cc-cv">${c.value}</span>`;
      return `<div class="cc-ctl${c.inactive ? ' off' : ''}"><label>${esc(label)}</label><div>${inp}</div><span class="dim cc-n">${esc(note)}${c.inactive ? ' (not used now)' : ''}</span></div>`;
    }).join('');
    $('#cc-cam').querySelectorAll('[data-c]').forEach(el => {
      const k = el.dataset.c;
      if (el.type === 'range') {
        el.oninput = () => { el.nextElementSibling.textContent = el.value; };
        el.onchange = () => setCtl(k, +el.value);
      } else el.onchange = () => setCtl(k, el.type === 'checkbox' ? (el.checked ? 1 : 0) : +el.value);
    });
  }
  async function setCtl(name, value) {
    try {
      const r = await P.post('/api/cam/control', { name, value });
      ctrls = r.controls || ctrls; camDirty = true; paintCam();
      P.setMsg($('#cc-msg'), `camera: ${CAM_UI[name] ? CAM_UI[name][0] : name} = ${r.value} (live now; Save keeps it)`, true);
    } catch (e) { P.setMsg($('#cc-msg'), e.message, false); }
  }
  async function lockNow() {   // freeze what auto chose: exposure, white balance and focus stop moving
    const cur = k => (ctrls.find(c => c.name === k) || {}).value;
    const exp = cur('exposure_time_absolute'), wb = cur('white_balance_temperature');
    for (const [k, v] of [['auto_exposure', 1], ['exposure_dynamic_framerate', 0], ['exposure_time_absolute', exp],
      ['white_balance_automatic', 0], ['white_balance_temperature', wb], ['focus_automatic_continuous', 0], ['power_line_frequency', 1]]) {
      if (v != null && ctrls.find(c => c.name === k)) await setCtl(k, v);
    }
    P.setMsg($('#cc-msg'), 'locked: manual exposure, fixed white balance and focus, flicker 50 Hz. Adjust the exposure if it is too dark/bright, then "Measure the empty belt" and Save.', true);
  }
  async function autoAll() {
    for (const [k, v] of [['auto_exposure', 3], ['white_balance_automatic', 1], ['focus_automatic_continuous', 1]]) {
      if (ctrls.find(c => c.name === k)) await setCtl(k, v);
    }
  }
  function measureBelt() {     // median brightness inside the belt area - the belt must be EMPTY
    if (!frame) return P.setMsg($('#cc-msg'), 'no picture yet', false);
    const R = roi(), vs = [], d = frame.img.data;
    for (let i = 0; i < W * H; i += 3) if (!R || R[i]) vs.push(Math.max(d[i * 4], d[i * 4 + 1], d[i * 4 + 2]));
    vs.sort((a, b) => a - b);
    beltV = vs[vs.length >> 1]; beltVChanged = true;
    $('#cc-beltvtxt').textContent = `empty belt V = ${beltV} (was ${cal.belt_v ?? '-'}) - vision scales the white limit with it`;
  }

  // ---------------------------------------------------------------- save
  function body() {
    const b = { colors: {}, shape, samples };
    names.forEach(n => { b.colors[n] = toRanges(ranges[n]); });
    if (beltVChanged) b.belt_v = beltV;
    b.camera = {};
    ctrls.forEach(c => { if (!c.inactive || ['exposure_time_absolute', 'white_balance_temperature', 'focus_absolute'].includes(c.name)) b.camera[c.name] = c.value; });
    return b;
  }
  async function check() {
    try {
      const r = await P.post('/api/cam/preview', body());
      if (!r.changes.length) { $('#cc-diff').innerHTML = '<div class="dim" style="margin-top:8px">nothing changed</div>'; return; }
      $('#cc-diff').innerHTML = `<div style="margin-top:10px"><b>These will change</b> (the old file goes to backup/vision/ first):
        <ul class="cc-dl">${r.changes.map(c => `<li class="mono">${esc(c)}</li>`).join('')}</ul>
        ${camDirty || r.changes.some(c => c.startsWith('camera')) ? (beltVChanged ? '' : '<div class="ov-hint">→ the camera settings changed: press "Measure the empty belt" (belt empty) before saving, so vision knows the new brightness.</div>') : ''}
        <div class="row"><button class="go" id="cc-save">Save - vision uses it in ~2 s</button><button id="cc-cancel">Not yet</button></div></div>`;
      $('#cc-save').onclick = save; $('#cc-cancel').onclick = () => { $('#cc-diff').innerHTML = ''; };
    } catch (e) { P.setMsg($('#cc-msg'), e.message, false); }
  }
  async function save() {
    try {
      const r = await P.post('/api/cam/save', body());
      $('#cc-diff').innerHTML = `<div class="msg ok" style="margin:8px 0 0">Saved ${r.changes.length} change(s). Backup: ${esc(r.backup.split('/').pop())}. Vision reloads it in ~2 s (look at the live Camera page).</div>`;
      camDirty = false; await load(false);
    } catch (e) { P.setMsg($('#cc-msg'), 'not saved: ' + e.message, false); }
  }
  // the one-button save: new labels -> colours fitted first, then saved at once (no extra step)
  async function saveNow() {
    const m = $('#cc-msg2'), btn = $('#cc-savenow');
    let fitted = [];
    if (labelsDirty) fitted = fit();
    btn.disabled = true;
    try {
      const r = await P.post('/api/cam/save', body());
      const real = r.changes.filter(c => !c.startsWith('samples'));
      m.className = 'cc-note msg ok';
      m.innerHTML = `✓ Saved ${new Date().toTimeString().slice(0, 5)} - vision uses it now.` +
        (real.length ? ` <span class="dim">(${real.length} setting${real.length === 1 ? '' : 's'} changed)</span>` : ' <span class="dim">(labels kept; the colours were already like this)</span>') +
        (fitted.some(x => / needs at least/.test(x)) ? `<div class="dim">${esc(fitted.filter(x => / needs at least/.test(x)).join(' · '))} - that colour was kept as it was.</div>` : '') +
        (fitted.some(x => /label \d+ more/.test(x)) ? `<div class="dim">${esc(fitted.filter(x => /label \d+ more/.test(x)).map(x => x.split(' · ').pop()).join(' · '))}</div>` : '');
      camDirty = false; await load(false);
    } catch (e) { m.className = 'cc-note msg bad'; m.textContent = 'Not saved: ' + e.message; }
    finally { btn.disabled = false; }
  }
  async function undo() {
    if (!confirm('Go back to the calibration before the last dashboard save? (the current one is backed up too)')) return;
    try {
      const r = await P.post('/api/cam/undo', {});
      $('#cc-msg2').className = 'cc-note msg ok'; $('#cc-msg2').textContent = '↶ Back to the calibration before your last save. Vision uses it now.';
      await load(true);
    } catch (e) { $('#cc-msg2').className = 'cc-note msg bad'; $('#cc-msg2').textContent = e.message; }
  }
})();
