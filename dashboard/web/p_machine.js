// p_machine.js - "Machine": health, analysis and log on one page, in plain words (2026-11-12, the operator:
// "health, log and analysis on the same page, easy to read: the machine is in good shape - no over power, no over
// temperature"). Every tile says OK / check / problem and WHY, with the number behind it.
(function () {
  const P = window.PRISM, $ = P.$, f = P.f, esc = P.esc;
  let timer = null, ov = null, lastPaint = 0;
  const LIM = {              // the limits behind the words (the same ones the cell's own checks use)
    railOk: 11.5, railBad: 10.5,                  // 12 V rail: below 11.5 V the feeder may not push
    servoWarm: 55, servoHot: 65,                  // STS3215 servo temperature, deg C
    servoMa: 1500,                                // a servo drawing more than this = straining
    beltWarm: 45, beltHot: 60,                    // belt motor (DS18B20), deg C
    piWarm: 70, piHot: 80,                        // Raspberry Pi SoC, deg C
  };

  P.page('/machine', {
    title: 'Machine',
    mount(el) {
      el.innerHTML = `
      <div class="card"><h2>Raspberry Pi <span class="r dim" id="mc-model"></span></h2><div id="mc-pi" class="ov-pi"></div></div>
      <div class="card ov-hero" id="mc-hero" style="margin-top:12px"><div class="ov-ready"><div class="ov-word" id="mc-word">…</div>
        <div class="dim" id="mc-why">checking</div></div></div>
      <div class="mc-tiles" style="margin-top:12px" id="mc-tiles"></div>
      <div class="grid g2" style="margin-top:12px">
        <div class="card"><h2>Programs</h2><table id="mc-prog" class="ov-tab"></table></div>
        <div class="card"><h2>Connections</h2><table id="mc-hw" class="ov-tab"></table></div>
      </div>
      <div class="card" style="margin-top:12px"><h2>Log <span class="r dim">newest first</span></h2><div id="mc-log" class="ov-log"></div></div>`;
      poll();
      timer = setInterval(poll, 2000);
    },
    update(L) { const now = Date.now(); if (now - lastPaint < 1000) return; lastPaint = now; paint(L); },   // 1 Hz is plenty
    unmount() { clearInterval(timer); },
  });

  async function poll() {
    try { ov = await P.api('/api/overview'); } catch (e) { /* keep the last one */ }
  }

  const lvl = (v, warn, bad, up = true) => v == null ? 'off' : (up ? v >= bad : v <= bad) ? 'bad' : (up ? v >= warn : v <= warn) ? 'warn' : 'ok';
  const WORD = { ok: 'OK', warn: 'Check', bad: 'Problem', off: 'No data' };

  function tile(name, state, big, why) {
    return { name, state, html: `<div class="mc-tile ${state}"><h3>${esc(name)}</h3><div class="mc-big">${WORD[state]}${big ? ' · ' + esc(big) : ''}</div>
      <div class="mc-why dim">${esc(why)}</div></div>` };
  }

  // the Raspberry Pi at the top (the operator: "the old Pi parameters - temperature, CPU, RAM - on top ... show off a
  // bit"): four ring gauges that glide to the new value, a 3-minute history of CPU and temperature, and chips.
  const HIST = 90, hist = { cpu: [], temp: [] };     // one sample per /api/overview poll (2 s) = 3 min
  let lastPiT = null;
  const GAUGES = [
    { k: 'cpu', name: 'CPU', unit: '%', max: 100, warn: 70, bad: 90 },
    { k: 'mem', name: 'Memory', unit: '%', max: 100, warn: 75, bad: 90 },
    { k: 'temp', name: 'Temperature', unit: '°C', max: 85, warn: 60, bad: 75 },
    { k: 'disk', name: 'Disk used', unit: '%', max: 100, warn: 85, bad: 95 },
  ];
  const R = 42, C = 2 * Math.PI * R, ARC = 0.75;      // a 270 deg arc, open at the bottom

  function piSkeleton(el) {
    el.innerHTML = `<div class="pi-gauges">${GAUGES.map(g => `
      <div class="pi-g" id="pg-${g.k}"><svg viewBox="0 0 100 100">
        <circle class="pi-track" cx="50" cy="50" r="${R}" stroke-dasharray="${C * ARC} ${C}" transform="rotate(135 50 50)"/>
        <circle class="pi-val" cx="50" cy="50" r="${R}" stroke-dasharray="0 ${C}" transform="rotate(135 50 50)"/>
        <text x="50" y="50" class="pi-num">-</text><text x="50" y="66" class="pi-unit">${g.unit}</text></svg>
        <div class="pi-gname">${g.name}</div><div class="pi-gsub dim mono"></div></div>`).join('')}</div>
      <div class="pi-hist"><div><span class="dim">CPU · last 3 min</span><svg viewBox="0 0 300 50" preserveAspectRatio="none" id="ph-cpu"><polyline/></svg></div>
        <div><span class="dim">Temperature · last 3 min</span><svg viewBox="0 0 300 50" preserveAspectRatio="none" id="ph-temp"><polyline/></svg></div></div>
      <div class="pi-chips" id="pi-chips"></div>`;
  }

  function setGauge(g, v, sub) {
    const el = $('#pg-' + g.k); if (!el) return;
    const st = v == null ? '' : v >= g.bad ? 'bad' : v >= g.warn ? 'warn' : 'ok';
    el.className = 'pi-g ' + st;
    const frac = v == null ? 0 : Math.max(0, Math.min(1, v / g.max));
    el.querySelector('.pi-val').setAttribute('stroke-dasharray', `${C * ARC * frac} ${C}`);
    el.querySelector('.pi-num').textContent = v == null ? '-' : Math.round(v);
    el.querySelector('.pi-gsub').textContent = sub || '';
  }

  function spark(id, arr, lo, hi) {
    const pl = document.querySelector(`#${id} polyline`); if (!pl) return;
    const n = arr.length; if (n < 2) { pl.setAttribute('points', ''); return; }
    const span = Math.max(1e-6, hi - lo);
    pl.setAttribute('points', arr.map((v, i) => `${(300 * (i + HIST - n) / (HIST - 1)).toFixed(1)},${(48 - 46 * (Math.min(hi, Math.max(lo, v)) - lo) / span).toFixed(1)}`).join(' '));
  }

  function paintPi(pi) {
    const el = $('#mc-pi'); if (!el) return;
    if (!el.querySelector('.pi-gauges')) piSkeleton(el);
    $('#mc-model').textContent = pi.model || '';
    if (pi.t != null && pi.t !== lastPiT) {           // a new sample (the page repaints faster than the Pi is read)
      lastPiT = pi.t;
      if (pi.cpu_pct != null) { hist.cpu.push(pi.cpu_pct); if (hist.cpu.length > HIST) hist.cpu.shift(); }
      if (pi.temp_c != null) { hist.temp.push(pi.temp_c); if (hist.temp.length > HIST) hist.temp.shift(); }
    }
    const memPct = pi.mem_total_mb ? 100 * pi.mem_used_mb / pi.mem_total_mb : null;
    const avg = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : null;
    setGauge(GAUGES[0], pi.cpu_pct, hist.cpu.length > 1 ? `avg ${f(avg(hist.cpu), 0)} %` : '');
    setGauge(GAUGES[1], memPct, pi.mem_total_mb ? `${(pi.mem_used_mb / 1024).toFixed(1)} / ${(pi.mem_total_mb / 1024).toFixed(1)} GB` : '');
    setGauge(GAUGES[2], pi.temp_c, hist.temp.length > 1 ? `max ${f(Math.max(...hist.temp), 1)} °C` : '');
    setGauge(GAUGES[3], pi.disk_used_pct, pi.disk_free_gb != null ? `${f(pi.disk_free_gb, 1)} GB free` : '');
    spark('ph-cpu', hist.cpu, 0, 100);
    const tl = hist.temp.length ? Math.min(...hist.temp) : 40, th = hist.temp.length ? Math.max(...hist.temp) : 60;
    spark('ph-temp', hist.temp, Math.floor(tl - 2), Math.ceil(th + 2));
    const thr = pi.throttled;
    const pwr = !thr ? ['', 'power: unknown'] : thr.flags.some(x => x.endsWith('now')) ? ['bad', 'power: ' + thr.flags.join(', ')]
      : thr.flags.length ? ['warn', 'power: ' + thr.flags.join(', ')] : ['ok', 'power supply good'];
    const cores = 4, ld = (pi.load || [])[0];
    const chip = (cls, txt) => `<span class="pi-chip ${cls}"><span class="dot ${cls}"></span>${esc(txt)}</span>`;
    $('#pi-chips').innerHTML = [
      chip('', `${pi.mhz ?? '-'} MHz`),
      chip(ld == null ? '' : ld > 2 * cores ? 'bad' : ld > cores ? 'warn' : 'ok', `load ${(pi.load || []).join(' / ') || '-'}`),
      chip('', `up ${pi.uptime_s != null ? P.dur(pi.uptime_s) : '-'}`),
      chip(pwr[0], pwr[1]),
    ].join('');
  }

  function paint(L) {
    if (!$('#mc-tiles')) return;
    const t = [];
    const b = L.belt && L.belt.d, bAge = L.belt && L.belt.age;
    const fresh = bAge != null && bAge < 5;
    // 12 V power
    const v = fresh ? b.voltage : null;
    const vs = v == null ? 'off' : v < LIM.railBad ? 'bad' : v < LIM.railOk ? 'warn' : 'ok';
    t.push(tile('12 V power', vs, v == null ? '' : f(v, 2) + ' V',
      v == null ? 'no belt telemetry (is autoen running?)' : vs === 'ok' ? 'steady - the feeder and the arm have full power'
        : vs === 'warn' ? `below ${LIM.railOk} V: pushes may be weak (the arm's grip pulls the shared supply down)`
        : `below ${LIM.railBad} V: the supply is collapsing - check it`));
    // belt motor
    const bt = fresh ? b.temp : null, bi = fresh ? b.current : null;
    const ab = L.an_belt && L.an_belt.d, blev = ab && ab.levels || {};
    let bs = lvl(bt, LIM.beltWarm, LIM.beltHot);
    if (blev.current === 'ALARM' || blev.temp === 'ALARM') bs = 'bad'; else if ((blev.current === 'WARN' || blev.temp === 'WARN') && bs === 'ok') bs = 'warn';
    t.push(tile('Belt motor', bs, bt == null ? '' : f(bt, 0) + ' °C',
      bt == null ? 'no data yet' : `${f(bi, 0)} mA · current ${blev.current || '-'} · temperature ${blev.temp || '-'}` +
        (bs === 'ok' ? ' - not over-powered, not hot' : '')));
    // arm servos
    const J = Object.values((L.arm && L.arm.joints) || {});
    const temps = J.map(j => j && j.temp).filter(x => x != null), curs = J.map(j => j && Math.abs(j.cur || 0));
    const tmax = temps.length ? Math.max(...temps) : null, cmax = curs.length ? Math.max(...curs) : null;
    let as = lvl(tmax, LIM.servoWarm, LIM.servoHot);
    if (cmax != null && cmax > LIM.servoMa && as === 'ok') as = 'warn';
    t.push(tile('Arm servos', as, tmax == null ? '' : `max ${f(tmax, 0)} °C`,
      tmax == null ? 'no arm telemetry yet' : `hottest ${f(tmax, 0)} °C (limit ${LIM.servoWarm}) · highest current ${f(cmax, 0)} mA (limit ${LIM.servoMa})`));
    // analysis
    const aa = L.an_arm && L.an_arm.d;
    const astate = aa ? aa.state : null;
    t.push(tile('Arm analysis', !aa ? 'off' : /ALARM|FAULT|OFF_PATH/.test(astate) ? 'bad' : /WARN|NO_DATA/.test(astate) ? 'warn' : 'ok',
      astate || '', aa ? (aa.detail || '') + (aa.cycles_ok != null ? ` · ${aa.cycles_ok} good cycles` : '') : 'the arm analyzer is not running'));
    t.push(tile('Belt analysis', !ab ? 'off' : /ALARM|FAULT/.test(ab.state) ? 'bad' : /WARN|NO_DATA/.test(ab.state) ? 'warn' : 'ok',
      ab ? ab.state : '', ab ? (ab.detail || '') + (ab.runs_total != null ? ` · ${ab.runs_ok}/${ab.runs_total} runs OK` : '') : 'the belt analyzer is not running'));
    // the Pi
    const pi = (ov && ov.pi) || {};
    const thr = pi.throttled, under = thr && thr.flags && thr.flags.some(x => x.endsWith('now'));
    let ps = lvl(pi.temp_c, LIM.piWarm, LIM.piHot);
    if (under) ps = 'bad';
    const mem = pi.mem_total_mb ? Math.round(100 * pi.mem_used_mb / pi.mem_total_mb) : null;
    if (ps === 'ok' && ((pi.cpu_pct || 0) > 90 || (mem || 0) > 92)) ps = 'warn';
    t.push(tile('Raspberry Pi', ov ? ps : 'off', pi.temp_c == null ? '' : f(pi.temp_c, 0) + ' °C',
      !ov ? 'waiting' : under ? 'UNDER-VOLTAGE now: the Pi power supply is too weak'
        : `CPU ${f(pi.cpu_pct, 0)} % · memory ${mem ?? '-'} % · power supply ${thr && thr.flags && thr.flags.length ? thr.flags.join(', ') : 'good'}`));
    t.push(tile('Messages (MQTT)', L.mqtt ? 'ok' : 'bad', L.mqtt ? 'connected' : 'DOWN',
      L.mqtt ? 'every program can talk to the others' : 'the broker is down: sudo systemctl start mosquitto'));

    $('#mc-tiles').innerHTML = t.map(x => x.html).join('');
    const bad = t.filter(x => x.state === 'bad'), warn = t.filter(x => x.state === 'warn');
    const machineOff = ['12 V power', 'Belt motor', 'Arm servos'].every(n => t.find(x => x.name === n).state === 'off');
    const hero = $('#mc-hero');
    hero.className = 'card ov-hero ' + (bad.length ? 'notready' : machineOff ? '' : 'ready');
    $('#mc-word').textContent = bad.length ? 'Needs attention' : machineOff ? 'Not running'
      : warn.length ? 'Good, with notes' : 'In good shape';
    $('#mc-why').textContent = bad.length ? 'Problem: ' + bad.map(x => x.name).join(', ')
      : machineOff ? 'No data from the arm, the belt or the power sensor - start the cell to check it.'
      : warn.length ? 'Check: ' + warn.map(x => x.name).join(', ') + ' - nothing is over the limits'
      : 'No over-power, no over-temperature, everything connected.';

    if (ov) paintPi(ov.pi || {});
    if (ov) {
      const DOT = { ok: 'ok', warn: 'warn', bad: 'bad', off: '' };
      $('#mc-prog').innerHTML = ov.programs.map(x => `<tr><td><span class="dot ${DOT[x.state] || ''}"></span>${esc(x.name)}</td>
        <td class="l ov-txt">${esc(x.text)}</td><td class="dim">${x.cpu_pct == null ? '' : f(x.cpu_pct, 1) + ' %'}</td></tr>`).join('');
      $('#mc-hw').innerHTML = ov.hardware.map(x => `<tr><td><span class="dot ${DOT[x.state] || ''}"></span>${esc(x.name)}</td>
        <td class="l ov-txt">${esc(x.text)}${x.hint ? `<div class="ov-hint">→ ${esc(x.hint)}</div>` : ''}</td></tr>`).join('');
    }
    const ev = (P.events || []).slice(-40).reverse();
    $('#mc-log').innerHTML = ev.map(e => `<div class="ev ${e.level}"><span class="t">${P.hms(e.t)}</span><span class="s">${esc(e.src)}</span><span class="x">${esc(e.text)}</span></div>`).join('')
      || '<div class="dim">nothing yet</div>';
  }
})();
