/* OVERVIEW page: the whole cell on one screen.
   READY  ·  clock  ·  Raspberry Pi  ·  programs  ·  hardware  ·  the optimizer talking  ·  vision + case array  ·  autoen  ·  log
   Data: /api/stream (10 Hz snapshot, painted at most 2x/s here), /api/overview (Pi, programs, hardware, READY; 2 s),
         /api/trace (the optimizer's sentences, when L.trace_last moves), /api/cases (10 s), /api/control/state (2 s). */
(function () {
  const P = window.PRISM, $ = P.$, f = P.f, esc = P.esc;
  const DOT = { ok: 'ok', warn: 'warn', bad: 'bad', off: '' };
  let lastAeStatus = null;
  let timers = [], traceSince = 0, traceAll = [], lastPaint = 0, ctl = null, editUnlocked = false, cfgFilled = false;

  // the PRISM mascot: a cloud whose face follows its mood (CSS colours it via .mood-*)
  const CLOUD_SVG = `<svg viewBox="0 0 120 80" class="cl-svg"><path class="cl-body" d="M30 68 Q8 68 10 50 Q12 34 30 36 Q34 16 56 16 Q76 16 82 32 Q100 28 106 44 Q114 66 92 68 Z"/>
    <g class="cl-eyes-open"><circle cx="48" cy="46" r="4"/><circle cx="72" cy="46" r="4"/></g>
    <g class="cl-eyes-shut"><path d="M43 46 q5 4 10 0 M67 46 q5 4 10 0"/></g>
    <path class="cl-mouth" d="M52 56 q8 6 16 0"/></svg>`;

  P.page('/overview', {
    title: 'Overview',
    mount(el) {
      traceSince = 0; traceAll = []; ctl = null; editUnlocked = false; cfgFilled = false;
      el.innerHTML = `
      <div class="ov-alarm card" id="ov-alarm" hidden></div>
      <div class="card cl-card mood-sleep" id="cl-card">
        <div class="cl-top">
          <div class="cl-mascot" id="cl-mascot">${CLOUD_SVG}</div>
          <div class="cl-say"><div class="cl-name">PRISM <span id="ov-optmode"></span></div>
            <div class="cl-head" id="cl-head">…</div><div class="cl-detail dim" id="cl-detail"></div></div>
        </div>
        <details class="cl-more" id="cl-more"><summary>What PRISM is doing (pushes, decisions, stories)</summary>
          <div class="ov-now" id="ov-now">…</div>
          <div class="ov-chips" id="ov-chips"></div>
          <div class="ov-pushes" id="ov-pushes" hidden></div>
          <div class="ov-chat" id="ov-chat"><div class="dim">waiting for PRISM's first words…</div></div>
        </details>
      </div>
      <div class="ov-hero card" id="ov-hero" style="margin-top:12px">
        <div class="ov-ready"><div class="ov-word" id="ov-word">…</div><div id="ov-readytext" class="dim">checking every part of the cell</div>
          <ul id="ov-missing" class="ov-missing"></ul></div>
        <div class="ov-clock"><div class="ov-time mono" id="ov-time">--:--:--</div><div class="dim" id="ov-date"></div>
          <div class="dim" id="ov-up"></div><div style="margin-top:6px"><span class="pill" id="ov-run">autoen: no data</span></div></div>
      </div>


      <div class="grid g2" style="margin-top:12px">
        <div class="card"><h2>Vision <span class="r dim" id="ov-camhead"></span></h2>
          <div class="ov-live"><img id="ov-img" alt="live camera"><div class="ov-noimg" id="ov-noimg" hidden>no live picture: is vision running?</div></div>
          <div class="grid g3 ov-mini" id="ov-cam"></div>
          <h2 style="margin-top:12px">Case array <span class="r dim" id="ov-casefile"></span></h2>
          <div class="scroll" style="max-height:230px"><table id="ov-cases"></table></div></div>
        <div class="card"><h2>Autoen <span class="r"><a href="#/control/run">all settings →</a></span></h2>
          <div class="row"><div class="big" id="ov-ae">-</div><span class="spacer"></span>
            <button class="go" id="ov-start">&#9654; Start</button><button class="warn" id="ov-pause">&#10074;&#10074; Pause</button><button class="stop" id="ov-stop">&#9632; Stop</button></div>
          <div class="dim" id="ov-aeprog" style="margin:6px 0"></div>
          <label class="row" id="ov-clearrow" style="font-size:13px"><input type="checkbox" id="ov-clear"> Arm area is clear and the E-stop is within reach</label>
          <div class="ov-lock" id="ov-lock" hidden></div>
          <form class="form" id="ov-form" style="margin-top:8px">
            <label>Belt rpm<input name="rpm" type="number" min="0.5" max="37" step="0.1"></label>
            <label>Arm speed<input name="arm_speed" type="number" min="0.2" max="8.8" step="0.1"></label>
            <label>Carry speed<input name="carry_speed" type="number" min="0.2" max="8.8" step="0.1"></label>
            <label>Arrive timeout s<input name="arrive_timeout" type="number" min="5" max="120" step="5"></label>
          </form>
          <div class="row" style="margin-top:8px"><button class="pri" id="ov-apply">Apply</button><span class="msg" id="ov-msg"></span></div>
          <h2 style="margin-top:12px">Last cycles</h2><div class="scroll" style="max-height:170px"><table id="ov-cyc"></table></div></div>
      </div>

      <div class="card" style="margin-top:12px"><h2>Log <span class="r dim">every program · newest first · the full list is in Events below</span></h2>
        <div id="ov-log" class="ov-log"></div></div>`;

      const form$ = () => { const o = {}; new FormData($('#ov-form')).forEach((v, k) => { if (v !== '') o[k] = +v; }); return o; };
      const act = async (name, body) => {
        try { await P.post('/api/control/' + name, body); P.setMsg($('#ov-msg'), name + ' sent', true); pollCtl(); }
        catch (e) { P.setMsg($('#ov-msg'), e.message, false); }
      };
      $('#ov-start').onclick = () => act('start', { clear: $('#ov-clear').checked, config: form$() });
      $('#ov-pause').onclick = () => act('pause', {});
      $('#ov-stop').onclick = () => { if (confirm('Stop the run? (the arm holds, the belt coasts; the block is ready for a new run afterwards)')) act('stop', {}); };
      $('#ov-apply').onclick = (e) => { e.preventDefault(); act('config', form$()); };
      if (P.info.read_only) ['#ov-start', '#ov-pause', '#ov-stop', '#ov-apply'].forEach(s => { $(s).disabled = true; });

      P.liveImg($('#ov-img'));
      pollOverview(); pollTrace(); pollCases(); pollCtl();
      timers = [setInterval(pollOverview, 2000), setInterval(pollCases, 10000), setInterval(pollCtl, 2000),
                setInterval(clock, 1000)];
      clock();
    },
    update(L) {
      if (L.trace_last && L.trace_last > traceSince) pollTrace();
      const now = performance.now();
      if (now - lastPaint < 1000) return;              // repaint at most once a second (lighter for the Pi's browser)
      lastPaint = now;
      paintOptimizer(L); paintVision(L); paintLog(); paintAutoenLive(L);
    },
    unmount() { timers.forEach(clearInterval); timers = []; }
  });

  // ---------------------------------------------------------------- clock
  function clock() {
    if (!$('#ov-time')) return;
    const d = new Date();
    $('#ov-time').textContent = d.toTimeString().slice(0, 8);
    $('#ov-date').textContent = d.toLocaleDateString(undefined, { weekday: 'long', day: 'numeric', month: 'long', year: 'numeric' });
  }

  // ---------------------------------------------------------------- READY, Pi, programs, hardware
  async function pollOverview() {
    let o; try { o = await P.api('/api/overview'); } catch (e) { return; }
    if (!$('#ov-word')) return;
    const r = o.ready, hero = $('#ov-hero');
    hero.className = 'ov-hero card ' + (r.ready ? 'ready' : 'notready');
    $('#ov-word').textContent = r.ready ? 'READY' : 'NOT READY';
    $('#ov-readytext').textContent = r.ready
      ? 'Everything is connected. Belt empty, cubes in the feeder, then press Start.'
      : `${r.missing.length} thing${r.missing.length > 1 ? 's' : ''} still missing. The page keeps working with whatever is connected.`;
    $('#ov-missing').innerHTML = r.missing.map(m => `<li class="bad">${esc(m)}</li>`).join('') +
      r.notes.map(m => `<li class="warn">${esc(m)}</li>`).join('');

    paintCloud(o.cloud);
    const pi = o.pi || {};
    $('#ov-up').textContent = pi.uptime_s != null ? 'Pi up ' + P.dur(pi.uptime_s) : '';
    if (!$('#ov-pi')) return;                       // Pi / programs / hardware live on the Machine page now
    $('#ov-model').textContent = pi.model || '';
    $('#ov-up').textContent = pi.uptime_s != null ? 'Pi up ' + P.dur(pi.uptime_s) : '';
    const tcls = pi.temp_c == null ? '' : pi.temp_c < 60 ? 'ok' : pi.temp_c < 75 ? 'warn' : 'bad';
    const memPct = pi.mem_total_mb ? 100 * pi.mem_used_mb / pi.mem_total_mb : null;
    const thr = pi.throttled;
    const pwr = !thr ? ['', 'unknown'] : thr.flags.some(x => x.endsWith('now')) ? ['bad', thr.flags.join(', ')]
      : thr.flags.length ? ['warn', thr.flags.join(', ')] : ['ok', 'good'];
    const bar = (pct, cls) => `<div class="bar ov-bar ${cls || ''}"><i style="width:${Math.min(100, pct || 0)}%"></i></div>`;
    $('#ov-pi').innerHTML = `
      <div class="ov-pirow"><span>CPU</span><b class="mono">${f(pi.cpu_pct, 0)} %</b>${bar(pi.cpu_pct, pi.cpu_pct > 90 ? 'bad' : pi.cpu_pct > 70 ? 'warn' : '')}</div>
      <div class="ov-pirow"><span>Memory</span><b class="mono">${pi.mem_used_mb ?? '-'} / ${pi.mem_total_mb ?? '-'} MB</b>${bar(memPct, memPct > 90 ? 'bad' : memPct > 75 ? 'warn' : '')}</div>
      <div class="ov-pirow"><span>Disk</span><b class="mono">${f(pi.disk_free_gb, 1)} GB free</b>${bar(pi.disk_used_pct, pi.disk_used_pct > 90 ? 'bad' : '')}</div>
      <dl class="kv" style="margin-top:8px">
        <dt>Temperature</dt><dd><span class="dot ${tcls}"></span>${f(pi.temp_c, 1)} °C</dd>
        <dt>CPU speed</dt><dd>${pi.mhz ?? '-'} MHz</dd>
        <dt>Load (1 / 5 / 15 min)</dt><dd>${(pi.load || []).join(' / ') || '-'}</dd>
        <dt>Power supply</dt><dd><span class="dot ${pwr[0]}"></span>${esc(pwr[1])}</dd>
      </dl>`;

    $('#ov-prog').innerHTML = o.programs.map(x => `<tr title="${esc(x.pids.length ? 'pid ' + x.pids.join(', ') : '')}">
      <td><span class="dot ${DOT[x.state]}"></span>${esc(x.name)}${x.required ? '' : ' <span class="tag">extra</span>'}${x.twice ? ' <span class="tag warnc">twice</span>' : ''}</td>
      <td class="l ov-txt">${esc(x.text)}</td><td class="dim">${x.cpu_pct == null ? '' : f(x.cpu_pct, 1) + ' %'}</td></tr>`).join('');
    $('#ov-hw').innerHTML = o.hardware.map(x => `<tr>
      <td><span class="dot ${DOT[x.state]}"></span>${esc(x.name)}</td>
      <td class="l ov-txt">${esc(x.text)}${x.hint ? `<div class="ov-hint">→ ${esc(x.hint)}</div>` : ''}</td></tr>`).join('');
  }

  // ---------------------------------------------------------------- the cloud (PRISM's voice) and its button
  function paintCloud(c) {
    const card = $('#cl-card'); if (!card || !c) return;
    const cls = 'card cl-card mood-' + (c.mood || 'info');
    if (card.className !== cls) card.className = cls;
    if ($('#cl-head').textContent !== c.headline) $('#cl-head').textContent = c.headline || '';
    if ($('#cl-detail').textContent !== c.detail) $('#cl-detail').textContent = c.detail || '';
  }

  // ---------------------------------------------------------------- the optimizer
  async function pollTrace() {
    let j; try { j = await P.api('/api/trace?since=' + traceSince); } catch (e) { return; }
    if (!j.trace.length) return;
    j.trace.forEach(e => { traceAll.push(e); traceSince = Math.max(traceSince, e.id); });
    if (traceAll.length > 300) traceAll.splice(0, traceAll.length - 300);
    paintChat();
  }
  function paintChat() {
    const box = $('#ov-chat'); if (!box) return;
    const atBottom = box.scrollHeight - box.scrollTop - box.clientHeight < 40;
    let html = '', inHistory = null;
    traceAll.forEach(e => {
      if (e.history !== inHistory) {
        html += `<div class="ov-sep">${e.history ? 'earlier (from the optimizer\'s log)' : 'live'}</div>`;
        inHistory = e.history;
      }
      const s = e.setting && e.setting.rpm != null ? `<span class="tag">${f(e.setting.rpm, 1)} rpm · arm ${f(e.setting.arm, 1)}</span>` : '';
      html += `<div class="ov-say ${e.level}${e.history ? ' old' : ''}"><div class="ov-meta"><span class="mono">${P.hms(e.t)}</span>${s}</div>
        <div>${esc(e.say)}</div>${e.why ? `<div class="ov-why">its reason: ${esc(e.why)}</div>` : ''}</div>`;
    });
    box.innerHTML = html;
    if (atBottom || !box.dataset.seen) { box.scrollTop = box.scrollHeight; box.dataset.seen = 1; }
  }
  // the feeder stopped delivering (optimize.py "feed stop"): a big banner on top, gone by itself when it carries on
  function paintAlarm(fs, reason) {
    const box = $('#ov-alarm'); if (!box) return;
    if (!fs) { box.hidden = true; return; }
    const dead = fs.kind === 'dead', next = fs.next_retry_s;
    const mins = s => s >= 90 ? `${Math.round(s / 60)} min` : `${s} s`;
    box.className = 'ov-alarm card ' + (dead ? 'bad' : 'info');
    box.innerHTML = `<div class="ov-alarm-t">${dead ? '⚠ ALARM: the feeder servo does not move' : 'Feeder empty: load cubes'}</div>
      <div>${dead ? 'No current on the push (12 V dip ' + esc(fs.dip_mV) + ' mV). Check the feeder servo\'s D12 plug and its 5 V supply, and that no cube is jammed.'
                  : 'The feeder IR sees no cube and the push drew no current. Put cubes in the feeder: I start again by myself.'}</div>
      <div class="dim">${dead ? 'No Resume needed: ' : ''}${next != null ? `next try in ${mins(next)}` : 'waiting for the pause to land'}${dead ? ' (or 5 s after you touch the feeder)' : ''}
        · for ${mins(fs.since_s)}${fs.tries ? ` · ${fs.tries} retr${fs.tries > 1 ? 'ies' : 'y'} so far` : ''}</div>`;
    box.hidden = false;
  }
  // pt2: the evidence of the last pushes (servo current, camera, feeder IR) and the verdict, newest first
  function paintPushes(list, flow) {
    const box = $('#ov-pushes'); if (!box) return;
    if (!list || !list.length) { box.hidden = true; return; }
    const cls = v => /=> FED/.test(v) ? 'ok' : /=> EMPTY/.test(v) ? 'bad' : 'warn';
    const tu = flow && flow.tuner, tr = tu && tu.trial;
    const head = flow ? `<div class="dim">Last pushes · ${esc(flow.in_flight)} of ${esc(flow.want)} cubes on the belt` +
      (flow.ride_mm ? ` · ${esc(flow.ride_mm)} mm feeder→IR, gap ${esc(flow.gap_mm)} mm` : ` · ride ${esc(flow.ride_s)} s`) +
      (flow.push_ms ? ` · push ${esc(flow.push_ms)} ms` : '') + `</div>` +
      (tr ? `<div class="dim">Experiment: ${esc(tr.knob)} = ${esc(tr.value)} (${esc(tr.left)} picks left)</div>`
          : (tu && tu.last ? `<div class="dim">Last experiment: ${esc(tu.last)}</div>` : '')) : '';
    box.innerHTML = head + list.slice().reverse().map(x => {
      const [what, verdict] = x.split(' => ');
      return `<div class="ov-push ${cls(x)}"><b>${esc(verdict || '')}</b> ${esc(what)}</div>`;
    }).join('');
    box.hidden = false;
  }
  function paintOptimizer(L) {
    const st = L.orchestrator && L.orchestrator.d, age = L.orchestrator && L.orchestrator.age;
    const now = $('#ov-now'); if (!now) return;
    if (!st || age == null || age > 10 || L.orchestrator.retained) {
      now.innerHTML = `<span class="dim">I'm not running, so nobody is choosing the settings. Start the cell (./start_cell.sh) and I'll take over.</span>`;
      $('#ov-optmode').innerHTML = '<span class="pill">not running</span>'; $('#ov-chips').innerHTML = '';
      $('#ov-alarm').hidden = true; return;
    }
    const sc = st.score || {}, mode = st.mode;
    paintAlarm(sc.feed_stop, st.reason);
    paintPushes(sc.pushes, sc.flow);
    const cls = { RUN: 'ok', IDLE: 'info', ALERT: 'bad', RECOVER: 'warn', MANUAL: 'warn', COOL: 'warn' }[mode] || '';
    $('#ov-optmode').innerHTML = `<span class="pill ${cls}">${esc(mode)}${st.shadow ? ' · shadow' : ''}</span>`;
    let s = `Right now I run the belt at <b>${f(st.rpm, 1)} rpm</b>, the arm at <b>${f(st.arm_speed, 1)}</b> and the carry at <b>${f(st.carry_speed, 1)}</b>.`;
    if (sc.n_live) s += ` On this setting <b>${sc.ok_live} of ${sc.n_live}</b> cycles went OK, so I'm at least <b>${Math.round(100 * sc.p_low)} %</b> sure the next one will.`;
    if (sc.t_live) s += ` A cycle takes about <b>${f(sc.t_live, 1)} s</b>` + (sc.J_live ? ` and <b>${f(sc.J_live, 1)} J</b> per cube.` : '.');
    if (sc.trial) s += ` I'm testing <b>${esc(sc.trial.setting)}</b>: ${sc.trial.left} cycles to go.`;
    if ((mode === 'IDLE' || mode === 'ALERT') && sc.next_probe_s != null) s += ` I'll try one feed in about ${sc.next_probe_s} s.`;
    if (sc.cam_trusted === false) s += ` I don't trust the camera yet (seen ${esc(sc.cam_seen || '-')}).`;
    if (sc.heat && sc.heat !== 'ok') s += ` The motors are ${esc(sc.heat)}.`;
    if (st.shadow) s += ' <i>Shadow mode: I only think out loud and send nothing.</i>';
    if (st.last_error) s += `<div class="ov-why">last problem: ${esc(st.last_error)}</div>`;
    now.innerHTML = s + `<div class="ov-why">what I'm doing: ${esc(st.reason || '')}</div>`;
    const chip = (k, v, c) => `<div class="ov-chip ${c || ''}"><span>${k}</span><b class="mono">${v}</b></div>`;
    const beyond = (sc.beyond_grid || []).length;
    $('#ov-chips').innerHTML = chip('success ≥', sc.p_low != null ? Math.round(100 * sc.p_low) + ' %' : '-', sc.p_low >= 0.9 ? 'ok' : sc.p_low >= 0.7 ? 'warn' : 'bad')
      + chip('s / cycle', f(sc.t_live, 1)) + chip('J / cube', f(sc.J_live, 1))
      + chip('camera', sc.cam_trusted ? 'trusted' : 'not yet', sc.cam_trusted ? 'ok' : 'warn')
      + chip('found beyond the old limits', beyond + ' setting' + (beyond === 1 ? '' : 's'), beyond ? 'ok' : '')
      + chip('autoen', esc(st.autoen || '-'));
  }

  // ---------------------------------------------------------------- vision + cases
  function paintVision(L) {
    const box = $('#ov-cam'); if (!box) return;
    $('#ov-noimg').hidden = !(L.jpeg_age == null || L.jpeg_age > 5);
    const per = L.camera && L.camera.perception, d = per && per.d;
    const live = d && per.age != null && per.age < 3 && !per.retained;
    $('#ov-camhead').textContent = live ? `${f(d.fps, 1)} fps · ${d.active} tracks` : 'no live camera data';
    const st = L.orchestrator && L.orchestrator.d, scene = st && st.score && st.score.scene || {};
    const onBelt = {}; (live ? d.tracks || [] : []).forEach(t => { const c = t.color || '?'; onBelt[c] = (onBelt[c] || 0) + 1; });
    const win = L.camera && L.camera.window, dem = L.camera && L.camera.demand;
    const colours = o => Object.entries(o || {}).map(([k, v]) => `<span class="ov-col ov-c-${esc(k)}">${esc(k)} <b>${v}</b></span>`).join(' ') || '<span class="dim">-</span>';
    const lightCls = { ok: 'ok', dim: 'warn', dark: 'bad', glare: 'bad' }[scene.light] || '';
    const sb = d && d.scene || {};
    box.innerHTML = `
      <div><div class="dim">light</div><div><span class="dot ${lightCls}"></span><b>${esc(scene.light || '-')}</b></div>
        <div class="dim" style="font-size:12px">V ${f(sb.v_mean, 0)} · glare ${f(sb.glare_pct, 1)} % · dark ${f(sb.dark_pct, 1)} %</div></div>
      <div><div class="dim">on the belt now</div><div>${colours(onBelt)}</div>
        <div class="dim" style="font-size:12px">waiting ${scene.backlog ?? '-'} · ${scene.train ? 'cubes touching' : 'spaced'}${scene.min_gap_mm != null ? ' · gap ' + f(scene.min_gap_mm, 0) + ' mm' : ''}</div></div>
      <div><div class="dim">counted in the last 10 s</div><div>${colours(dem && dem.by_color)}</div>
        <div class="dim" style="font-size:12px">last 3-min window: ${win ? win.total + ' · ' : ''}${win ? Object.entries(win.by_color || {}).map(([k, v]) => k + ' ' + v).join(', ') : '-'}</div></div>
      ${scene.stuck ? `<div class="ov-alert">a cube is stuck at u=${scene.stuck.u} for ${scene.stuck.secs} s</div>` : ''}`;
  }
  async function pollCases() {
    let j; try { j = await P.api('/api/cases?limit=12'); } catch (e) { return; }
    if (!$('#ov-cases')) return;
    $('#ov-casefile').textContent = j.file ? `${j.file} · ${j.total} rows · one row per cycle` : 'no cases yet: one row per cycle once a run starts';
    $('#ov-cases').innerHTML = '<tr><th>#</th><th class="l">result</th><th>s</th><th>rpm/arm</th><th class="l">colour</th><th>sure</th><th class="l">light</th><th>waiting</th><th>slip</th></tr>' +
      j.rows.slice().reverse().map(r => `<tr><td>${esc(r.n)}</td><td class="l ${r.result === 'OK' ? 'lvl-OK' : 'lvl-ALARM'}">${esc(r.result)}</td><td>${esc(r.secs)}</td>
        <td>${esc((+r.rpm || 0).toString())}/${esc((+r.arm_speed || 0).toString())}</td><td class="l">${esc(r.colour || '-')}</td>
        <td>${r.colour_share ? Math.round(100 * r.colour_share) + ' %' : '-'}</td><td class="l">${esc(r.light)}</td><td>${esc(r.backlog_max)}</td><td>${esc(r.slip || '-')}</td></tr>`).join('');
  }

  // ---------------------------------------------------------------- autoen
  async function pollCtl() {
    let keeper = null;
    try { ctl = await P.api('/api/control/state'); keeper = ctl.keeper || null; }
    catch (e) { ctl = null; keeper = e.body && e.body.keeper; }
    if (!$('#ov-ae')) return;
    if (!ctl) {
      lastAeStatus = null;
      $('#ov-ae').textContent = keeper ? 'new run' : 'not reachable';
      $('#ov-aeprog').textContent = keeper ? keeper + ' …' : 'autoen is not running: start the cell';
      ['#ov-start', '#ov-pause', '#ov-stop'].forEach(sl => { $(sl).style.display = 'none'; });
      return;
    }
    $('#ov-stop').style.display = '';
    const ready = ctl.status === 'ready';
    if (ready && lastAeStatus !== 'ready') {         // a fresh run: the block starts over like a new page
      cfgFilled = false; editUnlocked = false;
      $('#ov-clear').checked = false;
      $('#ov-cyc').innerHTML = '';
    }
    lastAeStatus = ctl.status;
    $('#ov-ae').textContent = ready ? 'ready' : `${ctl.status} · ${ctl.phase}`;
    const tot = ctl.sets * ctl.cycles_per_set;
    $('#ov-aeprog').innerHTML = keeper ? esc(keeper) + ' …'
      : ready ? 'Ready for a new run: set the values, tick the box, press <b>Start</b>.'
      : `cycle <b>${ctl.cycle_total}</b>${tot < 100000 ? ' of ' + tot : ''} · running ${P.dur(ctl.elapsed || 0)}${ctl.message ? ' · ' + esc(ctl.message) : ''}`;
    $('#ov-start').style.display = ready ? '' : 'none'; $('#ov-clearrow').style.display = ready ? '' : 'none';
    $('#ov-pause').style.display = ready ? 'none' : '';
    $('#ov-pause').innerHTML = ctl.paused ? '&#9654; Resume' : '&#10074;&#10074; Pause';
    if (!cfgFilled && ctl.config) {
      cfgFilled = true;
      ['rpm', 'arm_speed', 'carry_speed', 'arrive_timeout'].forEach(k => { const i = $(`#ov-form [name=${k}]`); if (i && ctl.config[k] != null) i.value = ctl.config[k]; });
    }
    $('#ov-cyc').innerHTML = '<tr><th>#</th><th class="l">result</th><th>s</th><th class="l">notes</th></tr>' +
      (ctl.cycles || []).slice(-12).reverse().map(c => `<tr><td>${c.n}</td><td class="l ${c.result === 'OK' ? 'lvl-OK' : 'lvl-ALARM'}">${esc(c.result)}</td><td>${c.secs}</td><td class="l">${esc(c.notes || '')}</td></tr>`).join('');
  }
  function paintAutoenLive(L) {
    const ae = L.autoen && L.autoen.d;
    const pill = $('#ov-run'); if (!pill) return;
    if (ae && L.autoen.age < 5) { pill.textContent = `autoen ${ae.status} · ${ae.phase} · cycle ${ae.cycle_total}`; pill.className = 'pill ' + (ae.status === 'running' ? 'ok' : ae.status === 'error' ? 'bad' : 'info'); }
    else { pill.textContent = 'autoen: no data'; pill.className = 'pill'; }
    // while the optimizer runs (not shadow) it owns rpm / arm / carry / timeout: a typed value would be changed back
    const st = L.orchestrator && L.orchestrator.d, age = L.orchestrator && L.orchestrator.age;
    const owns = st && age != null && age < 10 && !L.orchestrator.retained && !st.shadow && !['OFF', 'MANUAL'].includes(st.mode);
    const lock = $('#ov-lock');
    lock.hidden = !owns;
    if (owns && !lock.dataset.on) {
      lock.dataset.on = 1;
      lock.innerHTML = `<b>The optimizer is choosing these values.</b> If you change one, it sets its own value again from the next cycle.
        To choose them yourself, stop the optimizer or start the cell with <span class="mono">--shadow</span>. <button id="ov-unlock">Edit anyway</button>`;
      $('#ov-unlock').onclick = () => { editUnlocked = true; setLocked(false); };
    }
    if (!owns) lock.dataset.on = '';
    setLocked(owns && !editUnlocked);
  }
  function setLocked(on) {
    document.querySelectorAll('#ov-form input').forEach(i => { i.disabled = on; });
    $('#ov-apply').disabled = on || P.info.read_only;
  }

  // ---------------------------------------------------------------- log
  function paintLog() {
    const box = $('#ov-log'); if (!box) return;
    const ev = (P.events || []).filter(e => e.src !== 'optimizer').slice(-16).reverse();
    box.innerHTML = ev.map(e => `<div class="ev ${e.level}"><span class="t">${P.hms(e.t)}</span><span class="s">${esc(e.src)}</span><span class="x">${esc(e.text)}</span></div>`).join('')
      || '<div class="dim">nothing yet</div>';
  }
})();
