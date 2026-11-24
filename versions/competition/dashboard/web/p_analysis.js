/* ANALYSIS pages: Arm and Conveyor. Same layout, different data.
   live state card -> state timeline -> signal charts with the limits of the CURRENT state -> condition levels ->
   cycles / runs -> condition events -> threshold editor (writes the analyzer's own threshold file; it reloads within 5 s). */
(function () {
  const P = window.PRISM, $ = P.$, f = P.f, esc = P.esc;
  const JN = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
  const JCOL = ['#f08a24', '#5aa9ff', '#3ecf8e', '#f5c542', '#c084fc', '#ff7ab6'];
  const SC_ARM = { IDLE: '#5d6a79', APPROACHING: '#5aa9ff', WAITING: '#8a97a6', PICKING: '#f08a24', CARRYING: '#c084fc', PLACING: '#3ecf8e', RETURNING: '#2fb5b5', MOVING: '#a3b1c2', NO_DATA: '#2a3440' };
  const SC_BELT = { STOPPED: '#5d6a79', STARTING: '#f5c542', RUNNING: '#3ecf8e', STOPPING: '#f08a24', STALLED: '#ff5a4f', NO_DATA: '#2a3440' };

  const CFG = {
    arm: {
      route: '/analysis/arm', title: 'Arm analysis', node: 'arm analyzer', key: 'an_arm', ring: { tr: 'an_arm_transition', main: 'an_arm_cycle', cond: 'an_arm_condition' },
      colors: SC_ARM, db: 'arm_analyzer.db', mainTable: 'cycles', condTable: 'conditions', mainName: 'Cycles', start: 'python3 arm_analyzer_node.py'
    },
    belt: {
      route: '/analysis/belt', title: 'Conveyor analysis', node: 'belt analyzer', key: 'an_belt', ring: { tr: 'an_belt_transition', main: 'an_belt_run', cond: 'an_belt_condition' },
      colors: SC_BELT, db: 'belt_analyzer.db', mainTable: 'runs', condTable: 'conditions', mainName: 'Runs', start: 'python3 belt_analyzer_node.py'
    }
  };

  Object.keys(CFG).forEach(who => {
    const c = CFG[who];
    let tmr = null, ch = {}, tl = null, thr = null, hist = null, rings = null, jsel = 0;
    P.page(c.route, {
      title: c.title,
      mount(el) {
        const chartCards = who === 'arm' ?
          `<div class="card"><h2>Motor current <span class="r"><select id="an-j">${JN.map((j, i) => `<option value="${i}">limits: ${j}</option>`).join('')}</select></span></h2><canvas class="chart tall" id="an-ch-cur"></canvas><div class="legend" id="an-leg"></div></div>
           <div class="card"><h2>Temperature</h2><canvas class="chart tall" id="an-ch-temp"></canvas></div>` :
          `<div class="card"><h2>Motor current</h2><canvas class="chart tall" id="an-ch-cur"></canvas></div>
           <div class="card"><h2>Speed</h2><canvas class="chart tall" id="an-ch-spd"></canvas></div>
           <div class="card"><h2>Temperature</h2><canvas class="chart tall" id="an-ch-temp"></canvas></div>`;
        el.innerHTML = `
        <div id="an-na"></div>
        <div class="grid g4" id="an-cards"></div>
        <div class="card" style="margin-top:12px"><h2>State timeline <span class="r dim">last 3 minutes (same window as the charts)</span></h2><canvas id="an-tl" style="width:100%;height:46px;display:block"></canvas><div class="legend" id="an-tlleg"></div></div>
        <div class="grid g2" style="margin-top:12px">${chartCards}
          <div class="card"><h2>Condition levels now <span class="r dim">limits depend on the state</span></h2><div id="an-lv"></div></div></div>
        <div class="grid g2" style="margin-top:12px">
          <div class="card"><h2>${c.mainName} <span class="r dim" id="an-main-src"></span></h2><div class="scroll"><table id="an-main"></table></div></div>
          <div class="card"><h2>Condition events <span class="r dim" id="an-cond-src"></span></h2><div class="scroll"><table id="an-cond"></table></div></div></div>
        <div class="card" style="margin-top:12px"><h2>Thresholds <span class="r dim" id="an-thr-file"></span></h2><div id="an-thr"></div></div>`;
        const lim = () => limitsFor();
        ch.cur = new Charts.LineChart($('#an-ch-cur'), { series: who === 'arm' ? JN.map((n, i) => ({ name: n, color: JCOL[i] })) : [{ name: 'current', color: '#f08a24' }], unit: 'mA', yMin: 0 });
        ch.temp = new Charts.LineChart($('#an-ch-temp'), { series: who === 'arm' ? JN.map((n, i) => ({ name: n, color: JCOL[i] })) : [{ name: 'temp', color: '#f5c542' }], unit: '°C' });
        if (who === 'belt') ch.spd = new Charts.LineChart($('#an-ch-spd'), { series: [{ name: 'speed', color: '#5aa9ff' }], unit: 'rpm' });
        tl = new Charts.Timeline($('#an-tl'), { colors: c.colors });
        $('#an-tlleg').innerHTML = Object.entries(c.colors).filter(([k]) => k !== 'NO_DATA').map(([k, v]) => `<span><i style="background:${v}"></i>${k}</span>`).join('');
        if (who === 'arm') { $('#an-j').onchange = e => { jsel = +e.target.value; draw(); }; $('#an-leg').innerHTML = JN.map((n, i) => `<span><i style="background:${JCOL[i]}"></i>${n}</span>`).join(''); }
        loadThr(); refresh(); tmr = setInterval(refresh, 2000);
      },
      update(L) {
        if (!$('#an-cards')) return;
        const n = L.nodes[c.node], s = L[c.key] && L[c.key].d, age = L[c.key] ? L[c.key].age : null;
        $('#an-na').innerHTML = (!s || age > 10) ? P.na(`${c.node}: ${s ? 'silent for ' + f(age, 0) + ' s' : 'no data'}`, `Start it: ${c.start} (it publishes ${c.key.replace('_', '/')}/state). ${n ? n.reason : ''}`) : '';
        if (!s) { $('#an-cards').innerHTML = ''; return; }
        const stc = c.colors[s.state] || '#8a97a6';
        let cards = `<div class="card"><h2>State</h2><div class="big" style="color:${stc}">${s.state}</div><div class="dim" style="font-size:12px">${esc(s.detail || '')} · ${s.since_s == null ? '' : P.dur(s.since_s)}</div></div>`;
        if (who === 'arm') {
          cards += `<div class="card"><h2>Pose / gripper</h2><div class="big">${esc(s.pose || '-')}</div><div class="dim" style="font-size:12px">gripper ${esc(s.gripper || '-')} · ${s.arm_moving ? 'moving ' + f(s.arm_speed_ticks_s, 0) + ' t/s' : 'still'}${(s.off_path || []).length ? ' · OFF PATH ' + s.off_path.join(',') : ''}</div></div>
          <div class="card"><h2>Cycles OK</h2><div class="big">${s.cycles_ok}</div><div class="dim" style="font-size:12px">last ${s.last_cycle_s == null ? '-' : f(s.last_cycle_s, 1) + ' s'}</div></div>`;
        } else {
          cards += `<div class="card"><h2>Speed / current</h2><div class="big">${f(s.speed_rpm, 0)} <span class="dim" style="font-size:14px">rpm</span></div><div class="dim" style="font-size:12px">${f(s.current_mA, 0)} mA · ${s.temp_C == null ? 'temp -' : f(s.temp_C, 0) + ' °C'} · ${s.position_rev == null ? '' : f(s.position_rev, 1) + ' rev'}</div></div>
          <div class="card"><h2>Runs OK</h2><div class="big">${s.runs_ok} <span class="dim" style="font-size:14px">/ ${s.runs_total}</span></div><div class="dim" style="font-size:12px">${s.run_active ? 'a run is active' : 'idle'}</div></div>`;
        }
        const lv = worst(s.levels);
        cards += `<div class="card"><h2>Conditions</h2><div class="big ${P.lvlClass(lv)}">${lv}</div><div class="dim" style="font-size:12px">worst level over all values</div></div>`;
        $('#an-cards').innerHTML = cards;
        paintLevels(s);
      },
      unmount() { clearInterval(tmr); ch = {}; }
    });

    const worst = lv => { if (!lv) return 'NO_DATA'; const all = []; const walk = o => Object.values(o).forEach(v => typeof v === 'string' ? all.push(v) : walk(v)); walk(lv); return all.includes('ALARM') ? 'ALARM' : all.includes('WARN') ? 'WARN' : 'OK'; };
    function paintLevels(s) {
      const el = $('#an-lv'); if (!el) return;
      if (!s.levels) { el.innerHTML = '<div class="dim">NO_DATA: levels are not live</div>'; return; }
      if (who === 'arm') {
        el.innerHTML = '<table><tr><th>joint</th><th>current</th><th>temp</th></tr>' + JN.map(j => { const l = s.levels[j] || {}; return `<tr><td>${j}</td><td class="${P.lvlClass(l.current)}">${l.current || '-'}</td><td class="${P.lvlClass(l.temp)}">${l.temp || '-'}</td></tr>`; }).join('') + '</table>';
      } else {
        el.innerHTML = `<table><tr><th>value</th><th>level</th></tr><tr><td>current</td><td class="${P.lvlClass(s.levels.current)}">${s.levels.current}</td></tr><tr><td>temperature</td><td class="${P.lvlClass(s.levels.temp)}">${s.levels.temp}</td></tr></table>`;
      }
    }
    function limitsFor() {
      const s = P.live && P.live[c.key] && P.live[c.key].d; if (!s || !thr || !thr.thresholds) return { cur: [], temp: [] };
      const T = thr.thresholds, st = T[s.state];
      let cur = [], temp = [];
      if (who === 'arm') {
        const row = st && st[JN[jsel]];
        if (row) cur = [{ y: row.warn_current, color: '#f5c542', label: 'warn' }, { y: row.alarm_current, color: '#ff5a4f', label: 'alarm' }];
        const t = T.TEMPERATURE || {}, tj = t[JN[jsel]] || t; temp = [{ y: tj.warn, color: '#f5c542', label: 'warn' }, { y: tj.alarm, color: '#ff5a4f', label: 'alarm' }];
      } else {
        if (st) cur = [{ y: st.warn_current, color: '#f5c542', label: 'warn' }, { y: st.alarm_current, color: '#ff5a4f', label: 'alarm' }];
        const t = T.TEMPERATURE || {}; temp = [{ y: t.warn, color: '#f5c542', label: 'warn' }, { y: t.alarm, color: '#ff5a4f', label: 'alarm' }];
      }
      return { cur, temp };
    }
    async function refresh() {
      if (!$('#an-cards')) return;
      try {
        [hist, rings] = await Promise.all([P.api('/api/history?seconds=180'), P.api('/api/rings')]);
      } catch (e) { return; }
      draw(); paintTables();
    }
    function draw() {
      if (!hist || !ch.cur) return;
      const l = limitsFor();
      if (who === 'arm') {
        const a = hist.arm;
        ch.cur.set(a.t, JN.map((_, j) => a.cur.map(r => r[j])), l.cur);
        ch.temp.set(a.t, JN.map((_, j) => a.temp.map(r => r[j])), l.temp);
      } else {
        const b = hist.belt;
        ch.cur.set(b.t, [b.current.map(v => v == null ? null : Math.abs(v))], l.cur);
        ch.spd.set(b.t, [b.velocity.map(v => v == null ? null : Math.abs(v))]);
        ch.temp.set(b.t, [b.temp], l.temp);
      }
      // state timeline from the transition ring: a transition at t_i starts state `to`; it lasts until the next one
      const tr = (rings && rings[c.ring.tr]) || [], now = Date.now() / 1000, t0 = now - 180, segs = [];
      tr.forEach((e, i) => { const t1 = i + 1 < tr.length ? tr[i + 1].t : now; segs.push({ t0: e.t, t1, label: e.to, color: c.colors[e.to] }); });
      tl.set(segs.filter(s => s.t1 >= t0), t0, now);
    }
    async function paintTables() {
      const mainRing = (rings[c.ring.main] || []).slice().reverse(), condRing = (rings[c.ring.cond] || []).slice().reverse();
      let src1 = 'live (this session)', src2 = 'live (this session)', main = null, cond = null;
      try { const j = await P.api(`/api/db/rows?db=${c.db}&table=${c.mainTable}&limit=25`); main = j.rows.map(r => Object.fromEntries(j.columns.map((k, i) => [k, r[i]]))); src1 = `database ${c.db}`; } catch (e) { /* no db: use the ring */ }
      try { const j = await P.api(`/api/db/rows?db=${c.db}&table=${c.condTable}&limit=25`); cond = j.rows.map(r => Object.fromEntries(j.columns.map((k, i) => [k, r[i]]))); src2 = `database ${c.db}`; } catch (e) { /* ring */ }
      if (!$('#an-main')) return;
      $('#an-main-src').textContent = src1; $('#an-cond-src').textContent = src2;
      const resCls = r => r === 'OK' ? 'lvl-OK' : (r === 'ANOMALY' || r === 'ABORTED' || r === 'FAULT') ? 'lvl-ALARM' : 'lvl-WARN';
      if (who === 'arm') {
        const rows = main ? main.map(r => ({ t: r.start_iso ? r.start_iso.slice(11, 19) : '', s: r.seconds, dest: r.dest, res: r.result, note: r.conditions || r.unexpected || '' }))
          : mainRing.map(r => ({ t: P.hms(r.start), s: r.seconds, dest: r.dest, res: r.result, note: (r.conditions || []).concat(r.unexpected || []).join('; ') }));
        $('#an-main').innerHTML = '<tr><th>start</th><th>seconds</th><th>dest</th><th>result</th><th class="l">notes</th></tr>' + rows.map(r => `<tr><td>${r.t}</td><td>${f(r.s, 1)}</td><td>${esc(r.dest || '-')}</td><td class="${resCls(r.res)}">${r.res}</td><td class="l dim">${esc(r.note)}</td></tr>`).join('');
      } else {
        const rows = main ? main.map(r => ({ t: r.start_iso ? r.start_iso.slice(11, 19) : '', s: r.seconds, d: r.distance_rev, v: r.steady_speed_rpm, i: r.steady_current_mA, pk: r.peak_current_mA, res: r.result, note: r.reasons || '' }))
          : mainRing.map(r => ({ t: P.hms(r.start), s: r.seconds, d: r.distance_rev, v: r.steady_speed_rpm, i: r.steady_current_mA, pk: r.peak_current_mA, res: r.result, note: (r.reasons || []).join(', ') }));
        $('#an-main').innerHTML = '<tr><th>start</th><th>s</th><th>rev</th><th>rpm</th><th>mA</th><th>peak mA</th><th>result</th><th class="l">reasons</th></tr>' + rows.map(r => `<tr><td>${r.t}</td><td>${f(r.s, 1)}</td><td>${f(r.d, 1)}</td><td>${f(r.v, 0)}</td><td>${f(r.i, 0)}</td><td>${f(r.pk, 0)}</td><td class="${resCls(r.res)}">${r.res}</td><td class="l dim">${esc(r.note)}</td></tr>`).join('');
      }
      const crow = cond ? cond.map(r => ({ t: r.iso ? r.iso.slice(11, 19) : '', who: r.joint || '', m: r.metric, st: r.state, lv: r.level, v: r.value, th: r.threshold }))
        : condRing.map(r => ({ t: P.hms(r.t), who: r.joint || '', m: r.metric, st: r.state, lv: r.level, v: r.value, th: r.threshold }));
      $('#an-cond').innerHTML = '<tr><th>time</th>' + (who === 'arm' ? '<th>joint</th>' : '') + '<th>metric</th><th>state</th><th>level</th><th>value</th><th>limit</th></tr>' +
        crow.map(r => `<tr><td>${r.t}</td>${who === 'arm' ? `<td>${r.who}</td>` : ''}<td>${r.m}</td><td>${r.st}</td><td class="${P.lvlClass(r.lv)}">${r.lv}</td><td>${f(r.v, 0)}</td><td>${r.th == null ? '-' : f(r.th, 0)}</td></tr>`).join('') || '<tr><td class="dim">none</td></tr>';
    }

    // ---------- threshold editor ----------
    async function loadThr() {
      try { thr = await P.api('/api/thresholds?who=' + who); } catch (e) { thr = null; }
      paintThr();
    }
    function paintThr() {
      const el = $('#an-thr'); if (!el) return;
      if (!thr || !thr.available) { el.innerHTML = P.na('Threshold editor unavailable', (thr && thr.error) || 'the analyzer module was not found'); return; }
      $('#an-thr-file').textContent = (thr.exists ? thr.file : 'no file yet: defaults shown, Save creates ' + thr.file);
      const T = thr.thresholds, D = thr.defaults, ro = P.info.read_only;
      const inp = (path, v, d) => `<input type="number" step="any" data-p="${path}" data-d="${d}" value="${v}" ${v != d ? 'class="changed"' : ''}>`;
      let h = '';
      if (thr.problems && thr.problems.length) h += `<div class="lvl-WARN" style="margin-bottom:8px">${thr.problems.map(esc).join('<br>')}</div>`;
      if (who === 'arm') {
        const states = Object.keys(T).filter(k => T[k] && T[k].J1);
        h += '<div class="scroll" style="max-height:420px"><table class="thr"><tr><th>state</th>' + JN.map(j => `<th colspan="2">${j} <span class="warnc">warn</span>/<span class="alarmc">alarm</span> mA</th>`).join('') + '</tr>';
        states.forEach(s => { h += `<tr><td>${s}</td>` + JN.map(j => `<td>${inp(`${s}.${j}.warn_current`, T[s][j].warn_current, D[s][j].warn_current)}</td><td>${inp(`${s}.${j}.alarm_current`, T[s][j].alarm_current, D[s][j].alarm_current)}</td>`).join('') + '</tr>'; });
        h += '</table></div>';
      } else {
        h += '<table class="thr"><tr><th>state</th><th>warn mA</th><th>alarm mA</th></tr>';
        ['STOPPED', 'STARTING', 'RUNNING', 'STOPPING', 'STALLED'].forEach(s => { h += `<tr><td>${s}</td><td>${inp(`${s}.warn_current`, T[s].warn_current, D[s].warn_current)}</td><td>${inp(`${s}.alarm_current`, T[s].alarm_current, D[s].alarm_current)}</td></tr>`; });
        h += '</table>';
      }
      h += `<div class="row" style="margin-top:10px"><span>Temperature °C:</span> warn ${inp('TEMPERATURE.warn', T.TEMPERATURE.warn, D.TEMPERATURE.warn)} alarm ${inp('TEMPERATURE.alarm', T.TEMPERATURE.alarm, D.TEMPERATURE.alarm)}
        <span style="margin-left:12px">Hold s (a value must stay over a limit this long):</span> current ${inp('HOLD_S.current', T.HOLD_S.current, D.HOLD_S.current)} temp ${inp('HOLD_S.temp', T.HOLD_S.temp, D.HOLD_S.temp)}</div>
        <div class="row" style="margin-top:12px"><button class="pri" id="thr-save" ${ro ? 'disabled' : ''}>Save</button><button id="thr-reset">Reset to defaults</button><button id="thr-reload">Reload from file</button><span class="msg" id="thr-msg"></span>
        <span class="dim" style="font-size:12px">Orange border = differs from the built-in default. The analyzer reloads the file within 5 s; no restart.</span></div>`;
      el.innerHTML = h;
      el.querySelectorAll('input').forEach(i => i.oninput = () => i.classList.toggle('changed', +i.value !== +i.dataset.d));
      $('#thr-reload').onclick = loadThr;
      $('#thr-reset').onclick = () => { el.querySelectorAll('input').forEach(i => { i.value = i.dataset.d; i.classList.remove('changed'); }); };
      $('#thr-save').onclick = async () => {
        const body = {};
        el.querySelectorAll('input').forEach(i => { const p = i.dataset.p.split('.'); let o = body; p.slice(0, -1).forEach(k => { o = o[k] = o[k] || {}; }); o[p[p.length - 1]] = +i.value; });
        try { const r = await P.post('/api/thresholds', { who, thresholds: body }); thr.thresholds = r.thresholds; thr.exists = true; P.setMsg($('#thr-msg'), 'saved: the analyzer picks it up within 5 s', true); loadThr(); }
        catch (e) { P.setMsg($('#thr-msg'), e.message, false); }
      };
    }
  });
})();
