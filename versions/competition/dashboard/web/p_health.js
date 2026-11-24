/* HEALTH page: one row per node (dashboard's own view + health_monitor's view), broker, every topic with its rate, all events. */
(function () {
  const P = window.PRISM, $ = P.$, f = P.f, esc = P.esc;
  let tmr = null;
  const cls = s => 'st-' + String(s).replace(/\s.*/, '');
  P.page('/health', {
    title: 'Health',
    mount(el) {
      el.innerHTML = `
      <div class="grid g4" id="hl-cards"></div>
      <div class="grid g2" style="margin-top:12px">
        <div class="card"><h2>Nodes <span class="r dim">seen by this dashboard (message ages)</span></h2><div class="scroll"><table id="hl-nodes"></table></div></div>
        <div class="card"><h2>health_monitor.py <span class="r dim" id="hl-hm-age"></span></h2><div class="scroll"><table id="hl-hm"></table></div></div>
      </div>
      <div class="grid g2" style="margin-top:12px">
        <div class="card"><h2>Topics <span class="r dim">rate = live messages per second</span></h2><div class="scroll" style="max-height:420px"><table id="hl-topics"></table></div></div>
        <div class="card"><h2>Events</h2><div class="scroll" style="max-height:420px" id="hl-events"></div></div>
      </div>`;
      topics(); tmr = setInterval(topics, 2000);
    },
    update(L) {
      if (!$('#hl-cards')) return;
      const ns = Object.entries(L.nodes), cnt = s => ns.filter(([, v]) => v.status === s).length;
      $('#hl-cards').innerHTML = [
        ['Broker', L.mqtt ? 'connected' : 'DOWN', L.mqtt ? 'lvl-OK' : 'lvl-ALARM', P.info.mqtt || ''],
        ['Nodes OK', `${cnt('OK')} / ${ns.filter(([, v]) => v.status !== 'NOT AVAILABLE').length}`, cnt('OFFLINE') ? 'lvl-ALARM' : cnt('STALE') + cnt('NO DATA') ? 'lvl-WARN' : 'lvl-OK', 'nodes that report'],
        ['Stale / no data', cnt('STALE') + cnt('NO DATA'), cnt('STALE') + cnt('NO DATA') ? 'lvl-WARN' : 'lvl-OK', 'late or never seen'],
        ['Offline', cnt('OFFLINE'), cnt('OFFLINE') ? 'lvl-ALARM' : 'lvl-OK', 'silent or Last Will']
      ].map(c => `<div class="card"><h2>${c[0]}</h2><div class="big ${c[2]}">${c[1]}</div><div class="dim" style="font-size:12px">${esc(c[3])}</div></div>`).join('');
      $('#hl-nodes').innerHTML = '<tr><th>node</th><th>status</th><th class="l">why</th><th>last msg</th></tr>' + ns.map(([k, v]) =>
        `<tr><td>${esc(k)}</td><td class="${cls(v.status)}"><span class="dot ${P.nodeClass(v.status)}"></span>${v.status}</td><td class="l dim">${esc(v.reason || '')}</td><td>${v.age_s == null ? '-' : f(v.age_s, 1) + ' s'}</td></tr>`).join('');
      const hm = L.health, hmn = hm && hm.nodes;
      $('#hl-hm-age').textContent = hmn ? `updated ${f(L.health_age, 1)} s ago` : 'not running';
      $('#hl-hm').innerHTML = hmn ? '<tr><th>node</th><th>status</th><th class="l">reason</th></tr>' + Object.entries(hmn).map(([k, v]) =>
        `<tr><td>${esc(k)}</td><td class="${cls(v.status)}">${esc(v.status)}</td><td class="l dim">${esc(v.reason || v.detail || '')}</td></tr>`).join('') : `<tr><td class="dim">${esc('No health/status yet. Start health_monitor.py.')}</td></tr>`;
      const ev = (P.events || []).slice(-120).reverse();
      $('#hl-events').innerHTML = ev.map(e => `<div class="ev ${e.level}"><span class="t">${P.hms(e.t)}</span><span class="s">${esc(e.src)}</span><span class="x">${esc(e.text)}</span></div>`).join('') || '<div class="dim">no events yet</div>';
    },
    unmount() { clearInterval(tmr); }
  });
  async function topics() {
    if (!$('#hl-topics')) return;
    try {
      const j = await P.api('/api/topics');
      $('#hl-topics').innerHTML = '<tr><th class="l">topic</th><th>rate Hz</th><th>age s</th><th>msgs</th><th></th></tr>' + j.topics.map(t =>
        `<tr><td class="l mono">${esc(t.topic)}</td><td>${t.rate_hz ? f(t.rate_hz, 1) : '-'}</td><td class="${t.age_s > 10 ? 'lvl-WARN' : ''}">${f(t.age_s, 1)}</td><td>${t.n}</td><td>${t.retained ? '<span class="tag">retained</span>' : ''}</td></tr>`).join('');
    } catch (e) { /* backend down */ }
  }
})();
