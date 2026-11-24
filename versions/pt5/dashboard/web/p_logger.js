/* LOGGER page: is the logging alive, and what is in the databases / CSV files? Everything is read-only. */
(function () {
  const P = window.PRISM, $ = P.$, f = P.f, esc = P.esc;
  let sel = null, rows = { cols: [], data: [], total: 0 }, tmr = null;

  P.page('/logger', {
    title: 'Logger',
    mount(el) {
      el.innerHTML = `
      <div class="grid g3">
        <div class="card" id="lg-arm"><h2>logger (arm)</h2><div class="dim">waiting…</div></div>
        <div class="card" id="lg-camera"><h2>logger (camera)</h2><div class="dim">waiting…</div></div>
        <div class="card"><h2>Where the files are</h2><dl class="kv"><dt>databases</dt><dd id="lg-dir" style="text-align:left;word-break:break-all"></dd><dt>autoen CSVs</dt><dd id="lg-csv" style="text-align:left;word-break:break-all"></dd></dl>
          <div class="dim" style="font-size:12px;margin-top:6px">Read-only view. The dashboard never writes to these files.</div></div>
      </div>
      <div class="grid" style="grid-template-columns:minmax(260px,320px) 1fr;margin-top:12px;align-items:start">
        <div class="card"><h2>Files <button class="r" id="lg-refresh">Refresh</button></h2><div id="lg-tree" class="scroll" style="max-height:560px"></div></div>
        <div class="card"><h2 id="lg-title">Pick a table or a CSV on the left</h2>
          <div class="row" style="margin-bottom:8px"><button id="lg-newer" disabled>Latest</button><button id="lg-older" disabled>Older</button><span class="dim" id="lg-info"></span></div>
          <div class="scroll" style="max-height:560px"><table id="lg-table"></table></div></div>
      </div>`;
      $('#lg-dir').textContent = P.info.data_dir || ''; $('#lg-csv').textContent = P.info.log_dir || '';
      $('#lg-refresh').onclick = loadTree; $('#lg-newer').onclick = () => { open(sel, 0); }; $('#lg-older').onclick = () => { const last = rows.data.length ? rows.data[rows.data.length - 1][rows.idCol] : 0; open(sel, last); };
      loadTree(); tmr = setInterval(() => { if (sel && !sel.before) open(sel, 0, true); }, 4000);
    },
    update(L) {
      ['arm', 'camera'].forEach(k => {
        const c = $('#lg-' + k); if (!c) return;
        const n = L.nodes['logger (' + k + ')'], hb = L.logger && L.logger[k];
        c.innerHTML = `<h2>logger (${k}) <span class="r pill ${P.nodeClass(n.status)}">${n.status}</span></h2>` +
          (hb ? `<dl class="kv"><dt>rows written</dt><dd>${hb.written}</dd><dt>queued</dt><dd>${hb.queued}</dd><dt>last heartbeat</dt><dd>${f(n.signals['logger/' + k + '/heartbeat'], 1)} s ago</dd><dt>status topic</dt><dd>${n.online_flag == null ? '-' : n.online_flag ? 'online' : 'OFFLINE'}</dd></dl>`
            : `<div class="dim">${esc(n.reason || 'no heartbeat yet')}. Start: <span class="mono">python3 logger_node.py --topics ${k} --db ${k}.db</span></div>`);
      });
    },
    unmount() { clearInterval(tmr); sel = null; }
  });

  async function loadTree() {
    try {
      const j = await P.api('/api/db/list'); let h = '';
      if (!j.dbs.length) h += `<div class="dim">No .db files in ${esc(P.info.data_dir)}. Start the dashboard with --data-dir where the loggers write.</div>`;
      j.dbs.forEach(d => {
        h += `<div style="margin-bottom:8px"><b>${esc(d.name)}</b> <span class="dim">${(d.size / 1024).toFixed(0)} kB · ${P.hms(d.mtime)}</span>`;
        if (d.error) h += `<div class="lvl-ALARM">${esc(d.error)}</div>`;
        d.tables.forEach(t => { h += `<div><a href="#" data-db="${esc(d.name)}" data-t="${esc(t.name)}">${esc(t.name)}</a> <span class="dim">${t.rows} rows</span></div>`; });
        h += '</div>';
      });
      if (j.csv.length) { h += '<b>CSV logs</b>'; j.csv.forEach(c => { h += `<div><a href="#" data-csv="${esc(c.name)}">${esc(c.name)}</a> <span class="dim">${(c.size / 1024).toFixed(0)} kB · ${P.hms(c.mtime)}</span></div>`; }); }
      $('#lg-tree').innerHTML = h;
      $('#lg-tree').querySelectorAll('a').forEach(a => a.onclick = e => { e.preventDefault(); open(a.dataset.csv ? { csv: a.dataset.csv } : { db: a.dataset.db, table: a.dataset.t }, 0); });
    } catch (e) { $('#lg-tree').textContent = e.message; }
  }
  async function open(s, before, quiet) {
    sel = Object.assign({}, s, { before }); $('#lg-title').textContent = s.csv ? s.csv : `${s.db} › ${s.table}`;
    try {
      let head = [], body = [], total = 0;
      if (s.csv) {
        const j = await P.api(`/api/csv/tail?file=${encodeURIComponent(s.csv)}&limit=100`);
        head = j.header.split(','); body = j.lines.reverse().map(l => l.split(',')); total = j.total; rows.idCol = 0;
        $('#lg-older').disabled = true; $('#lg-newer').disabled = true;
      } else {
        const j = await P.api(`/api/db/rows?db=${encodeURIComponent(s.db)}&table=${encodeURIComponent(s.table)}&limit=50&before=${before || 0}`);
        head = j.columns; body = j.rows; total = j.total; rows.idCol = Math.max(0, head.indexOf('id'));
        $('#lg-older').disabled = !head.includes('id') || body.length < 50; $('#lg-newer').disabled = !before;
      }
      rows.data = body;
      $('#lg-info').textContent = `${total} rows in total · showing ${body.length} (newest first)`;
      $('#lg-table').innerHTML = '<tr>' + head.map(c => `<th>${esc(c)}</th>`).join('') + '</tr>' +
        body.map(r => '<tr>' + r.map(v => `<td class="l" style="max-width:340px;overflow:hidden;text-overflow:ellipsis">${esc(v == null ? '' : (typeof v === 'number' && !Number.isInteger(v) ? +v.toFixed(3) : v))}</td>`).join('') + '</tr>').join('');
    } catch (e) { if (!quiet) $('#lg-info').textContent = e.message; }
  }
})();
