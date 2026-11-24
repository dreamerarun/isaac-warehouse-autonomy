/* Warehouse Operations dashboard.
 * Static page, no build step. Talks to rosbridge (roslib 1.4.1, vendor/roslib.min.js).
 * URL params:  ?ws=ws://host:port   override the rosbridge URL
 *              ?mock=1              run a client-side simulation instead of ROS
 */
(function () {
  'use strict';

  // ------------------------------------------------------------------ utils
  const $ = (id) => document.getElementById(id);
  const isNum = (v) => typeof v === 'number' && isFinite(v);
  const num = (v, d) => (isNum(v) ? v : (d === undefined ? null : d));
  const str = (v, d) => (typeof v === 'string' ? v : (v === null || v === undefined ? (d === undefined ? '' : d) : String(v)));
  const arr = (v) => (Array.isArray(v) ? v : []);
  const objOr = (v) => (v && typeof v === 'object' && !Array.isArray(v) ? v : {});
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const fx = (v, d, dash) => (isNum(v) ? v.toFixed(d) : (dash === undefined ? '-' : dash));
  const TAU = Math.PI * 2;
  const wrapAng = (a) => { a = (a + Math.PI) % TAU; if (a < 0) a += TAU; return a - Math.PI; };
  const angLerp = (a, b, f) => a + wrapAng(b - a) * f;
  const now = () => performance.now();

  function fmtClock(t) {
    if (!isNum(t) || t < 0) return '--:--';
    t = Math.floor(t);
    const h = Math.floor(t / 3600), m = Math.floor((t % 3600) / 60), s = t % 60;
    const p = (n) => String(n).padStart(2, '0');
    return h > 0 ? h + ':' + p(m) + ':' + p(s) : p(m) + ':' + p(s);
  }
  function parseJson(s) {
    if (s && typeof s === 'object') return s;
    if (typeof s !== 'string') return null;
    try { return JSON.parse(s); } catch (e) { return null; }
  }
  function mk(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined) e.textContent = text;
    return e;
  }
  function safe(label, fn) {
    return function () {
      try { return fn.apply(this, arguments); } catch (e) { console.warn('[dashboard] ' + label + ':', e); return undefined; }
    };
  }
  function rectOf(r) {
    // accept [x0,x1,y0,y1] or {x0,x1,y0,y1}
    if (Array.isArray(r) && r.length >= 4 && r.slice(0, 4).every(isNum)) return { x0: Math.min(r[0], r[1]), x1: Math.max(r[0], r[1]), y0: Math.min(r[2], r[3]), y1: Math.max(r[2], r[3]) };
    if (r && isNum(r.x0) && isNum(r.x1) && isNum(r.y0) && isNum(r.y1)) return { x0: Math.min(r.x0, r.x1), x1: Math.max(r.x0, r.x1), y0: Math.min(r.y0, r.y1), y1: Math.max(r.y0, r.y1) };
    return null;
  }

  // ------------------------------------------------------------------ config
  const params = new URLSearchParams(location.search);
  const MOCK = ['1', 'true', 'yes', 'on'].indexOf(str(params.get('mock')).toLowerCase()) >= 0;
  const WS_URL = params.get('ws') || ((location.protocol === 'https:' ? 'wss' : 'ws') + '://' + (location.hostname || 'localhost') + ':9090');
  const RECONNECT_MS = 2000;
  const STALE_MS = 2000;
  const NO_SIGNAL_MS = 3000;
  const TRAIL_S = 10;
  const LS_LAYOUT = 'wh-dash.layout.' + WS_URL;
  const LS_FIT = 'wh-dash.fit';

  const STR = 'std_msgs/msg/String';
  const TOPICS = {
    state:        { name: '/warehouse/state', type: STR, dir: 'sub' },
    layout:       { name: '/warehouse/layout', type: STR, dir: 'sub' },
    events:       { name: '/warehouse/events', type: STR, dir: 'sub' },
    eventsRecent: { name: '/warehouse/events_recent', type: STR, dir: 'sub', throttle_rate: 5000, queue_length: 1 },
    camOverview:  { name: '/warehouse/camera/overview/compressed', type: 'sensor_msgs/msg/CompressedImage', dir: 'sub', throttle_rate: 200, queue_length: 1 },
    camFollow:    { name: '/warehouse/camera/follow/compressed', type: 'sensor_msgs/msg/CompressedImage', dir: 'sub', throttle_rate: 200, queue_length: 1 },
    wms:          { name: '/wms/status', type: STR, dir: 'sub' },
    followTarget: { name: '/warehouse/camera/follow_target', type: STR, dir: 'pub' },
    newOrder:     { name: '/wms/new_order', type: STR, dir: 'pub' },
    wmsConfig:    { name: '/wms/config', type: STR, dir: 'pub' },
  };
  const SERVICES = {
    pause:  { name: '/warehouse/pause', type: 'std_srvs/srv/Trigger' },
    resume: { name: '/warehouse/resume', type: 'std_srvs/srv/Trigger' },
    estop:  { name: '/warehouse/estop', type: 'std_srvs/srv/SetBool' },
  };

  // ------------------------------------------------------------------ model
  const DEFAULT_ZONE = { x0: -13, x1: 13, y0: -8, y1: 8 };
  const M = {
    conn: MOCK ? 'mock' : 'connecting',
    state: null, recvAt: 0, prevRecvAt: 0, prevIdx: null,
    layout: null, layoutSrc: 'none', derivedLayout: null,
    wms: null, wmsAt: 0,
    evLines: [], evBackfilled: false, followUserAt: 0,
    selected: null, hover: null,
    trails: {}, speeds: {},
    lastPos: {},
    fit: 'zone',
    estopBusy: false, pauseBusy: false,
    periodDirty: false,
  };
  const stats = {};

  function bump(key) {
    const t = now();
    const s = stats[key] || (stats[key] = { times: [], total: 0, first: t, last: 0 });
    s.times.push(t); s.total++; s.last = t;
    while (s.times.length && t - s.times[0] > 5000) s.times.shift();
  }
  function rateOf(key) {
    const s = stats[key];
    if (!s) return 0;
    const t = now();
    while (s.times.length && t - s.times[0] > 5000) s.times.shift();
    if (!s.times.length) return 0;
    const span = clamp((t - s.first) / 1000, 1, 5);
    return s.times.length / span;
  }

  // ------------------------------------------------------------------ layout normalisation
  function normLayout(L) {
    L = objOr(L);
    const zone = rectOf(L.zone) || DEFAULT_ZONE;
    const building = rectOf(L.building);
    const belts = arr(L.belts).filter((b) => b && isNum(b.x0) && isNum(b.x1) && isNum(b.y)).map((b) => ({ line: str(b.line, '?'), x0: b.x0, x1: b.x1, y: b.y, w: num(b.w, 0.6) }));
    const docks = arr(L.docks).filter((d) => d && isNum(d.x) && isNum(d.y)).map((d) => ({ line: str(d.line, ''), x: d.x, y: d.y, w: num(d.w, 1.2), h: num(d.h, 1.0) }));
    const strips = arr(L.forklift_strips).map((s) => { const r = rectOf(s); return r ? Object.assign(r, { name: str(s && s.name, '') }) : null; }).filter(Boolean);
    const pallets = arr(L.pallets).filter((p) => p && isNum(p.x) && isNum(p.y)).map((p) => ({ name: str(p.name, ''), x: p.x, y: p.y, w: num(p.w, 1.2), h: num(p.h, 1.0) }));
    const arms = arr(L.arms).filter((a) => a && isNum(a.x) && isNum(a.y)).map((a) => ({ name: str(a.name, 'arm'), x: a.x, y: a.y }));
    const statics = arr(L.statics).map(rectOf).filter(Boolean);
    const parking = [];
    const pk = objOr(L.parking);
    Object.keys(pk).forEach((k) => { const p = pk[k]; if (Array.isArray(p) && isNum(p[0]) && isNum(p[1])) parking.push({ name: k, x: p[0], y: p[1] }); else if (p && isNum(p.x) && isNum(p.y)) parking.push({ name: k, x: p.x, y: p.y }); });
    const c = objOr(L.crate);
    const crate = { l: num(c.l, 0.4), w: num(c.w, 0.3), h: num(c.h, 0.3) };
    return { zone, building, belts, docks, strips, pallets, arms, statics, parking, crate };
  }
  function deriveLayout(S) {
    // Used until /warehouse/layout arrives: enough geometry from the state message to draw belts/arms.
    return normLayout({
      zone: DEFAULT_ZONE,
      belts: arr(S && S.belts).map((b) => ({ line: b.line, x0: b.x0, x1: b.x1, y: b.y, w: 0.6 })),
      arms: arr(S && S.arms),
    });
  }
  function curLayout() { return M.layout || M.derivedLayout || normLayout({}); }

  // ------------------------------------------------------------------ colours (read from CSS variables)
  const COLOR_KEYS = ['text', 'muted', 'faint', 'panel', 'ok', 'warn', 'warn-strong', 'fault', 'info', 'idle', 'parked',
    'map-bg', 'map-outside', 'map-grid', 'map-grid5', 'map-zone', 'map-building', 'map-static', 'map-static-edge', 'map-belt', 'map-belt-edge',
    'map-chevron', 'map-dock', 'map-strip', 'map-pallet', 'map-pallet-edge', 'map-label', 'map-halo', 'crate', 'crate-edge', 'person', 'prop', 'forklift', 'trail'];
  let C = {};
  function readColors() {
    const cs = getComputedStyle(document.documentElement);
    C = {};
    COLOR_KEYS.forEach((k) => { C[k] = cs.getPropertyValue('--' + k).trim() || '#888'; });
  }
  const FONT = getComputedStyle(document.body).fontFamily || 'sans-serif';

  // ------------------------------------------------------------------ entity state helpers
  function speedOf(e) {
    if (!e) return null;
    if (isNum(e.v)) return e.v;
    const d = M.speeds[e.name];
    return isNum(d) ? d : null;
  }
  function carterCls(c, estop) {
    if (estop) return 'fault';
    if (c.parked) return 'parked';
    if (c.blocked_by_person || /yield|blocked/i.test(str(c.status))) return 'warn';
    const v = speedOf(c);
    if (isNum(v) && Math.abs(v) > 0.05) return 'ok';
    return 'idle';
  }
  function forkCls(f, estop) {
    if (estop) return 'fault';
    if (/fault|error/i.test(str(f.status))) return 'fault';
    if (/yield|blocked/i.test(str(f.status))) return 'warn';
    const v = speedOf(f);
    if ((isNum(v) && Math.abs(v) > 0.05) || /lift|stack|load|approach|mov|carry|return/i.test(str(f.status))) return 'ok';
    return 'idle';
  }
  function armCls(a, estop) {
    const s = str(a.status).toLowerCase();
    if (estop || /fault|error|halt/.test(s)) return 'fault';
    if (/block|yield/.test(s)) return 'warn';
    if (/pick|place|mov|grasp|lift|transfer/.test(s)) return 'ok';
    return 'idle';
  }
  const clsColor = (k) => ({ ok: C.ok, warn: C['warn-strong'], fault: C.fault, parked: C.parked, idle: C.idle }[k] || C.idle);

  // ------------------------------------------------------------------ incoming messages
  function onMessage(key, msg) {
    bump(key);
    msg = msg || {};
    switch (key) {
      case 'state': onState(msg.data); break;
      case 'layout': onLayout(msg.data); break;
      case 'events': M.evLines.push(str(msg.data)); if (M.evLines.length > 400) M.evLines.splice(0, 200); addEvent(msg.data); break;
      case 'eventsRecent': onEventsRecent(msg.data); break;
      case 'camOverview': onCam('overview', msg); break;
      case 'camFollow': onCam('follow', msg); break;
      case 'wms': onWms(msg.data); break;
      default: break;
    }
  }

  const onState = safe('state', function (raw) {
    const S = parseJson(raw);
    if (!S || typeof S !== 'object') return;
    const t = now();
    M.prevIdx = indexState(M.state);
    M.prevRecvAt = M.recvAt;
    M.state = S;
    M.recvAt = t;
    trackMotion(S);
    if (!M.layout) M.derivedLayout = deriveLayout(S);
    if (M.estopBusy && isBool(S.estop) && S.estop === M.estopWant) M.estopBusy = false;
    if (M.pauseBusy && isBool(S.paused) && S.paused === M.pauseWant) M.pauseBusy = false;
    renderHeader();
    renderKpis();
    renderFleet();
    syncFollowOptions();
    // the sim owns the follow camera: show its real target unless the user just picked one
    if (typeof S.follow === 'string' && S.follow && now() - (M.followUserAt || 0) > 3000 && followSel.value !== S.follow) {
      setFollowValue(S.follow);
      M.selected = S.follow === 'overview' ? null : S.follow;
    }
    renderControls();
    updateMapNote();
  });
  function isBool(v) { return typeof v === 'boolean'; }

  function indexState(S) {
    if (!S) return null;
    const idx = {};
    ['carters', 'forklifts', 'arms', 'people'].forEach((k) => {
      const m = new Map();
      arr(S[k]).forEach((e) => { if (e && e.name !== undefined) m.set(String(e.name), e); });
      idx[k] = m;
    });
    const pm = new Map();
    arr(S.packages).forEach((p) => { if (p && p.id !== undefined) pm.set(String(p.id), p); });
    idx.packages = pm;
    return idx;
  }

  function trackMotion(S) {
    const t = isNum(S.t) ? S.t : now() / 1000;
    ['carters', 'forklifts', 'people'].forEach((k) => {
      arr(S[k]).forEach((e) => {
        if (!e || !isNum(e.x) || !isNum(e.y)) return;
        const name = str(e.name);
        const lp = M.lastPos[name];
        if (lp && t > lp.t) {
          const d = Math.hypot(e.x - lp.x, e.y - lp.y);
          M.speeds[name] = d / (t - lp.t);
        } else if (lp && t < lp.t - 0.5) {
          M.speeds[name] = 0;
        }
        if (!lp || t !== lp.t) M.lastPos[name] = { t, x: e.x, y: e.y };
        if (k === 'carters') {
          let tr = M.trails[name] || (M.trails[name] = []);
          if (tr.length && t < tr[tr.length - 1].t - 0.5) tr = M.trails[name] = []; // sim reset
          if (!tr.length || t > tr[tr.length - 1].t) tr.push({ t, x: e.x, y: e.y });
          while (tr.length && tr[0].t < t - TRAIL_S) tr.shift();
        }
      });
    });
  }

  const onLayout = safe('layout', function (raw) {
    const L = parseJson(raw);
    if (!L || typeof L !== 'object') return;
    M.layout = normLayout(L);
    M.layoutSrc = 'topic';
    if (!MOCK) { try { localStorage.setItem(LS_LAYOUT, JSON.stringify(L)); } catch (e) { /* storage unavailable */ } }
    mapStaticDirty = true;
    updateFitButtons();
    updateMapNote();
  });

  const onWms = safe('wms', function (raw) {
    const W = parseJson(raw);
    if (!W || typeof W !== 'object') return;
    M.wms = W;
    M.wmsAt = now();
    renderOrders();
    renderKpis();
    renderControls();
  });

  // ------------------------------------------------------------------ header
  const elConn = $('conn'), elConnText = $('connText');
  function setConn(state, text) {
    M.conn = state;
    elConn.dataset.state = state;
    elConnText.textContent = text;
    elConn.title = text;
    renderControls();
  }
  function renderHeader() {
    const S = M.state || {};
    $('simTime').textContent = fmtClock(num(S.t));
    $('simRtf').textContent = isNum(S.rtf) ? S.rtf.toFixed(2) + 'x' : '-';
    const mode = $('simMode');
    mode.textContent = str(S.mode, '-') || '-';
    const estop = S.estop === true;
    $('estopBanner').hidden = !estop;
    document.body.classList.toggle('is-estop', estop);
  }
  function renderStateAge() {
    const el = $('simState');
    el.classList.remove('is-warn', 'is-fault');
    if (!M.recvAt) { el.textContent = 'no data'; el.classList.add('is-warn'); return; }
    const age = now() - M.recvAt;
    const S = M.state || {};
    if (age > STALE_MS) { el.textContent = 'stale ' + Math.round(age / 1000) + ' s'; el.classList.add('is-warn'); return; }
    if (S.estop === true) { el.textContent = 'E-STOP'; el.classList.add('is-fault'); return; }
    if (S.paused === true) { el.textContent = 'paused'; el.classList.add('is-warn'); return; }
    el.textContent = 'running';
  }

  // ------------------------------------------------------------------ controls
  const btnPause = $('btnPause'), btnEstop = $('btnEstop');
  function canAct() { return MOCK || M.conn === 'connected'; }
  function renderControls() {
    const S = M.state || {};
    const ok = canAct();
    const paused = S.paused === true;
    const estop = S.estop === true;
    btnPause.disabled = !ok || M.pauseBusy;
    btnPause.textContent = M.pauseBusy ? (M.pauseWant ? 'PAUSING' : 'RESUMING') : (paused ? 'RESUME' : 'PAUSE');
    btnPause.classList.toggle('is-paused', paused);
    btnPause.title = ok ? (paused ? 'Resume the simulation (' + SERVICES.resume.name + ')' : 'Pause the simulation (' + SERVICES.pause.name + ')') : 'Not connected';
    btnEstop.disabled = !ok || M.estopBusy;
    btnEstop.textContent = M.estopBusy ? (M.estopWant ? 'STOPPING' : 'RELEASING') : (estop ? 'RELEASE E-STOP' : 'E-STOP');
    btnEstop.classList.toggle('is-active', estop);
    btnEstop.title = ok ? (SERVICES.estop.name + (estop ? ' data:false (asks for confirmation)' : ' data:true')) : 'Not connected';

    const W = M.wms || {};
    const auto = W.auto === true;
    const bA = $('btnAuto');
    bA.disabled = !ok;
    bA.setAttribute('aria-pressed', auto ? 'true' : 'false');
    bA.textContent = auto ? 'ON' : 'OFF';
    ['inpPeriod', 'btnPeriod', 'btnOrderA', 'btnOrderB'].forEach((id) => { $(id).disabled = !ok; });
    const inp = $('inpPeriod');
    if (isNum(W.period_s) && !M.periodDirty && document.activeElement !== inp) inp.value = String(Math.round(W.period_s * 10) / 10);
  }

  // ------------------------------------------------------------------ KPIs
  function setKpi(id, value, cls, sub) {
    const el = $(id);
    if (!el) return;
    el.querySelector('.kpi-value').textContent = value;
    if (sub !== undefined) el.querySelector('.kpi-sub').textContent = sub;
    el.classList.remove('is-ok', 'is-warn', 'is-fault');
    if (cls) el.classList.add('is-' + cls);
  }
  function minSep(S) {
    const cs = arr(S.carters).filter((c) => c && isNum(c.x) && isNum(c.y));
    const ps = arr(S.people).filter((p) => p && isNum(p.x) && isNum(p.y) && p.kind !== 'prop');
    let cc = null, cp = null;
    for (let i = 0; i < cs.length; i++) {
      for (let j = i + 1; j < cs.length; j++) { const d = Math.hypot(cs[i].x - cs[j].x, cs[i].y - cs[j].y); if (cc === null || d < cc) cc = d; }
      for (let j = 0; j < ps.length; j++) { const d = Math.hypot(cs[i].x - ps[j].x, cs[i].y - ps[j].y); if (cp === null || d < cp) cp = d; }
    }
    return { cc, cp };
  }
  function wmsCounts() {
    const W = M.wms;
    if (!W) return null;
    const c = objOr(W.counts);
    const orders = arr(W.orders);
    const cnt = (f) => orders.filter(f).length;
    return {
      queued: num(c.queued, cnt((o) => o && o.state === 'queued')),
      progress: num(c.in_progress, cnt((o) => o && (o.state === 'released' || o.state === 'on_carter'))),
      delivered: num(c.delivered, cnt((o) => o && o.state === 'delivered')),
    };
  }
  function renderKpis() {
    const S = M.state || {};
    const K = objOr(S.kpi);
    let total = num(K.delivered_total);
    if (total === null && arr(S.forklifts).length) total = arr(S.forklifts).reduce((a, f) => a + num(f && f.delivered, 0), 0);
    setKpi('kpiDelivered', total === null ? '-' : String(total), total ? 'ok' : null);
    const wmsRate = M.wms ? num(M.wms.throughput_per_hour) : null;
    setKpi('kpiRate', fx(num(K.per_hour), 1), null, 'crates / h');
    $('kpiRate').title = 'state kpi.per_hour' + (wmsRate !== null ? '; WMS throughput_per_hour ' + wmsRate.toFixed(1) : '');
    const wc = wmsCounts();
    setKpi('kpiQueued', wc ? String(wc.queued) : '-', null);
    setKpi('kpiProgress', wc ? String(wc.progress) : '-', null);
    setKpi('kpiDone', wc ? String(wc.delivered) : '-', null);
    const ms = minSep(S);
    const cc = num(K.min_carter_sep, ms.cc);
    const cp = num(K.min_person_sep, ms.cp);
    setKpi('kpiCarterSep', fx(cc, 2), isNum(cc) && cc < 0.8 ? 'fault' : null);
    setKpi('kpiPersonSep', fx(cp, 2), isNum(cp) ? (cp < 1.0 ? 'fault' : null) : null);
    const y = num(K.person_yields);
    const st = num(K.stalls);
    const sh = num(K.pallets_shipped);
    setKpi('kpiYields', y === null ? '-' : String(y), st ? 'warn' : null,
      'robot stops' + (sh !== null ? ' · ' + sh + ' pallets shipped' : '') + (st ? ' · ' + st + ' stall escapes' : ''));
  }

  // ------------------------------------------------------------------ fleet table
  const fleetBody = $('fleetBody');
  const fleetRows = new Map();
  function fleetRow(key, isGroup) {
    let r = fleetRows.get(key);
    if (r) return r;
    const tr = mk('tr', isGroup ? 'group' : '');
    const cells = [];
    if (isGroup) { const td = mk('td'); td.colSpan = 5; tr.appendChild(td); cells.push(td); }
    else {
      for (let i = 0; i < 5; i++) { const td = mk('td', i === 2 ? 'num' : ''); tr.appendChild(td); cells.push(td); }
      const nameWrap = mk('span', 'uname');
      const dot = mk('i', 'sdot');
      const nm = mk('span');
      nameWrap.appendChild(dot); nameWrap.appendChild(nm);
      cells[0].appendChild(nameWrap);
      tr.dataset.name = key;
      r = { tr, cells, dot, nm };
    }
    if (!r) r = { tr, cells };
    fleetRows.set(key, r);
    return r;
  }
  function setCell(td, text) { if (td.textContent !== text) { td.textContent = text; td.title = text; } }
  function renderFleet() {
    const S = M.state || {};
    const estop = S.estop === true;
    const order = [];
    const groups = [
      ['carters', 'Carters (AMR)'],
      ['forklifts', 'Forklifts'],
      ['arms', 'Arms'],
    ];
    groups.forEach(([k, label]) => {
      const list = arr(S[k]).filter((e) => e && e.name !== undefined);
      if (!list.length) return;
      const g = fleetRow('g:' + k, true);
      g.cells[0].textContent = label;
      order.push(g.tr);
      list.forEach((e) => {
        const name = str(e.name);
        const r = fleetRow(name, false);
        let cls, cargo, detail;
        const v = speedOf(e);
        if (k === 'carters') {
          cls = carterCls(e, estop);
          cargo = e.cargo === true ? 'crate' : '-';
          const bits = ['job ' + (e.job ? str(e.job) : '-')];
          if (e.blocked_by_person) bits.push('person ahead');
          if (e.parked) bits.push('parked');
          detail = bits.join(', ');
        } else if (k === 'forklifts') {
          cls = forkCls(e, estop);
          cargo = e.cargo === true ? 'crate' : '-';
          detail = 'delivered ' + (isNum(e.delivered) ? e.delivered : '-') + ', lift ' + fx(num(e.lift), 2) + ' m';
        } else {
          cls = armCls(e, estop);
          cargo = e.vacuum === true ? 'vacuum on' : '-';
          detail = 'line ' + str(e.line, '-');
        }
        r.dot.className = 'sdot ' + cls;
        if (r.nm.textContent !== name) r.nm.textContent = name;
        setCell(r.cells[1], str(e.status, '-') || '-');
        setCell(r.cells[2], isNum(v) ? Math.abs(v).toFixed(2) : '-');
        setCell(r.cells[3], cargo);
        setCell(r.cells[4], detail);
        r.tr.classList.toggle('is-fault', estop);
        r.tr.classList.toggle('is-warn', !estop && (e.blocked_by_person === true));
        r.tr.classList.toggle('is-selected', M.selected === name);
        order.push(r.tr);
      });
    });
    if (!order.length) {
      if (!fleetBody.querySelector('tr.empty')) { fleetBody.textContent = ''; const tr = mk('tr', 'empty'); const td = mk('td', '', 'No state received'); td.colSpan = 5; tr.appendChild(td); fleetBody.appendChild(tr); }
      return;
    }
    const empty = fleetBody.querySelector('tr.empty');
    if (empty) empty.remove();
    // re-order only when needed
    const cur = Array.from(fleetBody.children);
    let same = cur.length === order.length;
    if (same) for (let i = 0; i < order.length; i++) if (cur[i] !== order[i]) { same = false; break; }
    if (!same) { fleetBody.textContent = ''; order.forEach((tr) => fleetBody.appendChild(tr)); }
  }
  fleetBody.addEventListener('click', (ev) => {
    const tr = ev.target.closest('tr');
    if (!tr || !tr.dataset.name) return;
    selectTarget(tr.dataset.name, true);
  });

  // ------------------------------------------------------------------ orders
  let ordersSig = '';
  function renderOrders() {
    const W = M.wms || {};
    const S = M.state || {};
    const simT = num(S.t);
    const orders = arr(W.orders).filter((o) => o && typeof o === 'object').slice();
    orders.sort((a, b) => (num(b.created_t, 0) - num(a.created_t, 0)) || str(b.id).localeCompare(str(a.id)));
    const last = orders.slice(0, 12);
    const wc = wmsCounts();
    $('wmsNote').textContent = wc ? (wc.queued + ' queued, ' + wc.progress + ' in progress, ' + wc.delivered + ' delivered') : 'no status';
    const rate = num(W.throughput_per_hour);
    $('wmsRate').textContent = (rate !== null ? rate.toFixed(1) + ' / h' : '') + (isNum(W.period_s) ? '  every ' + W.period_s + ' s' : '');
    const sig = JSON.stringify(last) + '|' + Math.floor(num(simT, 0));
    if (sig === ordersSig) return;
    ordersSig = sig;
    const tb = $('ordersBody');
    tb.textContent = '';
    if (!last.length) { const tr = mk('tr', 'empty'); const td = mk('td', '', 'No orders'); td.colSpan = 5; tr.appendChild(td); tb.appendChild(tr); return; }
    last.forEach((o) => {
      const tr = mk('tr');
      tr.appendChild(mk('td', '', str(o.id, '-')));
      const tdl = mk('td'); tdl.appendChild(mk('span', 'chip line', str(o.line, '-'))); tr.appendChild(tdl);
      const st = str(o.state, 'unknown');
      const tds = mk('td'); tds.appendChild(mk('span', 'chip st-' + st.replace(/[^a-z_]/gi, ''), st.replace(/_/g, ' '))); tr.appendChild(tds);
      tr.appendChild(mk('td', 'num', fmtClock(num(o.created_t))));
      let lead = '-';
      const ct = num(o.created_t), dt = num(o.done_t);
      const td5 = mk('td', 'num');
      if (ct !== null && dt !== null) lead = Math.round(dt - ct) + ' s';
      else if (ct !== null && simT !== null && simT >= ct) { lead = Math.round(simT - ct) + ' s'; td5.classList.add('muted'); td5.title = 'open, age so far'; }
      td5.textContent = lead;
      tr.appendChild(td5);
      tb.appendChild(tr);
    });
  }

  // ------------------------------------------------------------------ events
  const EV_MAX = 200;
  const evList = $('events');
  const evFilter = $('eventFilter');
  let evTotal = 0;
  function actorClass(a) {
    const s = str(a).trim();
    if (/^C\d+\b/i.test(s) || /carter|amr/i.test(s)) return 'carter';
    if (/^F\d+\b/i.test(s) || /fork/i.test(s)) return 'forklift';
    if (/ur5|franka|panda|\barm/i.test(s)) return 'arm';
    if (/belt|conveyor|^line\b/i.test(s)) return 'belt';
    if (/dispatch|wms|order|sched/i.test(s)) return 'dispatch';
    if (/person|people|human|prop|cart|dolly/i.test(s)) return 'person';
    return 'other';
  }
  function addEvent(raw) {
    const line = str(raw).replace(/\s+$/, '');
    if (!line) return;
    let t = '', rest = line;
    const m = /^\s*\[\s*([-+\d.]+)\s*s?\s*\]\s*(.*)$/.exec(line);
    if (m) { t = m[1] + 's'; rest = m[2]; }
    let actor = '', msg = rest;
    const a = /^([A-Za-z][\w\-./ ]{0,23}?):\s*(.*)$/.exec(rest);
    if (a) { actor = a[1]; msg = a[2]; }
    const li = mk('li', 'ev-' + actorClass(actor || msg));
    if (/e-?stop|fault|error|fail|collision|abort/i.test(msg)) li.classList.add('sev-fault');
    else if (/yield|blocked|timeout|retry|stuck/i.test(msg)) li.classList.add('sev-warn');
    li.appendChild(mk('span', 'ev-t', t));
    li.appendChild(mk('span', 'ev-a', actor));
    li.appendChild(mk('span', 'ev-m', msg));
    li.dataset.text = line.toLowerCase();
    li.title = line;
    const ph = evList.querySelector('.ev-empty');
    if (ph) ph.remove();
    applyFilter(li, evFilter.value.trim().toLowerCase());
    evList.insertBefore(li, evList.firstChild);
    evTotal++;
    while (evList.children.length > EV_MAX) evList.removeChild(evList.lastChild);
    updateEventCount();
  }
  function applyFilter(li, q) { li.hidden = !!q && li.dataset.text.indexOf(q) < 0; }
  function updateEventCount() {
    const n = evList.querySelectorAll('li:not(.ev-empty)').length;
    const shown = evList.querySelectorAll('li:not([hidden]):not(.ev-empty)').length;
    $('eventCount').textContent = (evFilter.value.trim() ? shown + ' of ' : '') + n + (evTotal > n ? ' (last ' + EV_MAX + ')' : '');
  }
  // history from the latched /warehouse/events_recent, merged once per connection with whatever
  // live lines already arrived (no duplicates, oldest first so the newest ends on top)
  const onEventsRecent = safe('eventsRecent', function (raw) {
    if (M.evBackfilled) return;
    const hist = parseJson(raw);
    if (!Array.isArray(hist)) return;
    M.evBackfilled = true;
    const seen = new Set(hist.map(String));
    const live = M.evLines.filter((l) => !seen.has(l));
    M.evLines = hist.map(String).concat(live);
    evList.textContent = '';
    M.evLines.forEach((l) => addEvent(l));
  });
  function evPlaceholder() { if (!evList.children.length) { const li = mk('li', 'ev-empty', 'No events yet on ' + TOPICS.events.name); evList.appendChild(li); } }
  evFilter.addEventListener('input', () => {
    const q = evFilter.value.trim().toLowerCase();
    evList.querySelectorAll('li:not(.ev-empty)').forEach((li) => applyFilter(li, q));
    updateEventCount();
  });
  $('btnClearLog').addEventListener('click', () => { evList.textContent = ''; evTotal = 0; evPlaceholder(); updateEventCount(); });

  // ------------------------------------------------------------------ cameras
  const cams = {};
  [['overview', 'camOverview'], ['follow', 'camFollow']].forEach(([k, id]) => {
    const root = $(id);
    const c = {
      key: id, root, img: root.querySelector('[data-img]'), meta: root.querySelector('[data-meta]'),
      nosig: root.querySelector('[data-nosig]'), nosigText: root.querySelector('[data-nosig-text]'),
      last: 0, times: [], size: '', busy: false, pending: null,
    };
    c.img.addEventListener('load', () => {
      c.busy = false;
      c.size = c.img.naturalWidth + 'x' + c.img.naturalHeight;
      if (c.pending) { const p = c.pending; c.pending = null; c.busy = true; c.img.src = p; }
    });
    c.img.addEventListener('error', () => { c.busy = false; c.pending = null; });
    cams[k] = c;
  });
  function bytesToB64(bytes) {
    let s = '';
    const CH = 0x8000;
    for (let i = 0; i < bytes.length; i += CH) s += String.fromCharCode.apply(null, Array.prototype.slice.call(bytes, i, i + CH));
    return btoa(s);
  }
  const onCam = safe('camera', function (which, msg) {
    const c = cams[which];
    if (!c || !msg) return;
    let d = msg.data;
    if (Array.isArray(d) || d instanceof Uint8Array) d = bytesToB64(d);
    if (typeof d !== 'string' || !d.length) return;
    const fmt = str(msg.format, 'jpeg').toLowerCase();
    const mime = fmt.indexOf('png') >= 0 ? 'image/png' : 'image/jpeg';
    const url = 'data:' + mime + ';base64,' + d;
    if (c.busy) c.pending = url; // drop intermediate frames while one is decoding
    else { c.busy = true; c.img.src = url; }
    const t = now();
    c.last = t;
    c.times.push(t);
    while (c.times.length && t - c.times[0] > 3000) c.times.shift();
  });
  function renderCams() {
    const t = now();
    Object.keys(cams).forEach((k) => {
      const c = cams[k];
      const live = c.last && t - c.last < NO_SIGNAL_MS;
      c.nosig.hidden = !!live;
      if (!live) {
        c.nosigText.textContent = c.last ? 'last frame ' + Math.round((t - c.last) / 1000) + ' s ago' : 'waiting for ' + TOPICS[c.key].name;
        c.meta.textContent = '-';
      } else {
        const fps = c.times.length > 1 ? (c.times.length - 1) / ((c.times[c.times.length - 1] - c.times[0]) / 1000) : 0;
        c.meta.textContent = fps.toFixed(1) + ' fps' + (c.size ? '  ' + c.size : '');
      }
    });
  }

  // follow target select
  const followSel = $('followSel');
  let followSig = '';
  function syncFollowOptions() {
    const S = M.state || {};
    const groups = [['Carters', arr(S.carters)], ['Forklifts', arr(S.forklifts)], ['Arms', arr(S.arms)], ['People', arr(S.people)]];
    const sig = groups.map((g) => g[1].map((e) => str(e && e.name)).join(',')).join('|');
    if (sig === followSig) return;
    followSig = sig;
    const cur = followSel.value;
    followSel.textContent = '';
    followSel.appendChild(new Option('overview', 'overview'));
    groups.forEach(([label, list]) => {
      const names = list.map((e) => str(e && e.name)).filter(Boolean);
      if (!names.length) return;
      const og = document.createElement('optgroup');
      og.label = label;
      names.forEach((n) => og.appendChild(new Option(n, n)));
      followSel.appendChild(og);
    });
    setFollowValue(cur || 'overview');
  }
  function setFollowValue(v) {
    if (!Array.from(followSel.options).some((o) => o.value === v)) followSel.appendChild(new Option(v, v));
    followSel.value = v;
  }
  followSel.addEventListener('change', () => {
    const v = followSel.value;
    selectTarget(v === 'overview' ? null : v, true, v);
  });
  function selectTarget(name, publishIt, explicit) {
    M.selected = name || null;
    if (publishIt) M.followUserAt = now();
    const target = explicit || name || 'overview';
    setFollowValue(target);
    if (publishIt) publish('followTarget', target, 'Follow camera: ' + target);
    renderFleet();
  }

  // ------------------------------------------------------------------ ROS info panel
  const rosRows = {};
  function buildRosPanel() {
    $('rosUrl').textContent = MOCK ? 'mock mode, no rosbridge connection' : WS_URL;
    const tb = $('rosBody');
    const shortType = (t) => t.replace('/msg/', '/').replace('/srv/', '/');
    Object.keys(TOPICS).forEach((k) => {
      const T = TOPICS[k];
      const tr = mk('tr');
      tr.appendChild(mk('td', '', T.name));
      tr.appendChild(mk('td', '', shortType(T.type)));
      tr.appendChild(mk('td', '', T.dir));
      const rate = mk('td', 'num', '-');
      tr.appendChild(rate);
      tr.title = T.name + (T.throttle_rate ? '  (throttle_rate ' + T.throttle_rate + ' ms, queue_length ' + T.queue_length + ')' : '');
      tb.appendChild(tr);
      rosRows['t:' + k] = rate;
    });
    Object.keys(SERVICES).forEach((k) => {
      const T = SERVICES[k];
      const tr = mk('tr');
      tr.appendChild(mk('td', '', T.name));
      tr.appendChild(mk('td', '', shortType(T.type)));
      tr.appendChild(mk('td', '', 'srv'));
      const rate = mk('td', 'num', '-');
      tr.appendChild(rate);
      tb.appendChild(tr);
      rosRows['s:' + k] = rate;
    });
  }
  function renderRosPanel() {
    Object.keys(TOPICS).forEach((k) => {
      const td = rosRows['t:' + k];
      const s = stats[k];
      td.classList.remove('rate-live', 'rate-fail');
      if (TOPICS[k].dir === 'pub') { td.textContent = s ? 'sent ' + s.total : '-'; return; }
      const r = rateOf(k);
      if (r > 0) { td.textContent = r.toFixed(1) + ' Hz'; td.classList.add('rate-live'); }
      else td.textContent = s ? 'got ' + s.total : '-';
    });
    Object.keys(SERVICES).forEach((k) => {
      const td = rosRows['s:' + k];
      const s = stats['srv:' + k];
      td.classList.remove('rate-live', 'rate-fail');
      if (!s) { td.textContent = '-'; return; }
      td.textContent = s.ok ? 'ok' : 'failed';
      td.title = s.msg || '';
      td.classList.add(s.ok ? 'rate-live' : 'rate-fail');
    });
  }

  // ------------------------------------------------------------------ toasts + confirm dialog
  function toast(kind, text, ms) {
    const box = $('toasts');
    const t = mk('div', 'toast ' + (kind || ''), text);
    box.appendChild(t);
    while (box.children.length > 5) box.removeChild(box.firstChild);
    setTimeout(() => t.remove(), ms || 3500);
  }
  function confirmDlg(title, text, okLabel) {
    const dlg = $('confirmDlg');
    if (!dlg || typeof dlg.showModal !== 'function') return Promise.resolve(window.confirm(title + '\n\n' + text));
    $('confirmTitle').textContent = title;
    $('confirmText').textContent = text;
    $('confirmOk').textContent = okLabel || 'Confirm';
    return new Promise((resolve) => {
      const done = () => { dlg.removeEventListener('close', done); resolve(dlg.returnValue === 'ok'); };
      dlg.returnValue = '';
      dlg.addEventListener('close', done);
      dlg.showModal();
      const cancel = dlg.querySelector('button[value="cancel"]');
      if (cancel) cancel.focus();
    });
  }

  // ------------------------------------------------------------------ actions
  let bus = null; // RosLink or MockSim
  function publish(key, payload, label) {
    const data = typeof payload === 'string' ? payload : JSON.stringify(payload);
    if (!bus || !canAct()) { toast('warn', 'Not connected: ' + (label || TOPICS[key].name) + ' not sent'); return false; }
    const ok = bus.publish(key, data);
    if (ok) { bump(key); if (MOCK) toast('', '[mock] ' + TOPICS[key].name + '  ' + data); }
    return ok;
  }
  function callService(key, req, label) {
    if (!bus || !canAct()) { toast('warn', 'Not connected: ' + label + ' not sent'); return Promise.resolve(null); }
    return bus.call(key, req || {}).then((res) => {
      res = objOr(res);
      const ok = res.success !== false;
      stats['srv:' + key] = { ok, msg: str(res.message), at: now() };
      toast(ok ? 'ok' : 'fault', label + (ok ? ': ok' : ': failed') + (res.message ? ' (' + res.message + ')' : ''));
      return res;
    }, (err) => {
      stats['srv:' + key] = { ok: false, msg: str(err && err.message, String(err)), at: now() };
      toast('fault', label + ': ' + str(err && err.message, String(err)));
      return null;
    });
  }
  function busyTimeout(flag) {
    setTimeout(() => { if (M[flag]) { M[flag] = false; renderControls(); } }, 4000);
  }
  btnPause.addEventListener('click', () => {
    const paused = (M.state || {}).paused === true;
    M.pauseBusy = true; M.pauseWant = !paused; renderControls(); busyTimeout('pauseBusy');
    const p = paused ? callService('resume', {}, 'Resume') : callService('pause', {}, 'Pause');
    p.then((res) => { if (!res || res.success === false) { M.pauseBusy = false; renderControls(); } });
  });
  btnEstop.addEventListener('click', () => {
    const estop = (M.state || {}).estop === true;
    const go = (want) => {
      M.estopBusy = true; M.estopWant = want; renderControls(); busyTimeout('estopBusy');
      callService('estop', { data: want }, want ? 'E-STOP' : 'Release E-STOP').then((res) => { if (!res || res.success === false) { M.estopBusy = false; renderControls(); } });
    };
    if (!estop) { go(true); return; }
    confirmDlg('Release E-STOP?', 'Robots will resume moving. Make sure the floor is clear before releasing.', 'Release E-STOP').then((yes) => { if (yes) go(false); });
  });
  $('btnAuto').addEventListener('click', () => {
    const W = M.wms || {};
    const period = currentPeriod(W);
    publish('wmsConfig', { auto: !(W.auto === true), period_s: period }, 'WMS config');
  });
  function currentPeriod(W) {
    const v = parseFloat($('inpPeriod').value);
    if (isFinite(v) && v > 0) return v;
    return isNum(W.period_s) ? W.period_s : 20;
  }
  $('inpPeriod').addEventListener('input', () => { M.periodDirty = true; });
  function applyPeriod() {
    const v = parseFloat($('inpPeriod').value);
    if (!isFinite(v) || v <= 0) { toast('warn', 'Order period must be a positive number of seconds'); return; }
    const W = M.wms || {};
    if (publish('wmsConfig', { auto: W.auto === true, period_s: v }, 'WMS config')) M.periodDirty = false;
  }
  $('btnPeriod').addEventListener('click', applyPeriod);
  $('inpPeriod').addEventListener('keydown', (e) => { if (e.key === 'Enter') applyPeriod(); });
  $('btnOrderA').addEventListener('click', () => publish('newOrder', { line: 'A', qty: 1 }, 'New order A'));
  $('btnOrderB').addEventListener('click', () => publish('newOrder', { line: 'B', qty: 1 }, 'New order B'));

  // ------------------------------------------------------------------ map renderer
  const mapWrap = $('mapWrap');
  const canvas = $('map');
  const ctx = canvas.getContext('2d');
  const tooltip = $('tooltip');
  const staticCanvas = document.createElement('canvas');
  const sctx = staticCanvas.getContext('2d');
  let mapStaticDirty = true;
  let mapView = null;
  let mapHits = [];
  let cssW = 0, cssH = 0, dpr = 1;
  let mouse = null;

  function makeView(b, W, H, pad) {
    pad = pad === undefined ? 8 : pad;
    const bw = Math.max(1, b.x1 - b.x0), bh = Math.max(1, b.y1 - b.y0);
    const s = Math.max(0.5, Math.min((W - 2 * pad) / bw, (H - 2 * pad) / bh));
    const ox = (W - bw * s) / 2, oy = (H - bh * s) / 2;
    return {
      s, b, W, H, ox, oy,
      X: (x) => ox + (x - b.x0) * s,
      Y: (y) => oy + (b.y1 - y) * s,
      wx: (px) => b.x0 + (px - ox) / s,
      wy: (py) => b.y1 - (py - oy) / s,
    };
  }
  function viewBounds(L) {
    const z = L.zone;
    let b = { x0: z.x0 - 1, x1: z.x1 + 1, y0: z.y0 - 1, y1: z.y1 + 1 };
    if (M.fit === 'building' && L.building) {
      const B = L.building;
      b = { x0: Math.min(b.x0, B.x0 - 0.5), x1: Math.max(b.x1, B.x1 + 0.5), y0: Math.min(b.y0, B.y0 - 0.5), y1: Math.max(b.y1, B.y1 + 0.5) };
    }
    return b;
  }
  function resizeMap() {
    const r = mapWrap.getBoundingClientRect();
    const w = Math.max(50, Math.floor(r.width)), h = Math.max(50, Math.floor(r.height));
    const d = window.devicePixelRatio || 1;
    if (w === cssW && h === cssH && d === dpr) return;
    cssW = w; cssH = h; dpr = d;
    canvas.width = Math.round(w * d); canvas.height = Math.round(h * d);
    staticCanvas.width = canvas.width; staticCanvas.height = canvas.height;
    mapStaticDirty = true;
  }

  // drawing primitives (all coordinates in CSS px; ctx carries the dpr transform)
  function rrect(c, x, y, w, h, r) {
    r = Math.min(r, w / 2, h / 2);
    c.beginPath();
    c.moveTo(x + r, y);
    c.arcTo(x + w, y, x + w, y + h, r);
    c.arcTo(x + w, y + h, x, y + h, r);
    c.arcTo(x, y + h, x, y, r);
    c.arcTo(x, y, x + w, y, r);
    c.closePath();
  }
  function wrect(v, r) { // world rect -> px rect
    const x = v.X(r.x0), y = v.Y(r.y1);
    return [x, y, v.X(r.x1) - x, v.Y(r.y0) - y];
  }
  function label(c, text, x, y, opts) {
    opts = opts || {};
    c.font = (opts.weight || '600') + ' ' + (opts.size || 11) + 'px ' + FONT;
    c.textAlign = opts.align || 'center';
    c.textBaseline = opts.base || 'middle';
    c.lineJoin = 'round';
    c.lineWidth = 3;
    c.strokeStyle = C['map-halo'];
    c.strokeText(text, x, y);
    c.fillStyle = opts.color || C['map-label'];
    c.fillText(text, x, y);
  }
  function hatch(c, x, y, w, h, step, color) {
    c.save();
    c.beginPath(); c.rect(x, y, w, h); c.clip();
    c.strokeStyle = color; c.lineWidth = 1; c.globalAlpha = 0.55;
    c.beginPath();
    for (let i = -h; i < w; i += step) { c.moveTo(x + i, y + h); c.lineTo(x + i + h, y); }
    c.stroke();
    c.restore();
  }

  function drawStatic(c, v, L, opts) {
    opts = opts || {};
    const s = v.s;
    c.fillStyle = C['map-outside'];
    c.fillRect(0, 0, v.W, v.H);
    // floor: building (if any) then zone
    if (L.building) {
      const br = wrect(v, L.building);
      c.save(); c.globalAlpha = 0.55; c.fillStyle = C['map-bg']; c.fillRect(br[0], br[1], br[2], br[3]); c.restore();
    }
    const zr = wrect(v, L.zone);
    c.fillStyle = C['map-bg'];
    c.fillRect(zr[0], zr[1], zr[2], zr[3]);

    // grid (1 m, stronger every 5 m) over the visible world
    const b = v.b;
    const gx0 = Math.ceil(b.x0), gx1 = Math.floor(b.x1), gy0 = Math.ceil(b.y0), gy1 = Math.floor(b.y1);
    const fine = s >= 5;
    c.lineWidth = 1;
    for (let pass = 0; pass < 2; pass++) {
      c.beginPath();
      for (let x = gx0; x <= gx1; x++) {
        const major = x % 5 === 0;
        if ((pass === 0 && (major || !fine)) || (pass === 1 && !major)) continue;
        const px = Math.round(v.X(x)) + 0.5;
        c.moveTo(px, v.Y(b.y1)); c.lineTo(px, v.Y(b.y0));
      }
      for (let y = gy0; y <= gy1; y++) {
        const major = y % 5 === 0;
        if ((pass === 0 && (major || !fine)) || (pass === 1 && !major)) continue;
        const py = Math.round(v.Y(y)) + 0.5;
        c.moveTo(v.X(b.x0), py); c.lineTo(v.X(b.x1), py);
      }
      c.strokeStyle = pass === 0 ? C['map-grid'] : C['map-grid5'];
      c.stroke();
    }

    // building outline
    if (L.building) {
      const br = wrect(v, L.building);
      c.save(); c.setLineDash([6, 4]); c.strokeStyle = C['map-building']; c.lineWidth = 1.5;
      c.strokeRect(br[0] + 0.5, br[1] + 0.5, br[2], br[3]); c.restore();
      if (!opts.noText && br[1] > 2) label(c, 'Building', br[0] + 6, br[1] + 10, { align: 'left', color: C.faint, weight: '500', size: 10 });
    }

    // static obstacles
    c.fillStyle = C['map-static']; c.strokeStyle = C['map-static-edge']; c.lineWidth = 1;
    L.statics.forEach((r) => { const p = wrect(v, r); c.fillRect(p[0], p[1], p[2], p[3]); c.strokeRect(p[0] + 0.5, p[1] + 0.5, p[2] - 1, p[3] - 1); });

    // forklift strips (hatched)
    L.strips.forEach((r) => {
      const p = wrect(v, r);
      hatch(c, p[0], p[1], p[2], p[3], 8, C['map-strip']);
      c.strokeStyle = C['map-strip']; c.lineWidth = 1; c.strokeRect(p[0] + 0.5, p[1] + 0.5, p[2] - 1, p[3] - 1);
      if (!opts.noText && r.name) label(c, r.name + ' strip', p[0] + 4, p[1] + p[3] - 8, { align: 'left', size: 10, weight: '500', color: C.muted });
    });

    // pallets
    L.pallets.forEach((pl) => {
      const p = wrect(v, { x0: pl.x - pl.w / 2, x1: pl.x + pl.w / 2, y0: pl.y - pl.h / 2, y1: pl.y + pl.h / 2 });
      c.fillStyle = C['map-pallet']; c.fillRect(p[0], p[1], p[2], p[3]);
      c.strokeStyle = C['map-pallet-edge']; c.lineWidth = 1; c.strokeRect(p[0] + 0.5, p[1] + 0.5, p[2] - 1, p[3] - 1);
      c.beginPath();
      for (let i = 1; i < 4; i++) { const yy = Math.round(p[1] + (p[3] * i) / 4) + 0.5; c.moveTo(p[0] + 1, yy); c.lineTo(p[0] + p[2] - 1, yy); }
      c.globalAlpha = 0.5; c.stroke(); c.globalAlpha = 1;
    });

    // docks (dashed)
    L.docks.forEach((d) => {
      const p = wrect(v, { x0: d.x - d.w / 2, x1: d.x + d.w / 2, y0: d.y - d.h / 2, y1: d.y + d.h / 2 });
      c.save(); c.setLineDash([4, 3]); c.strokeStyle = C['map-dock']; c.lineWidth = 1.5; c.strokeRect(p[0], p[1], p[2], p[3]); c.restore();
      if (!opts.noText) label(c, 'Dock ' + d.line, p[0] + p[2] / 2, d.y > 0 ? p[1] + p[3] + 8 : p[1] - 8, { size: 10, weight: '500', color: C.muted });
    });

    // belts
    L.belts.forEach((bt) => {
      const p = wrect(v, { x0: Math.min(bt.x0, bt.x1), x1: Math.max(bt.x0, bt.x1), y0: bt.y - bt.w / 2, y1: bt.y + bt.w / 2 });
      c.fillStyle = C['map-belt']; c.fillRect(p[0], p[1], p[2], p[3]);
      c.strokeStyle = C['map-belt-edge']; c.lineWidth = 1; c.strokeRect(p[0] + 0.5, p[1] + 0.5, p[2] - 1, p[3] - 1);
    });

    // parking spots
    L.parking.forEach((pk) => {
      const w = 0.9 * s, h = 0.75 * s;
      const x = v.X(pk.x) - w / 2, y = v.Y(pk.y) - h / 2;
      c.save(); c.setLineDash([2, 3]); c.strokeStyle = C.faint; c.lineWidth = 1; c.strokeRect(x, y, w, h); c.restore();
      if (!opts.noText) label(c, 'P ' + pk.name, x - 4, y + h / 2, { align: 'right', size: 9, weight: '500', color: C.faint });
    });

    // zone outline
    c.strokeStyle = C['map-zone']; c.lineWidth = 2;
    c.strokeRect(zr[0], zr[1], zr[2], zr[3]);

    if (!opts.noText) {
      // coordinate ticks every 5 m along the zone edges
      c.font = '500 10px ' + FONT; c.fillStyle = C.faint;
      c.textAlign = 'center'; c.textBaseline = 'top';
      for (let x = Math.ceil(L.zone.x0 / 5) * 5; x <= L.zone.x1; x += 5) c.fillText(String(x), v.X(x), zr[1] + zr[3] + 3);
      c.textAlign = 'right'; c.textBaseline = 'middle';
      for (let y = Math.ceil(L.zone.y0 / 5) * 5; y <= L.zone.y1; y += 5) c.fillText(String(y), zr[0] - 3, v.Y(y));
      label(c, 'Operating zone ' + Math.round(L.zone.x1 - L.zone.x0) + ' x ' + Math.round(L.zone.y1 - L.zone.y0) + ' m', zr[0] + 6, zr[1] + 10, { align: 'left', size: 10, weight: '500', color: C.muted });

      // scale bar
      const cands = [1, 2, 5, 10, 20];
      let len = cands[0];
      for (const cnd of cands) { if (cnd * s <= 140) len = cnd; }
      const sx = 12, sy = v.H - 14, sw = len * s;
      c.strokeStyle = C.text; c.lineWidth = 2;
      c.beginPath(); c.moveTo(sx, sy - 4); c.lineTo(sx, sy); c.lineTo(sx + sw, sy); c.lineTo(sx + sw, sy - 4); c.stroke();
      label(c, len + ' m', sx + sw + 6, sy - 2, { align: 'left', size: 10 });
      label(c, '+x east, +y north', v.W - 10, v.H - 12, { align: 'right', size: 10, weight: '500', color: C.faint });
    }
  }

  function interp(cur, idxMap, key, f) {
    const p = idxMap && idxMap.get(key);
    const yaw = num(cur.yaw, 0);
    if (!p || !isNum(p.x) || !isNum(p.y) || f >= 1) return { x: cur.x, y: cur.y, yaw };
    if (Math.hypot(cur.x - p.x, cur.y - p.y) > 3) return { x: cur.x, y: cur.y, yaw };
    return { x: p.x + (cur.x - p.x) * f, y: p.y + (cur.y - p.y) * f, yaw: angLerp(num(p.yaw, yaw), yaw, f) };
  }

  function drawDynamic(c, v, L, S, opts) {
    opts = opts || {};
    const hits = opts.hits || null;
    const s = v.s;
    const P = M.prevIdx;
    const f = opts.f === undefined ? 1 : opts.f;
    const estop = S && S.estop === true;
    const running = S && S.paused !== true && !estop;
    const tw = now() / 1000;

    // belts: chevrons + info
    const sBelts = arr(S && S.belts);
    const beltList = L.belts.length ? L.belts : sBelts.filter((b) => b && isNum(b.x0) && isNum(b.x1) && isNum(b.y)).map((b) => ({ line: str(b.line), x0: b.x0, x1: b.x1, y: b.y, w: 0.6 }));
    beltList.forEach((bt) => {
      const sb = sBelts.find((b) => b && str(b.line) === bt.line) || {};
      const xl = Math.min(bt.x0, bt.x1), xr = Math.max(bt.x0, bt.x1);
      const pick = num(sb.pick_x);
      const dir = pick !== null ? (pick >= (xl + xr) / 2 ? 1 : -1) : (bt.x1 >= bt.x0 ? 1 : -1);
      const p = wrect(v, { x0: xl, x1: xr, y0: bt.y - bt.w / 2, y1: bt.y + bt.w / 2 });
      const spacing = 0.8;
      const phase = running ? (tw * 0.35) % spacing : 0;
      c.save();
      c.beginPath(); c.rect(p[0], p[1], p[2], p[3]); c.clip();
      c.strokeStyle = C['map-chevron']; c.lineWidth = 1.5; c.globalAlpha = 0.85;
      const hh = bt.w * s * 0.3, aw = Math.min(0.18 * s, hh);
      c.beginPath();
      for (let x = xl - spacing; x <= xr + spacing; x += spacing) {
        const xx = dir > 0 ? x + phase : xr - (x - xl) - phase;
        const px = v.X(xx), py = v.Y(bt.y);
        c.moveTo(px - dir * aw, py - hh); c.lineTo(px, py); c.lineTo(px - dir * aw, py + hh);
      }
      c.stroke();
      c.restore();
      if (pick !== null) {
        c.beginPath(); c.arc(v.X(pick), v.Y(bt.y), Math.max(3, 0.12 * s), 0, TAU);
        c.fillStyle = sb.ready ? C.ok : C.panel; c.fill();
        c.strokeStyle = sb.ready ? C.ok : C.idle; c.lineWidth = 1.5; c.stroke();
      }
      if (!opts.noText) {
        const bits = ['Line ' + bt.line];
        if (sb.status) bits.push(str(sb.status));
        if (isNum(sb.placed)) bits.push('placed ' + sb.placed);
        if (isNum(sb.orders_pending)) bits.push('pending ' + sb.orders_pending);
        label(c, bits[0], p[0], bt.y >= 0 ? p[1] - 9 : p[1] + p[3] + 9, { align: 'left', size: 11, weight: '700' });
        c.font = '700 11px ' + FONT;
        const off = c.measureText(bits[0]).width + 10;
        if (bits.length > 1) label(c, bits.slice(1).join('  |  '), p[0] + off, bt.y >= 0 ? p[1] - 9 : p[1] + p[3] + 9, { align: 'left', size: 10, weight: '500', color: C.muted });
      }
    });

    // pallets: crates on the current pallet (on_pallet; full pallets ship), else forklifts[].delivered
    const forks = arr(S && S.forklifts);
    L.pallets.forEach((pl) => {
      const fk = forks.find((x) => x && str(x.name) === pl.name);
      const n = fk && isNum(fk.on_pallet) ? fk.on_pallet : (fk && isNum(fk.delivered) ? fk.delivered : null);
      const p = wrect(v, { x0: pl.x - pl.w / 2, x1: pl.x + pl.w / 2, y0: pl.y - pl.h / 2, y1: pl.y + pl.h / 2 });
      if (n !== null && n > 0) {
        const cols = 3, rows = 2, gap = 2;
        const cw = (p[2] - gap * (cols + 1)) / cols, ch = (p[3] - gap * (rows + 1)) / rows;
        const shown = Math.min(n, cols * rows);
        for (let i = 0; i < shown; i++) {
          const cx = p[0] + gap + (i % cols) * (cw + gap), cy = p[1] + gap + Math.floor(i / cols) * (ch + gap);
          c.fillStyle = C.crate; c.fillRect(cx, cy, cw, ch);
          c.strokeStyle = C['crate-edge']; c.lineWidth = 1; c.strokeRect(cx + 0.5, cy + 0.5, cw - 1, ch - 1);
        }
      }
      if (!opts.noText) label(c, (n === null ? '-' : String(n)) + (n === 1 ? ' crate' : ' crates'), p[0] + p[2] / 2, pl.y >= 0 ? p[1] - 9 : p[1] + p[3] + 9, { size: 11, weight: '700' });
    });

    // carter trails
    if (!opts.noTrails) {
      const tNow = S && isNum(S.t) ? S.t : null;
      c.lineWidth = 2; c.lineCap = 'round';
      Object.keys(M.trails).forEach((name) => {
        const tr = M.trails[name];
        if (tr.length < 2) return;
        const tl = tNow !== null ? tNow : tr[tr.length - 1].t;
        for (let i = 1; i < tr.length; i++) {
          const age = tl - tr[i].t;
          const a = clamp(0.4 * (1 - age / TRAIL_S), 0.03, 0.4);
          c.strokeStyle = 'rgba(' + C.trail + ',' + a.toFixed(3) + ')';
          c.beginPath(); c.moveTo(v.X(tr[i - 1].x), v.Y(tr[i - 1].y)); c.lineTo(v.X(tr[i].x), v.Y(tr[i].y)); c.stroke();
        }
      });
      c.lineCap = 'butt';
    }

    if (!S) return;

    // packages
    const cl = L.crate.l * s, cwid = L.crate.w * s;
    arr(S.packages).forEach((pk) => {
      if (!pk || !isNum(pk.x) || !isNum(pk.y)) return;
      const id = str(pk.id);
      const q = interp(pk, P && P.packages, id, f);
      const px = v.X(q.x), py = v.Y(q.y);
      c.save(); c.translate(px, py); c.rotate(-q.yaw);
      const w = Math.max(4, cl), h = Math.max(3, cwid);
      c.fillStyle = C.crate; c.fillRect(-w / 2, -h / 2, w, h);
      c.strokeStyle = C['crate-edge']; c.lineWidth = 1; c.strokeRect(-w / 2, -h / 2, w, h);
      c.restore();
      if (hits) hits.push({ kind: 'package', name: id, px, py, r: Math.max(5, cl / 2), ref: pk, prio: 0 });
    });

    // people / props
    arr(S.people).forEach((pe) => {
      if (!pe || !isNum(pe.x) || !isNum(pe.y)) return;
      const name = str(pe.name);
      const q = interp(pe, P && P.people, name, f);
      const prop = pe.kind === 'prop';
      const r = Math.max(4, (prop ? 0.32 : 0.25) * s);
      const px = v.X(q.x), py = v.Y(q.y);
      c.beginPath(); c.arc(px, py, r, 0, TAU);
      c.fillStyle = prop ? C.prop : C.person; c.fill();
      c.strokeStyle = C['map-halo']; c.lineWidth = 1.5; c.stroke();
      if (pe.blocked) { c.beginPath(); c.arc(px, py, r + 3, 0, TAU); c.strokeStyle = C['warn-strong']; c.lineWidth = 2; c.stroke(); }
      // heading tick
      const hl = r + Math.max(4, 0.2 * s);
      c.beginPath(); c.moveTo(px + Math.cos(q.yaw) * r * 0.4, py - Math.sin(q.yaw) * r * 0.4); c.lineTo(px + Math.cos(q.yaw) * hl, py - Math.sin(q.yaw) * hl);
      c.strokeStyle = prop ? C.prop : C.person; c.lineWidth = 2; c.stroke();
      if (!opts.noText && s >= 12) label(c, name, px, py + r + 8, { size: 9, weight: '500', color: C.muted });
      if (hits) hits.push({ kind: prop ? 'prop' : 'person', name, px, py, r: r + 2, ref: pe, prio: 2 });
    });

    // arms
    const sArms = arr(S.arms).length ? arr(S.arms) : L.arms;
    sArms.forEach((a) => {
      if (!a || !isNum(a.x) || !isNum(a.y)) return;
      const name = str(a.name);
      const px = v.X(a.x), py = v.Y(a.y);
      const r = Math.max(5, 0.3 * s);
      c.save(); c.setLineDash([2, 3]); c.beginPath(); c.arc(px, py, 0.85 * s, 0, TAU); c.strokeStyle = C.faint; c.lineWidth = 1; c.globalAlpha = 0.6; c.stroke(); c.restore();
      c.beginPath(); c.arc(px, py, r, 0, TAU);
      c.fillStyle = clsColor(armCls(a, estop)); c.fill();
      c.strokeStyle = C['map-label']; c.lineWidth = 1.5; c.stroke();
      if (a.vacuum) { c.beginPath(); c.arc(px, py, r * 0.4, 0, TAU); c.fillStyle = C['map-label']; c.fill(); }
      if (!opts.noText) label(c, name, px, py - r - 8, { size: 11 });
      if (hits) hits.push({ kind: 'arm', name, px, py, r: r + 2, ref: a, prio: 3 });
    });

    // forklifts
    forks.forEach((fk) => {
      if (!fk || !isNum(fk.x) || !isNum(fk.y)) return;
      const name = str(fk.name);
      const q = interp(fk, P && P.forklifts, name, f);
      const px = v.X(q.x), py = v.Y(q.y);
      const bl = 1.5 * s, bw = 0.9 * s, fl = 1.0 * s;
      c.save(); c.translate(px, py); c.rotate(-q.yaw);
      // forks (front = +x in local frame)
      c.fillStyle = C['map-label'];
      const tine = Math.max(1.5, 0.1 * s);
      c.fillRect(bl / 2, -0.22 * s - tine / 2, fl, tine);
      c.fillRect(bl / 2, 0.22 * s - tine / 2, fl, tine);
      // body
      rrect(c, -bl / 2, -bw / 2, bl, bw, Math.max(2, 0.12 * s));
      c.fillStyle = clsColor(forkCls(fk, estop)); c.fill();
      c.strokeStyle = C['map-label']; c.lineWidth = 1.5; c.stroke();
      // mast
      c.fillStyle = C['map-label']; c.fillRect(bl / 2 - Math.max(2, 0.1 * s), -bw / 2, Math.max(2, 0.1 * s), bw);
      if (fk.cargo) {
        const cs = 0.45 * s;
        c.fillStyle = C.crate; c.fillRect(bl / 2 + fl * 0.5 - cs / 2, -cs / 2, cs, cs);
        c.strokeStyle = C['crate-edge']; c.lineWidth = 1; c.strokeRect(bl / 2 + fl * 0.5 - cs / 2, -cs / 2, cs, cs);
      }
      c.restore();
      if (!opts.noText) {
        const ly = py - Math.max(bl, bw) / 2 - 12;
        label(c, name, px - 4, ly, { size: 11 });
        // lift height bar (0..2 m)
        const lift = clamp(num(fk.lift, 0) / 2.0, 0, 1);
        const bx = px + 10, bh = 14, by = ly - bh / 2;
        c.fillStyle = C['map-halo']; c.fillRect(bx - 1, by - 1, 6, bh + 2);
        c.strokeStyle = C['map-label']; c.lineWidth = 1; c.strokeRect(bx + 0.5, by + 0.5, 4, bh - 1);
        c.fillStyle = C.info; c.fillRect(bx + 1, by + bh - 1 - (bh - 2) * lift, 3, (bh - 2) * lift);
      }
      if (hits) hits.push({ kind: 'forklift', name, px, py, r: Math.max(8, 0.9 * s), ref: fk, prio: 3 });
    });

    // carters
    arr(S.carters).forEach((ct) => {
      if (!ct || !isNum(ct.x) || !isNum(ct.y)) return;
      const name = str(ct.name);
      const q = interp(ct, P && P.carters, name, f);
      const px = v.X(q.x), py = v.Y(q.y);
      const L0 = Math.max(9, 0.9 * s), W0 = Math.max(7, 0.75 * s);
      c.save(); c.translate(px, py); c.rotate(-q.yaw);
      rrect(c, -L0 / 2, -W0 / 2, L0, W0, Math.max(2, 0.15 * s));
      c.fillStyle = clsColor(carterCls(ct, estop)); c.fill();
      c.strokeStyle = C['map-label']; c.lineWidth = 1.5; c.stroke();
      // front marker
      c.beginPath(); c.moveTo(L0 / 2 - 2, 0); c.lineTo(L0 / 2 - Math.max(5, 0.22 * s), -W0 * 0.28); c.lineTo(L0 / 2 - Math.max(5, 0.22 * s), W0 * 0.28); c.closePath();
      c.fillStyle = C['map-halo']; c.fill();
      if (ct.cargo) {
        const cs = Math.max(4, 0.4 * s);
        c.fillStyle = C.crate; c.fillRect(-cs / 2 - L0 * 0.08, -cs / 2, cs, cs);
        c.strokeStyle = C['crate-edge']; c.lineWidth = 1; c.strokeRect(-cs / 2 - L0 * 0.08, -cs / 2, cs, cs);
      }
      c.restore();
      if (!opts.noText) label(c, name, px, py - Math.max(L0, W0) / 2 - 9, { size: 11, weight: '700' });
      if (hits) hits.push({ kind: 'carter', name, px, py, r: Math.max(8, 0.55 * s), ref: ct, prio: 4 });
    });

    // selection + hover rings
    const ring = (name, color, dash, w) => {
      const h = (hits || mapHits).find((x) => x.name === name && x.kind !== 'package');
      if (!h) return;
      c.save(); c.setLineDash(dash); c.strokeStyle = color; c.lineWidth = w;
      c.beginPath(); c.arc(h.px, h.py, h.r + 6, 0, TAU); c.stroke(); c.restore();
    };
    if (hits) {
      if (M.selected) ring(M.selected, C.info, [5, 3], 2);
      if (M.hover && M.hover.name !== M.selected) ring(M.hover.name, C.muted, [], 1);
    }
  }

  function drawOverlay(c, v) {
    const S = M.state;
    const tags = [];
    if (!S) tags.push(['No data on ' + TOPICS.state.name, C.muted]);
    else {
      if (S.estop === true) tags.push(['E-STOP', C.fault]);
      if (S.paused === true) tags.push(['SIM PAUSED', C.warn]);
      if (now() - M.recvAt > STALE_MS) tags.push(['STATE STALE', C.warn]);
    }
    let x = v.W - 10;
    tags.forEach(([t, col]) => {
      c.font = '700 12px ' + FONT;
      const w = c.measureText(t).width + 16;
      c.fillStyle = C.panel; c.fillRect(x - w, 8, w, 22);
      c.strokeStyle = col; c.lineWidth = 1.5; c.strokeRect(x - w + 0.5, 8.5, w - 1, 21);
      c.fillStyle = col; c.textAlign = 'center'; c.textBaseline = 'middle'; c.fillText(t, x - w / 2, 19.5);
      x -= w + 6;
    });
  }

  function frame() {
    try {
      resizeMap();
      const L = curLayout();
      const v = makeView(viewBounds(L), cssW, cssH, 18);
      if (!mapView || v.s !== mapView.s || v.ox !== mapView.ox || v.oy !== mapView.oy || v.b.x0 !== mapView.b.x0 || v.b.y1 !== mapView.b.y1) mapStaticDirty = true;
      mapView = v;
      if (mapStaticDirty) {
        sctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        drawStatic(sctx, v, L);
        mapStaticDirty = false;
      }
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.drawImage(staticCanvas, 0, 0);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      const dtMsg = M.recvAt - M.prevRecvAt;
      const f = M.prevRecvAt && dtMsg > 0 && dtMsg < 1500 ? clamp((now() - M.recvAt) / dtMsg, 0, 1) : 1;
      const hits = [];
      drawDynamic(ctx, v, L, M.state, { hits, f });
      mapHits = hits;
      drawOverlay(ctx, v);
      if (mouse) updateHover();
    } catch (e) {
      console.warn('[dashboard] map frame:', e);
    }
    requestAnimationFrame(frame);
  }

  function hitAt(px, py) {
    let best = null, bd = Infinity;
    mapHits.forEach((h) => {
      const d = Math.hypot(h.px - px, h.py - py);
      const lim = Math.max(h.r, 10);
      if (d <= lim) {
        const score = d - h.prio * 3; // prefer robots over crates under them
        if (score < bd) { bd = score; best = h; }
      }
    });
    return best;
  }
  function ttRow(k, v) { return '<div class="tt-row"><span>' + k + '</span><span>' + esc(v) + '</span></div>'; }
  function esc(s) { return str(s).replace(/[&<>"]/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[ch])); }
  function updateHover() {
    const h = hitAt(mouse.x, mouse.y);
    M.hover = h;
    canvas.classList.toggle('is-pointer', !!h && h.kind !== 'package');
    if (!h) { tooltip.hidden = true; return; }
    const e = h.ref || {};
    const sp = speedOf(e);
    let html = '<b>' + esc(h.name) + '</b>';
    const pos = isNum(e.x) && isNum(e.y) ? e.x.toFixed(2) + ', ' + e.y.toFixed(2) : '-';
    if (h.kind === 'carter') {
      html += ttRow('Status', str(e.status, '-')) + ttRow('Speed', isNum(sp) ? Math.abs(sp).toFixed(2) + ' m/s' : '-') + ttRow('Cargo', e.cargo ? 'crate' : 'none') + ttRow('Job', e.job ? str(e.job) : '-');
      if (e.blocked_by_person) html += ttRow('Blocked', 'person ahead');
      if (e.parked) html += ttRow('Parked', 'yes');
    } else if (h.kind === 'forklift') {
      html += ttRow('Status', str(e.status, '-')) + ttRow('Speed', isNum(sp) ? Math.abs(sp).toFixed(2) + ' m/s' : '-') + ttRow('Lift', fx(num(e.lift), 2) + ' m') + ttRow('Cargo', e.cargo ? 'crate' : 'none') + ttRow('Delivered', isNum(e.delivered) ? String(e.delivered) : '-');
    } else if (h.kind === 'arm') {
      html += ttRow('Status', str(e.status, '-')) + ttRow('Line', str(e.line, '-')) + ttRow('Vacuum', e.vacuum ? 'on' : 'off');
    } else if (h.kind === 'person' || h.kind === 'prop') {
      html += ttRow('Kind', h.kind) + ttRow('Speed', isNum(sp) ? Math.abs(sp).toFixed(2) + ' m/s' : '-') + ttRow('Blocked', e.blocked ? 'yes' : 'no');
    } else if (h.kind === 'package') {
      html += ttRow('Height', fx(num(e.z), 2) + ' m');
    }
    html += ttRow('Position', pos);
    tooltip.innerHTML = html;
    tooltip.hidden = false;
    const tw = tooltip.offsetWidth, th = tooltip.offsetHeight;
    let x = mouse.x + 14, y = mouse.y + 14;
    if (x + tw > cssW - 4) x = mouse.x - tw - 14;
    if (y + th > cssH - 4) y = mouse.y - th - 14;
    tooltip.style.left = Math.max(4, x) + 'px';
    tooltip.style.top = Math.max(4, y) + 'px';
  }
  canvas.addEventListener('mousemove', (e) => {
    const r = canvas.getBoundingClientRect();
    mouse = { x: e.clientX - r.left, y: e.clientY - r.top };
  });
  canvas.addEventListener('mouseleave', () => { mouse = null; M.hover = null; tooltip.hidden = true; canvas.classList.remove('is-pointer'); });
  canvas.addEventListener('click', (e) => {
    const r = canvas.getBoundingClientRect();
    const h = hitAt(e.clientX - r.left, e.clientY - r.top);
    if (h && h.kind !== 'package') selectTarget(h.name, true);
  });

  // map extent toggle
  const segBtns = Array.from(document.querySelectorAll('.seg-btn[data-fit]'));
  segBtns.forEach((b) => b.addEventListener('click', () => {
    if (b.disabled) return;
    M.fit = b.dataset.fit;
    try { localStorage.setItem(LS_FIT, M.fit); } catch (e) { /* ignore */ }
    mapStaticDirty = true;
    updateFitButtons();
  }));
  function updateFitButtons() {
    const hasB = !!curLayout().building;
    segBtns.forEach((b) => {
      if (b.dataset.fit === 'building') { b.disabled = !hasB; b.title = hasB ? 'Show the whole building' : 'No building outline in the layout'; }
      b.classList.toggle('is-on', b.dataset.fit === (hasB ? M.fit : 'zone'));
    });
  }
  function updateMapNote() {
    const el = $('mapNote');
    if (M.layoutSrc === 'topic') el.textContent = MOCK ? 'layout: mock' : 'layout: ' + TOPICS.layout.name;
    else if (M.layoutSrc === 'cache') el.textContent = 'layout: cached copy (waiting for ' + TOPICS.layout.name + ')';
    else if (M.state) el.textContent = 'layout: derived from state (no ' + TOPICS.layout.name + ' yet)';
    else el.textContent = 'waiting for layout';
  }

  // ------------------------------------------------------------------ rosbridge transport
  function RosLink(url) {
    this.url = url;
    this.ros = null;
    this.pubs = {};
    this.timer = null;
    this.attempt = 0;
  }
  RosLink.prototype.connect = function () {
    clearTimeout(this.timer);
    if (typeof window.ROSLIB === 'undefined') { setConn('disconnected', 'roslib failed to load (vendor/roslib.min.js)'); return; }
    this.attempt++;
    setConn('connecting', 'Connecting to ' + this.url + (this.attempt > 1 ? ' (attempt ' + this.attempt + ')' : ''));
    let ros;
    try { ros = new window.ROSLIB.Ros({ url: this.url }); } catch (e) { console.warn('[dashboard] ROSLIB.Ros:', e); this.retry(); return; }
    this.ros = ros;
    this.pubs = {};
    ros.on('connection', () => {
      if (this.ros !== ros) return;
      this.attempt = 0;
      setConn('connected', 'Connected  ' + this.url);
      this.subscribeAll(ros);
    });
    ros.on('error', () => { /* a close event follows */ });
    ros.on('close', () => {
      if (this.ros !== ros) return;
      this.ros = null;
      this.retry();
    });
  };
  RosLink.prototype.retry = function () {
    setConn('disconnected', 'Disconnected from ' + this.url + ', retrying every ' + RECONNECT_MS / 1000 + ' s');
    clearTimeout(this.timer);
    this.timer = setTimeout(() => this.connect(), RECONNECT_MS);
  };
  RosLink.prototype.subscribeAll = function (ros) {
    M.evBackfilled = false;
    Object.keys(TOPICS).forEach((key) => {
      const T = TOPICS[key];
      if (T.dir === 'pub') {
        // advertise up front: with ROS 2 the first message after a lazy advertise can be lost to discovery
        const tp = this.pubs[key] = new window.ROSLIB.Topic({ ros, name: T.name, messageType: T.type });
        try { tp.advertise(); } catch (e) { console.warn('[dashboard] advertise ' + T.name + ':', e); }
        return;
      }
      if (T.dir !== 'sub') return;
      const opts = { ros, name: T.name, messageType: T.type };
      if (T.throttle_rate) opts.throttle_rate = T.throttle_rate;
      if (T.queue_length) opts.queue_length = T.queue_length;
      const topic = new window.ROSLIB.Topic(opts);
      topic.subscribe((msg) => { try { onMessage(key, msg); } catch (e) { console.warn('[dashboard] ' + key + ':', e); } });
    });
  };
  RosLink.prototype.publish = function (key, data) {
    if (!this.ros) return false;
    const T = TOPICS[key];
    let tp = this.pubs[key];
    if (!tp) { tp = this.pubs[key] = new window.ROSLIB.Topic({ ros: this.ros, name: T.name, messageType: T.type }); }
    try { tp.publish(new window.ROSLIB.Message({ data })); return true; } catch (e) { toast('fault', 'Publish failed on ' + T.name + ': ' + e.message); return false; }
  };
  RosLink.prototype.call = function (key, req) {
    const ros = this.ros;
    const S = SERVICES[key];
    return new Promise((resolve, reject) => {
      if (!ros) { reject(new Error('not connected')); return; }
      const srv = new window.ROSLIB.Service({ ros, name: S.name, serviceType: S.type });
      const to = setTimeout(() => reject(new Error('no reply from ' + S.name + ' within 5 s')), 5000);
      srv.callService(new window.ROSLIB.ServiceRequest(req), (res) => { clearTimeout(to); resolve(res); }, (err) => { clearTimeout(to); reject(new Error(str(err, 'service error'))); });
    });
  };

  // ------------------------------------------------------------------ mock simulation (?mock=1)
  function MockSim() {
    const Lraw = {
      zone: { x0: -13, x1: 13, y0: -8, y1: 8 },
      building: { x0: -21, x1: 19, y0: -12, y1: 12 },
      belts: [{ line: 'A', x0: -11.5, x1: -5.5, y: 5, w: 0.6 }, { line: 'B', x0: -11.5, x1: -5.5, y: -5, w: 0.6 }],
      docks: [{ line: 'A', x: -4.0, y: 3.3, w: 1.4, h: 1.2 }, { line: 'B', x: -4.0, y: -3.3, w: 1.4, h: 1.2 }],
      forklift_strips: [{ name: 'F1', x0: 9.2, x1: 12.8, y0: 2.6, y1: 6.8 }, { name: 'F2', x0: 9.2, x1: 12.8, y0: -6.8, y1: -2.6 }],
      pallets: [{ name: 'F1', x: 11.8, y: 6.1, w: 1.2, h: 1.0 }, { name: 'F2', x: 11.8, y: -6.1, w: 1.2, h: 1.0 }],
      arms: [{ name: 'ur5', x: -5.0, y: 4.4 }, { name: 'franka', x: -5.0, y: -4.4 }],
      statics: [[-3, 7, 7.0, 7.8], [-3, 7, -7.8, -7.0], [-12.6, -11.8, -2.5, 2.5], [4.8, 5.2, -0.2, 0.2], [-9.4, -8.8, -0.9, -0.3], [7.6, 8.0, 6.6, 7.0], [7.6, 8.0, -7.0, -6.6], [-12.8, -12.0, 6.0, 7.6]],
      parking: { C1: [-1.5, 1.2], C2: [-1.5, -1.2], C3: [-1.5, 0] },
      crate: { l: 0.4, w: 0.3, h: 0.3 },
    };
    const sim = { t: 0, paused: false, estop: false, yields: 0, follow: 'overview', auto: true, period: 14, nextOrderT: 3, orderSeq: 0, crateSeq: { A: 0, B: 0 }, lineToggle: 0 };
    const orders = [];
    let warm = true;
    const events = [];
    const ev = (actor, text) => {
      const line = '[' + sim.t.toFixed(1).padStart(7, ' ') + 's] ' + actor + ': ' + text;
      if (warm) events.push(line); else onMessage('events', { data: line });
    };
    const mir = (pts, sgn) => pts.map((p) => [p[0], p[1] * sgn]);
    const lines = {};
    ['A', 'B'].forEach((ln, i) => {
      const sg = i === 0 ? 1 : -1;
      lines[ln] = {
        line: ln, sg, belt: Lraw.belts[i], pick_x: -5.9, crates: [], placed: 0,
        dock: [-4.0, 3.3 * sg], handoff: [8.4, 4.0 * sg],
        out: mir([[-1.0, 3.3], [6.0, 3.9], [8.4, 4.0]], sg),
        back: mir([[6.0, 2.2], [-1.0, 2.2], [-3.0, 2.6], [-4.0, 3.3]], sg),
        toDock: mir([[-3.0, 2.6], [-4.0, 3.3]], sg),
        toPark: mir([[-3.0, 2.4], [-1.5, 1.2]], sg),
        arm: { name: i === 0 ? 'ur5' : 'franka', x: -5.0, y: 4.4 * sg, phase: 'idle', timer: 0, vacuum: false, crate: null, status: 'waiting for crate' },
        fork: { name: i === 0 ? 'F1' : 'F2', home: [10.5, 4.0 * sg, Math.PI], x: 10.5, y: 4.0 * sg, yaw: Math.PI, lift: 0.05, cargo: false, delivered: 0, phase: 'idle', timer: 0, crate: null, status: 'idle', v: 0 },
      };
    });
    const carters = [
      { name: 'C1', line: 'A', park: [-1.5, 1.2] },
      { name: 'C2', line: 'B', park: [-1.5, -1.2] },
      { name: 'C3', line: null, park: [-1.5, 0] },
    ].map((c) => Object.assign(c, { x: c.park[0], y: c.park[1], yaw: 0, v: 0, phase: 'parked', wp: [], timer: 0, cargo: false, crate: null, blocked: false, status: 'parked', job: null, idleT: 0, yieldTo: null }));
    const people = [
      { name: 'person0', kind: 'person', pts: [[0, 6.2], [3.5, 6.2], [3.5, 2.9], [0, 2.9]], loop: true, speed: 1.0, d: 0 },
      { name: 'person1', kind: 'person', pts: [[2, -6.5], [2, -1.6]], loop: false, speed: 0.9, d: 2 },
      { name: 'person2', kind: 'person', pts: [[6.5, 6.0], [7.0, -6.0]], loop: false, speed: 0.8, d: 5 },
      { name: 'pushcart', kind: 'prop', pts: [[-10, -1.5], [-7, -1.5], [-7, 1.5], [-10, 1.5]], loop: true, speed: 0.5, d: 0 },
      { name: 'dolly', kind: 'prop', pts: [[8.0, 0.6], [12.0, 0.6]], loop: false, speed: 0.6, d: 1 },
    ].map((p) => Object.assign(p, { x: 0, y: 0, yaw: 0, v: 0, blocked: false }));

    function pathPos(pts, d, loop) {
      const segs = [];
      let total = 0;
      const n = loop ? pts.length : pts.length - 1;
      for (let i = 0; i < n; i++) { const a = pts[i], b = pts[(i + 1) % pts.length]; const l = Math.hypot(b[0] - a[0], b[1] - a[1]); segs.push([a, b, l]); total += l; }
      let dd = loop ? ((d % total) + total) % total : ((d % (2 * total)) + 2 * total) % (2 * total);
      let rev = false;
      if (!loop && dd > total) { dd = 2 * total - dd; rev = true; }
      for (const [a, b, l] of segs) {
        if (dd <= l || l === 0) {
          const k = l ? dd / l : 0;
          const yaw = Math.atan2(b[1] - a[1], b[0] - a[0]) + (rev ? Math.PI : 0);
          return { x: a[0] + (b[0] - a[0]) * k, y: a[1] + (b[1] - a[1]) * k, yaw };
        }
        dd -= l;
      }
      const last = segs[segs.length - 1];
      return { x: last[1][0], y: last[1][1], yaw: 0 };
    }
    function newOrder(line) {
      sim.orderSeq++;
      const o = { id: 'ORD-' + String(sim.orderSeq).padStart(4, '0'), line, state: 'queued', created_t: Math.round(sim.t * 10) / 10, done_t: null };
      orders.push(o);
      if (orders.length > 60) orders.splice(0, orders.length - 60);
      ev('dispatch', o.id + ' created for line ' + line);
      return o;
    }
    const lineHasWork = (L) => L.crates.length > 0 || orders.some((o) => o.line === L.line && o.state === 'queued') || L.arm.crate;

    function driveTo(c, dt) {
      if (!c.wp.length) { c.v = 0; return true; }
      const [tx, ty] = c.wp[0];
      const dx = tx - c.x, dy = ty - c.y, dist = Math.hypot(dx, dy);
      if (dist < 0.12) { c.wp.shift(); if (!c.wp.length) { c.v = 0; return true; } return false; }
      const head = Math.atan2(dy, dx);
      const err = wrapAng(head - c.yaw);
      c.yaw = wrapAng(c.yaw + clamp(err, -2.2 * dt, 2.2 * dt));
      // yield to people in front
      let blocker = null;
      const reach = c.blocked ? 2.0 : 1.6, cone = c.blocked ? 1.3 : 1.0; // hysteresis
      people.forEach((p) => {
        const px = p.x - c.x, py = p.y - c.y, d = Math.hypot(px, py);
        if (d < reach) {
          const rel = Math.abs(wrapAng(Math.atan2(py, px) - c.yaw));
          if (rel < cone) blocker = p;
        }
      });
      if (blocker) c.clearT = 0;
      else if (c.blocked) c.clearT = (c.clearT || 0) + dt;
      if (blocker || (c.blocked && c.clearT < 1.0)) {
        if (!c.blocked) { sim.yields++; ev(c.name, 'yielding to ' + blocker.name); }
        c.blocked = true; if (blocker) c.yieldTo = blocker.name; c.v = 0; return false;
      }
      if (c.blocked) ev(c.name, 'path clear, resuming');
      c.blocked = false; c.yieldTo = null;
      const vmax = 1.1;
      const last = c.wp.length === 1;
      let sp = vmax * Math.max(0, Math.cos(err));
      if (last) sp = Math.min(sp, dist * 1.4 + 0.08);
      c.v = sp;
      c.x += Math.cos(c.yaw) * sp * dt;
      c.y += Math.sin(c.yaw) * sp * dt;
      return false;
    }

    function stepCarter(c, dt) {
      const L = c.line ? lines[c.line] : null;
      c.timer += dt;
      if (c.phase === 'parked') {
        c.v = 0;
        if (L && lineHasWork(L)) { c.phase = 'to_dock'; c.wp = L.toDock.slice(); c.job = L.line; c.timer = 0; ev('dispatch', c.name + ' assigned to line ' + L.line); }
        else if (!L && c.timer > 28) { c.phase = 'roam'; c.wp = [[0.5, 0], [2.5, 0.9], [6.0, 0.9], [6.0, -0.9], [2.5, -0.9], [0.5, 0], c.park.slice()]; c.timer = 0; ev('dispatch', c.name + ' patrol started'); }
        c.status = L ? 'parked' : 'parked (standby)';
        return;
      }
      if (c.phase === 'roam') {
        c.status = c.blocked ? 'yielding to ' + c.yieldTo : 'patrolling';
        if (driveTo(c, dt)) { c.phase = 'parked'; c.timer = 0; c.yaw = 0; ev(c.name, 'patrol done, parked'); }
        return;
      }
      if (c.phase === 'to_dock') {
        c.status = c.blocked ? 'yielding to ' + c.yieldTo : 'heading to dock ' + L.line;
        if (driveTo(c, dt)) { c.phase = 'at_dock'; c.timer = 0; ev(c.name, 'at dock ' + L.line); }
        return;
      }
      if (c.phase === 'at_dock') {
        c.v = 0;
        // turn to face east while waiting
        c.yaw = wrapAng(c.yaw + clamp(wrapAng(0 - c.yaw), -2 * dt, 2 * dt));
        c.status = c.cargo ? 'loaded' : 'waiting for crate';
        if (c.cargo && c.timer > 0.6) { c.phase = 'to_handoff'; c.wp = L.out.slice(); ev(c.name, 'carrying ' + c.crate.id + ' to ' + L.fork.name); return; }
        if (!c.cargo && !lineHasWork(L) && c.timer > 10) { c.phase = 'to_park'; c.wp = L.toPark.slice(); c.job = null; ev(c.name, 'no work on line ' + L.line + ', going to park'); }
        return;
      }
      if (c.phase === 'to_handoff') {
        c.status = c.blocked ? 'yielding to ' + c.yieldTo : 'carrying to ' + L.fork.name;
        if (driveTo(c, dt)) { c.phase = 'handoff'; c.timer = 0; ev(c.name, 'at handoff, waiting for ' + L.fork.name); }
        return;
      }
      if (c.phase === 'handoff') {
        c.v = 0;
        c.status = 'handing over to ' + L.fork.name;
        if (!c.cargo && c.timer > 0.5) { c.phase = 'returning'; c.wp = L.back.slice(); }
        return;
      }
      if (c.phase === 'returning') {
        c.status = c.blocked ? 'yielding to ' + c.yieldTo : 'returning to dock ' + L.line;
        if (driveTo(c, dt)) { c.phase = 'at_dock'; c.timer = 0; }
        return;
      }
      if (c.phase === 'to_park') {
        c.status = c.blocked ? 'yielding to ' + c.yieldTo : 'going to park';
        if (driveTo(c, dt)) { c.phase = 'parked'; c.timer = 0; ev(c.name, 'parked'); }
      }
    }

    function stepLine(L, dt) {
      // release orders onto the belt
      if (L.crates.length < 2) {
        const o = orders.find((x) => x.line === L.line && x.state === 'queued');
        const spawnX = Math.min(L.belt.x0, L.belt.x1) + 0.3;
        if (o && !L.crates.some((c) => c.x < spawnX + 0.8)) {
          L.crateSeq = (L.crateSeq || 0) + 1;
          const crate = { id: 'Crate' + L.line + '_' + L.crateSeq, x: spawnX, order: o };
          o.state = 'released';
          L.crates.push(crate);
          ev('belt ' + L.line, 'released ' + crate.id + ' for ' + o.id);
        }
      }
      // move crates
      L.crates.sort((a, b) => b.x - a.x);
      let limit = L.pick_x;
      L.crates.forEach((c) => { c.x = Math.min(c.x + 0.32 * dt, limit); limit = c.x - 0.7; });
      const front = L.crates[0];
      const ready = !!front && front.x >= L.pick_x - 0.01;
      L.ready = ready;
      L.status = ready ? 'crate at pick' : (L.crates.length ? 'running' : 'idle');
      // arm
      const A = L.arm;
      const carter = carters.find((c) => c.line === L.line && c.phase === 'at_dock' && !c.cargo);
      A.timer += dt;
      if (A.phase === 'idle') {
        A.vacuum = false;
        A.status = ready ? (carter ? 'picking' : 'waiting for carter') : 'waiting for crate';
        if (ready && carter) { A.phase = 'picking'; A.timer = 0; A.crate = L.crates.shift(); A.target = carter; }
      } else if (A.phase === 'picking') {
        A.status = 'picking'; A.vacuum = A.timer > 1.0;
        if (A.timer > 2.0) { A.phase = 'placing'; A.timer = 0; }
      } else if (A.phase === 'placing') {
        A.status = 'placing'; A.vacuum = true;
        if (A.timer > 1.5) {
          const c = A.target;
          c.cargo = true; c.crate = A.crate; c.timer = 0;
          A.crate.order.state = 'on_carter';
          L.placed++;
          ev(A.name, 'placed ' + A.crate.id + ' on ' + c.name);
          A.crate = null; A.target = null; A.phase = 'idle'; A.timer = 0; A.vacuum = false;
        }
      }
      // forklift
      const F = L.fork;
      const sg = L.sg;
      const waiting = carters.find((c) => c.line === L.line && c.phase === 'handoff' && c.cargo);
      F.timer += dt;
      const seg = (from, to, dur, liftFrom, liftTo) => {
        const k = clamp(F.timer / dur, 0, 1);
        const e = k * k * (3 - 2 * k);
        const ox = F.x, oy = F.y;
        F.x = from[0] + (to[0] - from[0]) * e; F.y = from[1] + (to[1] - from[1]) * e;
        F.yaw = angLerp(from[2], to[2], e);
        F.lift = liftFrom + (liftTo - liftFrom) * e;
        F.v = dt > 0 ? Math.hypot(F.x - ox, F.y - oy) / dt : 0;
        return k >= 1;
      };
      const H = F.home, AP = [9.6, 4.0 * sg, Math.PI], RV = [11.8, 4.2 * sg, Math.PI / 2 * sg], ST = [11.8, 5.0 * sg, Math.PI / 2 * sg];
      const stackLift = Math.min(1.6, 0.2 + 0.15 * (F.delivered % 6));
      const next = (ph) => { F.phase = ph; F.timer = 0; };
      if (F.phase === 'idle') { F.v = 0; F.status = 'idle'; if (waiting) { next('approach'); ev(F.name, 'approaching ' + waiting.name); } }
      else if (F.phase === 'approach') { F.status = 'approaching carter'; if (seg(H, AP, 1.6, 0.05, 0.05)) next('pick'); }
      else if (F.phase === 'pick') {
        F.status = 'lifting crate'; F.v = 0;
        F.lift = 0.05 + 0.2 * clamp(F.timer / 1.0, 0, 1);
        if (F.timer > 0.3 && !F.cargo && waiting) { F.cargo = true; F.crate = waiting.crate; waiting.cargo = false; waiting.crate = null; waiting.timer = 0; }
        if (F.timer > 1.0) next('reverse');
      } else if (F.phase === 'reverse') { F.status = 'moving to pallet'; if (seg(AP, RV, 2.4, 0.25, 0.25)) next('stack'); }
      else if (F.phase === 'stack') { F.status = 'stacking, level ' + (F.delivered % 6 + 1); if (seg(RV, ST, 1.6, 0.25, stackLift)) next('drop'); }
      else if (F.phase === 'drop') {
        F.v = 0; F.status = 'placing on pallet';
        if (F.timer > 1.0) {
          F.cargo = false; F.delivered++;
          if (F.crate && F.crate.order) { F.crate.order.state = 'delivered'; F.crate.order.done_t = Math.round(sim.t * 10) / 10; }
          F.crate = null;
          ev(F.name, 'delivered total ' + F.delivered);
          next('return');
        }
      } else if (F.phase === 'return') { F.status = 'returning'; if (seg(ST, H, 2.0, stackLift, 0.05)) { next('idle'); F.x = H[0]; F.y = H[1]; F.yaw = H[2]; } }
    }

    function step(dt) {
      if (sim.paused) return;
      sim.t += dt;
      // people always move (they are not robots)
      people.forEach((p) => {
        p.d += p.speed * dt;
        const q = pathPos(p.pts, p.d, p.loop);
        p.x = q.x; p.y = q.y; p.yaw = q.yaw; p.v = p.speed;
        p.blocked = carters.some((c) => Math.hypot(c.x - p.x, c.y - p.y) < 1.3);
      });
      // WMS auto orders
      if (sim.auto && sim.t >= sim.nextOrderT) {
        newOrder(sim.lineToggle++ % 2 === 0 ? 'A' : 'B');
        sim.nextOrderT = sim.t + sim.period;
      }
      if (sim.estop) { carters.forEach((c) => { c.v = 0; c.status = 'E-STOP'; }); Object.values(lines).forEach((L) => { L.fork.v = 0; L.fork.status = 'E-STOP'; L.arm.status = 'halted (E-STOP)'; L.status = 'stopped (E-STOP)'; }); return; }
      carters.forEach((c) => stepCarter(c, dt));
      Object.values(lines).forEach((L) => stepLine(L, dt));
    }

    function packages() {
      const out = [];
      Object.values(lines).forEach((L) => {
        L.crates.forEach((c) => out.push({ id: c.id, x: c.x, y: L.belt.y, z: 0.85, yaw: 0 }));
        const A = L.arm;
        if (A.crate) {
          const px = L.pick_x, py = L.belt.y;
          let x = px, y = py, z = 0.85;
          if (A.phase === 'placing' && A.target) { const k = clamp(A.timer / 1.5, 0, 1); x = px + (A.target.x - px) * k; y = py + (A.target.y - py) * k; z = 1.1 - 0.5 * k; }
          else if (A.phase === 'picking') z = 0.85 + 0.25 * clamp((A.timer - 1.0) / 1.0, 0, 1);
          out.push({ id: A.crate.id, x, y, z, yaw: 0 });
        }
        const F = L.fork;
        if (F.cargo && F.crate) out.push({ id: F.crate.id, x: F.x + Math.cos(F.yaw) * 1.25, y: F.y + Math.sin(F.yaw) * 1.25, z: F.lift + 0.1, yaw: F.yaw });
      });
      carters.forEach((c) => { if (c.cargo && c.crate) out.push({ id: c.crate.id, x: c.x, y: c.y, z: 0.5, yaw: c.yaw }); });
      return out;
    }
    const r2 = (v) => Math.round(v * 100) / 100;
    function stateMsg() {
      const cs = carters.map((c) => ({ name: c.name, x: r2(c.x), y: r2(c.y), yaw: r2(c.yaw), v: r2(c.v), status: c.status, cargo: c.cargo, job: c.job, blocked_by_person: c.blocked, parked: c.phase === 'parked' }));
      let cc = Infinity, cp = Infinity;
      for (let i = 0; i < carters.length; i++) {
        for (let j = i + 1; j < carters.length; j++) cc = Math.min(cc, Math.hypot(carters[i].x - carters[j].x, carters[i].y - carters[j].y));
        people.forEach((p) => { if (p.kind === 'person') cp = Math.min(cp, Math.hypot(carters[i].x - p.x, carters[i].y - p.y)); });
      }
      const total = Object.values(lines).reduce((a, L) => a + L.fork.delivered, 0);
      return {
        t: r2(sim.t), rtf: r2(sim.paused ? 0 : 0.96 + 0.03 * Math.sin(sim.t / 7)), paused: sim.paused, estop: sim.estop, mode: sim.auto ? 'orders' : 'continuous',
        carters: cs,
        forklifts: Object.values(lines).map((L) => ({ name: L.fork.name, x: r2(L.fork.x), y: r2(L.fork.y), yaw: r2(L.fork.yaw), lift: r2(L.fork.lift), cargo: L.fork.cargo, delivered: L.fork.delivered, status: L.fork.status })),
        arms: Object.values(lines).map((L) => ({ name: L.arm.name, line: L.line, x: L.arm.x, y: L.arm.y, status: L.arm.status, vacuum: L.arm.vacuum })),
        belts: Object.values(lines).map((L) => ({ line: L.line, y: L.belt.y, x0: L.belt.x0, x1: L.belt.x1, pick_x: L.pick_x, ready: !!L.ready, placed: L.placed, orders_pending: orders.filter((o) => o.line === L.line && (o.state === 'queued' || o.state === 'released')).length, status: L.status || 'idle' })),
        people: people.map((p) => ({ name: p.name, kind: p.kind, x: r2(p.x), y: r2(p.y), yaw: r2(p.yaw), v: r2(p.v), blocked: p.blocked })),
        packages: packages().map((p) => ({ id: p.id, x: r2(p.x), y: r2(p.y), z: r2(p.z), yaw: r2(p.yaw) })),
        kpi: { delivered_total: total, per_hour: sim.t > 60 ? Math.round((total / (sim.t / 3600)) * 10) / 10 : 0, min_carter_sep: r2(cc), min_person_sep: r2(cp), person_yields: sim.yields },
      };
    }
    function wmsMsg() {
      const counts = { queued: 0, in_progress: 0, delivered: 0 };
      orders.forEach((o) => { if (o.state === 'queued') counts.queued++; else if (o.state === 'delivered') counts.delivered++; else counts.in_progress++; });
      return { auto: sim.auto, period_s: sim.period, orders: orders.slice(-40), counts, throughput_per_hour: sim.t > 60 ? Math.round((counts.delivered / (sim.t / 3600)) * 10) / 10 : 0 };
    }

    // camera frames: render the scene into an offscreen canvas and ship it as base64 jpeg
    const camCanvas = document.createElement('canvas');
    camCanvas.width = 640; camCanvas.height = 360;
    const cctx = camCanvas.getContext('2d');
    const Lnorm = normLayout(Lraw);
    function camFrame(which) {
      const S = M.state;
      if (!S) return null;
      let target = which === 'overview' ? 'overview' : sim.follow;
      let v;
      if (target === 'overview') v = makeView({ x0: -14, x1: 14, y0: -8.5, y1: 8.5 }, 640, 360, 4);
      else {
        const all = [].concat(arr(S.carters), arr(S.forklifts), arr(S.arms), arr(S.people));
        const e = all.find((x) => x && str(x.name) === target);
        if (!e) { target = target + ' (not found)'; v = makeView({ x0: -14, x1: 14, y0: -8.5, y1: 8.5 }, 640, 360, 4); }
        else v = makeView({ x0: e.x - 4.8, x1: e.x + 4.8, y0: e.y - 2.7, y1: e.y + 2.7 }, 640, 360, 0);
      }
      cctx.setTransform(1, 0, 0, 1, 0, 0);
      drawStatic(cctx, v, Lnorm, { noText: true });
      drawDynamic(cctx, v, Lnorm, S, { noText: true, noTrails: true });
      // camera look: slight tint, vignette band, OSD text
      cctx.fillStyle = 'rgba(10, 14, 20, 0.22)'; cctx.fillRect(0, 0, 640, 360);
      cctx.fillStyle = 'rgba(0, 0, 0, 0.55)'; cctx.fillRect(0, 0, 640, 24); cctx.fillRect(0, 336, 640, 24);
      cctx.font = '600 13px ' + FONT; cctx.fillStyle = '#e6e9ec'; cctx.textBaseline = 'middle'; cctx.textAlign = 'left';
      cctx.fillText((which === 'overview' ? 'CAM overview' : 'CAM follow -> ' + target), 10, 12);
      cctx.textAlign = 'right'; cctx.fillText('t ' + fmtClock(S.t), 630, 12);
      cctx.textAlign = 'left'; cctx.fillStyle = '#9aa4ae'; cctx.fillText('MOCK FRAME (generated client-side)', 10, 348);
      return camCanvas.toDataURL('image/jpeg', 0.7).split(',')[1];
    }

    // warm-up so the page opens on a populated scene
    for (let i = 0; i < 1400; i++) step(0.05);
    warm = false;

    this.start = function () {
      onMessage('layout', { data: JSON.stringify(Lraw) });
      events.slice(-120).forEach((l) => onMessage('events', { data: l }));
      onMessage('state', { data: JSON.stringify(stateMsg()) });
      onMessage('wms', { data: JSON.stringify(wmsMsg()) });
      let last = now();
      setInterval(() => { const t = now(); const dt = Math.min(0.25, (t - last) / 1000); last = t; const sub = Math.max(1, Math.ceil(dt / 0.05)); for (let i = 0; i < sub; i++) step(dt / sub); }, 50);
      setInterval(() => onMessage('state', { data: JSON.stringify(stateMsg()) }), 200);
      setInterval(() => onMessage('wms', { data: JSON.stringify(wmsMsg()) }), 1000);
      setInterval(() => {
        const a = camFrame('overview'); if (a) onMessage('camOverview', { format: 'jpeg', data: a });
        const b = camFrame('follow'); if (b) onMessage('camFollow', { format: 'jpeg', data: b });
      }, 250);
    };
    this.publish = function (key, data) {
      console.info('[mock] publish', TOPICS[key].name, data);
      const p = parseJson(data);
      if (key === 'followTarget') { sim.follow = data; ev('camera', 'follow target set to ' + data); }
      else if (key === 'newOrder' && p) { const q = clamp(Math.round(num(p.qty, 1)), 1, 20); const ln = str(p.line).toUpperCase() === 'B' ? 'B' : 'A'; for (let i = 0; i < q; i++) newOrder(ln); }
      else if (key === 'wmsConfig' && p) {
        if (typeof p.auto === 'boolean') { if (p.auto && !sim.auto) sim.nextOrderT = sim.t + 1; sim.auto = p.auto; }
        if (isNum(p.period_s) && p.period_s > 0) sim.period = p.period_s;
        ev('dispatch', 'WMS config auto=' + sim.auto + ' period=' + sim.period + ' s');
      }
      setTimeout(() => onMessage('wms', { data: JSON.stringify(wmsMsg()) }), 50);
      return true;
    };
    this.call = function (key, req) {
      console.info('[mock] call', SERVICES[key].name, JSON.stringify(req));
      return new Promise((resolve) => setTimeout(() => {
        let message = '';
        if (key === 'pause') { sim.paused = true; message = 'mock: simulation paused'; ev('dispatch', 'simulation paused'); }
        else if (key === 'resume') { sim.paused = false; message = 'mock: simulation resumed'; ev('dispatch', 'simulation resumed'); }
        else if (key === 'estop') { sim.estop = !!(req && req.data); message = 'mock: e-stop ' + (sim.estop ? 'engaged' : 'released'); ev('dispatch', sim.estop ? 'E-STOP engaged by operator' : 'E-STOP released by operator'); }
        onMessage('state', { data: JSON.stringify(stateMsg()) });
        resolve({ success: true, message });
      }, 150));
    };
  }

  // ------------------------------------------------------------------ boot
  function boot() {
    readColors();
    try { const f = localStorage.getItem(LS_FIT); if (f === 'zone' || f === 'building') M.fit = f; } catch (e) { /* ignore */ }
    const mq = window.matchMedia ? window.matchMedia('(prefers-color-scheme: dark)') : null;
    if (mq) {
      const onTheme = () => { readColors(); mapStaticDirty = true; };
      if (mq.addEventListener) mq.addEventListener('change', onTheme); else if (mq.addListener) mq.addListener(onTheme);
    }
    if (window.ResizeObserver) new ResizeObserver(() => { mapStaticDirty = true; }).observe(mapWrap);
    buildRosPanel();
    evPlaceholder();
    updateEventCount();
    updateFitButtons();
    renderHeader(); renderKpis(); renderControls(); renderCams(); renderStateAge();

    if (MOCK) {
      document.title = 'Warehouse Operations (mock)';
      setConn('mock', 'MOCK mode (simulated, no ROS)');
      bus = new MockSim();
      bus.start();
    } else {
      try {
        const cached = localStorage.getItem(LS_LAYOUT);
        if (cached) { const L = parseJson(cached); if (L) { M.layout = normLayout(L); M.layoutSrc = 'cache'; } }
      } catch (e) { /* ignore */ }
      bus = new RosLink(WS_URL);
      bus.connect();
    }
    updateFitButtons();
    updateMapNote();
    setInterval(() => { renderStateAge(); renderCams(); }, 500);
    setInterval(renderRosPanel, 1000);
    renderRosPanel();
    requestAnimationFrame(frame);
  }
  // read-only debug hook (used by tests): entity hit boxes in canvas CSS px, current model
  window.__warehouseDash = { hits: () => mapHits.map((h) => ({ kind: h.kind, name: h.name, x: h.px, y: h.py })), model: M, mock: MOCK };
  boot();
})();
