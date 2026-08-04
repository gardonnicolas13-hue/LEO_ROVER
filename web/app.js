/* =============================================================================
   LEO·OPS — Cockpit logic (app.js)
   -----------------------------------------------------------------------------
   Pilotage du LEO Rover (AgileX) via rosbridge + web_video_server.

     · WebSocket rosbridge .......... ws://<PC>:9090
     · Flux MJPEG (web_video_server)  http://<PC>:8080/stream?topic=/mission/image_annotated
     · Publie  /mission/command  (std_msgs/String, JSON)
     · Écoute  /mission/telemetry (std_msgs/String, JSON ~10 Hz)
     · Écoute  /mission/log       (std_msgs/String)
     · Écoute  /camera/imu/data_raw (sensor_msgs/Imu) — artificial horizon

   Actions JSON envoyées :
     set_mode {mode}, stop, reset, clear_map, park,
     set_params {hue_low,hue_high,v_min,minled}, manual {lin,ang}, set_view {mask}
   ========================================================================== */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);

  const T = (key, vars) => {
    let s = (window.I18N ? I18N.t(key) : key);
    if (vars) Object.keys(vars).forEach(k => { s = s.replace('{' + k + '}', vars[k]); });
    return s;
  };

  /* ===========================================================================
     CHAMP D'ÉTOILES (décor partagé landing / cockpit)
     ========================================================================= */
  function starfield(canvasId, opts) {
    const cv = document.getElementById(canvasId);
    if (!cv) return;
    opts = opts || {};
    const ctx = cv.getContext('2d');
    const DENSITY = opts.density || 0.00018;
    const SHOOT = opts.shooting !== false;
    let stars = [], shoots = [], w = 0, h = 0, raf = 0, visible = !document.hidden;

    function resize() {
      w = cv.width = cv.offsetWidth * devicePixelRatio;
      h = cv.height = cv.offsetHeight * devicePixelRatio;
      const n = Math.max(60, Math.floor(w * h * DENSITY / devicePixelRatio));
      stars = Array.from({ length: n }, () => ({
        x: Math.random() * w, y: Math.random() * h,
        z: Math.random() * 0.8 + 0.2, r: Math.random() * 1.3 + 0.2,
        tw: Math.random() * Math.PI * 2, sp: Math.random() * 0.02 + 0.004,
      }));
    }
    function spawnShoot() {
      if (!SHOOT || Math.random() > 0.012) return;
      shoots.push({ x: Math.random() * w, y: Math.random() * h * 0.5,
        vx: (Math.random() * 4 + 5) * devicePixelRatio,
        vy: (Math.random() * 2 + 1) * devicePixelRatio, life: 1 });
    }
    function frame() {
      raf = requestAnimationFrame(frame);
      if (!visible) return;
      ctx.clearRect(0, 0, w, h);
      for (const s of stars) {
        s.tw += s.sp;
        const a = 0.35 + 0.65 * (0.5 + 0.5 * Math.sin(s.tw));
        ctx.beginPath();
        ctx.fillStyle = 'rgba(' + (180 + Math.floor(s.z * 60)) + ',200,255,' + (a * s.z).toFixed(3) + ')';
        ctx.arc(s.x, s.y, s.r * s.z * devicePixelRatio, 0, 6.2832);
        ctx.fill();
      }
      spawnShoot();
      for (let i = shoots.length - 1; i >= 0; i--) {
        const sh = shoots[i];
        const grad = ctx.createLinearGradient(sh.x, sh.y, sh.x - sh.vx * 6, sh.y - sh.vy * 6);
        grad.addColorStop(0, 'rgba(103,232,249,' + sh.life + ')');
        grad.addColorStop(1, 'rgba(103,232,249,0)');
        ctx.strokeStyle = grad; ctx.lineWidth = 1.6 * devicePixelRatio;
        ctx.beginPath(); ctx.moveTo(sh.x, sh.y);
        ctx.lineTo(sh.x - sh.vx * 6, sh.y - sh.vy * 6); ctx.stroke();
        sh.x += sh.vx; sh.y += sh.vy; sh.life -= 0.02;
        if (sh.life <= 0 || sh.x > w || sh.y > h) shoots.splice(i, 1);
      }
    }
    resize(); cancelAnimationFrame(raf); frame();
    window.addEventListener('resize', () => { clearTimeout(cv._t); cv._t = setTimeout(resize, 150); });
    document.addEventListener('visibilitychange', () => { visible = !document.hidden; });
  }

  window.LEO = { starfield };

  /* ===========================================================================
     TUNNEL CLOUDFLARE — quand la page est servie depuis leo-rover-gardon.dev,
     rosbridge/vidéo passent par le MÊME hôte que la page (chemins /rosbridge
     et /stream), pas par des sous-domaines séparés. Cloudflare Access pose son
     cookie d'auth par hôte : un sous-domaine différent (ws./video.) ne le
     reçoit pas automatiquement, et un WebSocket ne peut pas suivre une
     redirection de login comme le ferait une page -> connexion bloquée en
     silence (statut bloqué sur OFFLINE). Même origine = même cookie garanti.
     ========================================================================= */
  const TUNNEL_ROOT = 'leo-rover-gardon.dev';
  const isTunnel = () => location.hostname === TUNNEL_ROOT || location.hostname.endsWith('.' + TUNNEL_ROOT);
  const wsURL = (host) => isTunnel() ? ('wss://' + location.host + '/rosbridge') : ('ws://' + host + ':9090');
  const videoOrigin = (host) => isTunnel() ? ('https://' + location.host) : ('http://' + host + ':8080');

  /* ===========================================================================
     INITIALISATION DU COCKPIT
     ========================================================================= */
  function initCockpit() {
    if (!$('videoStream')) return;
    if (window.lucide) lucide.createIcons();
    if (window.AudioMgr) AudioMgr.bindAll();

    if (isTunnel()) {
      const hostEl = $('host');
      hostEl.value = TUNNEL_ROOT;
      hostEl.readOnly = true;
      hostEl.title = 'Tunnel Cloudflare — domaine fixe';
      hostEl.classList.add('opacity-60', 'cursor-not-allowed');
    } else {
      const servedHost = (location.protocol.startsWith('http') && location.hostname)
        ? location.hostname : '';
      $('host').value = servedHost || localStorage.getItem('leo_host') || '10.0.0.10';
    }

    if (location.protocol === 'file:') {
      console.warn('[LEO] file:// mode — rosbridge and video inaccessible');
      const b = document.createElement('div');
      b.className = 'fixed inset-x-0 top-0 z-[60] bg-alert px-4 py-2 text-center text-sm font-semibold text-white shadow-glow-alert';
      b.innerHTML = T('ops_js_file_warn')
        + ' <span class="font-mono underline">http://&lt;IP-du-PC&gt;:8000/ops.html</span>';
      document.body.prepend(b);
      if ($('videoMsg')) $('videoMsg').textContent = T('ops_js_file_msg');
    }

    let ros = null, connected = false, reconnectTimer = null, wantConnected = false;
    let topics = {};
    let lastTelemetry = null;
    let _sysThrottleN = 0;   // counts telemetry packets; sysinfo updated every 5th (≈2Hz)

    /* ── Ring Buffer O(1) — pas de freeze à 10 000 pts ─────────────────── */
    class _RingBuffer {
      constructor(cap) { this._d = new Array(cap); this._h = 0; this._sz = 0; this._cap = cap; }
      push(v) {
        this._d[(this._h + this._sz) % this._cap] = v;
        if (this._sz < this._cap) this._sz++; else this._h = (this._h + 1) % this._cap;
      }
      get(i)       { return this._d[(this._h + i) % this._cap]; }
      get length() { return this._sz; }
      last()       { return this._sz > 0 ? this._d[(this._h + this._sz - 1) % this._cap] : null; }
      clear()      { this._h = 0; this._sz = 0; }
    }

    /* ── Outlier detection — log isolé, n'affecte jamais le rendu ──────── */
    const MAP_MAX_RANGE  = 500;  // m — au-delà = aberrant certain
    const _mapOutlierLog = [];   // entrées individuelles, capé à 200
    let   _mapOutlierN   = 0;    // compteur total (incluant hors-log)

    function _mapValidate(x, y, src) {
      if (!Number.isFinite(x) || !Number.isFinite(y)
          || Math.abs(x) > MAP_MAX_RANGE || Math.abs(y) > MAP_MAX_RANGE) {
        _mapOutlierN++;
        if (_mapOutlierLog.length < 200)
          _mapOutlierLog.push({ ts: Date.now(), x, y, src,
            err: (!Number.isFinite(x) || !Number.isFinite(y)) ? 'NaN/Inf' : 'overflow' });
        return false;
      }
      return true;
    }

    /* ── Bounds incrémentaux O(1) — recompute complet toutes les 5 s ───── */
    let _mapBounds    = { minX: -1, maxX: 1, minY: -1, maxY: 1 };
    let _mapBoundsAge = 0;

    function _mapExpandBounds(x, y) {
      if (x < _mapBounds.minX) _mapBounds.minX = x;
      else if (x > _mapBounds.maxX) _mapBounds.maxX = x;
      if (y < _mapBounds.minY) _mapBounds.minY = y;
      else if (y > _mapBounds.maxY) _mapBounds.maxY = y;
    }
    function _mapFullRecompute() {
      let b = { minX: -1, maxX: 1, minY: -1, maxY: 1 };
      const ex = (x, y) => {
        if (x < b.minX) b.minX = x; else if (x > b.maxX) b.maxX = x;
        if (y < b.minY) b.minY = y; else if (y > b.maxY) b.maxY = y;
      };
      ex(0, 0);
      for (let i = 0; i < globalTraj.length; i++) { const p = globalTraj.get(i); ex(p[0], p[1]); }
      globalBeacons.forEach(b2 => ex(b2.x, b2.y));
      globalMapMarkers.forEach(m => ex(m.x, m.y));
      _mapBounds = b;
    }

    /* ── Global trajectory ─────────────────────────────────────────────── */
    let globalTraj       = new _RingBuffer(10000);
    let globalBeacons    = [];
    let globalMapMarkers = [];   /* marqueurs permanents /map/markers — survivent aux resets */
    let gOffset          = { x: 0, y: 0 };
    let gOrientation     = 0;
    let gPrevPose        = { x: 0, y: 0 };
    let gPrevRawYaw      = null;
    let gPrevBeaconCount = 0;

    function toGlobal(lx, ly) {
      const c = Math.cos(gOrientation), s = Math.sin(gOrientation);
      return [gOffset.x + c * lx - s * ly, gOffset.y + s * lx + c * ly];
    }
    function resetGlobalHistory() {
      globalTraj.clear(); globalBeacons = [];
      gOffset = { x: 0, y: 0 }; gOrientation = 0;
      gPrevPose = { x: 0, y: 0 }; gPrevRawYaw = null; gPrevBeaconCount = 0;
      _mapBounds = { minX: -1, maxX: 1, minY: -1, maxY: 1 };
    }
    function clearAllMapData() {
      resetGlobalHistory();
      globalMapMarkers = [];
    }

    /* ── Console ─────────────────────────────────────────────────────────── */
    function logLine(text, cls) {
      const box = $('console'); if (!box) return;
      const p = document.createElement('p');
      p.className = cls || 'text-emerald-300';
      const ts = new Date().toLocaleTimeString('fr-FR', { hour12: false });
      p.innerHTML = '<span class="text-zinc-600">' + ts + '</span>  ' + text.replace(/</g, '&lt;');
      box.appendChild(p);
      while (box.children.length > 220) box.removeChild(box.firstChild);
      box.scrollTop = box.scrollHeight;
    }

    /* ── Status pill ─────────────────────────────────────────────────────── */
    function setStatus(state) {
      const dot = $('statusDot'), txt = $('statusText'), pill = $('statusPill');
      if (!pill) return;
      pill.className = 'flex items-center gap-2 rounded-full border border-white/10 px-3 py-1.5 text-xs font-medium';
      if (state === 'on') {
        dot.className = 'led-dot text-emerald-400';
        pill.classList.add('bg-emerald-500/10', 'text-emerald-300');
        txt.textContent = T('status_on');
      } else if (state === 'wait') {
        dot.className = 'led-dot text-amber-400 animate-pulse-glow';
        pill.classList.add('bg-amber-500/10', 'text-amber-300');
        txt.textContent = T('status_wait');
      } else {
        dot.className = 'led-dot text-alert';
        pill.classList.add('bg-alert/10', 'text-alert');
        txt.textContent = T('status_off');
      }
    }

    /* ── Rosbridge connect/disconnect ─────────────────────────────────────── */
    let currentGen = 0;
    let cmdDropWarned = false;
    let _stopPending  = false;

    function connect() {
      const host = $('host').value.trim() || '10.0.0.10';
      _currentHost = host;
      localStorage.setItem('leo_host', host);
      wantConnected = true;
      const lbl = $('connectLabel');
      if (lbl) lbl.textContent = T('btn_disconnect');
      startVideo(host);
      setStatus('wait');
      const url = wsURL(host);
      logLine(T('ops_js_conn', { url }), 'text-amber-300');

      const gen = ++currentGen;
      ros = new ROSLIB.Ros({ url });

      ros.on('connection', () => {
        if (gen !== currentGen) return;
        connected = true;
        cmdDropWarned = false;
        setStatus('on');
        logLine(T('ops_js_ok'), 'text-accent');
        setupTopics();
        startHeartbeat();
        if (_stopPending) {
          _stopPending = false;
          setTimeout(() => {
            if (connected && topics.command)
              topics.command.publish(new ROSLIB.Message({ data: JSON.stringify({ action: 'stop' }) }));
            logLine('// STOP exécuté à la reconnexion', 'text-alert');
          }, 200);
        }
        if (window.AudioMgr) AudioMgr.play('connect');
      });
      ros.on('error', (err) => {
        if (gen !== currentGen) return;
        console.error('[LEO] rosbridge ERROR:', err);
        setStatus('wait');
        if (window.AudioMgr) AudioMgr.play('alert');
      });
      ros.on('close', () => {
        if (gen !== currentGen) return;
        connected = false;
        stopHeartbeat();
        setStatus(wantConnected ? 'wait' : 'off');
        if (wantConnected) {
          logLine(T('ops_js_lost'), 'text-alert');
          clearTimeout(reconnectTimer);
          reconnectTimer = setTimeout(connect, 1000);
          if (window.AudioMgr) AudioMgr.play('disconnect');
        }
      });
    }

    /* ── Operator heartbeat ───────────────────────────────────────────────────
       Pings the backend twice per second. If the backend stops receiving these
       while in AUTO (closed tab, frozen page, dropped WS), it engages its own
       failsafe: force MANUAL + zero velocity. See HEARTBEAT_TIMEOUT backend-side. */
    const HEARTBEAT_MS = 200;   /* 200 ms — 5×/s, well within backend 1.5 s failsafe */
    let heartbeatTimer = null;
    function startHeartbeat() {
      clearInterval(heartbeatTimer);
      const ping = () => {
        if (connected && topics.command)
          topics.command.publish(new ROSLIB.Message({ data: JSON.stringify({ action: 'heartbeat' }) }));
      };
      ping();                                   // arm immediately
      heartbeatTimer = setInterval(ping, HEARTBEAT_MS);
    }
    function stopHeartbeat() {
      clearInterval(heartbeatTimer);
      heartbeatTimer = null;
    }

    function disconnect() {
      wantConnected = false;
      const lbl = $('connectLabel');
      if (lbl) lbl.textContent = T('btn_connect');
      clearTimeout(reconnectTimer);
      stopHeartbeat();
      if (ros) ros.close();
      setStatus('off'); stopVideo();
      /* Reset stream degradation state on disconnect */
      _streamDegraded = false; _degradeHits = 0; _recoverHits = 0;
      const badge = $('streamDegBadge');
      if (badge) badge.classList.add('hidden');
      logLine(T('ops_js_man'), 'text-zinc-500');
    }

    /* ── Flux vidéo MJPEG ─────────────────────────────────────────────────── */
    let videoRetryTimer = null;
    let _currentHost    = '';    /* set on connect(), used by stream degradation */

    function videoStreamURL(host, q, w) {
      let url = videoOrigin(host) + '/stream'
        + '?topic=/mission/image_annotated&type=mjpeg&quality=' + (q || 70);
      if (w) url += '&width=' + w;
      url += '&t=' + Date.now();
      return url;
    }
    function startVideo(host) {
      clearTimeout(videoRetryTimer);
      const v = $('videoStream');
      $('videoPlaceholder').classList.remove('hidden');
      $('videoMsg').textContent = T('ops_vid_load');
      $('btnVideoRetry').classList.remove('hidden');
      const link = $('videoLink');
      link.href = videoOrigin(host) + '/stream_viewer?topic=/mission/image_annotated';
      link.classList.remove('hidden');
      v.onload = () => { $('videoPlaceholder').classList.add('hidden'); };
      v.onerror = () => {
        $('videoPlaceholder').classList.remove('hidden');
        if (wantConnected) {
          clearTimeout(videoRetryTimer);
          videoRetryTimer = setTimeout(() => startVideo(host), 3000);
        }
      };
      v.src = videoStreamURL(host);
      /* Anti-gel MJPEG (2026-07-13) : quand web_video_server redémarre
         (respawn supervision), la connexion multipart du <img> meurt SANS
         événement — l'image fige sur la dernière frame, indéfiniment.
         Aucun signal fiable n'existe côté navigateur : on ré-arme donc le
         flux toutes les 45 s (re-handshake MJPEG ~100 ms, imperceptible). */
      clearInterval(window._vidKeepalive);
      window._vidKeepalive = setInterval(() => {
        if (wantConnected && !document.hidden) v.src = videoStreamURL(host);
      }, 45000);
    }
    function stopVideo() {
      clearTimeout(videoRetryTimer);
      clearInterval(window._vidKeepalive);
      const v = $('videoStream');
      v.onload = v.onerror = null; v.src = '';
      $('videoPlaceholder').classList.remove('hidden');
      $('videoMsg').textContent = T('ops_js_no_conn');
      $('btnVideoRetry').classList.add('hidden');
      $('videoLink').classList.add('hidden');
    }

    /* ── Topics ROS ─────────────────────────────────────────────────────────── */
    function setupTopics() {
      topics.command = new ROSLIB.Topic({ ros, name: '/mission/command', messageType: 'std_msgs/String' });
      topics.command.advertise();
      console.log('[ROS] Command pipeline ready — /mission/command advertised');

      topics.velocityConfig = new ROSLIB.Topic({ ros, name: '/leo_rover/config/velocity', messageType: 'std_msgs/String' });
      topics.velocityConfig.advertise();

      topics.telemetry = new ROSLIB.Topic({ ros, name: '/mission/telemetry', messageType: 'std_msgs/String' });
      topics.log = new ROSLIB.Topic({ ros, name: '/mission/log', messageType: 'std_msgs/String' });

      topics.telemetry.subscribe((m) => {
        try {
          const d = JSON.parse(m.data);
          recordLatency();
          onTelemetry(d);
          /* Heartbeat piggyback (2026-07-13) : les navigateurs étranglent les
             timers des onglets CACHÉS (jusqu'à 1/min) — le setInterval du
             heartbeat mourait dès qu'on regardait Trajectory, et le failsafe
             backend coupait l'AUTO toutes les 60 s. Les événements de RÉCEPTION
             WebSocket, eux, ne sont pas étranglés : on émet donc aussi un
             heartbeat à chaque télémétrie reçue (throttlé à 2/s). */
          const now = Date.now();
          if (connected && topics.command && now - (window._lastHbT || 0) > 500) {
            window._lastHbT = now;
            topics.command.publish(new ROSLIB.Message({ data: JSON.stringify({ action: 'heartbeat' }) }));
          }
        } catch (e) {
          console.error('[LEO:tel] PARSE ERROR:', e.message);
        }
      });

      topics.log.subscribe((m) => {
        logLine(m.data, 'text-emerald-300');
        // Confirmation visuelle directe qu'un reset LED vient de tirer
        // (2026-07-27) — matche le message exact de _try_led_reset_event().
        if (m.data.indexOf('LED RESET') !== -1) flashBeaconReset();
      });

      /* /map/markers — marqueurs permanents balises + obstacles */
      topics.markers = new ROSLIB.Topic({ ros, name: '/map/markers', messageType: 'std_msgs/String' });
      topics.markers.subscribe((m) => {
        try {
          const mk = JSON.parse(m.data);
          if (!globalMapMarkers.find(e => e.id === mk.id && e.type === mk.type)) {
            globalMapMarkers.push({ type: mk.type, x: mk.x, y: mk.y, label: mk.label, id: mk.id });
            const col = mk.type === 'obstacle' ? 'text-alert' : 'text-emerald-300';
            logLine('[MAP] ' + mk.type.toUpperCase() + ' ' + mk.label
                    + ' at (' + mk.x.toFixed(2) + ', ' + mk.y.toFixed(2) + ')', col);
          }
        } catch (e) { /* ignore malformed */ }
      });

      /* Optional IMU subscription — D455 /camera/imu/data_raw */
      try {
        topics.imu = new ROSLIB.Topic({ ros, name: '/camera/imu/data_raw', messageType: 'sensor_msgs/Imu' });
        topics.imu.subscribe((m) => {
          const ax = m.linear_acceleration.x;
          const ay = m.linear_acceleration.y;
          const az = m.linear_acceleration.z;
          imuRoll  = Math.atan2(ay, az);
          imuPitch = Math.atan2(-ax, Math.sqrt(ay * ay + az * az));
          imuAge   = Date.now();
          updateHorizonDisplay();
        });
      } catch (e) {
        console.warn('[LEO] IMU topic subscription failed — horizon will show offline');
      }

      /* Kalibr calibration health — fed by calibration_monitor.py (see
         start_web.sh). Latched + low-rate: only republished when the
         camchain file on disk actually changes. */
      topics.calibration = new ROSLIB.Topic({
        ros, name: '/leo_vision/calibration_status', messageType: 'std_msgs/String'
      });
      topics.calibration.subscribe((m) => {
        try {
          updateCalibrationStatus(JSON.parse(m.data));
        } catch (e) {
          console.error('[LEO:calib] PARSE ERROR:', e.message);
        }
      });
    }

    /* ── Calibration Status panel (ops.html) ───────────────────────────── */
    const REPROJ_OK_PX = 0.5; // <-- green below this, orange above (per acceptance criteria)

    function updateCalibrationStatus(d) {
      const body = $('calibBody'), na = $('calibUnavailable');
      if (!body || !na) return;

      if (!d || !d.available) {
        body.classList.add('hidden');
        na.classList.remove('hidden');
        return;
      }
      na.classList.add('hidden');
      body.classList.remove('hidden');

      const cam0 = (d.cameras || []).find(c => c.cam_id === 0) || d.cameras[0];
      const reproj = cam0 ? cam0.reprojection_error_px : null;

      setTxt('calibReproj', reproj != null ? reproj.toFixed(3) + 'px' : '—');
      const badge = $('calibBadge');
      if (badge) {
        const ok = reproj != null && reproj < REPROJ_OK_PX;
        badge.textContent = reproj == null ? '—' : (ok ? T('ops_calib_good') : T('ops_calib_check'));
        badge.className = 'net-badge mt-2 '
          + (reproj == null ? 'bg-zinc-700/40 text-zinc-400'
             : ok ? 'bg-emerald-500/15 text-emerald-300'
                  : 'bg-amber-500/15 text-amber-300');
      }

      setTxt('calibDate', d.calibrated_at
        ? new Date(d.calibrated_at * 1000).toLocaleString() : '—');
      setTxt('calibFocal', cam0 && cam0.intrinsics
        ? cam0.intrinsics[0].toFixed(1) + ' / ' + cam0.intrinsics[1].toFixed(1) + ' px' : '—');
      setTxt('calibCams', (d.cameras || []).length || '—');

      const alertEl = $('calibDriftAlert');
      if (alertEl) {
        const drift = d.drift || {};
        if (drift.alert && drift.reasons && drift.reasons.length) {
          alertEl.textContent = T('ops_calib_drift') + ' ' + drift.reasons.join('; ');
          alertEl.classList.remove('hidden');
        } else {
          alertEl.classList.add('hidden');
        }
      }
    }

    /* ── Beacon LED Reset panel (ops.html) — repurposed 2026-07-27 ─────────
       Shows the ROBOT's own local pose (self.pose from telemetry, zeroed by
       leo_backend's LED-only reset) and the 4-LED capture status. No longer
       fed by carolus_astrobee's /pose (that was the beacon's pose, not the
       robot's — a source of real confusion) nor by /mins/external_ref/carolus
       (never populated, beacon_link is never published — see report
       §sec:july24tf2). The badge briefly flashes "RESET!" when
       leo_backend's log line ("LED RESET — ... — odometry = 0,0,0") is seen,
       giving direct visual confirmation the event actually fired. */
    let carolusResetFlashT = null;
    function updateRobotPoseInBeaconPanel(d) {
      const p = d.pose || {};
      if (p.x != null) setTxt('caroX', p.x.toFixed(3));
      if (p.y != null) setTxt('caroY', p.y.toFixed(3));
      if (p.yaw_deg != null) setTxt('caroYw', p.yaw_deg.toFixed(1) + '°');
    }
    /* ══ FIX GLOBAL CAROLUS — 6 DDL ══════════════════════════════════════
       A ne PAS confondre avec updateRobotPoseInBeaconPanel, qui affiche la
       pose INTERNE du robot. Ici : les six degres de liberte du fix balise,
       lus en LECTURE SEULE. La FRAICHEUR est decidee par le backend (champ
       `stale`), jamais recalculee ici : les deux machines n'ont pas la meme
       horloge. Trois etats : aucun fix (gris), perime (ambre, valeurs
       CONSERVEES et datees), frais (violet). */
    function renderCarolusFix(f) {
      const badge = $('caroFixAge');
      const cells = ['cfX','cfY','cfZ','cfR','cfP','cfYw'];
      if (!badge) return;
      if (!f) {
        badge.textContent = 'no fix';
        badge.className = 'ml-auto rounded px-1.5 py-0.5 font-mono text-[9px] font-bold text-zinc-500 ring-1 ring-white/10';
        cells.forEach(function (id) { setTxt(id, '\u2014'); });
        setTxt('cfFrame', '\u2014');
        const cv0 = $('cfConv'), sg0 = $('cfSigns');
        if (cv0) { cv0.textContent = 'repère \u2014';
          cv0.className = 'rounded px-1.5 py-0.5 font-bold ring-1 ring-white/10 text-zinc-500'; }
        if (sg0) sg0.textContent = 'signes \u2014';
        return;
      }
      setTxt('cfX',  f.x.toFixed(3));
      setTxt('cfY',  f.y.toFixed(3));
      setTxt('cfZ',  f.z.toFixed(3));
      setTxt('cfR',  f.roll.toFixed(1)  + '\u00b0');
      setTxt('cfP',  f.pitch.toFixed(1) + '\u00b0');
      setTxt('cfYw', f.yaw.toFixed(1)   + '\u00b0');
      setTxt('cfFrame', f.frame || '?');
      /* LOT B : le repère et la combinaison de signes sont affichés en clair.
         Vert = permutation appliquée (valeurs en base_link) ; ambre = valeurs
         BRUTES caméra, donc à ne pas lire comme une pose robot. */
      const cv = $('cfConv'), sg = $('cfSigns');
      if (cv) {
        cv.textContent = 'repère ' + (f.frame || '?');
        cv.className = 'rounded px-1.5 py-0.5 font-bold ring-1 '
          + (f.converted ? 'bg-emerald-400/15 text-emerald-300 ring-emerald-300/30'
                         : 'bg-amber-400/15 text-amber-300 ring-amber-300/30');
      }
      if (sg) {
        sg.textContent = 'signes ' + (f.signs || '—');
        sg.className = 'rounded px-1.5 py-0.5 text-zinc-400 ring-1 ring-white/10';
      }
      cells.forEach(function (id) {
        const el = $(id); if (!el) return;
        const base = (id === 'cfR' || id === 'cfP' || id === 'cfYw')
                   ? 'text-fuchsia-200' : 'text-violet-200';
        el.className = 'text-sm font-bold ' + (f.stale ? 'text-zinc-500' : base);
      });
      badge.textContent = (f.stale ? 'STALE ' : '') + f.age_s.toFixed(1) + 's';
      badge.className = 'ml-auto rounded px-1.5 py-0.5 font-mono text-[9px] font-bold ring-1 '
        + (f.stale ? 'bg-amber-400/15 text-amber-300 ring-amber-300/30'
                   : 'bg-violet-400/15 text-violet-200 ring-violet-300/30');
    }

    function flashBeaconReset() {
      const dot = $('carolusFixDot'), lbl = $('carolusFixLbl');
      if (!dot) return;
      const inner = dot.querySelector('span:first-child');
      dot.className = 'ml-auto inline-flex items-center gap-1 rounded px-1.5 py-0.5 text-[10px] font-bold text-emerald-300 ring-1 ring-emerald-400/30';
      if (inner) inner.className = 'block h-1.5 w-1.5 rounded-full bg-emerald-400 animate-pulse-glow';
      if (lbl) lbl.textContent = T('ops_carolus_fix');
      clearTimeout(carolusResetFlashT);
      carolusResetFlashT = setTimeout(() => {
        dot.className = 'ml-auto inline-flex items-center gap-1 rounded px-1.5 py-0.5 text-[10px] font-bold text-zinc-500 ring-1 ring-white/10';
        if (inner) inner.className = 'block h-1.5 w-1.5 rounded-full bg-zinc-600';
        if (lbl) lbl.textContent = T('ops_carolus_nofix');
      }, 2500);
    }

    /* ── 4-LED beacon status cross (ops.html) ──────────────────────────────
       Driven from telemetry detection.led_reset.count (0-4 clustered beacon
       LEDs) + detection.dist. The axis mapping follows the beacon's known_points
       geometry (2 LEDs on ±X, 1 on Y, 1 on Z) purely as a visual proxy for how
       many of the 4 are currently captured — X needs ≥2, Y needs ≥3, Z needs
       all 4. AprilTag is on standby (2026-07-27): this cross is now the sole
       beacon-capture indicator. */
    const LED_ON = 'block h-4 w-4 rounded-full ring-1 ring-white/20 transition';
    const LED_OFF = 'block h-4 w-4 rounded-full bg-zinc-700 ring-1 ring-white/10 transition';
    function setLed(id, on, color) {
      const el = $(id); if (!el) return;
      if (on) { el.className = LED_ON; el.style.background = color; el.style.boxShadow = '0 0 8px ' + color; }
      else    { el.className = LED_OFF; el.style.background = ''; el.style.boxShadow = ''; }
    }
    function updateLedCross(det) {
      const lr = (det && det.led_reset) || {};
      const count = lr.count || 0;
      const dist = det && det.dist;
      setTxt('ledCount', count + '/4');
      const cnt = $('ledCount');
      if (cnt) cnt.className = 'font-mono text-xs font-bold ' + (count >= 3 ? 'text-emerald-300' : count >= 1 ? 'text-amber-300' : 'text-zinc-400');
      const AX = '#34d399';  // emerald once captured
      setLed('ledX', count >= 2, AX);
      setLed('ledY', count >= 3, AX);
      setLed('ledZ', count >= 4, AX);
      setLed('ledDist', dist != null, '#22d3ee');
      setTxt('ledDistVal', dist != null ? dist.toFixed(2) + ' m' : '—');
    }

    /* ── Trajectory recorder + Matlab export panel (ops.html) ─────────────── */
    let opsExportPending = false;     // ce client a cliqué "Export" -> auto-DL
    let opsExportSeenTs = null;
    let opsMlTs = null, opsExportTimer = null;

    /* Verrouillage du bouton Export. Retour operateur : « le bouton ne fait
       rien ». L'export marchait ; c'est le lancement de MATLAB qui echouait
       (licence, error 5001) dans un processus DETACHE, sans que rien ne le
       signale au cockpit. */
    function opsExportLock(on, label) {
      const b = $('btnExportMatlab');
      if (!b) return;
      b.disabled = !!on;
      b.style.opacity = on ? '0.55' : '';
      b.style.cursor  = on ? 'wait' : '';
      const sp = b.querySelector('span');
      if (sp) sp.textContent = on ? (label || 'Exporting\u2026')
                                  : T('ops_export_btn');
      clearTimeout(opsExportTimer);
      if (on) {
        opsExportTimer = setTimeout(function () {
          opsExportLock(false);
          logLine('[DATA] aucune reponse du backend apres 150 s', 'text-red-400');
        }, 150000);
      }
    }
    function updateTrajPanel(d) {
      const tr = d.traj_rec; if (!tr) return;
      setTxt('trajMins', tr.mins || 0);
      setTxt('trajVins', tr.vins || 0);
      setTxt('trajCaro', tr.carolus || 0);
      const ex = tr.last_export, box = $('exportLinks');
      if (ex && box) {
        box.classList.remove('hidden');
        setTxt('exportTs', ex.ts || '');
        // Nouvel export ET c'est CE client qui a cliqué -> téléchargement
        // immédiat du .mat (même serveur statique -> chemin même-origine).
        // Un rechargement de page ne re-télécharge pas un export antérieur.
        if (ex.ts && ex.ts !== opsExportSeenTs) {
          opsExportSeenTs = ex.ts;
          if (opsExportPending) {
            opsExportPending = false;
            if (ex.mat) {
              const a = document.createElement('a');
              a.href = location.origin + '/exports/' + ex.mat;
              a.download = ex.mat;
              document.body.appendChild(a); a.click(); a.remove();
              logLine('[DATA] ' + ex.mat + ' téléchargé', 'text-accent');
            }
          }
        }
        const list = $('exportFileList');
        if (list && list.dataset.ts !== (ex.ts || '')) {
          list.dataset.ts = ex.ts || '';
          list.innerHTML = '';
          // Exports are written into web/exports/ and served by the SAME static
          // server that served this page (serve.py) — always same-origin,
          // whether local (:8000) or through the tunnel. Not the ROS host.
          const add = (label, fname) => {
            if (!fname) return;
            const a = document.createElement('a');
            a.href = location.origin + '/exports/' + fname;
            a.textContent = label;
            a.target = '_blank';
            a.className = 'text-accent underline hover:text-accent/80';
            list.appendChild(a);
          };
          const f = ex.files || {};
          add('MINS.csv', f.mins); add('VINS.csv', f.vins); add('Carolus.csv', f.carolus);
          add('.mat', ex.mat);
        }
      }

      /* Verdict du lancement MATLAB, publie par le backend qui surveille le
         journal du processus DETACHE. Place ICI, APRES la fermeture de
         `if (ex && box)` : l'avoir mis DEDANS lors d'une premiere tentative
         avait ferme ce bloc trop tot, laissant le rendu de la liste hors de
         sa garde -> `ex.ts` evalue sur un `ex` indefini -> TypeError dans le
         callback de telemetrie, qui cassait tout le cockpit, bouton compris.
         `ex` est ici potentiellement indefini : on le teste avant usage. */
      const ml = tr.matlab_launch;
      if (ml && ml.ts !== opsMlTs) {
        opsMlTs = ml.ts;
        if (ml.state === 'running') {
          opsExportPending = false;
          opsExportLock(false);
          logLine('[DATA] MATLAB ouvert sur le trace', 'text-emerald-400');
        } else if (ml.state === 'closed') {
          opsExportLock(false);
          logLine('[DATA] MATLAB ferme - .mat conserve dans web/exports/', 'text-zinc-400');
        } else if (ml.state === 'failed' || ml.state === 'unknown') {
          opsExportLock(false);
          logLine('[DATA] MATLAB n a pas pu s ouvrir - ' + ml.detail, 'text-red-400');
          /* REPLI : les fichiers existent, seul l affichage a echoue. */
          if (opsExportPending && ex && ex.mat) {
            opsExportPending = false;
            const a = document.createElement('a');
            a.href = location.origin + '/exports/' + ex.mat;
            a.download = ex.mat;
            document.body.appendChild(a); a.click(); a.remove();
            logLine('[DATA] repli : ' + ex.mat + ' telecharge', 'text-amber-300');
          }
        }
      }
    }

    /* ── Carolus tunable params panel (ops.html) ───────────────────────────
       Values parsed from the Carolus launch file by the backend
       (carolus_params in telemetry). Editable; a change sends set_carolus_param
       (effective on the next Carolus launch — no dynamic_reconfigure). */
    let carolusParamsRendered = false;
    const CAROLUS_LABELS = {
      min_circularity: 'Circularité min', saturation_threshold: 'Seuil saturation',
      min_area: 'Aire min (px²)', max_area: 'Aire max (px²)',
      max_distance_lim: 'Distance max (mm)', lb_hue: 'Teinte basse',
      ub_hue: 'Teinte haute', image_threshold: 'Seuil image'
    };
    /* LOT C : cinématique firmware, LECTURE SEULE. Les valeurs viennent du
       serveur de paramètres (valeur active), pas du fichier. `null` signifie
       que le robot n'était pas joignable au chargement — le backend re-tente,
       donc l'affichage se remplit tout seul dès qu'il répond. */
    /* LOT D : bandeau de perte de balise. `lost` distingue une PERTE (on a vu
       la balise puis plus rien) d'une ABSENCE (jamais vue) — le backend ne
       lève `lost` que dans le premier cas, pour qu'un démarrage sans balise
       n'alarme pas inutilement. */
    function renderBeaconWatch(bw) {
      const box = $('beaconLostBanner');
      if (!box) return;
      if (!bw || !bw.lost) { box.style.display = 'none'; return; }
      box.style.display = 'block';
      setTxt('beaconLostTxt',
             'Balise perdue depuis ' + (bw.age_s != null ? bw.age_s : '?') + ' s');
      let sub = 'seuil ' + bw.timeout_s + ' s \u00b7 état ' + (bw.state || '?');
      if (bw.estop_fired)      sub += ' \u00b7 ARRÊT DÉCLENCHÉ';
      else if (!bw.state_depends) sub += ' \u00b7 état indépendant de la balise, pas d\'arrêt';
      else if (bw.estop_armed) sub += ' \u00b7 arrêt armé';
      else                     sub += ' \u00b7 arrêt désarmé (signalement seul)';
      setTxt('beaconLostSub', sub);
    }

    function renderDriveParams(dp) {
      const map = { dpRadius: 'wheel_radius', dpSep: 'wheel_separation',
                    dpAngMul: 'angular_velocity_multiplier' };
      for (const id in map) {
        const el = $(id); if (!el) continue;
        const v = dp ? dp[map[id]] : null;
        el.textContent = (v === null || v === undefined) ? '\u2014' : String(v);
        el.style.opacity = (v === null || v === undefined) ? '0.5' : '';
      }
    }

    function renderCarolusParams(params) {
      const host = $('carolusParamList');
      if (!host || !params) return;
      const keys = Object.keys(CAROLUS_LABELS).filter(k => k in params);
      if (!keys.length) return;
      if (!carolusParamsRendered) {
        host.innerHTML = '';
        keys.forEach(k => {
          const row = document.createElement('div');
          row.className = 'flex items-center justify-between gap-2';
          const lab = document.createElement('span');
          lab.className = 'text-zinc-400'; lab.textContent = CAROLUS_LABELS[k];
          const inp = document.createElement('input');
          inp.type = 'number'; inp.step = 'any'; inp.id = 'carop_' + k;
          inp.value = params[k];
          inp.className = 'w-20 rounded border border-white/10 bg-white/5 px-2 py-1 text-right text-plasma outline-none focus:border-plasma/50';
          inp.addEventListener('change', () => {
            const v = parseFloat(inp.value);
            if (!isNaN(v)) {
              sendCommand({ action: 'set_carolus_param', key: k, value: v });
              logLine('[Carolus] ' + k + ' = ' + v + ' (next launch)', 'text-plasma');
            }
          });
          row.appendChild(lab); row.appendChild(inp);
          host.appendChild(row);
        });
        carolusParamsRendered = true;
      } else {
        // Refresh values only if the user isn't editing that field.
        keys.forEach(k => {
          const inp = $('carop_' + k);
          if (inp && document.activeElement !== inp) inp.value = params[k];
        });
      }
    }

    /* ── Pose Source (VINS/MINS, leo_navigation/pose_selector) ──────────── */
    // Un bouton par source — sélection directe (2026-07-23). Un clic sur une
    // source pas encore prête l'ARME (2026-07-24, ré-introduit sur demande
    // opérateur explicite après l'essai annulé du 23/07) : la bascule
    // s'applique automatiquement dès qu'elle publie, sans reclic. La leçon
    // du 23/07 ("trop d'états implicites") est traitée ici en rendant l'état
    // armé impossible à manquer — un LABEL TEXTE persistant sous le bouton
    // (pas juste une couleur ou une animation), piloté par le SEUL topic
    // source de vérité (pose_source_pending), jamais déduit ailleurs.
    const POSE_SRC_STYLE = {
      MINS: { active: ['border-plasma/40', 'bg-plasma/15', 'text-plasma'], dot: 'bg-plasma' },
      VINS: { active: ['border-accent/40', 'bg-accent/15', 'text-accent'], dot: 'bg-accent' },
    };
    const POSE_SRC_IDLE = ['border-white/10', 'bg-white/5', 'text-zinc-400'];

    function updatePoseSourceBtn(source, available, pending) {
      const btns = { MINS: $('poseSourceBtnMINS'), VINS: $('poseSourceBtnVINS') };
      const dots = { MINS: $('poseSourceDotMINS'), VINS: $('poseSourceDotVINS') };
      const armedLbl = { MINS: $('poseSourceArmedMINS'), VINS: $('poseSourceArmedVINS') };
      if (!btns.MINS || !btns.VINS || !dots.MINS || !dots.VINS) return;

      ['MINS', 'VINS'].forEach(s => {
        const btn = btns[s], dot = dots[s], st = POSE_SRC_STYLE[s];
        const active = available && source === s;
        const armed = available && !active && pending === s;

        btn.classList.toggle('opacity-50', !available);
        btn.title = available ? '' : T('ops_pose_source_na');

        // Toujours retirer TOUS les jeux de classes avant d'appliquer l'état
        // courant — jamais d'accumulation, la fonction tourne à chaque tick.
        // Armé = directement la couleur de LA SOURCE CIBLÉE (2026-07-24bis,
        // demande opérateur : plus d'ambre neutre, VINS passe direct en
        // bleu dès le clic) — seul le point clignote encore (animate-pulse-glow)
        // + le label texte, pour garder un minimum de distinction avec
        // l'état vraiment actif sans réintroduire l'ambiguïté du 23/07.
        POSE_SRC_IDLE.forEach(c => btn.classList.remove(c));
        st.active.forEach(c => btn.classList.remove(c));
        const styleSet = (active || armed) ? st.active : POSE_SRC_IDLE;
        styleSet.forEach(c => btn.classList.add(c));

        dot.className = 'block h-2 w-2 shrink-0 rounded-full ' +
          (active ? st.dot : (armed ? st.dot + ' animate-pulse-glow' : 'bg-zinc-600'));

        if (armedLbl[s]) armedLbl[s].classList.toggle('hidden', !armed);
      });
    }

    function sendCommand(obj) {
      if (!connected || !topics.command) {
        if (obj.action === 'stop' || obj.action === 'park') _stopPending = true;
        if (!cmdDropWarned) {
          logLine(T('ops_js_ign'), 'text-alert');
          const hudM = $('hudMode');
          if (hudM) { hudM.style.color = '#FC3D21'; setTimeout(() => { hudM.style.color = ''; }, 600); }
          cmdDropWarned = true;
        }
        if (obj.action !== 'heartbeat')
          console.warn('[CTRL:DROP] not connected — cmd dropped:', obj.action);
        return;
      }
      cmdDropWarned = false;
      if (obj.action !== 'heartbeat')
        console.log('[CTRL:pub]', obj.action, JSON.stringify(obj));
      topics.command.publish(new ROSLIB.Message({ data: JSON.stringify(obj) }));
    }

    function gotoBeacon(beaconId) {
      const bid = parseInt(beaconId, 10);
      if (isNaN(bid) || bid < 1) { logLine(T('ops_js_goto_invalid'), 'text-alert'); return; }
      const storedBeacons = globalMapMarkers.filter(m => m.type === 'beacon');
      if (storedBeacons.length === 0) {
        logLine(T('ops_js_goto_none'), 'text-alert'); return;
      }
      if (bid > storedBeacons.length) {
        logLine(T('ops_js_goto_oob', { n: bid, max: storedBeacons.length }), 'text-alert'); return;
      }
      sendCommand({ action: 'goto_beacon', beacon_id: bid });
      logLine(T('ops_js_goto_nav', { n: bid }), 'text-violet-400');
    }

    /* ── Telemetry → UI ─────────────────────────────────────────────────────── */
    const modeOrder  = { MANUEL: 'AUTO', AUTO: 'MANUEL' };
    const setTxt = (id, v) => { const e = $(id); if (e) e.textContent = v; };

    /* ── Panneau PID live (2026-07-22) ────────────────────────────────────
       Injecté en JS pur, aucune dépendance sur le HTML existant — évite de
       toucher ops.html à l'aveugle. Affiche les sorties déjà calculées par
       le backend ce tick (jamais recalculées côté client). */
    function ensurePidPanel() {
      let el = document.getElementById('pidPanel');
      if (el) return el;
      el = document.createElement('div');
      el.id = 'pidPanel';
      el.className = 'fixed bottom-4 left-4 z-40 rounded-xl border border-white/10 '
        + 'bg-zinc-900/90 backdrop-blur px-3 py-2 font-mono text-[11px] text-zinc-300 '
        + 'shadow-lg space-y-1 min-w-[180px]';
      el.innerHTML = `
        <div class="text-[10px] uppercase tracking-wide text-zinc-500 mb-1">PID live</div>
        <div class="flex justify-between gap-3"><span>Vitesse (obstacle)</span><span id="pidObsScale">—</span></div>
        <div class="h-1.5 rounded-full bg-zinc-700 overflow-hidden mb-1">
          <div id="pidObsBar" class="h-full bg-emerald-400 transition-all" style="width:100%"></div>
        </div>
        <div class="flex justify-between gap-3"><span>Corridor</span><span id="pidCorridor">—</span></div>
        <div class="flex justify-between gap-3"><span>Cap patrouille</span><span id="pidPatrolAng">—</span></div>
        <div class="flex justify-between gap-3"><span>Cap balise</span><span id="pidLockAng">—</span></div>
      `;
      document.body.appendChild(el);
      return el;
    }

    function updatePidPanel(d) {
      const p = d.pid;
      if (!p) return;
      ensurePidPanel();
      const scalePct = Math.round((p.obstacle_scale != null ? p.obstacle_scale : 1) * 100);
      setTxt('pidObsScale', scalePct + ' %');
      const bar = $('pidObsBar');
      if (bar) {
        bar.style.width = scalePct + '%';
        bar.className = 'h-full transition-all ' + (
          scalePct >= 90 ? 'bg-emerald-400' : scalePct >= 50 ? 'bg-amber-400' : 'bg-alert');
      }
      setTxt('pidCorridor', p.corridor_mm != null ? (p.corridor_mm / 1000).toFixed(2) + ' m' : '—');
      setTxt('pidPatrolAng', (p.patrol_ang >= 0 ? '+' : '') + p.patrol_ang.toFixed(3) + ' rad/s');
      const lockAng = d.mode === 'AUTO' && d.auto_state === 'LOCK'
        ? (Math.abs(p.lock_align_ang) > Math.abs(p.lock_approach_ang) ? p.lock_align_ang : p.lock_approach_ang)
        : null;
      setTxt('pidLockAng', lockAng != null ? (lockAng >= 0 ? '+' : '') + lockAng.toFixed(3) + ' rad/s' : '—');
    }

    function onTelemetry(d) {
      lastTelemetry = d;
      updatePidPanel(d);

      /* Global trajectory — world coords from backend (authoritative) */
      if (d.world_heading != null) gOrientation = d.world_heading;

      if (d.beacon_count > gPrevBeaconCount) {
        const bx = d.pose.wx != null ? d.pose.wx : toGlobal(gPrevPose.x, gPrevPose.y)[0];
        const by = d.pose.wx != null ? d.pose.wy : toGlobal(gPrevPose.x, gPrevPose.y)[1];
        if (_mapValidate(bx, by, 'beacon')) {
          globalBeacons.push({ x: bx, y: by, label: T('ops_beacon_label') + d.beacon_count });
          _mapExpandBounds(bx, by);
          if (globalBeacons.length > 500) globalBeacons.splice(0, 1);
        }
        gOffset = { x: bx, y: by };
        gPrevBeaconCount = d.beacon_count;
        if (window.AudioMgr) AudioMgr.play('beacon');
      }
      const gx = d.pose.wx != null ? d.pose.wx : toGlobal(d.pose.x, d.pose.y)[0];
      const gy = d.pose.wx != null ? d.pose.wy : toGlobal(d.pose.x, d.pose.y)[1];
      if (_mapValidate(gx, gy, 'pose')) {
        const gLast = globalTraj.last();
        if (!gLast || Math.hypot(gx - gLast[0], gy - gLast[1]) > 0.02) {
          globalTraj.push([gx, gy]);
          _mapExpandBounds(gx, gy);
        }
      }
      gPrevPose   = { x: d.pose.x, y: d.pose.y };
      gPrevRawYaw = d.pose.raw_yaw != null ? d.pose.raw_yaw : gOrientation;

      const mm = String(Math.floor(d.mission_s / 60)).padStart(2, '0');
      const ss = String(d.mission_s % 60).padStart(2, '0');
      setTxt('mMission', mm + ':' + ss);
      setTxt('mBeacons', d.beacon_count);
      setTxt('hudBeacons', T('ops_hud_beacons').replace('{n}', d.beacon_count));
      setTxt('hudMode', d.mode + (d.mode === 'AUTO' ? '·' + d.auto_state : ''));
      setTxt('modeBtnLabel', d.mode + ' → ' + (modeOrder[d.mode] || 'AUTO'));
      updatePoseSourceBtn(d.pose_source, d.pose_source_available, d.pose_source_pending);
      /* Beacon LED reset / trajectory-export panels (2026-07-27) */
      updateLedCross(d.detection);
      updateRobotPoseInBeaconPanel(d);
      renderCarolusFix(d.carolus_fix);
      updateTrajPanel(d);
      if (d.carolus_params) renderCarolusParams(d.carolus_params);
      renderDriveParams(d.drive_params);
      renderBeaconWatch(d.beacon_watch);
      setTxt('mCamHz', d.hz.cam + ' Hz');
      setTxt('mOdomHz', d.hz.odom + ' Hz');
      setTxt('hudFps', d.hz.cam + ' Hz');
      setTxt('railMode', d.mode);
      setTxt('railLin', (d.vel.lin >= 0 ? '+' : '') + d.vel.lin.toFixed(2));
      setTxt('railAng', (d.vel.ang >= 0 ? '+' : '') + d.vel.ang.toFixed(2));
      setTxt('railBeacons', d.beacon_count);
      setTxt('railCam', d.hz.cam);

      if (d.hz.cam > 0 && d.hz.cam < 8) logLine(T('ops_js_cam_low'), 'text-amber-400');

      /* Battery */
      if (d.battery.v != null) {
        setTxt('mBattV', d.battery.v.toFixed(1) + ' V');
        const pct = d.battery.pct || 0;
        const bar = $('battBar');
        if (bar) {
          bar.style.width = pct + '%';
          bar.className = 'h-full rounded-full transition-all '
            + (pct > 50 ? 'bg-gradient-to-r from-emerald-400 to-emerald-500'
               : pct > 20 ? 'bg-gradient-to-r from-amber-400 to-amber-500'
                 : 'bg-gradient-to-r from-alert to-alert');
        }
        setTxt('mBattPct', pct + '%');
      } else {
        setTxt('mBattV', '-- V'); setTxt('mBattPct', '—');
      }

      /* Detection badge */
      const det = d.detection;
      setTxt('vLeds', det.cluster + '/' + det.leds_all);
      const ck = $('vChecker');
      if (ck) {
        if (det.checker) {
          ck.textContent = det.checker_size ? det.checker_size.join('×') : T('ops_ck_yes');
          ck.className = 'font-mono text-lg font-bold text-emerald-400';
        } else { ck.textContent = '—'; ck.className = 'font-mono text-lg font-bold text-zinc-500'; }
      }
      const badge = $('detBadge');
      if (badge) {
        if (det.valid) {
          badge.textContent = T('ops_det_valid') + (det.dist ? ' · ' + det.dist + ' m' : '');
          badge.className = 'rounded-full bg-emerald-500/20 px-3 py-1 text-xs font-semibold text-emerald-300 ring-1 ring-emerald-400/30';
        } else if (det.checker) {
          badge.textContent = T('ops_det_ck').replace('{n}', det.cluster);
          badge.className = 'rounded-full bg-amber-500/20 px-3 py-1 text-xs font-medium text-amber-300 ring-1 ring-amber-400/20';
        } else if (det.cluster > 0) {
          badge.textContent = det.cluster + ' ' + T('ops_det_led');
          badge.className = 'rounded-full bg-amber-500/10 px-3 py-1 text-xs font-medium text-amber-200';
        } else {
          badge.textContent = d.cam_alive ? T('ops_det_none') : T('ops_det_noimg');
          badge.className = 'rounded-full bg-zinc-700/40 px-3 py-1 text-xs font-medium text-zinc-300';
        }
      }

      /* Position + compass */
      setTxt('posX', d.pose.wx != null ? d.pose.wx.toFixed(2) : d.pose.x.toFixed(2));
      setTxt('posY', d.pose.wx != null ? d.pose.wy.toFixed(2) : d.pose.y.toFixed(2));
      const worldYawDeg = (d.world_heading || 0) * (180 / Math.PI) + (d.pose.yaw_deg || 0);
      setTxt('posYaw', Math.round(worldYawDeg) + '°');
      const needle = $('compassNeedle');
      if (needle) needle.style.transform = 'rotate(' + worldYawDeg + 'deg)';
      setTxt('compassHeading', (((Math.round(worldYawDeg) % 360) + 360) % 360) + '°');

      /* Health LEDs */
      setHealth('hCam', d.health.cam); setHealth('hOdom', d.health.odom);
      setHealth('hBatt', d.health.batt); setHealth('hWheels', d.health.wheels);

      updateWheelIndicators(d);
      pushChart(d);
      updateMissionStatus(d);
      updateVideoCanvas(d);

      /* Sysinfo throttled to 2Hz (every 5th packet) to reduce DOM churn */
      _sysThrottleN++;
      if (d.sysinfo && (_sysThrottleN % 5 === 0)) updateSysinfo(d.sysinfo);

      /* Sync permanent map markers from telemetry (handles page reload) */
      if (d.map_markers && d.map_markers.length !== globalMapMarkers.length) {
        globalMapMarkers = d.map_markers.map(m => ({
          type: m.type, x: m.x, y: m.y, label: m.label, id: m.id
        }));
      }

      /* WAIT countdown block */
      const waitWrap = $('msWaitCountdown');
      const waitNum  = $('msWaitNum');
      if (waitWrap) {
        if (d.auto_state === 'WAIT' && d.wait_remaining != null) {
          if (waitNum) waitNum.textContent = d.wait_remaining + 's';
          waitWrap.classList.remove('hidden');
        } else {
          waitWrap.classList.add('hidden');
        }
      }

      /* Global critical alert overlay — fires ONCE per event, not per packet.
         Hysteresis prevents threshold jitter from spamming the overlay:
           CPU  : fires >80°C, resets <75°C
           Batt : fires <15%,  resets >20%
           Ping : fires >150ms, resets <100ms                              */
      (function () {
        const overlay = $('globalAlertOverlay');
        if (!overlay) return;

        /* Respect 1-hour mute: suppress ALL alert display while muted */
        if (_isMuted()) {
          if (!overlay.classList.contains('hidden')) overlay.classList.add('hidden');
          return;
        }

        const cpuTemp = d.sysinfo && d.sysinfo.cpu_temp != null ? d.sysinfo.cpu_temp : null;
        const battPct = d.battery  && d.battery.pct     != null ? d.battery.pct      : null;

        /* Active conditions (trigger thresholds) */
        const isCpuHot   = cpuTemp != null && cpuTemp  > 80;
        const isBattLow  = battPct != null && battPct  < 15;
        const isPingHigh = netAvgMs > 150;

        /* Reset fired flags only when value drops BELOW hysteresis threshold.
           This prevents a 80.1°C → 79.9°C oscillation from re-arming the flag. */
        if (cpuTemp != null && cpuTemp  < 75)  _alertFiredCpu  = false;
        if (battPct != null && battPct  > 20)  _alertFiredBatt = false;
        if (netAvgMs < 100)                    _alertFiredPing = false;

        /* Detect NEW transitions : condition active AND not yet fired this session */
        const newCpu   = isCpuHot   && !_alertFiredCpu;
        const newBatt  = isBattLow  && !_alertFiredBatt;
        const newPing  = isPingHigh && !_alertFiredPing;

        /* Build current message (always, to keep it fresh if overlay is visible) */
        const msgs = [];
        if (isCpuHot)   msgs.push(T('ops_alert_cpu_temp').replace('{v}', Math.round(cpuTemp)));
        if (isBattLow)  msgs.push(T('ops_alert_batt').replace('{v}', Math.round(battPct)));
        if (isPingHigh) msgs.push(T('ops_alert_ping').replace('{v}', Math.round(netAvgMs)));

        if (newCpu || newBatt || newPing) {
          /* ── New event : mark as fired, show overlay, play sound once ── */
          if (newCpu)  _alertFiredCpu  = true;
          if (newBatt) _alertFiredBatt = true;
          if (newPing) _alertFiredPing = true;
          $('globalAlertMsg').textContent = msgs.join(' · ');
          overlay._dismissed = false;
          overlay.classList.remove('hidden');
          if (!overlay._sounded) {
            overlay._sounded = true;
            if (window.AudioMgr) AudioMgr.play('emergency');
          }
        } else if (msgs.length > 0) {
          /* ── Condition still active but already fired — keep text fresh, stay quiet ── */
          $('globalAlertMsg').textContent = msgs.join(' · ');
        } else {
          /* ── All conditions clear — full reset ── */
          overlay.classList.add('hidden');
          overlay._sounded   = false;
          overlay._dismissed = false;
        }
      })();
    }

    function setHealth(id, ok) {
      const e = $(id); if (e) e.className = 'led-dot ' + (ok ? 'text-emerald-400' : 'text-alert');
    }

    /* ══════════════════════════════════════════════════════════════════════
       MISSION STATUS BLOCK
       ══════════════════════════════════════════════════════════════════════ */
    const AUTO_STATES_LABELS = {
      /* legacy states (kept for compatibility) */
      SCAN360:  { label: 'SCANNING',  cls: 'auto' },
      APPROACH: { label: 'APPROACH',  cls: 'emerald' },
      RESET:    { label: 'RESET',     cls: 'emerald' },
      TURN180:  { label: 'TURN 180°', cls: 'auto' },
      ADVANCE:  { label: 'ADVANCE',   cls: 'auto' },
      /* new autonomy states */
      PATROL:   { label: 'PATROL',    cls: 'auto' },
      LOCK:     { label: 'LOCK·ON',   cls: 'emerald' },
      WAIT:     { label: 'WAIT 30s',  cls: 'emerald' },
      U_TURN:   { label: 'U·TURN',    cls: 'auto' },
      RTB:      { label: 'RTB←BASE',  cls: 'amber' },
    };

    /* FSM state → i18n key + colour (matches backend FSM_* constants) */
    const FSM_LABELS = {
      MANUAL:         { i18nKey: 'fsm_manual',       dot: 'text-accent',       ring: 'ring-accent/40 bg-accent/10' },
      AUTO_PATROL:    { i18nKey: 'fsm_auto_patrol',  dot: 'text-sky-400',      ring: 'ring-sky-400/40 bg-sky-500/10' },
      AUTO_NO_CAMERA: { i18nKey: 'fsm_auto_nocam',   dot: 'text-alert',        ring: 'ring-alert/40 bg-alert/15' },
      AUTO_NO_DEPTH:  { i18nKey: 'fsm_auto_nodepth', dot: 'text-alert',        ring: 'ring-alert/40 bg-alert/15' },
      LOCK_BEACON:    { i18nKey: 'fsm_lock_beacon',  dot: 'text-emerald-400',  ring: 'ring-emerald-400/40 bg-emerald-500/10' },
      RESET_ODOMETRY: { i18nKey: 'fsm_reset_odom',   dot: 'text-amber-400',    ring: 'ring-amber-400/40 bg-amber-500/10' },
      AVOID_OBSTACLE: { i18nKey: 'fsm_avoid_obs',    dot: 'text-alert',        ring: 'ring-alert/40 bg-alert/15' },
      GOTO_BEACON:    { i18nKey: 'fsm_goto_beacon',  dot: 'text-violet-400',   ring: 'ring-violet-400/40 bg-violet-500/10' },
    };

    function updateMissionStatus(d) {
      /* ── FSM banner: single exclusive state from the backend ───────────── */
      const fsm  = d.fsm_state || (d.mode === 'AUTO' ? 'AUTO_PATROL' : 'MANUAL');
      // état inconnu (nouvelle version backend + vieux js en cache) :
      // afficher l'état BRUT en rouge plutôt que de mentir avec MANUAL
      const info = FSM_LABELS[fsm]
        || { i18nKey: null, raw: fsm, dot: 'text-alert', ring: 'ring-alert/40 bg-alert/15' };
      const banner = $('fsmBanner');
      setTxt('fsmState', info.i18nKey ? T(info.i18nKey) : String(info.raw).replace(/_/g, ' '));
      const fsmDot = $('fsmDot');
      if (fsmDot) fsmDot.className = 'led-dot ' + info.dot;
      if (banner)
        banner.className = 'mb-2 flex items-center justify-center gap-2 rounded-lg px-3 py-2 ring-1 transition ' + info.ring;

      /* ── Heartbeat banners ────────────────────────────────────────────── */
      const hb = $('hbFailsafe');
      if (hb) hb.classList.toggle('hidden', !(d.heartbeat && d.heartbeat.lost));
      // Stale: armed=false (backend reset it) OR age_s > HEARTBEAT_TIMEOUT
      const hbs = $('hbStale');
      if (hbs) {
        const stale = d.heartbeat && (
          !d.heartbeat.armed ||
          (d.heartbeat.age_s != null && d.heartbeat.age_s > 1.5)
        );
        hbs.classList.toggle('hidden', !stale);
      }

      setTxt('msMode', d.mode);
      const msBadge = $('msModeState');
      if (msBadge) {
        msBadge.textContent = d.mode;
        msBadge.className = 'ms-badge ' + (d.mode === 'AUTO' ? 'auto' : d.mode === 'STOP' ? 'alert' : 'manual');
      }
      setTxt('msSubstate', d.auto_state || '—');
      const asBadge = $('msAutoState');
      if (asBadge) {
        const sub = AUTO_STATES_LABELS[d.auto_state];
        asBadge.textContent = (sub ? sub.label : d.auto_state) || T('ops_ms_idle');
        asBadge.className = 'ms-badge text-[9px] ' + (d.mode === 'AUTO' && sub ? sub.cls : 'manual');
      }
    }

    /* ══════════════════════════════════════════════════════════════════════
       VIDEO CANVAS OVERLAY — differentiated target tracking visualisation.

       Rendering is split between two layers:
         • MJPEG stream  (backend)  — spotlight isolation (dim + colour restore)
         • Canvas overlay (here)    — measurement markers + HUD

       Two public drawing functions (called from the RAF loop):
         drawChessboardOverlay(ctx, lm, vr, fw, fh, dpr)
         drawLedReticles(ctx, lr, vr, fw, fh, dpr)
       Plus a HUD strip at canvas bottom.
       ══════════════════════════════════════════════════════════════════════ */
    const vcv   = $('videoCanvas');
    let   vcCtx = vcv ? vcv.getContext('2d') : null;
    let   lastDet    = null;
    let   lastMeta   = null;   /* {vision_mode, lm_miss, lm_hits, led_stable} */
    let   _telLatMs  = 0;      /* telemetry age in ms when RAF fires */
    let   _telRxTs   = 0;      /* performance.now() when last telemetry arrived */
    let   _rafLastTs = 0;      /* throttle: last RAF tick that actually drew */

    /* ── Canvas resize (pixel-perfect DPR scaling) ─────────────────── */
    function resizeVideoCanvas() {
      const container = vcv && vcv.parentElement;
      if (!container) return;
      vcv.width  = container.offsetWidth  * devicePixelRatio;
      vcv.height = container.offsetHeight * devicePixelRatio;
      vcv.style.width  = container.offsetWidth  + 'px';
      vcv.style.height = container.offsetHeight + 'px';
    }
    if (vcv) {
      resizeVideoCanvas();
      window.addEventListener('resize', () => {
        clearTimeout(vcv._rt); vcv._rt = setTimeout(resizeVideoCanvas, 150);
      });
    }

    /* ── Coordinate helpers ─────────────────────────────────────────── */
    /* Returns the pixel rect of the letterboxed video inside the canvas.
       fw/fh — actual camera frame dimensions (drives aspect ratio calculation). */
    function getVideoRect(fw, fh) {
      if (!vcv) return null;
      const cW = vcv.width / devicePixelRatio;
      const cH = vcv.height / devicePixelRatio;
      if (!cW || !cH) return null;   /* canvas not yet laid out */
      const vidAr = (fw && fh) ? fw / fh : 4 / 3;
      const conAr = cW / cH;
      let vW, vH, vX, vY;
      if (vidAr > conAr) {
        /* wider than container → letterbox top/bottom */
        vW = cW; vH = cW / vidAr; vX = 0; vY = (cH - vH) / 2;
      } else {
        /* taller than container → pillarbox left/right */
        vH = cH; vW = cH * vidAr; vX = (cW - vW) / 2; vY = 0;
      }
      return { x: vX * devicePixelRatio, y: vY * devicePixelRatio,
               w: vW * devicePixelRatio, h: vH * devicePixelRatio };
    }

    /* Maps image-space (ix, iy) in a fw×fh frame to canvas physical pixels. */
    function imgToPx(ix, iy, vr, fw, fh) {
      return [vr.x + (ix / fw) * vr.w,
              vr.y + (iy / fh) * vr.h];
    }

    /* ════════════════════════════════════════════════════════════════════
       drawChessboardOverlay — Main Target (damier)
       Color: green (#00dc9a) if confidence ≥ 0.6, orange (#ff8c00) otherwise.
       Draws: grid dots on all inner corners · topology quad · dashed bbox
              · centre crosshair · distance / confidence annotation.
       ════════════════════════════════════════════════════════════════════ */
    function drawChessboardOverlay(ctx, lm, vr, fw, fh, dpr) {
      if (!lm || !lm.valid) return;
      const conf   = lm.confidence || 0;
      const isLock = conf >= 0.6;
      const isSrch = conf >= 0.1;
      const col    = isLock ? '#00dc9a' : '#ff8c00';

      ctx.save();
      /* Full alpha when locked, dimmer while searching, faint on first frames */
      ctx.globalAlpha = isLock ? 0.92 : (isSrch ? 0.70 : 0.40);

      /* ── Grid dots on detected inner corners (rainbow by index) ───── */
      const corners = lm.corners || [];
      const nCorners = Math.max(corners.length, 1);
      corners.forEach(([ix, iy], i) => {
        const [px, py] = imgToPx(ix, iy, vr, fw, fh);
        ctx.fillStyle = `hsl(${Math.round((i / nCorners) * 270)},100%,55%)`;
        ctx.beginPath();
        ctx.arc(px, py, 2.5 * dpr, 0, Math.PI * 2);
        ctx.fill();
      });

      /* ── Full board outline (physical boundary, 1 square beyond inner corners) ── */
      const boardPts = lm.board_corners;   // TL, TR, BR, BL in image coords
      if (boardPts && boardPts.length === 4) {
        ctx.strokeStyle = col;
        ctx.lineWidth   = 2 * dpr;
        ctx.setLineDash([]);
        ctx.beginPath();
        const [bpx0, bpy0] = imgToPx(boardPts[0][0], boardPts[0][1], vr, fw, fh);
        ctx.moveTo(bpx0, bpy0);
        for (let i = 1; i < 4; i++) {
          const [xi, yi] = imgToPx(boardPts[i][0], boardPts[i][1], vr, fw, fh);
          ctx.lineTo(xi, yi);
        }
        ctx.closePath();
        ctx.stroke();
      } else {
        /* Fallback: topology quad on inner corners */
        const outer = lm.outer_corners;
        if (outer && outer.length === 4) {
          ctx.strokeStyle = col;
          ctx.lineWidth   = 2 * dpr;
          ctx.setLineDash([]);
          ctx.beginPath();
          const [x0, y0] = imgToPx(outer[0][0], outer[0][1], vr, fw, fh);
          ctx.moveTo(x0, y0);
          for (let i = 1; i < 4; i++) {
            const [xi, yi] = imgToPx(outer[i][0], outer[i][1], vr, fw, fh);
            ctx.lineTo(xi, yi);
          }
          ctx.closePath();
          ctx.stroke();
        }
      }

      /* ── Centre crosshair ──────────────────────────────────────────── */
      if (lm.center) {
        const [cx, cy] = imgToPx(lm.center[0], lm.center[1], vr, fw, fh);
        const arm = 16 * dpr, gap = 4 * dpr;
        ctx.strokeStyle = col;
        ctx.lineWidth   = 2 * dpr;
        ctx.beginPath();
        ctx.moveTo(cx - arm, cy); ctx.lineTo(cx - gap, cy);
        ctx.moveTo(cx + gap, cy); ctx.lineTo(cx + arm, cy);
        ctx.moveTo(cx, cy - arm); ctx.lineTo(cx, cy - gap);
        ctx.moveTo(cx, cy + gap); ctx.lineTo(cx, cy + arm);
        ctx.stroke();
      }

      /* ── Annotation: distance + flags ──────────────────────────────── */
      if (lm.bbox) {
        const [bx0, by0] = imgToPx(lm.bbox[0], lm.bbox[1], vr, fw, fh);
        const pose3d = lm.pose3d;
        const dist   = (pose3d && pose3d.reliable) ? pose3d.distance_m : lm.dist_est;
        const parts  = [];
        if (dist   != null)     parts.push(dist.toFixed(2) + ' m');
        if (lm.active_search)   parts.push('[AS]');
        if (!isLock)            parts.push('~' + Math.round(conf * 100) + '%');
        if (parts.length) {
          ctx.font         = `bold ${10 * dpr}px "JetBrains Mono", monospace`;
          ctx.fillStyle    = col;
          ctx.textAlign    = 'left';
          ctx.textBaseline = 'bottom';
          ctx.fillText(parts.join('  '), bx0, by0 - 4 * dpr);
        }
        /* Checker size (top-right corner of bbox) */
        if (lm.checker_size) {
          const [bx1] = imgToPx(lm.bbox[2], lm.bbox[1], vr, fw, fh);
          ctx.font         = `${9 * dpr}px "JetBrains Mono", monospace`;
          ctx.fillStyle    = 'rgba(255,255,255,0.55)';
          ctx.textAlign    = 'right';
          ctx.textBaseline = 'bottom';
          ctx.fillText(lm.checker_size[0] + '×' + lm.checker_size[1],
                       bx1, by0 - 4 * dpr);
        }
      }

      ctx.restore();
    }

    /* ════════════════════════════════════════════════════════════════════
       drawLedReticles — Secondary Targets (4 LEDs)
       Each LED: precision crosshair with centre gap + ring + "LED N" label.
       LEDs are sorted left-to-right and numbered 1–4 for diagnostics.
       ════════════════════════════════════════════════════════════════════ */
    function drawLedReticles(ctx, lr, vr, fw, fh, dpr) {
      if (!lr) return;
      /* Only show cluster_pos — these are the confirmed tight-group beacon LEDs.
         leds_pos is NOT used as fallback: it contains ALL bright spots in the scene
         (ceiling lights, reflections) which produces false-positive markers. */
      const leds = lr.cluster_pos || [];
      if (!leds.length) return;

      /* Sort by X so LED numbers are stable across frames */
      const sorted = leds
        .map((pos, i) => ({ pos, i }))
        .sort((a, b) => a.pos[0] - b.pos[0]);

      sorted.forEach(({ pos: [ix, iy] }, rank) => {
        const [px, py] = imgToPx(ix, iy, vr, fw, fh);
        const n        = rank + 1;
        const arm      = 11 * dpr;
        const gap      = 4  * dpr;
        const ring     = 6  * dpr;

        ctx.save();

        /* Crosshair */
        ctx.strokeStyle = '#ff4400';
        ctx.lineWidth   = 1.5 * dpr;
        ctx.beginPath();
        ctx.moveTo(px - arm, py); ctx.lineTo(px - gap, py);
        ctx.moveTo(px + gap, py); ctx.lineTo(px + arm, py);
        ctx.moveTo(px, py - arm); ctx.lineTo(px, py - gap);
        ctx.moveTo(px, py + gap); ctx.lineTo(px, py + arm);
        ctx.stroke();

        /* Centre dot */
        ctx.fillStyle = '#ff4400';
        ctx.beginPath();
        ctx.arc(px, py, 2 * dpr, 0, Math.PI * 2);
        ctx.fill();

        /* Outer ring */
        ctx.strokeStyle = 'rgba(255,80,0,0.75)';
        ctx.beginPath();
        ctx.arc(px, py, ring, 0, Math.PI * 2);
        ctx.stroke();

        /* "LED N" label below reticle */
        ctx.font         = `bold ${8.5 * dpr}px "JetBrains Mono", monospace`;
        ctx.fillStyle    = '#ff7755';
        ctx.textAlign    = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText('LED ' + n, px, py + ring + 2 * dpr);

        ctx.restore();
      });
    }

    /* ════════════════════════════════════════════════════════════════════
       drawHUD — Status bar at canvas bottom (NASA-grade)
       Shows: DETECTION STATUS · CB state · LED count · pose data
       Corner: CV pipeline latency
       ════════════════════════════════════════════════════════════════════ */
    function drawHUD(ctx, det, meta, latMs, cW, cH, dpr) {
      const BAR_H = 22 * dpr;
      const PAD   =  8 * dpr;
      const y0    = cH - BAR_H;

      ctx.save();

      /* Background strip */
      ctx.fillStyle = 'rgba(0,0,0,0.72)';
      ctx.fillRect(0, y0, cW, BAR_H);

      /* No telemetry yet */
      if (!det) {
        ctx.font      = `bold ${9.5 * dpr}px "JetBrains Mono", monospace`;
        ctx.fillStyle = 'rgba(255,255,255,0.30)';
        ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
        ctx.fillText('DETECTION STATUS: NO DATA', PAD, y0 + BAR_H / 2);
        ctx.restore();
        return;
      }

      const lm    = det.map_landmark || {};
      const lr    = det.led_reset    || {};
      const conf  = lm.confidence || 0;
      const ledCt = (lr.cluster_pos || []).length;
      const allLed= (lr.leds_pos    || []).length;
      const pose  = lm.pose3d;

      /* CB state */
      let cbState, cbCol;
      if (!lm.valid || conf < 0.1)  { cbState = 'LOST';  cbCol = '#ff3333'; }
      else if (conf >= 0.6)          { cbState = 'LOCK';  cbCol = '#00dc9a'; }
      else                           { cbState = 'SRCH';  cbCol = '#ff8c00'; }
      if (lm.active_search)          { cbState = 'SCAN+'; }

      /* FSM diagnostic */
      const vMode    = meta ? meta.vision_mode : 'LED';
      const lmMiss   = meta ? meta.lm_miss     : 0;
      const lmHits   = meta ? meta.lm_hits     : 0;
      const ledStbl  = meta ? meta.led_stable  : 0;

      const midY = y0 + BAR_H / 2;
      ctx.textBaseline = 'middle';

      /* FSM mode pill — left anchor */
      ctx.font      = `bold ${9.5 * dpr}px "JetBrains Mono", monospace`;
      ctx.fillStyle = vMode === 'LANDMARK' ? 'rgba(80,180,255,0.80)' : 'rgba(255,200,60,0.70)';
      ctx.textAlign = 'left';
      const modeStr = vMode === 'LANDMARK'
        ? 'LM·' + (lmMiss > 0 ? 'miss:' + lmMiss : 'hits:' + lmHits)
        : 'LED·stbl:' + ledStbl;
      ctx.fillText(modeStr, PAD, midY);
      let xCursor = PAD + ctx.measureText(modeStr).width + 8 * dpr;

      /* Separator */
      ctx.fillStyle = 'rgba(255,255,255,0.22)';
      ctx.fillText('·', xCursor, midY);
      xCursor += ctx.measureText('·').width + 8 * dpr;

      /* CB state pill */
      ctx.fillStyle = cbCol;
      ctx.fillText(cbState, xCursor, midY);
      xCursor += ctx.measureText(cbState).width + 8 * dpr;

      /* Separator */
      ctx.fillStyle = 'rgba(255,255,255,0.22)';
      ctx.fillText('·', xCursor, midY);
      xCursor += ctx.measureText('·').width + 8 * dpr;

      /* LED count */
      const ledStr = ledCt + '/' + allLed + ' LED' + (allLed !== 1 ? 's' : '');
      ctx.fillStyle = ledCt > 0 ? '#ff9955' : 'rgba(255,255,255,0.32)';
      ctx.fillText(ledStr, xCursor, midY);
      xCursor += ctx.measureText(ledStr).width + 8 * dpr;

      /* Pose3D quick read */
      if (pose && pose.reliable) {
        ctx.fillStyle = 'rgba(255,255,255,0.22)';
        ctx.fillText('·', xCursor, midY);
        xCursor += ctx.measureText('·').width + 8 * dpr;
        ctx.fillStyle = '#00dc9a';
        const pStr = pose.distance_m.toFixed(2) + 'm  Y:' + pose.yaw_deg.toFixed(1) + '°';
        ctx.fillText(pStr, xCursor, midY);
      }

      /* CV latency + frame resolution — top-right */
      const fw2 = det.frame_w || 640, fh2 = det.frame_h || 480;
      const latStr = fw2 + '×' + fh2 + '  CV: ' + Math.round(latMs) + 'ms';
      ctx.font      = `${8.5 * dpr}px "JetBrains Mono", monospace`;
      ctx.fillStyle = latMs < 120 ? 'rgba(160,255,180,0.55)' : 'rgba(255,160,60,0.70)';
      ctx.textAlign = 'right';
      ctx.fillText(latStr, cW - PAD, midY);

      ctx.restore();
    }

    /* ── updateVideoCanvas — bridges onTelemetry → canvas state ──────── */
    function updateVideoCanvas(d) {
      if (!d || !d.detection) return;
      lastDet  = d.detection;
      lastMeta = {
        vision_mode: d.vision_mode || 'LED',
        lm_miss:     d.lm_miss     || 0,
        lm_hits:     d.lm_hits     || 0,
        led_stable:  d.led_stable  || 0,
      };
      _telRxTs  = performance.now();
      /* Telemetry age: diff between wall-clock now and the server timestamp.
         d.t is the backend unix time (float seconds); multiply by 1000 for ms.
         Clamp to 0 to avoid negative values from clock skew. */
      _telLatMs = d.t ? Math.max(0, Date.now() - d.t * 1000) : 0;
    }

    /* ══════════════════════════════════════════════════════════════════════
       RENDER LOOP — strictly isolated from the control pipeline.
       This RAF loop NEVER reads or writes any variable used by sendCommand,
       startDrive, stopDrive, or the keyboard/gamepad handlers.
       Control commands flow independently via WebSocket (rosbridge).
       ══════════════════════════════════════════════════════════════════════ */
    const _WD_TIMEOUT_MS = 500;   /* watchdog threshold — 500 ms of WS silence */

    function _drawWatchdogAlert(ctx, cW, cH, dpr, ageMs) {
      /* Full-width critical banner — two sub-cases distinguished:
         • WS down   → "WS DISCONNECTED" (rosbridge not reachable)
         • TEL lost  → "TELEMETRY LOST"  (WS alive but backend silent > 500 ms) */
      const wsOk = connected;   /* roslib `connected` flag in outer closure */
      const label = wsOk
        ? `⚠  CRITICAL: TELEMETRY LOST  (${Math.round(ageMs)} ms)  ⚠`
        : `⚠  CRITICAL: WS DISCONNECTED — reconnect rosbridge  ⚠`;
      const BH = 28 * dpr, y0 = (cH - BH) / 2;
      ctx.save();
      ctx.fillStyle = wsOk ? 'rgba(160,60,0,0.88)' : 'rgba(180,0,0,0.88)';
      ctx.fillRect(0, y0, cW, BH);
      ctx.font         = `bold ${10 * dpr}px "JetBrains Mono", monospace`;
      ctx.fillStyle    = '#fff';
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(label, cW / 2, y0 + BH / 2);
      ctx.restore();
    }

    function _visionRAF(ts) {
      requestAnimationFrame(_visionRAF);
      if (ts - _rafLastTs < 50) return;   /* ~20 fps cap */
      _rafLastTs = ts;
      if (!vcCtx || !vcv) return;

      /* Canvas may be 0-sized on first tick if layout hadn't run yet */
      if (vcv.width === 0 || vcv.height === 0) {
        resizeVideoCanvas();
        if (vcv.width === 0) return;
      }

      vcCtx.clearRect(0, 0, vcv.width, vcv.height);

      const dpr    = devicePixelRatio;
      const det    = lastDet;
      const ageMs  = _telRxTs ? performance.now() - _telRxTs : Infinity;
      const linkOk = ageMs < _WD_TIMEOUT_MS;

      /* HUD always drawn — control-state-independent */
      drawHUD(vcCtx, det, lastMeta, linkOk ? ageMs : 0, vcv.width, vcv.height, dpr);

      /* Marker overlays — only when we have valid telemetry */
      if (det) {
        const fw = det.frame_w || 640;
        const fh = det.frame_h || 480;
        const vr = getVideoRect(fw, fh);
        if (vr) {
          drawChessboardOverlay(vcCtx, det.map_landmark, vr, fw, fh, dpr);
          drawLedReticles(vcCtx,       det.led_reset,    vr, fw, fh, dpr);
        }
      }

      /* Watchdog banner — drawn last so it always overlays everything.
         Fires on: WS disconnected (connected===false) OR telemetry stale > 500 ms.
         Suppressed on fresh page load before first telemetry arrives (_telRxTs===0)
         AND while WS is not yet connected (wantConnected===false → user hasn't clicked). */
      const wdFire = !connected
        ? wantConnected                      /* WS lost while user wanted connection */
        : (!linkOk && _telRxTs !== 0);      /* WS alive but backend went silent */
      if (wdFire) {
        _drawWatchdogAlert(vcCtx, vcv.width, vcv.height, dpr, ageMs);
      }
    }

    /* Start RAF loop once canvas is ready */
    if (vcv) requestAnimationFrame(_visionRAF);

    /* ══════════════════════════════════════════════════════════════════════
       NETWORK LATENCY — rolling window of WS message intervals
       ══════════════════════════════════════════════════════════════════════ */
    const NET_WIN = 20;
    let latencyTs  = [];   /* circular buffer of Date.now() timestamps */
    let netBars    = null; /* DOM elements */
    let netBarData = new Array(NET_WIN).fill(0);
    let netAvgMs   = 0;    /* exposed for alert overlay */

    /* Per-condition "already fired" flags — prevent re-show on threshold jitter.
       Each flag is set true when the overlay fires for that condition, and reset
       only when the value drops BELOW the hysteresis threshold (not just the
       trigger threshold), so a temperature hovering at 80°C doesn't spam. */
    let _alertFiredCpu  = false;
    let _alertFiredBatt = false;
    let _alertFiredPing = false;

    /* ── Alert mute (localStorage persistence across page reloads) ──────── */
    const MUTE_KEY      = 'leo_mute_alert_until';   // localStorage key
    const MUTE_DURATION = 3600 * 1000;              // 1 hour in ms

    function _isMuted() {
      return Date.now() < parseInt(localStorage.getItem(MUTE_KEY) || '0', 10);
    }

    function _muteUntilDate() {
      const ts = parseInt(localStorage.getItem(MUTE_KEY) || '0', 10);
      if (!ts || Date.now() >= ts) return null;
      return new Date(ts);
    }

    function _updateMuteIndicator() {
      const pill  = $('alertMuteStatus');
      const label = $('alertMuteUntil');
      if (!pill) return;
      const until = _muteUntilDate();
      if (until) {
        const hh = String(until.getHours()).padStart(2, '0');
        const mm = String(until.getMinutes()).padStart(2, '0');
        const timeStr = `${hh}:${mm}`;
        const tpl = T('ops_alert_muted_until');
        if (label) label.textContent = tpl.replace('{t}', timeStr);
        pill.classList.remove('hidden');
      } else {
        pill.classList.add('hidden');
      }
    }

    /* Refresh the pill every 30s in case the mute expires between WS messages */
    setInterval(_updateMuteIndicator, 30000);
    _updateMuteIndicator();   /* run once on page load */

    /* ── Auto stream degradation when latency > 200ms ────────────────────── */
    const DEGRADE_MS   = 100;   /* ms — trigger degradation (3 consecutive hits) */
    const RECOVER_MS   = 60;    /* ms — recovery threshold (3 consecutive hits) */
    const DEGRADE_HITS = 3;     /* consecutive measurements before switching mode */
    let _streamDegraded = false;
    let _degradeHits    = 0;
    let _recoverHits    = 0;

    function _applyStreamQuality(degrade) {
      const host = _currentHost || localStorage.getItem('leo_host') || '10.0.0.10';
      const v    = $('videoStream');
      if (!v || !host) return;
      v.src = degrade ? videoStreamURL(host, 25, 320) : videoStreamURL(host, 50, null);
      const badge = $('streamDegBadge');
      if (badge) badge.classList.toggle('hidden', !degrade);
    }

    function _checkStreamQuality(avg) {
      if (!_streamDegraded) {
        if (avg > DEGRADE_MS) {
          _recoverHits = 0;
          if (++_degradeHits >= DEGRADE_HITS) {
            _streamDegraded = true;
            _degradeHits    = 0;
            _applyStreamQuality(true);
          }
        } else {
          _degradeHits = 0;
        }
      } else {
        if (avg < RECOVER_MS) {
          _degradeHits = 0;
          if (++_recoverHits >= DEGRADE_HITS) {
            _streamDegraded = false;
            _recoverHits    = 0;
            _applyStreamQuality(false);
          }
        } else {
          _recoverHits = 0;
        }
      }
    }

    function initNetworkUI() {
      const spark = $('netSparkline');
      if (!spark) return;
      spark.innerHTML = '';
      netBars = [];
      for (let i = 0; i < NET_WIN; i++) {
        const d = document.createElement('div');
        d.className = 'spark-bar flex-1 bg-zinc-700/50';
        d.style.height = '4px';
        spark.appendChild(d);
        netBars.push(d);
      }
    }
    initNetworkUI();

    function recordLatency() {
      const now = Date.now();
      latencyTs.push(now);
      if (latencyTs.length > NET_WIN + 1) latencyTs.shift();

      if (latencyTs.length < 2) return;

      /* Compute all consecutive intervals */
      const intervals = [];
      for (let i = 1; i < latencyTs.length; i++) {
        intervals.push(latencyTs[i] - latencyTs[i - 1]);
      }
      netBarData = intervals.slice(-NET_WIN);

      const avg = intervals.reduce((a, b) => a + b, 0) / intervals.length;
      netAvgMs = avg;
      const min = Math.min(...intervals);
      const max = Math.max(...intervals);
      const latest = intervals[intervals.length - 1];

      /* Update displays */
      const msEl    = $('netMs');
      const badgeEl = $('netBadge');
      const minEl   = $('netMin');
      const avgEl   = $('netAvg');
      const maxEl   = $('netMax');
      const railEl  = $('railNet');

      if (msEl) msEl.textContent = Math.round(latest);
      if (minEl) minEl.textContent = Math.round(min) + ' ms';
      if (avgEl) avgEl.textContent = Math.round(avg) + ' ms';
      if (maxEl) maxEl.textContent = Math.round(max) + ' ms';
      if (railEl) railEl.textContent = Math.round(avg);

      /* Color thresholds: ≤120ms nominal, ≤200ms degraded, >200ms critical */
      let badgeClass, badgeText, barColor;
      if (avg <= 120) {
        badgeClass = 'net-badge bg-emerald-500/15 text-emerald-300';
        badgeText  = T('ops_net_nominal');
        barColor   = '#34d399';
      } else if (avg <= 200) {
        badgeClass = 'net-badge bg-amber-500/15 text-amber-300';
        badgeText  = T('ops_net_degraded');
        barColor   = '#f59e0b';
      } else {
        badgeClass = 'net-badge bg-alert/15 text-alert';
        badgeText  = T('ops_net_critical');
        barColor   = '#FC3D21';
      }
      if (badgeEl) { badgeEl.className = badgeClass; badgeEl.textContent = badgeText; }

      /* Update sparkline */
      if (netBars) {
        const maxBar = Math.max(300, max);
        netBars.forEach((bar, i) => {
          const val = netBarData[i] || 0;
          const h   = Math.max(4, Math.round((val / maxBar) * 40));
          bar.style.height    = h + 'px';
          bar.style.background = val > 200 ? '#FC3D21' : val > 120 ? '#f59e0b' : barColor;
          bar.style.opacity   = (0.4 + (i / NET_WIN) * 0.6).toFixed(2);
        });
      }

      /* Auto-degrade stream resolution when latency is critical */
      _checkStreamQuality(avg);
    }

    /* ══════════════════════════════════════════════════════════════════════
       CPU & THERMAL
       ══════════════════════════════════════════════════════════════════════ */
    /* CPU arc radius 45, circumference = 2π × 45 ≈ 283 */
    const CPU_CIRC = 283;
    let cpuCoreEls = null;

    function initCpuCores(n) {
      const row = $('cpuCoresRow');
      if (!row) return;
      row.innerHTML = '';
      cpuCoreEls = [];
      for (let i = 0; i < Math.min(n, 16); i++) {
        const wrap = document.createElement('div');
        wrap.className = 'flex flex-col items-center gap-0.5';
        wrap.style.flex = '1';
        const bar = document.createElement('div');
        bar.style.cssText = 'width:100%;border-radius:2px;background:#3b74e4;transition:height .4s ease;height:4px;';
        const lbl = document.createElement('span');
        lbl.style.cssText = 'font-family:JetBrains Mono,monospace;font-size:7px;color:#52525b;';
        lbl.textContent = i;
        wrap.appendChild(bar); wrap.appendChild(lbl);
        row.appendChild(wrap);
        cpuCoreEls.push(bar);
      }
    }

    function updateSysinfo(si) {
      const cpuPct  = si.cpu_pct;
      const cpuTemp = si.cpu_temp;
      const perCore = si.cpu_per_core || [];

      /* Arc */
      const arcEl = $('cpuArc');
      if (arcEl && cpuPct != null) {
        const offset = CPU_CIRC - (cpuPct / 100) * CPU_CIRC;
        arcEl.style.strokeDashoffset = offset;
        arcEl.style.stroke = cpuPct > 85 ? '#FC3D21' : cpuPct > 60 ? '#f59e0b' : '#3b74e4';
      }
      setTxt('cpuPct', cpuPct != null ? Math.round(cpuPct) + '%' : '—');
      setTxt('railCpu', cpuPct != null ? Math.round(cpuPct) : '—');

      /* Temperature bar (0–100°C) */
      const tempBar = $('cpuTempBar');
      if (tempBar && cpuTemp != null) {
        const pct = Math.min(100, (cpuTemp / 100) * 100);
        tempBar.style.width = pct + '%';
        tempBar.className = 'h-full rounded-full transition-all duration-500 '
          + (cpuTemp > 80 ? 'bg-gradient-to-r from-alert to-alert'
             : cpuTemp > 60 ? 'bg-gradient-to-r from-amber-400 to-amber-500'
               : 'bg-gradient-to-r from-accent to-emerald-400');
      }
      setTxt('cpuTempVal', cpuTemp != null ? cpuTemp.toFixed(1) + ' °C' : '— °C');

      /* Per-core bars */
      if (perCore.length) {
        if (!cpuCoreEls || cpuCoreEls.length !== perCore.length) initCpuCores(perCore.length);
        if (cpuCoreEls) {
          perCore.forEach((pct, i) => {
            if (!cpuCoreEls[i]) return;
            const h = Math.max(4, Math.round((pct / 100) * 32));
            cpuCoreEls[i].style.height = h + 'px';
            cpuCoreEls[i].style.background = pct > 85 ? '#FC3D21' : pct > 60 ? '#f59e0b' : '#3b74e4';
          });
        }
      }

      /* N/A message */
      const naMsg = $('cpuNaMsg');
      if (naMsg) naMsg.classList.toggle('hidden', cpuPct != null);
    }

    /* ══════════════════════════════════════════════════════════════════════
       ARTIFICIAL HORIZON
       ══════════════════════════════════════════════════════════════════════ */
    const hcv  = $('horizonCanvas');
    const hctx = hcv ? hcv.getContext('2d') : null;
    let imuRoll  = 0;
    let imuPitch = 0;
    let imuAge   = 0;
    const IMU_STALE_MS = 2000;

    function drawHorizon(roll, pitch, online) {
      if (!hctx || !hcv) return;
      const W = hcv.width, H = hcv.height;
      const cx = W / 2, cy = H / 2;
      const R  = Math.min(W, H) / 2;
      hctx.clearRect(0, 0, W, H);

      if (!online) {
        /* Offline — flat grey horizon */
        hctx.save();
        hctx.fillStyle = '#18181b';
        hctx.fillRect(0, 0, W, H);
        /* Faint horizon line */
        hctx.strokeStyle = 'rgba(255,255,255,.12)';
        hctx.lineWidth = 1;
        hctx.beginPath(); hctx.moveTo(0, cy); hctx.lineTo(W, cy); hctx.stroke();
        hctx.restore();
        const naMsg = $('horizonNaMsg');
        if (naMsg) naMsg.style.display = 'flex';
        return;
      }
      const naMsg = $('horizonNaMsg');
      if (naMsg) naMsg.style.display = 'none';

      /* ── Circular clip ─────────────────────────────────────────────── */
      hctx.save();
      hctx.beginPath(); hctx.arc(cx, cy, R - 2, 0, Math.PI * 2); hctx.clip();

      /* ── Sky & Ground (rotated by roll) ────────────────────────────── */
      hctx.save();
      hctx.translate(cx, cy);
      hctx.rotate(-roll);

      /* Pitch offset: 1° = R/45 px */
      const pxPerDeg = R / 45;
      const pitchOff = pitch * (180 / Math.PI) * pxPerDeg;

      /* Sky */
      hctx.fillStyle = '#0B3D91';
      hctx.fillRect(-W, -H / 2 + pitchOff, W * 2, H);
      /* Ground */
      hctx.fillStyle = '#4a2f12';
      hctx.fillRect(-W, pitchOff, W * 2, H);

      /* Horizon line */
      hctx.strokeStyle = '#ffffff';
      hctx.lineWidth = 2;
      hctx.beginPath();
      hctx.moveTo(-W, pitchOff); hctx.lineTo(W, pitchOff);
      hctx.stroke();

      /* Pitch ladder — every 10° */
      hctx.strokeStyle = 'rgba(255,255,255,.55)';
      hctx.fillStyle   = 'rgba(255,255,255,.55)';
      hctx.lineWidth   = 1;
      hctx.font        = '9px JetBrains Mono, monospace';
      hctx.textAlign   = 'right';
      for (let deg = -30; deg <= 30; deg += 10) {
        if (deg === 0) continue;
        const y = pitchOff - deg * pxPerDeg;
        const len = Math.abs(deg) >= 20 ? 20 : 14;
        hctx.beginPath();
        hctx.moveTo(-len, y); hctx.lineTo(len, y);
        hctx.stroke();
        hctx.fillText(deg + '°', -len - 2, y + 3);
      }

      hctx.restore(); /* un-rotate */

      /* ── Fixed aircraft symbol ──────────────────────────────────────── */
      hctx.strokeStyle = '#ffffff';
      hctx.lineWidth   = 2.5;
      /* Left wing */
      hctx.beginPath();
      hctx.moveTo(cx - R * 0.55, cy); hctx.lineTo(cx - R * 0.2, cy);
      hctx.lineTo(cx - R * 0.2, cy + 6); hctx.stroke();
      /* Right wing */
      hctx.beginPath();
      hctx.moveTo(cx + R * 0.55, cy); hctx.lineTo(cx + R * 0.2, cy);
      hctx.lineTo(cx + R * 0.2, cy + 6); hctx.stroke();
      /* Centre dot */
      hctx.fillStyle = '#ffffff';
      hctx.beginPath(); hctx.arc(cx, cy, 3, 0, Math.PI * 2); hctx.fill();

      /* ── Roll arc at top ────────────────────────────────────────────── */
      const arcR = R * 0.82;
      hctx.strokeStyle = 'rgba(255,255,255,.35)';
      hctx.lineWidth   = 1;
      hctx.beginPath();
      hctx.arc(cx, cy, arcR, -Math.PI * 1.1, -0.05);
      hctx.stroke();
      /* Roll tick at current angle */
      const tickAngle = -Math.PI / 2 - roll;
      hctx.strokeStyle = '#3b74e4';
      hctx.lineWidth   = 2.5;
      hctx.beginPath();
      hctx.moveTo(cx + (arcR - 6) * Math.cos(tickAngle), cy + (arcR - 6) * Math.sin(tickAngle));
      hctx.lineTo(cx + (arcR + 4) * Math.cos(tickAngle), cy + (arcR + 4) * Math.sin(tickAngle));
      hctx.stroke();

      /* ── Border ─────────────────────────────────────────────────────── */
      hctx.restore(); /* un-clip */
      hctx.strokeStyle = 'rgba(255,255,255,.1)';
      hctx.lineWidth   = 1;
      hctx.beginPath(); hctx.arc(cx, cy, R - 2, 0, Math.PI * 2); hctx.stroke();
    }

    function updateHorizonDisplay() {
      const online = (Date.now() - imuAge) < IMU_STALE_MS && imuAge > 0;
      const rollDeg  = imuRoll  * 180 / Math.PI;
      const pitchDeg = imuPitch * 180 / Math.PI;

      setTxt('imuRoll',  online ? rollDeg.toFixed(1)  + '°' : '—°');
      setTxt('imuPitch', online ? pitchDeg.toFixed(1) + '°' : '—°');

      const dot = $('imuStatusDot');
      const txt = $('imuStatusTxt');
      if (dot) dot.className = 'led-dot ' + (online ? 'text-emerald-400' : 'text-zinc-700');
      if (txt) txt.textContent = online ? '/camera/imu/data_raw · ' + Math.round(1000 / (Date.now() - imuAge + 1)) + ' Hz' : '/camera/imu/data_raw';

      drawHorizon(imuRoll, imuPitch, online);
    }

    /* Draw horizon at init + keep refreshing even without IMU data */
    drawHorizon(0, 0, false);
    setInterval(updateHorizonDisplay, 100);

    /* ══════════════════════════════════════════════════════════════════════
       WHEEL INDICATORS
       ══════════════════════════════════════════════════════════════════════ */
    const WHEEL_LABELS = ['FL', 'FR', 'RL', 'RR'];
    const WHEEL_CIRC   = 176;

    function updateWheelIndicators(d) {
      if (!d || !d.wheels) return;
      const vel = d.wheels.vel || [0, 0, 0, 0];
      const pwm = d.wheels.pwm || [0, 0, 0, 0];

      WHEEL_LABELS.forEach(function(label, i) {
        const absV = Math.abs(vel[i] || 0);
        const absP = Math.abs(pwm[i] || 0);
        const rpm  = Math.round(absV * 9.549);

        const stall    = absP > 0.50 && absV < 0.08;
        const highLoad = absP > 0.65 && !stall;
        let color, shadow;
        if (stall)         { color = '#FC3D21'; shadow = 'drop-shadow(0 0 6px rgba(252,61,33,.75))'; }
        else if (highLoad) { color = '#f59e0b'; shadow = 'drop-shadow(0 0 5px rgba(245,158,11,.65))'; }
        else               { color = '#34d399'; shadow = 'drop-shadow(0 0 4px rgba(52,211,153,.55))'; }

        const offset = WHEEL_CIRC - Math.min(absP, 1) * WHEEL_CIRC;
        const arcEl  = $('wa_' + label);
        const valEl  = $('wv_' + label);
        const pwmEl  = $('wp_' + label);
        const contEl = $('wheel_' + label);
        const ringEl = contEl ? contEl.querySelector('.wheel-ring') : null;

        if (arcEl) { arcEl.style['stroke-dashoffset'] = offset; arcEl.style.stroke = color; arcEl.style.filter = shadow; }
        if (valEl) valEl.textContent = rpm;
        if (pwmEl) pwmEl.textContent = Math.round(absP * 100) + '%';
        if (ringEl) { absV > 0.06 ? ringEl.classList.add('wheel-active') : ringEl.classList.remove('wheel-active'); }
      });
    }

    /* ══════════════════════════════════════════════════════════════════════
       CHART.JS — Velocity & Distance
       ══════════════════════════════════════════════════════════════════════ */
    const MAXPTS = 60;
    const commonOpts = (yTitle) => ({
      responsive: true, maintainAspectRatio: false, animation: false,
      plugins: { legend: { labels: { color: '#9fb3c8', boxWidth: 10, font: { size: 10 } } } },
      scales: {
        x: { display: false },
        y: {
          title: { display: true, text: yTitle, color: '#5b6b82', font: { size: 9 } },
          grid: { color: 'rgba(255,255,255,.05)' }, ticks: { color: '#5b6b82', font: { size: 9 } },
        },
      },
    });
    let velChart = null, distChart = null;
    if (window.Chart && $('chartVel')) {
      velChart = new Chart($('chartVel'), {
        type: 'line',
        data: {
          labels: [], datasets: [
            { label: 'v lin (m/s)', data: [], borderColor: '#3b74e4', backgroundColor: 'rgba(59,116,228,.12)', borderWidth: 2, pointRadius: 0, tension: .35, fill: true },
            { label: 'ω ang (rad/s)', data: [], borderColor: '#c0142e', backgroundColor: 'rgba(192,20,46,.10)', borderWidth: 2, pointRadius: 0, tension: .35, fill: true },
          ],
        }, options: commonOpts('speed'),
      });
      distChart = new Chart($('chartDist'), {
        type: 'line',
        data: {
          labels: [], datasets: [
            { label: 'beacon dist (m)', data: [], borderColor: '#67e8f9', backgroundColor: 'rgba(103,232,249,.12)', borderWidth: 2, pointRadius: 0, tension: .35, fill: true },
          ],
        }, options: commonOpts('distance'),
      });
    }

    function pushChart(d) {
      if (!velChart) return;
      [velChart, distChart].forEach(c => {
        c.data.labels.push('');
        if (c.data.labels.length > MAXPTS) c.data.labels.shift();
      });
      velChart.data.datasets[0].data.push(d.vel.lin);
      velChart.data.datasets[1].data.push(d.vel.ang);
      velChart.data.datasets.forEach(ds => { if (ds.data.length > MAXPTS) ds.data.shift(); });
      distChart.data.datasets[0].data.push(d.detection.dist);
      if (distChart.data.datasets[0].data.length > MAXPTS) distChart.data.datasets[0].data.shift();
      velChart.update('none'); distChart.update('none');
    }

    /* ══════════════════════════════════════════════════════════════════════
       MISSION MAP — Mission Control, DPR-aware, auto-scale, 60 FPS
       ══════════════════════════════════════════════════════════════════════ */

    /* ── Unit test : round-trip pixel ↔ monde, tolérance < 0.001 % ──────── */
    function _mapUnitTest(toPx, fromPx) {
      const cases = [[0,0],[1,0],[0,1],[-1,-1],[5.5,-3.2],[47.8,-91.3]];
      let worst = 0;
      for (const [wx, wy] of cases) {
        const [px, py] = toPx(wx, wy);
        const [wx2, wy2] = fromPx(px, py);
        const ref = Math.max(Math.abs(wx), Math.abs(wy), 1e-9);
        worst = Math.max(worst,
          Math.abs(wx2 - wx) / ref * 100,
          Math.abs(wy2 - wy) / ref * 100);
      }
      if (worst > 0.001)
        console.warn('[MAP-TEST] round-trip error ' + worst.toExponential(3) + '% > 0.001% threshold');
      return worst;
    }

    /* ── État de la vue (lerp fluide vers auto-fit) ──────────────────────── */
    let _mapSweep  = 0;
    let _mapScale  = 38;   // px/m courant (lerp)
    let _mapVCx    = 0;    // centre de vue X en coordonnées monde
    let _mapVCy    = 0;    // centre de vue Y en coordonnées monde

    function drawMap() {
      const cv = $('mapCanvas');
      if (!cv) { requestAnimationFrame(drawMap); return; }

      /* ── DPR-aware : résolution = pixels physiques réels ────────────────── */
      const dpr  = window.devicePixelRatio || 1;
      const cssW = cv.clientWidth  || 280;
      const cssH = cv.clientHeight || 260;
      if (cv.width !== Math.round(cssW * dpr) || cv.height !== Math.round(cssH * dpr)) {
        cv.width  = Math.round(cssW * dpr);
        cv.height = Math.round(cssH * dpr);
      }
      const ctx = cv.getContext('2d');
      /* Toutes les coords de dessin en pixels CSS ; DPR géré par setTransform */
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      const W = cssW, H = cssH;

      /* ── Recompute périodique des bounds (zoom-in automatique) ──────────── */
      _mapBoundsAge++;
      if (_mapBoundsAge >= 300) { _mapFullRecompute(); _mapBoundsAge = 0; }

      /* ── Auto-fit : calcule la vue cible ──────────────────────────────── */
      const pad    = 0.25;
      const rangeX = Math.max(_mapBounds.maxX - _mapBounds.minX, 0.5);
      const rangeY = Math.max(_mapBounds.maxY - _mapBounds.minY, 0.5);
      const tgtScale = Math.min(
        (W * (1 - 2 * pad)) / rangeX,
        (H * (1 - 2 * pad)) / rangeY,
        110);
      const tgtCx = (_mapBounds.minX + _mapBounds.maxX) / 2;
      const tgtCy = (_mapBounds.minY + _mapBounds.maxY) / 2;
      const lf = 0.035;
      _mapScale += (tgtScale - _mapScale) * lf;
      _mapVCx   += (tgtCx   - _mapVCx)   * lf;
      _mapVCy   += (tgtCy   - _mapVCy)   * lf;

      const sc = _mapScale;
      const ocx = W / 2 - _mapVCx * sc;  // origine canvas en CSS px
      const ocy = H / 2 + _mapVCy * sc;

      const toPx   = (wx, wy) => [ocx + wx * sc, ocy - wy * sc];
      const fromPx = (px, py) => [(px - ocx) / sc, -(py - ocy) / sc];

      /* ── Unit test (inline, ~0.1 µs) ──────────────────────────────────── */
      _mapUnitTest(toPx, fromPx);

      /* ── Fond ──────────────────────────────────────────────────────────── */
      ctx.fillStyle = '#08080f';
      ctx.fillRect(0, 0, W, H);

      /* ── Grille adaptative avec labels de distance ─────────────────────── */
      const gridSteps = [0.25, 0.5, 1, 2, 5, 10, 25, 50, 100];
      const gridM = gridSteps.find(s => s * sc >= 38) || 100;
      const gx0 = Math.floor((_mapBounds.minX - gridM) / gridM) * gridM;
      const gx1 = Math.ceil((_mapBounds.maxX  + gridM) / gridM) * gridM;
      const gy0 = Math.floor((_mapBounds.minY - gridM) / gridM) * gridM;
      const gy1 = Math.ceil((_mapBounds.maxY  + gridM) / gridM) * gridM;

      ctx.lineWidth = 1;
      for (let gx2 = gx0; gx2 <= gx1; gx2 += gridM) {
        const [px] = toPx(gx2, 0);
        if (px < -1 || px > W + 1) continue;
        ctx.strokeStyle = Math.abs(gx2) < gridM * 0.01
          ? 'rgba(255,255,255,.20)' : 'rgba(59,116,228,.12)';
        ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke();
      }
      for (let gy2 = gy0; gy2 <= gy1; gy2 += gridM) {
        const [, py] = toPx(0, gy2);
        if (py < -1 || py > H + 1) continue;
        ctx.strokeStyle = Math.abs(gy2) < gridM * 0.01
          ? 'rgba(255,255,255,.20)' : 'rgba(59,116,228,.12)';
        ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(W, py); ctx.stroke();
      }

      /* Labels de grille */
      const [, axY] = toPx(0, 0);
      const [axX]   = toPx(0, 0);
      const lblY = Math.max(10, Math.min(axY - 3, H - 10));
      const lblX = Math.max(3,  Math.min(axX + 3, W - 30));
      ctx.font = '7px JetBrains Mono, monospace';
      for (let gx2 = gx0 + gridM; gx2 < gx1; gx2 += gridM) {
        if (Math.abs(gx2) < gridM * 0.1) continue;
        const [px] = toPx(gx2, 0);
        if (px < 12 || px > W - 12) continue;
        ctx.fillStyle = 'rgba(90,130,200,.6)';
        ctx.textAlign = 'center';
        ctx.fillText((gridM < 1 ? gx2.toFixed(2) : gx2.toFixed(0)) + 'm', px, lblY);
      }
      for (let gy2 = gy0 + gridM; gy2 < gy1; gy2 += gridM) {
        if (Math.abs(gy2) < gridM * 0.1) continue;
        const [, py] = toPx(0, gy2);
        if (py < 8 || py > H - 8) continue;
        ctx.fillStyle = 'rgba(90,130,200,.6)';
        ctx.textAlign = 'right';
        ctx.fillText((gridM < 1 ? gy2.toFixed(2) : gy2.toFixed(0)) + 'm', lblX, py - 2);
      }

      /* ── Sweep radar (clippé à un cercle pour réduire le coût GPU) ──────── */
      _mapSweep += 0.015; if (_mapSweep >= 6.2832) _mapSweep -= 6.2832;
      const swR = Math.hypot(W, H) * 0.55;
      const cgrad = ctx.createConicGradient
        ? ctx.createConicGradient(_mapSweep, W / 2, H / 2) : null;
      if (cgrad) {
        cgrad.addColorStop(0,    'rgba(59,116,228,.16)');
        cgrad.addColorStop(0.06, 'rgba(59,116,228,0)');
        cgrad.addColorStop(1,    'rgba(59,116,228,0)');
        ctx.save();
        ctx.beginPath(); ctx.arc(W / 2, H / 2, swR, 0, 6.2832); ctx.clip();
        ctx.fillStyle = cgrad; ctx.fillRect(0, 0, W, H);
        ctx.restore();
      }
      ctx.strokeStyle = 'rgba(59,116,228,.4)'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(W / 2, H / 2);
      ctx.lineTo(W / 2 + Math.cos(_mapSweep) * swR, H / 2 + Math.sin(_mapSweep) * swR);
      ctx.stroke();

      /* ── Marqueur BASE à (0, 0) ──────────────────────────────────────────── */
      {
        const [bx0, by0] = toPx(0, 0);
        ctx.save();
        ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 2;
        ctx.shadowColor = '#f59e0b'; ctx.shadowBlur  = 8;
        ctx.beginPath(); ctx.arc(bx0, by0, 7, 0, 6.2832); ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(bx0 - 5, by0); ctx.lineTo(bx0 + 5, by0);
        ctx.moveTo(bx0, by0 - 5); ctx.lineTo(bx0, by0 + 5);
        ctx.stroke();
        ctx.shadowBlur = 0;
        ctx.fillStyle  = '#f59e0b'; ctx.font = 'bold 8px JetBrains Mono';
        ctx.textAlign  = 'center';
        ctx.fillText('BASE', bx0, by0 - 12);
        ctx.restore();
      }

      /* ── Trajectoire avec dégradé temporel (plus récent = plus lumineux) ── */
      if (globalTraj.length > 1) {
        const n = globalTraj.length;
        const startIdx = Math.max(0, n - 3000);
        const SEGS = 5;
        const segSz = Math.max(1, Math.ceil((n - startIdx) / SEGS));
        for (let s = 0; s < SEGS; s++) {
          const iA = startIdx + s * segSz;
          const iB = Math.min(n, iA + segSz + 1);
          if (iA >= n) break;
          const alpha = (0.15 + (s / SEGS) * 0.85).toFixed(2);
          ctx.strokeStyle = 'rgba(59,116,228,' + alpha + ')';
          ctx.lineWidth   = 1.2 + (s / SEGS) * 0.8;
          ctx.beginPath();
          let first = true;
          for (let i = iA; i < iB; i++) {
            const p = globalTraj.get(i);
            const [px, py] = toPx(p[0], p[1]);
            first ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
            first = false;
          }
          ctx.stroke();
        }
      }

      /* ── Marqueurs permanents (/map/markers) ─────────────────────────────── */
      const tgtBid = (lastTelemetry && lastTelemetry.target_beacon_id) || null;
      let beaconIdx = 0;
      globalMapMarkers.forEach(m => {
        const [mx, my] = toPx(m.x, m.y);
        if (mx < -15 || mx > W + 15 || my < -15 || my > H + 15) {
          if (m.type === 'beacon') beaconIdx++;
          return;
        }
        ctx.save();
        if (m.type === 'beacon') {
          beaconIdx++;
          const isTarget = (tgtBid !== null && beaconIdx === tgtBid);
          const col = isTarget ? '#c084fc' : '#34d399';
          ctx.fillStyle = col; ctx.strokeStyle = col; ctx.lineWidth = isTarget ? 2.5 : 1.5;
          ctx.shadowColor = col; ctx.shadowBlur = isTarget ? 18 : 10;
          if (isTarget) {
            // Pulsing ring for target beacon
            ctx.beginPath(); ctx.arc(mx, my, 14, 0, Math.PI * 2);
            ctx.strokeStyle = 'rgba(192,132,252,0.4)'; ctx.lineWidth = 4; ctx.stroke();
            ctx.strokeStyle = col; ctx.lineWidth = 2.5;
          }
          ctx.beginPath();
          ctx.moveTo(mx, my - 11); ctx.lineTo(mx + 8, my);
          ctx.lineTo(mx, my + 11); ctx.lineTo(mx - 8, my); ctx.closePath();
          ctx.fill(); ctx.stroke();
          ctx.shadowBlur = 0;
          ctx.font = isTarget ? 'bold 9px JetBrains Mono' : 'bold 8px JetBrains Mono';
          ctx.fillStyle = col;
          ctx.textAlign = 'center'; ctx.fillText(m.label, mx, my - 15);
        } else if (m.type === 'obstacle') {
          ctx.fillStyle = '#FC3D21'; ctx.strokeStyle = '#FC3D21'; ctx.lineWidth = 1.5;
          ctx.shadowColor = '#FC3D21'; ctx.shadowBlur = 8;
          ctx.beginPath();
          ctx.moveTo(mx, my - 11); ctx.lineTo(mx + 9, my + 8); ctx.lineTo(mx - 9, my + 8);
          ctx.closePath(); ctx.fill(); ctx.stroke();
          ctx.shadowBlur = 0;
          ctx.font = 'bold 7px JetBrains Mono'; ctx.fillStyle = '#FC3D21';
          ctx.textAlign = 'center'; ctx.fillText(m.label, mx, my - 14);
        }
        ctx.restore();
      });

      /* ── Balises mission ─────────────────────────────────────────────────── */
      globalBeacons.forEach(b => {
        const [bx2, by2] = toPx(b.x, b.y);
        if (bx2 < -15 || bx2 > W + 15 || by2 < -15 || by2 > H + 15) return;
        ctx.fillStyle = '#34d399'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(bx2, by2, 7, 0, 6.2832); ctx.fill(); ctx.stroke();
        ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(bx2 - 4, by2); ctx.lineTo(bx2 + 4, by2);
        ctx.moveTo(bx2, by2 - 4); ctx.lineTo(bx2, by2 + 4);
        ctx.stroke();
        ctx.fillStyle = '#34d399'; ctx.font = 'bold 8px JetBrains Mono';
        ctx.textAlign = 'center'; ctx.fillText(b.label, bx2, by2 - 12);
      });

      /* ── Robot (point + flèche cap + coords) ─────────────────────────────── */
      const d = lastTelemetry;
      if (d && d.pose) {
        const rgx = d.pose.wx != null ? d.pose.wx : toGlobal(d.pose.x, d.pose.y)[0];
        const rgy = d.pose.wx != null ? d.pose.wy : toGlobal(d.pose.x, d.pose.y)[1];
        if (_mapValidate(rgx, rgy, 'robot_render')) {
          const [rx, ry] = toPx(rgx, rgy);
          const yaw      = (d.world_heading || 0) + (d.pose.yaw || 0);
          const hLen     = Math.max(14, sc * 0.35);

          /* Flèche cap */
          const ex2 = rx + hLen * Math.cos(yaw);
          const ey2 = ry - hLen * Math.sin(yaw);
          ctx.strokeStyle = '#fff'; ctx.lineWidth = 2.5;
          ctx.shadowColor = 'rgba(255,255,255,.4)'; ctx.shadowBlur = 5;
          ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(ex2, ey2); ctx.stroke();
          /* Tête de flèche */
          const ah = 6, aa = 0.45;
          ctx.beginPath();
          ctx.moveTo(ex2, ey2);
          ctx.lineTo(ex2 - ah * Math.cos(yaw - aa), ey2 + ah * Math.sin(yaw - aa));
          ctx.lineTo(ex2 - ah * Math.cos(yaw + aa), ey2 + ah * Math.sin(yaw + aa));
          ctx.closePath();
          ctx.fillStyle = '#fff'; ctx.shadowBlur = 0; ctx.fill();

          /* Corps robot */
          ctx.fillStyle = '#fff';
          ctx.beginPath(); ctx.arc(rx, ry, 5, 0, 6.2832); ctx.fill();
          ctx.strokeStyle = '#3b74e4'; ctx.lineWidth = 2;
          ctx.beginPath(); ctx.arc(rx, ry, 5, 0, 6.2832); ctx.stroke();

          /* Coordonnées sous le point */
          ctx.fillStyle = 'rgba(255,255,255,.65)';
          ctx.font = '7px JetBrains Mono'; ctx.textAlign = 'center';
          ctx.fillText('(' + rgx.toFixed(3) + ', ' + rgy.toFixed(3) + ')', rx, ry + 15);

          /* Si RTB actif : ligne pointillée vers BASE + distance restante */
          if (d.auto_state === 'RTB') {
            const [bxR, byR] = toPx(0, 0);
            ctx.save();
            ctx.setLineDash([5, 4]);
            ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.5;
            ctx.shadowColor = '#f59e0b'; ctx.shadowBlur = 6;
            ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(bxR, byR); ctx.stroke();
            ctx.setLineDash([]);
            ctx.shadowBlur = 0;
            ctx.fillStyle  = '#f59e0b';
            ctx.font       = 'bold 9px JetBrains Mono'; ctx.textAlign = 'center';
            const distRTB  = Math.hypot(rgx, rgy);
            ctx.fillText('RTB ' + distRTB.toFixed(2) + 'm',
              (rx + bxR) / 2, (ry + byR) / 2 - 5);
            ctx.restore();
          }
        }
      } else {
        const [ox, oy] = toPx(0, 0);
        ctx.fillStyle = 'rgba(255,255,255,.35)';
        ctx.beginPath(); ctx.arc(ox, oy, 4, 0, 6.2832); ctx.fill();
      }

      /* ── Barre d'échelle (bas gauche) ───────────────────────────────────── */
      {
        const barSteps = [0.1, 0.25, 0.5, 1, 2, 5, 10, 25, 50, 100];
        const barM  = barSteps.find(s => s * sc >= 35) || 100;
        const barPx = barM * sc;
        const sbx0  = 10, sbx1 = sbx0 + barPx, sby = H - 10;
        ctx.strokeStyle = '#67e8f9'; ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(sbx0, sby);     ctx.lineTo(sbx1, sby);
        ctx.moveTo(sbx0, sby - 3); ctx.lineTo(sbx0, sby + 3);
        ctx.moveTo(sbx1, sby - 3); ctx.lineTo(sbx1, sby + 3);
        ctx.stroke();
        ctx.fillStyle = '#67e8f9'; ctx.font = '7px JetBrains Mono'; ctx.textAlign = 'left';
        ctx.fillText(barM >= 1 ? barM + ' m' : (barM * 100).toFixed(0) + ' cm', sbx0, sby - 4);
      }

      /* ── Badge outliers (bas droit canvas + header section) ────────────────── */
      if (_mapOutlierN > 0) {
        ctx.fillStyle = 'rgba(252,61,33,.85)';
        ctx.font      = '7px JetBrains Mono'; ctx.textAlign = 'right';
        ctx.fillText('⚠ ' + _mapOutlierN + ' outlier' + (_mapOutlierN > 1 ? 's' : ''),
          W - 6, H - 6);
      }
      const _obEl = $('mapOutlierBadge');
      if (_obEl) {
        if (_mapOutlierN > 0) { _obEl.textContent = '⚠ ' + _mapOutlierN; _obEl.classList.remove('hidden'); }
        else _obEl.classList.add('hidden');
      }

      requestAnimationFrame(drawMap);
    }
    requestAnimationFrame(drawMap);

    /* ══════════════════════════════════════════════════════════════════════
       BOUTONS DE COMMANDE
       ══════════════════════════════════════════════════════════════════════ */
    document.querySelectorAll('[data-cmd]').forEach(btn => btn.addEventListener('click', () => {
      const c = btn.dataset.cmd;
      if (c === 'mode') {
        const nextMode = lastTelemetry ? (modeOrder[lastTelemetry.mode] || 'AUTO') : 'MANUEL';
        sendCommand({ action: 'set_mode', mode: nextMode });
      }
      else if (c === 'stop')  { sendCommand({ action: 'stop' }); flashAlert('#FC3D21'); }
      else if (c === 'reset') sendCommand({ action: 'reset' });
      else if (c === 'clear') { sendCommand({ action: 'clear_map' }); clearAllMapData(); }
      else if (c === 'park') {
        sendCommand({ action: 'park' });
        logLine(T('ops_js_park'), 'text-amber-300');
        flashAlert('#f59e0b');
      }
      else if (c === 'hardreset') {
        if (!confirm('Node Reset: disconnect, flush all state and reconnect?\nThe robot will stop.')) return;
        sendCommand({ action: 'stop' });
        logLine(T('ops_js_hreset'), 'text-plasma');
        resetGlobalHistory(); /* hardreset: clear traj only — map markers persist */
        setTimeout(() => { disconnect(); setTimeout(connect, 800); }, 400);
        flashAlert('#c0142e');
      }
      else if (c === 'pose_source') {
        if (lastTelemetry && !lastTelemetry.pose_source_available) {
          logLine(T('ops_pose_source_na'), 'text-alert');
          return;
        }
        // Sélection directe (2026-07-23) : chaque bouton porte sa propre
        // source (data-source="MINS"|"VINS") plutôt qu'un seul bouton qui
        // bascule vers "l'autre" — plus besoin de deviner l'état courant.
        // Ne PAS logguer "-> next" ici comme si la bascule était acquise
        // (2026-07-24) : un clic peut aussi bien ARMER que basculer
        // immédiatement (voir pose_selector.py _switch_to()) — seul le
        // message relayé par leo_backend juste après (résultat réel du
        // service) doit affirmer ce qui s'est vraiment passé.
        const next = btn.dataset.source;
        sendCommand({ action: 'set_pose_source', source: next });
        logLine('[NAV] pose source: requested ' + next + '…', 'text-violet-400');
      }
      else if (c === 'export_matlab') {
        // Si MATLAB est présent sur le PC : OUVRIR l'application directement
        // sur le tracé (demande opérateur). Sinon : repli téléchargement du .mat.
        const hasMatlab = lastTelemetry && lastTelemetry.traj_rec &&
                          lastTelemetry.traj_rec.matlab_available;
        if (hasMatlab) {
          /* pending reste VRAI : si MATLAB echoue, on veut quand meme le .mat */
          opsExportPending = true;
          opsExportLock(true);
          sendCommand({ action: 'export_matlab', open_matlab: true });
          logLine(T('ops_js_openmatlab'), 'text-accent');
        } else {
          opsExportPending = true;               // -> auto-DL du .mat au retour
          opsExportLock(true);
          sendCommand({ action: 'export_matlab' });
          logLine(T('ops_js_export'), 'text-accent');
        }
      }
      else if (c === 'traj_reset') {
        sendCommand({ action: 'traj_reset' });
        logLine(T('ops_js_trajreset'), 'text-zinc-400');
      }
    }));

    /* ── Goto Beacon button ─────────────────────────────────────────────── */
    const gotoBtn = $('gotoBeaconBtn');
    if (gotoBtn) gotoBtn.addEventListener('click', () => {
      const inp = $('beaconIdInput');
      gotoBeacon(inp ? inp.value : '');
    });
    const beaconInp = $('beaconIdInput');
    if (beaconInp) beaconInp.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') gotoBeacon(beaconInp.value);
    });

    function flashAlert(color) {
      const el = document.body;
      el.style.outline = '4px solid ' + color;
      el.style.outlineOffset = '-4px';
      setTimeout(() => { el.style.outline = ''; el.style.outlineOffset = ''; }, 220);
    }

    /* ── Masque HSV ─────────────────────────────────────────────────────── */
    let maskOn = false;
    const btnMask = $('btnMask');
    if (btnMask) btnMask.addEventListener('click', () => {
      maskOn = !maskOn;
      btnMask.classList.toggle('bg-accent/20', maskOn);
      btnMask.classList.toggle('text-accent', maskOn);
      btnMask.classList.toggle('ring-accent/40', maskOn);
      sendCommand({ action: 'set_view', mask: maskOn });
    });

    /* ── Sliders set_params ─────────────────────────────────────────────── */
    let paramTimer = null;
    function pushParams() {
      clearTimeout(paramTimer);
      paramTimer = setTimeout(() => {
        sendCommand({ action: 'set_params',
          hue_low: +$('hueLow').value, hue_high: +$('hueHigh').value,
          v_min: +$('vMin').value, minled: +$('minLed').value });
      }, 120);
    }
    const bindSlider = (id, lbl) => {
      const el = $(id); if (!el) return;
      el.addEventListener('input', () => { setTxt(lbl, el.value); pushParams(); });
    };
    bindSlider('hueLow', 'vHueLow'); bindSlider('hueHigh', 'vHueHigh');
    bindSlider('vMin', 'vVmin'); bindSlider('minLed', 'vMinled');

    /* ── Pilotage manuel ────────────────────────────────────────────────── */
    const LIN = 0.2, ANG = 0.6;
    // Boost (2026-07-24) : le D-pad/clavier envoie une vitesse FIXE,
    // indépendante des sliders de plafond (linVelSlider/angVelSlider,
    // eux publient max_lin/max_ang — un plafond, pas une consigne). Sans ce
    // toggle, remonter les sliders ne rend jamais la conduite manuelle plus
    // rapide. BOOST_LIN=0.4 m/s reprend la vitesse linéaire max officielle
    // FictionLab (v1.9) ; BOOST_ANG volontairement modeste (0.9, pas 2×ANG)
    // — l'autorité de lacet réelle de ce rover plafonne empiriquement autour
    // de ~0.4 rad/s (mesure rampe 13/07, cf. TURN_SPEED côté backend) à
    // cause du glissement des rouleaux mecanum : au-delà, la commande sature
    // sans virage réellement plus rapide. _publish() côté backend clampe de
    // toute façon tout au plafond des sliders — le boost ne peut jamais le
    // dépasser, il ne fait qu'utiliser la plage déjà autorisée.
    const BOOST_LIN = 0.4, BOOST_ANG = 0.9;
    let boostActive = false;
    let driveVec = { lin: 0, ang: 0 }, driveTimer = null;

    function setDriveHUD(active) {
      const hudM = $('hudMode'); if (!hudM) return;
      hudM.classList.toggle('ring-1',    active);
      hudM.classList.toggle('ring-accent/60', active);
      hudM.classList.toggle('bg-accent/20',   active);
    }
    function startDrive(lin, ang) {
      console.log('[CTRL:key] startDrive →', lin, ang);
      driveVec = { lin, ang };
      sendCommand({ action: 'manual', lin, ang });
      setDriveHUD(true);
      clearInterval(driveTimer);
      driveTimer = setInterval(() => sendCommand({ action: 'manual', lin: driveVec.lin, ang: driveVec.ang }), 150);
    }
    function stopDrive() {
      console.log('[CTRL:key] stopDrive');
      clearInterval(driveTimer);
      driveVec = { lin: 0, ang: 0 };
      sendCommand({ action: 'manual', lin: 0, ang: 0 });
      setDriveHUD(false);
    }
    function dirVec(name) {
      const l = boostActive ? BOOST_LIN : LIN, a = boostActive ? BOOST_ANG : ANG;
      return { up: [l, 0], down: [-l, 0], left: [0, a], right: [0, -a], stop: [0, 0] }[name];
    }
    document.querySelectorAll('[data-drive]').forEach(b => {
      const press = (e) => {
        e.preventDefault();
        if (b.dataset.drive === 'stop') { stopDrive(); return; }
        const [l, a] = dirVec(b.dataset.drive);
        startDrive(l, a);
      };
      b.addEventListener('mousedown', press); b.addEventListener('touchstart', press, { passive: false });
      b.addEventListener('mouseup', stopDrive); b.addEventListener('mouseleave', stopDrive); b.addEventListener('touchend', stopDrive);
    });
    const keyDirs = { ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right' };
    let keyHeld = null;
    document.addEventListener('keydown', (e) => {
      if (keyDirs[e.key] && keyHeld !== e.key) {
        keyHeld = e.key;
        const [l, a] = dirVec(keyDirs[e.key]);
        startDrive(l, a);
        if (window.AudioMgr) AudioMgr.play('keydown');
        e.preventDefault();
      }
    });
    document.addEventListener('keyup', (e) => {
      if (keyDirs[e.key]) { keyHeld = null; stopDrive(); e.preventDefault(); }
    });

    /* ── Boost toggle ──────────────────────────────────────────────────── */
    const btnBoost = $('btnBoost');
    function setBoost(on) {
      boostActive = on;
      if (btnBoost) {
        btnBoost.setAttribute('aria-pressed', String(on));
        btnBoost.classList.toggle('bg-plasma/30', on);
        btnBoost.classList.toggle('ring-2', on);
        btnBoost.classList.toggle('ring-plasma/60', on);
        btnBoost.classList.toggle('shadow-glow-plasma', on);
      }
      const lbl = $('btnBoostLabel');
      if (lbl) lbl.textContent = T(on ? 'ops_boost_on' : 'ops_boost_off');
      const hint = $('boostSpeedHint');
      if (hint) {
        hint.textContent = (on ? BOOST_LIN : LIN).toFixed(2) + ' m/s · '
          + (on ? BOOST_ANG : ANG).toFixed(2) + ' rad/s';
      }
      // Réarme immédiatement une direction déjà maintenue avec la nouvelle
      // vitesse (bouton cliqué pendant une flèche tenue) plutôt que d'attendre
      // un relâcher/re-presser.
      if (keyHeld && keyDirs[keyHeld]) {
        const [l, a] = dirVec(keyDirs[keyHeld]);
        startDrive(l, a);
      }
    }
    if (btnBoost) btnBoost.addEventListener('click', () => setBoost(!boostActive));
    document.addEventListener('keydown', (e) => {
      if ((e.key === 'b' || e.key === 'B')
          && !['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) {
        setBoost(!boostActive);
        e.preventDefault();
      }
    });
    setBoost(false);

    /* ── Boutons vidéo ──────────────────────────────────────────────────── */
    $('btnVideoRetry').addEventListener('click', () => startVideo($('host').value.trim() || '10.0.0.10'));

    /* Relance caméra (2026-07-20) : déclenche côté backend la même échelle
       que le watchdog (cycle rosmon, purge republish, params ré-appliqués) —
       cooldown 15 s côté backend, désactivation visuelle 20 s ici (la relance
       prend ~10-15 s, on laisse une marge avant de ré-autoriser le clic). */
    $('btnCamRecover').addEventListener('click', () => {
      if (!connected || !topics.command) return;
      const btn = $('btnCamRecover');
      btn.disabled = true;
      btn.classList.add('opacity-50', 'cursor-not-allowed');
      btn.innerHTML = '<i data-lucide="loader-2" class="h-3 w-3 animate-spin"></i> ' + T('ops_vid_recovering');
      if (window.lucide) lucide.createIcons();
      topics.command.publish(new ROSLIB.Message({ data: JSON.stringify({ action: 'recover_camera' }) }));
      setTimeout(() => {
        btn.disabled = false;
        btn.classList.remove('opacity-50', 'cursor-not-allowed');
        btn.innerHTML = '<i data-lucide="refresh-cw" class="h-3 w-3"></i> ' + T('ops_vid_recover');
        if (window.lucide) lucide.createIcons();
        startVideo($('host').value.trim() || '10.0.0.10');
      }, 20000);
    });
    $('btnConnect').addEventListener('click', () => wantConnected ? disconnect() : connect());
    $('host').addEventListener('keydown', (e) => { if (e.key === 'Enter') { disconnect(); connect(); } });

    /* ── Velocity config sliders ────────────────────────────────────────────── */
    let velTimer = null;
    function pushVelocityConfig() {
      clearTimeout(velTimer);
      velTimer = setTimeout(() => {
        const linV = parseFloat($('linVelSlider').value);
        const angV = parseFloat($('angVelSlider').value);
        if (topics.velocityConfig && connected) {
          topics.velocityConfig.publish(new ROSLIB.Message({ data: JSON.stringify({ max_lin: linV, max_ang: angV }) }));
        }
        logLine('[CFG] vel limits → lin≤' + linV.toFixed(2) + ' m/s  ang≤' + angV.toFixed(1) + ' rad/s', 'text-zinc-400');
      }, 120);
    }
    ['linVelSlider', 'angVelSlider'].forEach(id => {
      const el = $(id); if (!el) return;
      el.addEventListener('input', () => {
        setTxt('vLinVel', parseFloat($('linVelSlider').value).toFixed(2) + ' m/s');
        setTxt('vAngVel', parseFloat($('angVelSlider').value).toFixed(1) + ' rad/s');
        pushVelocityConfig();
      });
    });

    /* ── Alert overlay controls ─────────────────────────────────────────── */
    window.dismissAlert = function () {
      const overlay = $('globalAlertOverlay');
      if (!overlay) return;
      overlay.classList.add('hidden');
      overlay._dismissed = true;   /* suppress re-show until condition clears */
      overlay._sounded   = false;
    };

    window.muteAlert = function () {
      localStorage.setItem(MUTE_KEY, String(Date.now() + MUTE_DURATION));
      const overlay = $('globalAlertOverlay');
      if (overlay) {
        overlay.classList.add('hidden');
        overlay._dismissed = true;
        overlay._sounded   = false;
      }
      /* Reset fired flags so alerts re-arm after mute expires */
      _alertFiredCpu  = false;
      _alertFiredBatt = false;
      _alertFiredPing = false;
      _updateMuteIndicator();
    };

    window.unmuteAlert = function () {
      localStorage.removeItem(MUTE_KEY);
      _updateMuteIndicator();
    };

    /* ── Export mission report ───────────────────────────────────────────── */
    function _downloadJson(obj, filename) {
      const blob = new Blob([JSON.stringify(obj, null, 2)], { type: 'application/json' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = filename;
      a.click();
      URL.revokeObjectURL(a.href);
    }

    window.exportMissionReport = function () {
      const ts = new Date().toISOString();
      const report = {
        exported_at: ts,
        session_telemetry: lastTelemetry ? {
          mission_s:    lastTelemetry.mission_s,
          beacon_count: lastTelemetry.beacon_count,
          battery:      lastTelemetry.battery,
          cpu_temp:     lastTelemetry.sysinfo ? lastTelemetry.sysinfo.cpu_temp : null,
          vel_limits:   lastTelemetry.vel_limits || null,
        } : null,
        map_markers: globalMapMarkers,
        trajectory_points: globalTraj.length,
      };
      fetch('auto_entries.json?' + Date.now())
        .then(r => r.json())
        .then(data => { report.auto_log = data; })
        .catch(() => {})
        .finally(() => {
          _downloadJson(report, 'leo_mission_report_' + ts.slice(0, 19).replace(/:/g, '-') + '.json');
        });
    };

    /* ── Démarrage ──────────────────────────────────────────────────────── */
    logLine(T('ops_js_init'), 'text-accent');
    if (window.I18N) { I18N.apply(); }
    /* Rebind tout bouton potentiellement injecté dynamiquement */
    if (window.AudioMgr) setTimeout(() => AudioMgr.bindAll(), 300);
    connect();
  }

  if (document.readyState === 'loading')
    document.addEventListener('DOMContentLoaded', initCockpit);
  else initCockpit();
})();
