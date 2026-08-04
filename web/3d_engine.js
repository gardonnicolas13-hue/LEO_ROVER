/* =============================================================================
   3d_engine.js — LEO Rover · Extension 3D Space Exploration
   Florida Tech Robotics Lab · 2026
   Three.js r128 · GSAP 3.12.5 + ScrollTrigger
   Scènes : Satellite (ops.html) · Capsule spatiale (logbook.html)
   Fond   : globe filaire + anneaux, parallaxe souris (ops.html/logbook.html)
            et/ou défilement via opts.scroll (index.html, 2026-07-23 —
            initBackground() unifié, ne duplique plus ce globe séparément).
============================================================================= */
(function () {
  'use strict';

  var C = {
    BLUE:   0x3b74e4, SOFT:  0x7aa7ff, CRIMSON: 0xc0142e,
    METAL:  0x1e2a3a, GOLD:  0xc9a227, GOLD_DK: 0x0d1a3a,
    SILVER: 0x8fa0b4, WHITE: 0xcfe0ff, HEAT:    0x3a2418,
  };

  /* ── Éclairage partagé ───────────────────────────────────────────────── */
  function makeLights(scene) {
    scene.add(new THREE.AmbientLight(0x334466, 0.8));
    var sun = new THREE.DirectionalLight(C.SOFT, 1.5);
    sun.position.set(5, 7, 4);
    scene.add(sun);
    var rim = new THREE.DirectionalLight(C.CRIMSON, 0.55);
    rim.position.set(-4, -3, -4);
    scene.add(rim);
    var fill = new THREE.DirectionalLight(0x334466, 0.4);
    fill.position.set(0, -5, 3);
    scene.add(fill);
  }

  /* ── Champ de particules d'ambiance ──────────────────────────────────── */
  function makeParticles(scene, n, sp) {
    n = n || 80; sp = sp || 10;
    var pos = new Float32Array(n * 3);
    for (var i = 0; i < n; i++) {
      pos[i*3]   = (Math.random()-.5)*sp;
      pos[i*3+1] = (Math.random()-.5)*sp;
      pos[i*3+2] = (Math.random()-.5)*sp;
    }
    var geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    var pts = new THREE.Points(geo, new THREE.PointsMaterial({
      color: C.SOFT, size: 0.06, transparent: true, opacity: 0.5,
      blending: THREE.AdditiveBlending, depthWrite: false,
    }));
    scene.add(pts);
    return pts;
  }

  /* ══════════════════════════════════════════════════════════════════════
     MODÈLE SATELLITE (ops.html)
  ══════════════════════════════════════════════════════════════════════ */
  function buildSatellite() {
    var root = new THREE.Group();

    /* Corps principal */
    var bodyMat = new THREE.MeshPhongMaterial({
      color: C.METAL, emissive: 0x080e18, emissiveIntensity: 0.35, shininess: 55,
    });
    root.add(new THREE.Mesh(new THREE.BoxGeometry(1.05, 0.58, 0.72), bodyMat));

    /* Rainures décoratives */
    var grooveMat = new THREE.MeshPhongMaterial({ color: 0x2a3a55, shininess: 10 });
    [-0.18, 0.0, 0.18].forEach(function(y) {
      var m = new THREE.Mesh(new THREE.BoxGeometry(1.07, 0.022, 0.74), grooveMat);
      m.position.y = y;
      root.add(m);
    });

    /* Matériaux panneaux solaires */
    var solarMat = new THREE.MeshPhongMaterial({
      color: C.GOLD_DK, emissive: 0x060e1c, emissiveIntensity: 0.5, shininess: 95,
      side: THREE.DoubleSide,
    });
    var cellMat = new THREE.LineBasicMaterial({ color: C.GOLD, transparent: true, opacity: 0.6 });

    function makeSolarPanel(dx) {
      var grp = new THREE.Group();
      var W = 1.95, H = 0.72, D = 0.024;
      grp.add(new THREE.Mesh(new THREE.BoxGeometry(W, H, D), solarMat));
      grp.add(new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.BoxGeometry(W, H, D)),
        new THREE.LineBasicMaterial({ color: C.GOLD })));

      /* Grille cellules */
      var rows = 5, cols = 9, r, c, g;
      for (r = 0; r <= rows; r++) {
        var y = -H/2 + r*(H/rows);
        g = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(-W/2, y, D/2+0.001),
          new THREE.Vector3(W/2,  y, D/2+0.001),
        ]);
        grp.add(new THREE.LineSegments(g, cellMat));
      }
      for (c = 0; c <= cols; c++) {
        var x = -W/2 + c*(W/cols);
        g = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(x, -H/2, D/2+0.001),
          new THREE.Vector3(x,  H/2, D/2+0.001),
        ]);
        grp.add(new THREE.LineSegments(g, cellMat));
      }
      grp.position.x = dx;

      /* Bras de support */
      var arm = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, 1.06, 6),
        new THREE.MeshPhongMaterial({ color: C.SILVER }));
      arm.rotation.z = Math.PI/2;
      arm.position.x = dx > 0 ? 0.53 : -0.53;
      root.add(arm);

      return grp;
    }

    root.add(makeSolarPanel(-1.52));
    root.add(makeSolarPanel(1.52));

    /* Antenne parabolique */
    var dish = new THREE.Mesh(
      new THREE.SphereGeometry(0.30, 10, 10, 0, Math.PI*2, 0, Math.PI/1.8),
      new THREE.MeshPhongMaterial({ color: 0xd2dce8, shininess: 90, side: THREE.DoubleSide }));
    dish.rotation.x = Math.PI;
    dish.position.set(-0.08, 0.44, 0.12);
    root.add(dish);

    root.add(function() {
      var m = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.25, 5),
        new THREE.MeshPhongMaterial({ color: C.SILVER }));
      m.position.set(-0.08, 0.31, 0.12);
      return m;
    }());

    /* Antenne bâton + voyant rouge */
    root.add(function() {
      var m = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.52, 4),
        new THREE.MeshPhongMaterial({ color: C.SILVER }));
      m.position.set(0.32, 0.56, 0.0);
      return m;
    }());
    var beacon = new THREE.Mesh(new THREE.SphereGeometry(0.028, 7, 7),
      new THREE.MeshPhongMaterial({ color: 0xff3333, emissive: 0xcc0000, emissiveIntensity: 0.9 }));
    beacon.position.set(0.32, 0.84, 0.0);
    root.add(beacon);

    /* Réflecteur latéral */
    var refl = new THREE.Mesh(new THREE.CylinderGeometry(0.17, 0.17, 0.018, 8),
      new THREE.MeshPhongMaterial({ color: 0xe0e8f0, shininess: 130 }));
    refl.rotation.x = Math.PI/2;
    refl.position.set(0.32, -0.16, 0.41);
    root.add(refl);

    return { group: root, beacon: beacon };
  }

  /* ══════════════════════════════════════════════════════════════════════
     MODÈLE CAPSULE SPATIALE / MODULE LUNAIRE (logbook.html)
  ══════════════════════════════════════════════════════════════════════ */
  function buildCapsule() {
    var root = new THREE.Group();

    /* Module de commande (cône hexagonal) */
    var cmdMat = new THREE.MeshPhongMaterial({
      color: 0x9898b0, emissive: 0x08081a, emissiveIntensity: 0.22, shininess: 45,
    });
    var cmd = new THREE.Mesh(new THREE.ConeGeometry(0.62, 0.98, 6), cmdMat);
    cmd.position.y = 0.88;
    root.add(cmd);

    /* Nervures structurelles sur le cône */
    for (var ri = 0; ri < 6; ri++) {
      var ra = (ri/6)*Math.PI*2;
      var rib = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.95, 0.03),
        new THREE.MeshPhongMaterial({ color: 0x707088 }));
      rib.position.set(Math.cos(ra)*0.4, 0.88, Math.sin(ra)*0.4);
      rib.rotation.y = -ra;
      root.add(rib);
    }

    /* Hublots (2 sur le cône) */
    var winMat = new THREE.MeshPhongMaterial({
      color: C.SOFT, emissive: 0x1a3060, emissiveIntensity: 0.95,
      shininess: 220, transparent: true, opacity: 0.92,
    });
    [-0.28, 0.28].forEach(function(wx) {
      var win = new THREE.Mesh(new THREE.CircleGeometry(0.092, 8), winMat);
      win.position.set(wx, 0.88, 0.56);
      root.add(win);
    });

    /* Bouclier thermique */
    root.add(function() {
      var m = new THREE.Mesh(new THREE.CylinderGeometry(0.63, 0.63, 0.10, 6),
        new THREE.MeshPhongMaterial({ color: C.HEAT, emissive: 0x1a0c08, shininess: 18 }));
      m.position.y = 0.41;
      return m;
    }());

    /* Module de service */
    var srv = new THREE.Mesh(new THREE.CylinderGeometry(0.52, 0.52, 1.10, 6),
      new THREE.MeshPhongMaterial({ color: 0x5a6878, emissive: 0x08101a, shininess: 20 }));
    srv.position.y = -0.20;
    root.add(srv);

    /* Bandes bleutées */
    [0.10, -0.14, -0.38].forEach(function(by) {
      var band = new THREE.Mesh(new THREE.CylinderGeometry(0.535, 0.535, 0.042, 6),
        new THREE.MeshPhongMaterial({ color: C.BLUE, emissive: 0x1a3060, emissiveIntensity: 0.5 }));
      band.position.y = by;
      root.add(band);
    });

    /* Cloche moteur principale */
    var bellMat = new THREE.MeshPhongMaterial({ color: 0x9a7a5a, side: THREE.DoubleSide, shininess: 38 });
    var bell = new THREE.Mesh(new THREE.ConeGeometry(0.40, 0.58, 6, 1, true), bellMat);
    bell.rotation.x = Math.PI;
    bell.position.y = -1.05;
    root.add(bell);

    /* Buses secondaires (4×) */
    for (var bi = 0; bi < 4; bi++) {
      var ba = (bi/4)*Math.PI*2 + Math.PI/4;
      var nozz = new THREE.Mesh(new THREE.ConeGeometry(0.095, 0.34, 5, 1, true), bellMat);
      nozz.rotation.x = Math.PI;
      nozz.position.set(Math.cos(ba)*0.44, -0.90, Math.sin(ba)*0.44);
      root.add(nozz);
    }

    /* Panneaux solaires radiaux (4×) */
    var panMat = new THREE.MeshPhongMaterial({
      color: C.GOLD_DK, emissive: 0x050c1e, shininess: 55, side: THREE.DoubleSide,
    });
    for (var pi = 0; pi < 4; pi++) {
      var pa = (pi/4)*Math.PI*2;
      var pgrp = new THREE.Group();
      var panel = new THREE.Mesh(new THREE.BoxGeometry(0.88, 0.44, 0.022), panMat);
      panel.position.x = 0.54;
      pgrp.add(panel);
      pgrp.add(new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.BoxGeometry(0.88, 0.44, 0.022)),
        new THREE.LineBasicMaterial({ color: C.GOLD })));
      var parm = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.40, 4),
        new THREE.MeshPhongMaterial({ color: C.SILVER }));
      parm.rotation.z = Math.PI/2;
      parm.position.x = 0.19;
      pgrp.add(parm);
      pgrp.rotation.y = pa;
      pgrp.position.y = -0.20;
      root.add(pgrp);
    }

    /* Antenne parabolique (dessus) */
    var dish2 = new THREE.Mesh(
      new THREE.SphereGeometry(0.21, 8, 8, 0, Math.PI*2, 0, Math.PI/1.8),
      new THREE.MeshPhongMaterial({ color: 0xd2dce8, shininess: 80, side: THREE.DoubleSide }));
    dish2.rotation.x = Math.PI;
    dish2.position.set(0.16, 1.42, 0.12);
    root.add(dish2);
    root.add(function() {
      var m = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 0.18, 4),
        new THREE.MeshPhongMaterial({ color: C.SILVER }));
      m.position.set(0.16, 1.36, 0.12);
      return m;
    }());

    return { group: root };
  }

  /* ── Planete pour vignette (2026-07-30) ──────────────────────────────────
     Meme vocabulaire visuel que le globe de fond (icosaedre filaire bleu +
     anneaux), mais construit comme buildSatellite/buildCapsule pour passer
     par initScene : c'est initScene qui dimensionne sur clientWidth/Height.
     initBackground(), lui, force window.innerWidth — correct pour un fond
     plein ecran, deformant dans une tuile. D'ou ce builder plutot qu'un
     appel direct au fond.
     Rayon 1.2 : a cameraZ 5.5 et fov 44, la demi-hauteur visible vaut
     tan(22 deg)*5.5 = 2.22 ; l'anneau externe (1.7 R = 2.04) tient dedans,
     et il est vu presque par la tranche donc son extension verticale reelle
     est bien moindre. ────────────────────────────────────────────────── */
  function buildPlanet() {
    var root = new THREE.Group();
    var GR = 1.2;
    var geo = new THREE.IcosahedronGeometry(GR, 2);

    root.add(new THREE.LineSegments(
      new THREE.WireframeGeometry(geo),
      new THREE.LineBasicMaterial({ color: C.BLUE, transparent: true, opacity: 0.30 })));

    root.add(new THREE.LineSegments(
      new THREE.WireframeGeometry(new THREE.IcosahedronGeometry(GR * 0.66, 1)),
      new THREE.LineBasicMaterial({ color: C.CRIMSON, transparent: true, opacity: 0.16 })));

    root.add(new THREE.Points(geo, new THREE.PointsMaterial({
      color: C.WHITE, size: 0.038, transparent: true, opacity: 0.75,
      blending: THREE.AdditiveBlending, depthWrite: false,
    })));

    /* Halo : sphere pleine vue de l'interieur, additive — donne l'atmosphere
       sans post-traitement (pas de passe bloom sur cette page). */
    root.add(new THREE.Mesh(
      new THREE.SphereGeometry(GR * 1.14, 24, 18),
      new THREE.MeshBasicMaterial({
        color: C.SOFT, transparent: true, opacity: 0.05,
        side: THREE.BackSide, blending: THREE.AdditiveBlending, depthWrite: false,
      })));

    var ringDefs = [
      { r: GR * 1.42, color: C.BLUE,    op: 0.34, rx: Math.PI / 2.2 },
      { r: GR * 1.70, color: C.CRIMSON, op: 0.17, rx: Math.PI / 2.6, ry: 0.5 },
    ];
    for (var i = 0; i < ringDefs.length; i++) {
      var d = ringDefs[i];
      var m = new THREE.Mesh(
        new THREE.TorusGeometry(d.r, 0.006, 10, 120),
        new THREE.MeshBasicMaterial({ color: d.color, transparent: true, opacity: d.op }));
      m.rotation.x = d.rx;
      if (d.ry) m.rotation.y = d.ry;
      root.add(m);
    }
    return { group: root };
  }

  /* ══════════════════════════════════════════════════════════════════════
     MOTEUR DE RENDU GÉNÉRIQUE
  ══════════════════════════════════════════════════════════════════════ */
  function initScene(canvasId, buildFn, opts) {
    opts = opts || {};
    var canvas = typeof canvasId === 'string'
      ? document.getElementById(canvasId) : canvasId;
    if (!canvas || typeof THREE === 'undefined') return null;

    var renderer;
    try {
      renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true, alpha: true });
    } catch(e) { return null; }

    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x000000, 0);

    var scene  = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(44, 1, 0.1, 200);
    camera.position.z = opts.cameraZ || 5.5;

    makeLights(scene);
    var particles = makeParticles(scene, opts.particles || 70, opts.spread || 10);

    var built = buildFn();
    scene.add(built.group);

    function resize() {
      var w = canvas.clientWidth  || 300;
      var h = canvas.clientHeight || 300;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    }
    resize();
    window.addEventListener('resize', resize, { passive: true });

    /* Zoom progressif au défilement (GSAP ScrollTrigger) */
    var scrollState = { zoom: 0 };
    if (opts.scroll !== false && window.gsap && window.ScrollTrigger) {
      gsap.registerPlugin(ScrollTrigger);
      gsap.to(scrollState, {
        zoom: 1, ease: 'none',
        scrollTrigger: {
          trigger: document.documentElement,
          start: 'top top',
          end: 'bottom bottom',
          scrub: 0.8,
        },
      });
    }

    var clock   = new THREE.Clock();
    var baseZ   = opts.cameraZ   || 5.5;
    var zoomAmt = opts.zoomAmt   || 2.2;
    var tiltBase= opts.tiltBase  || 0.15;
    var rotSp   = opts.rotSpeed  || 0.0045;
    var raf;

    /* Scintillement du voyant (satellite uniquement) */
    var beaconMesh = built.beacon || null;
    var beaconT = 0;

    function frame() {
      raf = requestAnimationFrame(frame);
      var t = clock.getElapsedTime();
      built.group.rotation.y += rotSp;
      built.group.rotation.x = tiltBase + Math.sin(t * 0.18) * 0.04;
      particles.rotation.y   = t * 0.007;
      camera.position.z = baseZ - scrollState.zoom * zoomAmt;

      /* Clignotement voyant rouge du satellite */
      if (beaconMesh) {
        beaconT = t;
        var blink = 0.5 + 0.5 * Math.sin(beaconT * 3.8);
        beaconMesh.material.emissiveIntensity = 0.3 + blink * 0.7;
      }

      renderer.render(scene, camera);
    }
    frame();

    document.addEventListener('visibilitychange', function() {
      if (document.hidden) cancelAnimationFrame(raf);
      else frame();
    });

    return {
      renderer: renderer, scene: scene, camera: camera,
      model: built.group,
      stop: function() { cancelAnimationFrame(raf); },
      resize: resize,
    };
  }

  /* ══════════════════════════════════════════════════════════════════════
     SCÈNE DE FOND INTERACTIVE — pointer-events:none, z-index:-1
     Parallaxe souris : rotation XY + zoom Z lissé par lerp interne.
     Optimisée : FPS throttle, antialias désactivé, pixelRatio capé.
  ══════════════════════════════════════════════════════════════════════ */
  function initBackground(canvasId, opts) {
    opts = Object.assign({
      baseZ:       14,
      rotAmp:      0.28,   /* amplitude rotation max (radians)         */
      zoomRange:   2.0,    /* delta caméra Z centre↔bord               */
      lerpFactor:  0.055,  /* 0=figé · 1=instantané                    */
      targetFps:   35,     /* fps cible rendu fond                     */
      nParticles:  1200,   /* particules profondes                     */
      rings:       2,      /* anneaux orbitaux (0-3)                   */
      lite:        false,  /* mode économique (ops.html)               */
      /* Pilotage scroll (2026-07-23) : en plus de la parallaxe souris
         déjà existante ci-dessous, additionne une rotation/zoom pilotés
         par la position de défilement de la page (GSAP ScrollTrigger),
         combinés avec la souris exactement comme index.html le faisait
         dans son implémentation Three.js dédiée avant unification —
         désactivé par défaut (ops.html/logbook.html, pilotage souris
         seul, inchangé). */
      scroll:      false,
      scrollSpin:  Math.PI * 2.4,  /* rotation Y totale sur tout le scroll */
      scrollTilt:  0.5,            /* inclinaison X additionnelle max     */
      scrollZoom:  7,              /* recul caméra Z max (unités monde)   */
    }, opts || {});

    var canvas = typeof canvasId === 'string'
      ? document.getElementById(canvasId) : canvasId;
    if (!canvas || typeof THREE === 'undefined') return null;

    var renderer;
    try {
      renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: false, alpha: true });
    } catch(e) { return null; }

    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.6));
    renderer.setClearColor(0x000000, 0);

    var scene  = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(52, 1, 0.1, 240);
    camera.position.z = opts.baseZ;

    /* Éclairage minimal */
    scene.add(new THREE.AmbientLight(0x334466, 0.5));
    var dlight = new THREE.DirectionalLight(C.SOFT, 0.8);
    dlight.position.set(3, 5, 4);
    scene.add(dlight);

    /* ── Objet principal ───────────────────────────────────────────── */
    var core = new THREE.Group();
    scene.add(core);

    var GR = opts.lite ? 3.6 : 4.2;
    var icoGeo = new THREE.IcosahedronGeometry(GR, opts.lite ? 1 : 2);

    /* Globe filaire externe (Bleu NASA) */
    core.add(new THREE.LineSegments(
      new THREE.WireframeGeometry(icoGeo),
      new THREE.LineBasicMaterial({ color: C.BLUE, transparent: true, opacity: 0.28 })));

    /* Sphère interne (Crimson FIT, contre-rotation) */
    var innerMesh = null;
    if (!opts.lite) {
      innerMesh = new THREE.LineSegments(
        new THREE.WireframeGeometry(new THREE.IcosahedronGeometry(GR * 0.68, 1)),
        new THREE.LineBasicMaterial({ color: C.CRIMSON, transparent: true, opacity: 0.14 }));
      core.add(innerMesh);
    }

    /* Points aux sommets */
    core.add(new THREE.Points(icoGeo, new THREE.PointsMaterial({
      color: C.WHITE, size: 0.11, transparent: true, opacity: 0.75,
      blending: THREE.AdditiveBlending, depthWrite: false,
    })));

    /* Anneaux orbitaux */
    var ringMeshes = [];
    var ringDefs = [
      { r: GR*1.42, color: C.BLUE,    op: 0.35, rx: Math.PI/2.2 },
      { r: GR*1.70, color: C.CRIMSON, op: 0.18, rx: Math.PI/2.6, ry: 0.5 },
      { r: GR*2.00, color: C.BLUE,    op: 0.10, rx: Math.PI/1.9, rz: 0.6 },
    ];
    var nRings = Math.min(opts.rings, ringDefs.length);
    for (var ri = 0; ri < nRings; ri++) {
      var rd = ringDefs[ri];
      var rm = new THREE.Mesh(
        new THREE.TorusGeometry(rd.r, 0.011, 12, 160),
        new THREE.MeshBasicMaterial({ color: rd.color, transparent: true, opacity: rd.op }));
      rm.rotation.x = rd.rx || 0;
      if (rd.ry) rm.rotation.y = rd.ry;
      if (rd.rz) rm.rotation.z = rd.rz;
      core.add(rm);
      ringMeshes.push(rm);
    }

    /* Champ de particules profondes */
    var NP   = opts.lite ? Math.ceil(opts.nParticles * 0.5) : opts.nParticles;
    var ppos = new Float32Array(NP * 3);
    for (var pi = 0; pi < NP; pi++) {
      var pr = 16 + Math.random()*32;
      var pt = Math.random()*Math.PI*2;
      var pp = Math.acos(2*Math.random()-1);
      ppos[pi*3]   = pr*Math.sin(pp)*Math.cos(pt);
      ppos[pi*3+1] = pr*Math.sin(pp)*Math.sin(pt);
      ppos[pi*3+2] = pr*Math.cos(pp);
    }
    var pGeo = new THREE.BufferGeometry();
    pGeo.setAttribute('position', new THREE.BufferAttribute(ppos, 3));
    var dust = new THREE.Points(pGeo, new THREE.PointsMaterial({
      color: C.SOFT, size: 0.065, transparent: true, opacity: 0.38,
      blending: THREE.AdditiveBlending, depthWrite: false,
    }));
    scene.add(dust);

    /* ── Suivi souris ──────────────────────────────────────────────── */
    /* Valeurs normalisées -1..+1 */
    var mxN = 0, myN = 0;
    /* État courant (lissé) */
    var curRX = 0.0, curRY = 0.0, curZOff = 0.0;
    /* Cibles */
    var tgtRX = 0.0, tgtRY = 0.0, tgtZ = 0.0;

    var ROT_AMP    = opts.rotAmp;
    var ZOOM_RANGE = opts.zoomRange;
    var L          = opts.lerpFactor;

    function onMouseMove(e) {
      mxN = (e.clientX / window.innerWidth)  * 2.0 - 1.0;
      myN = (e.clientY / window.innerHeight) * 2.0 - 1.0;
      /* Rotation : l'objet s'incline vers le curseur */
      tgtRY =  mxN * ROT_AMP;
      tgtRX = -myN * ROT_AMP * 0.55;
      /* Zoom : centre → zoom in, bords → zoom out       */
      var dist = Math.min(Math.sqrt(mxN*mxN + myN*myN) / Math.SQRT2, 1.0);
      tgtZ = (dist - 0.5) * ZOOM_RANGE;
    }
    document.addEventListener('mousemove', onMouseMove, { passive: true });

    /* ── Pilotage scroll (optionnel, voir opts.scroll) ────────────────── */
    var scrollState = { spin: 0, tilt: 0, zoom: 0 };
    if (opts.scroll && window.gsap && window.ScrollTrigger &&
        !(window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches)) {
      gsap.registerPlugin(ScrollTrigger);
      gsap.to(scrollState, {
        spin: opts.scrollSpin, tilt: opts.scrollTilt, zoom: 1, ease: 'none',
        scrollTrigger: {
          trigger: document.documentElement,
          start: 'top top', end: 'bottom bottom', scrub: 0.6,
        },
      });
    }

    /* ── Resize ────────────────────────────────────────────────────── */
    function resize() {
      renderer.setSize(window.innerWidth, window.innerHeight, false);
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
    }
    resize();
    window.addEventListener('resize', resize, { passive: true });

    /* ── Boucle de rendu (FPS throttle) ───────────────────────────── */
    var INTERVAL = 1000.0 / opts.targetFps;
    var lastTs   = 0.0;
    var clock    = new THREE.Clock();
    var raf, visible = true;
    /* prefers-reduced-motion : on gèle la dérive AUTONOME (rotation lente,
       anneaux, particules) mais on laisse la réponse directe à la souris
       (curRX/curRY/curZOff) et au scroll (scrollState, déjà à 0 dans ce cas
       puisque le bloc GSAP ci-dessus ne s'arme pas) continuer de répondre —
       même sémantique que l'ancien index.html (seul `t` gelait). */
    var reduceMotion = !!(window.matchMedia &&
      window.matchMedia('(prefers-reduced-motion: reduce)').matches);

    function frame(ts) {
      raf = requestAnimationFrame(frame);
      if (!visible || (ts - lastTs) < INTERVAL) return;
      lastTs = ts;

      var t = reduceMotion ? 0 : clock.getElapsedTime();

      /* Lerp vers les cibles (douceur du mouvement) */
      curRX   += (tgtRX  - curRX)   * L;
      curRY   += (tgtRY  - curRY)   * L;
      curZOff += (tgtZ   - curZOff) * L;

      /* Rotation globale : dérive lente + parallaxe souris + scroll (additif) */
      core.rotation.y = t * 0.055 + curRY + scrollState.spin;
      core.rotation.x = 0.15 + curRX + scrollState.tilt * 0.5;
      if (innerMesh) {
        innerMesh.rotation.y = -t * 0.10;
        innerMesh.rotation.z =  t * 0.035;
      }
      if (ringMeshes[0]) ringMeshes[0].rotation.z =  t * 0.07;
      if (ringMeshes[1]) ringMeshes[1].rotation.z = -t * 0.05;
      if (ringMeshes[2]) ringMeshes[2].rotation.y =  t * 0.04;
      dust.rotation.y = t * 0.008;

      /* Camera Z : zoom centre-bords (souris) + recul progressif (scroll) */
      camera.position.z = opts.baseZ + curZOff - scrollState.zoom * opts.scrollZoom;

      renderer.render(scene, camera);
    }
    requestAnimationFrame(frame);

    document.addEventListener('visibilitychange', function() {
      visible = !document.hidden;
    });

    return {
      renderer: renderer, scene: scene, camera: camera,
      stop:   function() { cancelAnimationFrame(raf); },
      resize: resize,
    };
  }

  /* ── API publique ─────────────────────────────────────────────────── */
  window.LEO3D = {
    initSatellite: function(id, opts) {
      return initScene(id, buildSatellite, Object.assign({
        cameraZ: 5.5, zoomAmt: 2.0, rotSpeed: 0.004, tiltBase: 0.12,
        scroll: true, particles: 60, spread: 9,
      }, opts || {}));
    },
    initCapsule: function(id, opts) {
      return initScene(id, buildCapsule, Object.assign({
        cameraZ: 4.8, zoomAmt: 1.8, rotSpeed: 0.003, tiltBase: 0.08,
        scroll: true, particles: 50, spread: 8,
      }, opts || {}));
    },
    /* Vignette planete (ops.html, panneau « vue orbitale ») : passe par
       initScene, donc se dimensionne sur le canvas et non sur la fenetre.
       scroll:false — une vignette ne doit pas zoomer au defilement de page. */
    initPlanet: function(id, opts) {
      return initScene(id, buildPlanet, Object.assign({
        cameraZ: 5.5, zoomAmt: 0, rotSpeed: 0.0022, tiltBase: 0.30,
        scroll: false, particles: 90, spread: 12,
      }, opts || {}));
    },
    /* Fond interactif (parallaxe souris) pour ops.html et logbook.html */
    initBackground: function(id, opts) {
      return initBackground(id, opts);
    },
  };

})();
