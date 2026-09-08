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

  /* ══════════════════════════════════════════════════════════════════════
     REPÈRES CAPTEURS (mins_tuning.html) — orbite à la souris
     ──────────────────────────────────────────────────────────────────────
     Ce n'est PAS un décor : la scène montre les trois repères dont la page
     explique les transformations. T_imu_cam et T_imu_wheel sont le concept
     le plus coûteux du document (une erreur de 90° y a produit une dérive
     monotone en z que la calibration en ligne ne rattrape pas), et une
     figure orbitable est le seul support où l'on VOIT que le repère roues
     pointe à l'opposé du repère caméra — le fameux Rz(pi).

     Palette : strictement celle de mission.css. x=plasma, y=ok, z=accent.
     Aucune rotation automatique : la scène ne bouge QUE sous la souris,
     pour qu'on puisse s'arrêter sur un angle et le lire.
  ══════════════════════════════════════════════════════════════════════ */
  var F = { X: 0xff6b35, Y: 0x3fdc97, Z: 0x22d3ee, BODY: 0x2a3546, DIM: 0x6b7684 };

  /* Triade d'axes. len = longueur, w = épaisseur du trait. */
  function makeTriad(len, op) {
    var g = new THREE.Group();
    [[F.X, new THREE.Vector3(1,0,0)],
     [F.Y, new THREE.Vector3(0,1,0)],
     [F.Z, new THREE.Vector3(0,0,1)]].forEach(function (ax) {
      var col = ax[0], dir = ax[1];
      var geo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0,0,0), dir.clone().multiplyScalar(len)
      ]);
      g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({
        color: col, transparent: true, opacity: op || 0.95,
      })));
      /* pointe : un cône orienté sur l'axe */
      var cone = new THREE.Mesh(
        new THREE.ConeGeometry(len * 0.075, len * 0.2, 12),
        new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: op || 0.95 })
      );
      cone.position.copy(dir.clone().multiplyScalar(len));
      /* ConeGeometry pointe vers +y : on l'aligne sur la direction voulue */
      cone.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), dir);
      g.add(cone);
    });
    return g;
  }

  /* Trait pointillé entre deux points — matérialise une translation. */
  function makeLink(a, b, col) {
    var geo = new THREE.BufferGeometry().setFromPoints([a, b]);
    var m = new THREE.Line(geo, new THREE.LineDashedMaterial({
      color: col, dashSize: 0.09, gapSize: 0.07,
      transparent: true, opacity: 0.8,
    }));
    m.computeLineDistances();
    return m;
  }

  function buildFrames() {
    var root = new THREE.Group();

    /* ── Châssis filaire : la caisse du rover, échelle 1 unité ≈ 0.2 m ──
       Les décalages réels (0.15 m caméra, 0.03 m roues) sont AMPLIFIÉS
       pour rester lisibles à l'écran ; les DIRECTIONS, elles, sont exactes. */
    var body = new THREE.Mesh(
      new THREE.BoxGeometry(2.2, 0.62, 1.55),
      new THREE.MeshPhongMaterial({
        color: F.BODY, emissive: 0x142033, emissiveIntensity: 0.8, shininess: 40,
        transparent: true, opacity: 0.42,
      })
    );
    root.add(body);
    /* Arêtes franches : sur un panneau sombre, c'est le filaire qui porte la
       forme, pas la face. Opacité montée après rendu de contrôle — à 0.5 le
       châssis disparaissait dans le fond. */
    root.add(new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(2.2, 0.62, 1.55)),
      new THREE.LineBasicMaterial({ color: 0x93a3b8, transparent: true, opacity: 0.85 })
    ));

    /* ── Quatre roues ─────────────────────────────────────────────────── */
    [[-0.78, 0.86], [0.78, 0.86], [-0.78, -0.86], [0.78, -0.86]].forEach(function (p) {
      var w = new THREE.Mesh(
        new THREE.CylinderGeometry(0.34, 0.34, 0.17, 22),
        new THREE.MeshPhongMaterial({ color: 0x222c3a, shininess: 26 })
      );
      w.rotation.x = Math.PI / 2;
      w.position.set(p[0], -0.32, p[1]);
      root.add(w);
      root.add(new THREE.Mesh(
        new THREE.TorusGeometry(0.34, 0.018, 8, 26),
        new THREE.MeshBasicMaterial({ color: 0x93a3b8, transparent: true, opacity: 0.75 })
      ).translateX(p[0]).translateY(-0.32).translateZ(p[1]));
    });

    /* ── Repère IMU : origine, au centre du châssis ────────────────────── */
    var imu = makeTriad(1.25, 1.0);
    root.add(imu);
    root.add(new THREE.Mesh(
      new THREE.SphereGeometry(0.075, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xffffff })
    ));

    /* ── Repère CAMÉRA : en avant (+x) et en hauteur (+z du corps) ──────
       T_imu_cam applique R : cam-z -> +x, cam-x -> -y, cam-y -> -z.
       On reproduit cette rotation exactement, pas une approximation. */
    var camG = new THREE.Group();
    camG.position.set(1.18, 0.52, 0);
    var Rcam = new THREE.Matrix4().set(
      0, 0, 1, 0,
     -1, 0, 0, 0,
      0,-1, 0, 0,
      0, 0, 0, 1
    );
    camG.quaternion.setFromRotationMatrix(Rcam);
    camG.add(makeTriad(0.9, 1.0));
    /* boîtier stéréo : deux objectifs séparés par la baseline 90.2 mm */
    var housing = new THREE.Mesh(
      new THREE.BoxGeometry(0.16, 0.2, 0.72),
      new THREE.MeshPhongMaterial({ color: 0x1b2432, shininess: 60 })
    );
    camG.add(housing);
    [-0.22, 0.22].forEach(function (dz) {
      var lens = new THREE.Mesh(
        new THREE.CylinderGeometry(0.062, 0.062, 0.05, 18),
        new THREE.MeshPhongMaterial({ color: 0x0d1520, emissive: F.Z,
                                      emissiveIntensity: 0.42, shininess: 90 })
      );
      lens.rotation.z = Math.PI / 2;
      lens.position.set(0.09, 0, dz);
      camG.add(lens);
    });
    root.add(camG);
    root.add(makeLink(new THREE.Vector3(0,0,0), camG.position.clone(), F.Z));

    /* ── Repère ROUES : Rz(pi) — il pointe à l'OPPOSÉ de la caméra ──────
       C'est le fait que la page explique et qu'aucun texte ne rend
       évident : sans cette rotation, chaque mesure roue contredisait la
       prédiction inertielle et le chi2 les rejetait silencieusement. */
    var whG = new THREE.Group();
    whG.position.set(0, -0.30, 0);
    whG.rotation.z = Math.PI;
    whG.add(makeTriad(0.9, 1.0));
    root.add(whG);
    root.add(makeLink(new THREE.Vector3(0,0,0), whG.position.clone(), F.X));

    return { group: root };
  }

  /* Orbite pilotée par la souris, amortie. Pas d'auto-rotation. */
  function initFrames(canvasId, opts) {
    opts = opts || {};
    var canvas = typeof canvasId === 'string'
      ? document.getElementById(canvasId) : canvasId;
    if (!canvas || typeof THREE === 'undefined') return null;

    var renderer;
    try {
      renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true, alpha: true });
    } catch (e) { return null; }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x000000, 0);

    var scene  = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
    camera.position.set(0, 0, opts.cameraZ || 4.9);

    scene.add(new THREE.AmbientLight(0x5b6f8f, 1.35));
    var key = new THREE.DirectionalLight(0xbcd4ff, 1.6);
    key.position.set(4, 6, 5); scene.add(key);
    var rim = new THREE.DirectionalLight(F.X, 0.75);
    rim.position.set(-5, -2, -3); scene.add(rim);

    var built = buildFrames();
    scene.add(built.group);

    function resize() {
      var w = canvas.clientWidth || 300, h = canvas.clientHeight || 300;
      renderer.setSize(w, h, false);
      camera.aspect = w / h; camera.updateProjectionMatrix();
    }
    resize();
    window.addEventListener('resize', resize, { passive: true });

    /* Cible d'orientation : suit la souris sur TOUTE la fenêtre, pour que la
       figure réagisse pendant la lecture et pas seulement au survol direct. */
    var tgtY = 0.55, tgtX = 0.18, curY = 0.55, curX = 0.18;
    var reduce = !!(window.matchMedia &&
                    window.matchMedia('(prefers-reduced-motion: reduce)').matches);

    function aim(cx, cy) {
      var r = canvas.getBoundingClientRect();
      var nx = (cx - (r.left + r.width  / 2)) / (window.innerWidth  / 2);
      var ny = (cy - (r.top  + r.height / 2)) / (window.innerHeight / 2);
      tgtY = 0.55 + nx * 0.85;
      tgtX = 0.18 + ny * 0.42;
    }
    if (!reduce) {
      window.addEventListener('mousemove', function (e) { aim(e.clientX, e.clientY); },
                              { passive: true });
      canvas.addEventListener('touchmove', function (e) {
        if (e.touches && e.touches[0]) aim(e.touches[0].clientX, e.touches[0].clientY);
      }, { passive: true });
    }

    var visible = true, raf, clock = new THREE.Clock();
    document.addEventListener('visibilitychange', function () { visible = !document.hidden; });

    function frame() {
      raf = requestAnimationFrame(frame);
      if (!visible) return;
      /* amorti : 0.055 ≈ suit sans coller, s'arrête sans rebond */
      curY += (tgtY - curY) * 0.055;
      curX += (tgtX - curX) * 0.055;
      built.group.rotation.y = curY;
      built.group.rotation.x = curX;
      /* respiration très légère — la figure reste vivante à souris immobile */
      built.group.position.y = Math.sin(clock.getElapsedTime() * 0.5) * 0.035;
      renderer.render(scene, camera);
    }
    frame();

    return {
      renderer: renderer, scene: scene, camera: camera,
      stop: function () { cancelAnimationFrame(raf); }, resize: resize,
    };
  }

  /* ══════════════════════════════════════════════════════════════════════
     EXTRINSÈQUES SUPERPOSÉES (tf_validator.html) — orbite à la souris
     ──────────────────────────────────────────────────────────────────────
     La page compare la convention caméra↔IMU des trois estimateurs, mais
     dans TROIS panneaux séparés : constater qu'ils décrivent — ou non — la
     même géométrie oblige à comparer de tête. Cette scène les met dans un
     SEUL repère IMU, où l'accord et le désaccord se voient.

     Sur les données actuelles : MINS et openVINS coïncident exactement
     (cam0 à 0,15/0/0,10, base stéréo 90,2 mm), sqrtVINS est à ~15 cm.

     Marqueurs CONCENTRIQUES de rayons décroissants : deux estimateurs qui
     coïncident se lisent comme des anneaux emboîtés. Avec des sphères de
     même taille, celui du dessous disparaîtrait et l'accord ressemblerait
     à une absence — la maquette l'a montré avant que ce soit codé.
  ══════════════════════════════════════════════════════════════════════ */
  function initTfCompare(canvasId, opts) {
    opts = opts || {};
    var canvas = typeof canvasId === 'string'
      ? document.getElementById(canvasId) : canvasId;
    if (!canvas || typeof THREE === 'undefined') return null;

    var renderer;
    try {
      renderer = new THREE.WebGLRenderer({ canvas: canvas, alpha: true, antialias: true });
    } catch (e) { return null; }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x000000, 0);

    var scene  = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(40, 1, 0.01, 100);
    camera.position.set(0, 0, 0.62);

    scene.add(new THREE.AmbientLight(0x6b8199, 1.15));
    var key = new THREE.DirectionalLight(0xd6e6ff, 0.95); key.position.set(-1, 1.4, 1);
    scene.add(key);

    var monde = new THREE.Group(); scene.add(monde);

    /* sol : repère métrique, pas de la décoration — pas de 5 cm */
    var grille = new THREE.GridHelper(0.4, 8, 0x2b3542, 0x1d242e);
    grille.rotation.x = Math.PI / 2;          // GridHelper est en XZ, on veut XY
    monde.add(grille);

    /* repère IMU commun */
    monde.add(makeTriad(0.07, 1.0));
    monde.add(new THREE.Mesh(
      new THREE.SphereGeometry(0.005, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xffffff })));

    var api_frames = [];
    function ajouteEstimateur(nom, couleur, frames, rang) {
      var col = new THREE.Color(couleur);
      frames.forEach(function (f) {
        if (!f || !f.nom || f.nom.indexOf('cam') !== 0) return;
        var t = [f.T[0][3], f.T[1][3], f.T[2][3]];
        // Rayon décroissant selon le rang, et FILAIRE : deux estimateurs qui
        // coïncident se lisent alors comme des cages emboîtées. Une sphère
        // pleine, même à 0,92 d'opacité, masque complètement celle du
        // dessous — testé, openVINS disparaissait entièrement sous MINS.
        var r = 0.013 - rang * 0.0032;
        var m = new THREE.Mesh(
          new THREE.SphereGeometry(Math.max(r, 0.005), 14, 10),
          new THREE.MeshBasicMaterial({ color: col, wireframe: true,
                                        transparent: true, opacity: 0.95 }));
        m.position.set(t[0], t[1], t[2]);
        monde.add(m);
        monde.add(makeLink(new THREE.Vector3(0, 0, 0),
                           new THREE.Vector3(t[0], t[1], t[2]), col.getHex()));
        api_frames.push({ est: nom, nom: f.nom, t: t });
      });
    }

    /* orbite souris amortie, même comportement qu'initFrames */
    var tgtY = 0.7, tgtX = 0.35, curY = 0.7, curX = 0.35;
    var reduce = !!(window.matchMedia &&
                    window.matchMedia('(prefers-reduced-motion: reduce)').matches);
    function aim(cx, cy) {
      var r = canvas.getBoundingClientRect();
      tgtY = 0.7 + (cx - (r.left + r.width / 2)) / (window.innerWidth / 2) * 0.9;
      tgtX = 0.35 + (cy - (r.top + r.height / 2)) / (window.innerHeight / 2) * 0.45;
    }
    if (!reduce) {
      window.addEventListener('mousemove', function (e) { aim(e.clientX, e.clientY); },
                              { passive: true });
      canvas.addEventListener('touchmove', function (e) {
        if (e.touches && e.touches[0]) aim(e.touches[0].clientX, e.touches[0].clientY);
      }, { passive: true });
    }

    function resize() {
      var w = canvas.clientWidth || 300, h = canvas.clientHeight || 300;
      renderer.setSize(w, h, false);
      camera.aspect = w / h; camera.updateProjectionMatrix();
    }
    resize();
    window.addEventListener('resize', resize, { passive: true });

    var visible = true, raf;
    document.addEventListener('visibilitychange', function () { visible = !document.hidden; });
    function frame() {
      raf = requestAnimationFrame(frame);
      if (!visible) return;
      curY += (tgtY - curY) * 0.055;
      curX += (tgtX - curX) * 0.055;
      monde.rotation.z = curY;
      monde.rotation.x = -1.15 + curX;      // vue plongeante par défaut
      renderer.render(scene, camera);
    }
    renderer.render(scene, camera);   /* première image immédiate, cf. ci-dessus */
    frame();

    return {
      renderer: renderer, scene: scene, camera: camera, resize: resize,
      stop: function () { cancelAnimationFrame(raf); },
      /* Les données arrivent de tf_frames.json, chargé par la page : le
         moteur ne va pas les chercher lui-même, il les reçoit. */
      setFrames: function (estimateurs, couleurs) {
        Object.keys(estimateurs).forEach(function (nom, i) {
          var e = estimateurs[nom];
          if (e && e.frames) ajouteEstimateur(nom, couleurs[nom] || 0xffffff, e.frames, i);
        });
      },
      frames: function () { return api_frames; },
    };
  }

  /* ══════════════════════════════════════════════════════════════════════
     TOPOLOGIE DE FUSION (navigation_modes.html) — orbite à la souris
     ──────────────────────────────────────────────────────────────────────
     La page oppose VINS, MINS et sqrtVINS en prose. Le fait qui explique
     tout le reste tient en une ligne : les trois reçoivent l'IMU et la
     caméra stéréo, mais SEUL MINS consomme l'odométrie des roues. C'est ce
     qui lui donne son ancrage en x/y quand la vision décroche — et c'est
     aussi pourquoi lui seul survit à des trajets où les deux VIO purs
     divergent.

     La scène ne dit rien d'autre : trois capteurs, trois estimateurs, et
     une liaison qui n'existe que pour un seul. Le lien « roues » est
     volontairement plus épais et animé pour qu'on le remarque.
  ══════════════════════════════════════════════════════════════════════ */
  function initSensorFlow(canvasId, opts) {
    opts = opts || {};
    var canvas = typeof canvasId === 'string'
      ? document.getElementById(canvasId) : canvasId;
    if (!canvas || typeof THREE === 'undefined') return null;

    var renderer;
    try {
      renderer = new THREE.WebGLRenderer({ canvas: canvas, alpha: true, antialias: true });
    } catch (e) { return null; }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x000000, 0);

    var scene  = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(42, 1, 0.1, 200);
    camera.position.set(0, 0, 9.2);

    scene.add(new THREE.AmbientLight(0x6b8199, 1.2));
    var key = new THREE.DirectionalLight(0xd6e6ff, 0.9); key.position.set(-1, 1.2, 1.4);
    scene.add(key);

    var monde = new THREE.Group(); scene.add(monde);

    var CAPTEURS = [
      { nom: 'IMU',     y:  1.7, c: 0x9fb2c8 },
      { nom: 'caméra',  y:  0.0, c: 0x9fb2c8 },
      { nom: 'roues',   y: -1.7, c: 0x3fdc97 },   // le seul qui n'alimente qu'un estimateur
    ];
    var ESTIM = [
      { nom: 'MINS',     y:  1.7, c: 0xff6b35, roues: true  },
      { nom: 'openVINS', y:  0.0, c: 0x22d3ee, roues: false },
      { nom: 'sqrtVINS', y: -1.7, c: 0xfbbf24, roues: false },
    ];
    var XG = -2.6, XD = 2.6;

    function boite(x, y, coul, large) {
      var g = new THREE.Group();
      var geo = new THREE.BoxGeometry(large ? 1.9 : 1.5, 0.72, 0.5);
      g.add(new THREE.Mesh(geo, new THREE.MeshPhongMaterial({
        color: 0x18202b, emissive: coul, emissiveIntensity: 0.12,
        shininess: 30, transparent: true, opacity: 0.85 })));
      g.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo),
        new THREE.LineBasicMaterial({ color: coul, transparent: true, opacity: 0.9 })));
      g.position.set(x, y, 0);
      monde.add(g);
      return g;
    }
    CAPTEURS.forEach(function (s) { boite(XG, s.y, s.c, false); });
    ESTIM.forEach(function (e) { boite(XD, e.y, e.c, true); });

    /* Liaisons. Celle des roues est la seule qui ne parte pas vers les trois :
       c'est l'information que la scène existe pour porter. */
    var fluxRoues = [];
    CAPTEURS.forEach(function (s) {
      ESTIM.forEach(function (e) {
        if (s.nom === 'roues' && !e.roues) return;
        var roue = (s.nom === 'roues');
        var pts = [new THREE.Vector3(XG + 0.78, s.y, 0),
                   new THREE.Vector3(0, (s.y + e.y) / 2, roue ? 0.55 : 0),
                   new THREE.Vector3(XD - 0.96, e.y, 0)];
        var courbe = new THREE.QuadraticBezierCurve3(pts[0], pts[1], pts[2]);
        var geo = new THREE.BufferGeometry().setFromPoints(courbe.getPoints(28));
        monde.add(new THREE.Line(geo, new THREE.LineBasicMaterial({
          color: roue ? 0x3fdc97 : 0x46566b,
          transparent: true, opacity: roue ? 0.95 : 0.45 })));
        if (roue) {
          // pastille qui parcourt la liaison : attire l'œil sur la seule
          // entrée que MINS possède et que les deux autres n'ont pas
          var p = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 12),
                    new THREE.MeshBasicMaterial({ color: 0x3fdc97 }));
          monde.add(p);
          fluxRoues.push({ courbe: courbe, mesh: p, t: 0 });
        }
      });
    });

    var tgtY = 0.0, tgtX = 0.0, curY = 0.0, curX = 0.0;
    var reduce = !!(window.matchMedia &&
                    window.matchMedia('(prefers-reduced-motion: reduce)').matches);
    function aim(cx, cy) {
      var r = canvas.getBoundingClientRect();
      tgtY = (cx - (r.left + r.width / 2)) / (window.innerWidth / 2) * 0.42;
      tgtX = (cy - (r.top + r.height / 2)) / (window.innerHeight / 2) * 0.26;
    }
    if (!reduce) {
      window.addEventListener('mousemove', function (e) { aim(e.clientX, e.clientY); },
                              { passive: true });
      canvas.addEventListener('touchmove', function (e) {
        if (e.touches && e.touches[0]) aim(e.touches[0].clientX, e.touches[0].clientY);
      }, { passive: true });
    }

    function resize() {
      var w = canvas.clientWidth || 300, h = canvas.clientHeight || 300;
      renderer.setSize(w, h, false);
      camera.aspect = w / h; camera.updateProjectionMatrix();
    }
    resize();
    window.addEventListener('resize', resize, { passive: true });

    var visible = true, raf, clock = new THREE.Clock();
    document.addEventListener('visibilitychange', function () { visible = !document.hidden; });
    function frame() {
      raf = requestAnimationFrame(frame);
      if (!visible) return;
      curY += (tgtY - curY) * 0.06;
      curX += (tgtX - curX) * 0.06;
      monde.rotation.y = curY;
      monde.rotation.x = curX;
      if (!reduce) {
        var t = clock.getElapsedTime();
        fluxRoues.forEach(function (f, i) {
          var u = (t * 0.32 + i * 0.5) % 1;
          f.mesh.position.copy(f.courbe.getPoint(u));
        });
      }
      renderer.render(scene, camera);
    }
    /* Première image dessinée TOUT DE SUITE, sans attendre la boucle : en
       vrai navigateur le contenu apparaît dès le premier rendu au lieu du
       premier rAF, et une capture headless (qui n'attend pas les rAF) voit
       enfin la scène. */
    renderer.render(scene, camera);
    frame();

    return { renderer: renderer, scene: scene, camera: camera, resize: resize,
             stop: function () { cancelAnimationFrame(raf); } };
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
    /* Repères capteurs orbitables (mins_tuning.html). Pas d'auto-rotation :
       la scène ne bouge que sous la souris, pour qu'un angle puisse être
       tenu et lu. */
    initFrames: function(id, opts) {
      return initFrames(id, opts);
    },
    /* Extrinsèques des trois estimateurs superposées dans un repère IMU
       commun (tf_validator.html). Les données lui sont FOURNIES par la page
       via setFrames() : le moteur ne connaît pas tf_frames.json. */
    initTfCompare: function(id, opts) {
      return initTfCompare(id, opts);
    },
    /* Topologie de fusion : qui consomme quoi (navigation_modes.html).
       Seul MINS reçoit les roues — c'est le fait que la scène porte. */
    initSensorFlow: function(id, opts) {
      return initSensorFlow(id, opts);
    },
  };

})();
