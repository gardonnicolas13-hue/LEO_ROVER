/* =============================================================================
   LEO·OPS — Mouse Engine  (mouse_engine.js)
   Premium parallax & 3D tilt driven by mouse position.

   Three depth layers, each with its own LERP inertia:
     [bg]    bg3d canvas + stars  → translates OPPOSITE to mouse (deep, heavy)
     [orbit] #heroOrbit CSS div   → 3D perspective tilt + slight follow
     [label] [data-parallax] els  → slight follow, depth-scaled

   Three.js integration: exposes window._bg3dMouse so the globe render loop
   inside index.html can read tilt offsets without breaking its closure.

   Mobile / reduced-motion: effect disabled, _bg3dMouse still set to 0.
   ============================================================================= */
(function () {
  'use strict';

  /* ── Feature gates ───────────────────────────────────────────────────────── */
  const isMobile =
    'ontouchstart' in window ||
    navigator.maxTouchPoints > 0 ||
    (window.innerWidth < 768);

  const reduceMotion =
    window.matchMedia &&
    window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  /* Always expose the shared state so Three.js doesn't crash on mobile. */
  window._bg3dMouse = { tiltX: 0, tiltY: 0, camX: 0, camY: 0 };

  if (isMobile || reduceMotion) return;

  /* ── Configuration ───────────────────────────────────────────────────────── */
  const CFG = {
    /* Per-layer LERP factors — lower = more inertia (heavier feel). */
    lerpBg:    0.032,   /* deep background: very slow, majestic             */
    lerpOrbit: 0.068,   /* hero orbital: medium, precise                    */
    lerpLabel: 0.105,   /* labels: snappy, appear closest to viewer         */

    /* Displacement amplitudes (px or deg) */
    bgTransX:    22,    /* bg3d horizontal travel (px, opposite direction)  */
    bgTransY:    16,    /* bg3d vertical travel                              */
    starsTransX: 11,    /* starfield horizontal (less than bg3d → depth gap)*/
    starsTransY:  8,
    orbitRotX:   11,    /* hero tilt around X axis (deg)                    */
    orbitRotY:   13,    /* hero tilt around Y axis (deg)                    */
    orbitTransX:  9,    /* hero drift (px)                                  */
    orbitTransY:  6,
    threeTiltX:   0.32, /* Three.js core.rotation.y offset (rad)            */
    threeTiltY:   0.22, /* Three.js core.rotation.x offset                  */
    threeCamX:    2.2,  /* Three.js camera.position.x offset                */
    threeCamY:    1.5,
    labelBase:    8,    /* label translate base (px × data-parallax depth)  */
    labelBaseY:   5,

    /* Intensity curve: tanh saturation factor.
       Higher → response plateaus sooner at edges (softer clamp).
       Lower  → near-linear, stronger at edges (more aggressive). */
    curveSat: 1.25,
  };

  /* ── State ───────────────────────────────────────────────────────────────── */
  const raw = { x: 0, y: 0 };

  /* Each entry: { x, y } = current lerped position, f = lerp factor. */
  const L = {
    bg:    { x: 0, y: 0, f: CFG.lerpBg    },
    orbit: { x: 0, y: 0, f: CFG.lerpOrbit },
    label: { x: 0, y: 0, f: CFG.lerpLabel },
  };

  /* ── Intensity curve ─────────────────────────────────────────────────────── */
  /*
   * Maps the normalized mouse position [-1, 1] through tanh so that:
   *   - Small displacements (mouse near centre) → near-linear, very sensitive
   *   - Large displacements (mouse near edges)  → soft plateau (no jarring extremes)
   * Multiplied by amplitude to give the final px / deg value.
   */
  function curve(v, amplitude) {
    return Math.tanh(v * CFG.curveSat) * amplitude;
  }

  /* ── Mouse / viewport listeners ──────────────────────────────────────────── */
  document.addEventListener('mousemove', function (e) {
    raw.x = (e.clientX / window.innerWidth  - 0.5) * 2;
    raw.y = (e.clientY / window.innerHeight - 0.5) * 2;
  }, { passive: true });

  /* Smoothly settle back to centre when the mouse leaves the viewport. */
  document.addEventListener('mouseleave', function () {
    raw.x = 0;
    raw.y = 0;
  }, { passive: true });

  /* ── Element cache ───────────────────────────────────────────────────────── */
  let heroOrbit    = null;
  let bg3dCanvas   = null;
  let starsCanvas  = null;
  let parallaxEls  = [];

  function cacheElements() {
    heroOrbit   = document.getElementById('heroOrbit');
    bg3dCanvas  = document.getElementById('bg3d');
    starsCanvas = document.getElementById('stars');
    parallaxEls = Array.from(document.querySelectorAll('[data-parallax]'));

    /* GPU compositing hints — avoids paint each frame. */
    [heroOrbit, bg3dCanvas, starsCanvas].forEach(function (el) {
      if (el) el.style.willChange = 'transform';
    });
    parallaxEls.forEach(function (el) {
      el.style.willChange = 'transform';
    });
  }

  /* ── Visibility guard ────────────────────────────────────────────────────── */
  let visible = !document.hidden;
  document.addEventListener('visibilitychange', function () {
    visible = !document.hidden;
  });

  /* ── Main animation loop ─────────────────────────────────────────────────── */
  function tick() {
    requestAnimationFrame(tick);
    if (!visible) return;

    /* 1) Advance LERP for all layers toward raw mouse. */
    const layers = [L.bg, L.orbit, L.label];
    for (var i = 0; i < layers.length; i++) {
      var l = layers[i];
      l.x += (raw.x - l.x) * l.f;
      l.y += (raw.y - l.y) * l.f;
    }

    /* 2) Background canvases — move OPPOSITE to mouse.
          scale > 1 hides canvas edge gaps when translated.  */
    if (bg3dCanvas) {
      var bx = -curve(L.bg.x, CFG.bgTransX);
      var by = -curve(L.bg.y, CFG.bgTransY);
      bg3dCanvas.style.transform =
        'translate(' + bx + 'px,' + by + 'px) scale(1.07)';
    }
    if (starsCanvas) {
      var sx = -curve(L.bg.x, CFG.starsTransX);
      var sy = -curve(L.bg.y, CFG.starsTransY);
      starsCanvas.style.transform =
        'translate(' + sx + 'px,' + sy + 'px) scale(1.04)';
    }

    /* 3) Three.js shared state.
          The render loop in index.html reads these every frame to add
          a mouse-driven offset on top of its time-based animation.
          tiltX/Y are in radians, added to core.rotation.y/x.
          camX/Y are in world units, set on camera.position.x/y.      */
    var m = window._bg3dMouse;
    m.tiltX =  curve(L.bg.x,  CFG.threeTiltX);
    m.tiltY = -curve(L.bg.y,  CFG.threeTiltY);
    m.camX  =  curve(L.bg.x,  CFG.threeCamX);
    m.camY  = -curve(L.bg.y,  CFG.threeCamY);

    /* 4) Hero orbital CSS 3D tilt.
          perspective() on the element itself so child CSS animations
          (animate-spin-slow, animate-floaty, animate-sweep) are unaffected:
          they operate on their own transform contexts (separate elements). */
    if (heroOrbit) {
      var rx = curve(L.orbit.y, -CFG.orbitRotX); /* mouseY → tilt back/fwd  */
      var ry = curve(L.orbit.x,  CFG.orbitRotY); /* mouseX → tilt left/right*/
      var tx = curve(L.orbit.x,  CFG.orbitTransX);
      var ty = curve(L.orbit.y,  CFG.orbitTransY);
      heroOrbit.style.transform =
        'perspective(950px)' +
        ' rotateX(' + rx + 'deg)' +
        ' rotateY(' + ry + 'deg)' +
        ' translate(' + tx + 'px,' + ty + 'px)';
    }

    /* 5) [data-parallax] elements.
          depth attribute: 1.0 = close (fast), 0.3 = far (slow).
          They FOLLOW the mouse (same direction), giving the impression
          of floating in front of the background.                       */
    for (var j = 0; j < parallaxEls.length; j++) {
      var el    = parallaxEls[j];
      var depth = parseFloat(el.dataset.parallax) || 0.6;
      var px    = curve(L.label.x, CFG.labelBase  * depth);
      var py    = curve(L.label.y, CFG.labelBaseY * depth);
      el.style.transform = 'translate(' + px + 'px,' + py + 'px)';
    }
  }

  /* ── Boot ────────────────────────────────────────────────────────────────── */
  function boot() {
    cacheElements();
    tick();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }

  /* ── Public API (debug / hot-reload) ─────────────────────────────────────── */
  window.MouseEngine = {
    config:  CFG,
    state:   L,
    raw:     raw,
    refresh: cacheElements,
  };

})();
