/* =============================================================================
   LEO·OPS — Audio Manager  (audio_manager.js)
   -----------------------------------------------------------------------------
   Retour sonore haute-fidélité pour interface Mission Control.

   Architecture : synthèse procédurale via Web Audio API.
   Avantages vs fichiers .mp3/.wav :
     · latence nulle (pas de décodage réseau)
     · fonctionne hors-ligne
     · enveloppes et fréquences réglables à la milliseconde
     · aucune dépendance externe

   Si vous souhaitez substituer des fichiers audios réels :
     AudioMgr.loadFile('click', 'assets/audio/click.mp3').then(...)
   La méthode loadFile() décrit en bas de fichier.

   API publique
   ─────────────
   AudioMgr.play(name)        — joue un son par nom
   AudioMgr.setVolume(v)      — volume maître 0.0–1.0
   AudioMgr.toggleMute()      — coupe / rétablit le son (persisté localStorage)
   AudioMgr.bindAll()         — attache hover + click à tous les éléments interactifs
   AudioMgr.muted             — getter : état muet actuel

   Sons disponibles
   ─────────────────
   hover       UI tick discret — survol de bouton
   click       Clic mécanique — action générique
   success     Confirmation bitonale — opération réussie
   alert       Double sweep descendant — avertissement
   emergency   Triple impulsion — arrêt d'urgence
   connect     Sweep ascendant — rosbridge connecté
   disconnect  Sweep descendant — connexion perdue
   park        Thump grave + ping — retour à la base
   hardreset   Tri-ton descendant — reset nœud
   beacon      Arpège ascendant — balise acquise
   keydown     Impulsion brève — touche drive enfoncée
   ============================================================================= */
window.AudioMgr = (function () {
  'use strict';

  /* ── Persistance ─────────────────────────────────────────────────────────── */
  const LS_MUTED  = 'leo_audio_muted';
  const LS_VOLUME = 'leo_audio_volume';

  let muted  = localStorage.getItem(LS_MUTED)  === '1';
  let volume = parseFloat(localStorage.getItem(LS_VOLUME) || '0.42');
  if (isNaN(volume) || volume < 0 || volume > 1) volume = 0.42;

  /* ── Web Audio context (lazy — débloqué par la 1ère interaction) ─────────── */
  let ctx = null;
  let masterGain = null;

  /* Buffers audio optionnels chargés via loadFile() */
  const fileBuffers = {};

  function getCtx() {
    if (!ctx) {
      ctx = new (window.AudioContext || window.webkitAudioContext)();
      masterGain = ctx.createGain();
      masterGain.gain.setValueAtTime(muted ? 0 : volume, ctx.currentTime);
      masterGain.connect(ctx.destination);
    }
    /* Les navigateurs suspendent le contexte après inactivité */
    if (ctx.state === 'suspended') ctx.resume();
    return ctx;
  }

  /* ══════════════════════════════════════════════════════════════════════════
     PRIMITIVES DE SYNTHÈSE
     ══════════════════════════════════════════════════════════════════════════ */

  /**
   * Oscillateur simple avec enveloppe ADSR simplifiée (attack / release).
   * @param {number} freq    fréquence Hz
   * @param {string} type    waveform : 'sine' | 'square' | 'triangle' | 'sawtooth'
   * @param {number} dur     durée totale en secondes
   * @param {number} peak    gain crête (0-1, sera multiplié par masterGain)
   * @param {number} [delay=0]  décalage de départ par rapport à ctx.currentTime
   */
  function osc(freq, type, dur, peak, delay) {
    delay = delay || 0;
    const c = getCtx();
    const t = c.currentTime + delay;
    const atk = Math.min(0.003, dur * 0.1);
    const rel = Math.min(0.018, dur * 0.35);

    const g = c.createGain();
    g.connect(masterGain);
    g.gain.setValueAtTime(0, t);
    g.gain.linearRampToValueAtTime(peak, t + atk);
    g.gain.setValueAtTime(peak, t + dur - rel);
    g.gain.linearRampToValueAtTime(0, t + dur);

    const o = c.createOscillator();
    o.type = type;
    o.frequency.setValueAtTime(freq, t);
    o.connect(g);
    o.start(t);
    o.stop(t + dur + 0.01);
  }

  /**
   * Oscillateur avec glissement de fréquence exponentiel (sweep).
   */
  function sweep(f0, f1, type, dur, peak, delay) {
    delay = delay || 0;
    const c = getCtx();
    const t = c.currentTime + delay;
    const atk = Math.min(0.004, dur * 0.1);
    const rel = Math.min(0.025, dur * 0.38);

    const g = c.createGain();
    g.connect(masterGain);
    g.gain.setValueAtTime(0, t);
    g.gain.linearRampToValueAtTime(peak, t + atk);
    g.gain.setValueAtTime(peak, t + dur - rel);
    g.gain.linearRampToValueAtTime(0, t + dur);

    const o = c.createOscillator();
    o.type = type;
    o.frequency.setValueAtTime(f0, t);
    /* exponentialRamp exige des valeurs > 0 */
    o.frequency.exponentialRampToValueAtTime(Math.max(f1, 10), t + dur);
    o.connect(g);
    o.start(t);
    o.stop(t + dur + 0.01);
  }

  /**
   * Burst de bruit blanc filtré (bandpass) — simule un clic mécanique.
   */
  function noise(dur, peak, filterFreq, filterQ, delay) {
    filterFreq = filterFreq || 1000;
    filterQ    = filterQ    || 3;
    delay      = delay      || 0;
    const c = getCtx();
    const t = c.currentTime + delay;

    const samples = Math.ceil(c.sampleRate * dur);
    const buf     = c.createBuffer(1, samples, c.sampleRate);
    const data    = buf.getChannelData(0);
    for (let i = 0; i < samples; i++) data[i] = Math.random() * 2 - 1;

    const src = c.createBufferSource();
    src.buffer = buf;

    const bp  = c.createBiquadFilter();
    bp.type   = 'bandpass';
    bp.frequency.value = filterFreq;
    bp.Q.value         = filterQ;

    const g = c.createGain();
    g.gain.setValueAtTime(peak, t);
    g.gain.exponentialRampToValueAtTime(0.0001, t + dur);

    src.connect(bp);
    bp.connect(g);
    g.connect(masterGain);
    src.start(t);
    src.stop(t + dur + 0.01);
  }

  /* ══════════════════════════════════════════════════════════════════════════
     PALETTE SONORE — HIGH-TECH / NASA
     ══════════════════════════════════════════════════════════════════════════

     Principes de design :
       · Brefs (< 350 ms) — ne couvrent jamais une conversation
       · Discrets — gain calibré pour moniteur à volume moyen
       · Registre médium-aigu — audible sur un PC de travail
       · Aucun son "cartoon" — uniquement sinusoïdes, triangles, bruit filtré
  */
  const SOUNDS = {

    /* ── UI / Navigation ─────────────────────────────────────────────────── */

    hover: function () {
      /* Tick quasi-sub-perceptif : à peine audible, juste assez pour le retour haptique visuel */
      osc(1380, 'sine', 0.016, 0.024);
    },

    click: function () {
      /* Claquement mécanique — bruit filtré à 950 Hz + impulsion sinusoïdale */
      noise(0.022, 0.22, 950, 2.8);
      osc(880, 'sine', 0.014, 0.055, 0.004);
    },

    /* ── Système ─────────────────────────────────────────────────────────── */

    connect: function () {
      /* Sweep ascendant 200→1100 Hz — liaison établie */
      sweep(200, 1100, 'sine', 0.30, 0.085);
      /* Ping de confirmation */
      osc(1100, 'sine', 0.09, 0.060, 0.28);
    },

    disconnect: function () {
      /* Sweep descendant — connexion perdue */
      sweep(900, 180, 'sine', 0.24, 0.072);
    },

    success: function () {
      /* Bitonale ascendante — A5 puis E6 (quinte juste) */
      osc(880, 'sine', 0.058, 0.090);
      osc(1320, 'sine', 0.082, 0.085, 0.052);
    },

    /* ── Avertissements ──────────────────────────────────────────────────── */

    alert: function () {
      /* Double sweep descendant 600→280 Hz — warning non-urgent */
      sweep(600, 280, 'triangle', 0.175, 0.140);
      sweep(600, 280, 'triangle', 0.175, 0.105, 0.220);
    },

    emergency: function () {
      /* Triple impulsion carrée 550 Hz — arrêt d'urgence */
      for (var i = 0; i < 3; i++) {
        osc(550, 'square', 0.072, 0.155, i * 0.125);
      }
    },

    /* ── Commandes Robot ─────────────────────────────────────────────────── */

    park: function () {
      /* Thump grave (atterrissage) + ping de confirmation */
      osc(75, 'sine', 0.120, 0.190);          /* sub grave */
      osc(1100, 'sine', 0.095, 0.068, 0.095); /* confirmation */
    },

    hardreset: function () {
      /* Tri-ton descendant — mise hors tension séquentielle */
      osc(1000, 'triangle', 0.062, 0.105);
      osc(750,  'triangle', 0.062, 0.105, 0.075);
      osc(500,  'triangle', 0.082, 0.100, 0.155);
    },

    /* ── Mission ─────────────────────────────────────────────────────────── */

    beacon: function () {
      /* Arpège ascendant 3 notes — cible acquise */
      osc(660,  'sine', 0.038, 0.068);
      osc(880,  'sine', 0.038, 0.075, 0.040);
      osc(1320, 'sine', 0.072, 0.082, 0.082);
    },

    keydown: function () {
      /* Bref clic solénoïde — chaque pression de touche de pilotage */
      noise(0.016, 0.090, 1600, 3.5);
    },

  };

  /* ══════════════════════════════════════════════════════════════════════════
     CHARGEMENT DE FICHIERS AUDIO (optionnel)
     ══════════════════════════════════════════════════════════════════════════ */

  /**
   * Charge un fichier audio et remplace le son synthétique correspondant.
   * Exemple : AudioMgr.loadFile('click', 'assets/audio/click.mp3')
   * @param {string} name  nom du son (clé de SOUNDS)
   * @param {string} url   chemin vers le fichier .mp3 / .wav
   * @returns {Promise<void>}
   */
  function loadFile(name, url) {
    return fetch(url)
      .then(function (r) { return r.arrayBuffer(); })
      .then(function (ab) { return getCtx().decodeAudioData(ab); })
      .then(function (decoded) {
        fileBuffers[name] = decoded;
        SOUNDS[name] = function () {
          var c = getCtx();
          var t = c.currentTime;
          var src = c.createBufferSource();
          src.buffer = decoded;
          var g = c.createGain();
          g.gain.setValueAtTime(1.0, t);
          src.connect(g);
          g.connect(masterGain);
          src.start(t);
        };
      })
      .catch(function (e) {
        console.warn('[AudioMgr] loadFile(' + name + ') failed:', e.message);
      });
  }

  /* ══════════════════════════════════════════════════════════════════════════
     API PUBLIQUE
     ══════════════════════════════════════════════════════════════════════════ */

  function play(name) {
    if (muted) return;
    if (!SOUNDS[name]) {
      console.warn('[AudioMgr] unknown sound:', name);
      return;
    }
    try {
      SOUNDS[name]();
    } catch (e) {
      console.warn('[AudioMgr] play(' + name + '):', e.message);
    }
  }

  function setVolume(v) {
    volume = Math.max(0, Math.min(1, v));
    localStorage.setItem(LS_VOLUME, volume);
    if (masterGain && !muted) {
      masterGain.gain.setTargetAtTime(volume, getCtx().currentTime, 0.04);
    }
  }

  function toggleMute() {
    muted = !muted;
    localStorage.setItem(LS_MUTED, muted ? '1' : '0');
    if (masterGain) {
      masterGain.gain.setTargetAtTime(
        muted ? 0 : volume, getCtx().currentTime, 0.04
      );
    }
    _updateMuteBtn();
    return muted;
  }

  /* ── Mise à jour visuelle du bouton mute ────────────────────────────────── */
  function _updateMuteBtn() {
    var btn = document.getElementById('btnMuteAudio');
    if (!btn) return;

    var volSpan  = btn.querySelector('.audio-icon-vol');
    var muteSpan = btn.querySelector('.audio-icon-mute');
    if (volSpan)  volSpan.style.display  = muted ? 'none' : '';
    if (muteSpan) muteSpan.style.display = muted ? ''     : 'none';

    btn.title     = muted ? 'Unmute audio' : 'Mute audio';
    btn.setAttribute('aria-label', muted ? 'Unmute audio' : 'Mute audio');
    btn.classList.toggle('opacity-35', muted);
    btn.classList.toggle('text-zinc-600', muted);
  }

  /* ── Binding global hover / click ───────────────────────────────────────── */

  /**
   * Attache les sons hover + click sur tous les éléments interactifs de la page.
   * Idempotent : un élément déjà bindé (data-audiobound="1") est ignoré.
   * À appeler après chaque injection dynamique de boutons.
   */
  function bindAll() {
    var selector = 'button, a[href], [data-cmd], [data-drive], .lang-btn';
    document.querySelectorAll(selector).forEach(function (el) {
      if (el.dataset.audiobound) return;
      el.dataset.audiobound = '1';

      /* Hover discret */
      el.addEventListener('mouseenter', function () {
        play('hover');
      }, { passive: true });

      /* Click — son contextuel selon la fonction du bouton */
      el.addEventListener('click', function () {
        var cmd = el.dataset.cmd;
        if      (cmd === 'stop')       play('emergency');
        else if (cmd === 'park')       play('park');
        else if (cmd === 'hardreset')  play('hardreset');
        else if (cmd === 'mode')       play('success');
        else if (cmd === 'reset' || cmd === 'clear') play('alert');
        else                           play('click');
      });
    });

    _updateMuteBtn();
  }

  /* ── Initialisation au chargement ───────────────────────────────────────── */
  /* Applique l'état mute persisté dès que le DOM est disponible */
  (function () {
    if (document.readyState !== 'loading') {
      _updateMuteBtn();
    } else {
      document.addEventListener('DOMContentLoaded', _updateMuteBtn);
    }
  })();

  /* ── Export ─────────────────────────────────────────────────────────────── */
  return {
    play       : play,
    setVolume  : setVolume,
    toggleMute : toggleMute,
    bindAll    : bindAll,
    loadFile   : loadFile,
    get muted() { return muted; },
    get volume() { return volume; },
  };

})();
