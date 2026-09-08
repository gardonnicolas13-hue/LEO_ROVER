#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — live IMU sanitizer
==============================
Subscribes the raw onboard IMU (/firmware/imu, leo_msgs/Imu) and republishes a
spike-free sensor_msgs/Imu on /imu/data_clean for VINS/MINS to consume.

WHY THIS EXISTS: the onboard IMU emits rare single-sample glitches (measured
offline in bag_final.bag: ~2 samples in 8973, e.g. one accel_z at 5e16 m/s^2,
one gyro_y at 5e7 rad/s — almost certainly I2C/SPI read or rosserial framing
hiccups). The robot's firmware_message_converter copies raw values straight
through with no clamping, so /imu/data_raw carries these glitches live. A single
1e16 sample makes any inertial estimator (OpenVINS, MINS) diverge instantly.
Offline this broke kalibr_calibrate_imu_camera (spurious -16 s time shift ->
"Optimization failed!"); online it would blow up the running filter the first
time it fired. This node is the runtime guard.

FILTER (identical thresholds to calibration_leo_d455/sanitize_imu_bag.py so the
online and offline pipelines match): reject a sample if any gyro axis exceeds
GYRO_LIMIT rad/s or any accel axis exceeds ACCEL_LIMIT m/s^2 (both far above real
robot motion, far below the garbage). A rejected sample is replaced by the last
good one (zero-order hold), keeping the IMU cadence intact. Glitches are isolated
single samples so ZOH is exact in practice. Rejections are counted and logged.

PUBLISHES  /imu/data_clean   sensor_msgs/Imu
           ~zupt_topic (default /imu_sanitizer/is_stationary)  std_msgs/Bool
           (2026-08-19 — see ZUPT DETECTOR below)
SUBSCRIBES ~in_topic (default /firmware/imu)  leo_msgs/Imu
           ~wheel_topic (default /firmware/wheel_states)  leo_msgs/WheelStates
           (always, not just when gyro_recal_period > 0 — see ZUPT DETECTOR)

FREEZE GUARD: a second, distinct firmware failure mode observed live
(2026-07-04): after a rosserial desync burst, the LeoCore firmware kept
retransmitting the SAME IMU sample at full rate — bit-identical values,
physically absurd (|a| != g, gyro >> 0 while wheels report standstill). A real
sensor always has noise, so N consecutive bit-identical samples is unambiguous.
MINS cannot initialize on a frozen IMU (gyro says "spinning", wheels say
"still"). The proven remedy is a firmware board reset
(rosservice /firmware/reset_board), so this node now detects the freeze and
triggers that reset itself (throttled), logging loudly each time.

ACCEL SCALE CORRECTION (2026-07-28): the CORE2 accelerometer under-reports by
~10.6%. Measured with the robot verified stationary (wheel odom = 0.000 m/s,
502 samples, std 0.02): |a| = 8.7525 m/s^2 on the RAW /firmware/imu, versus a
true local gravity of 9.790 m/s^2 (Melbourne FL, lat 28 deg). The error is in
the sensor itself, not this node: raw 8.7525 vs sanitized 8.7475 are identical.
It is not a mounting tilt either — the NORM of a vector is rotation-invariant,
so a tilted-but-correct sensor would still read 9.790.

Consequence, and why this matters: OpenVINS assumes gravity_mag = 9.81, so it
sees a permanent ~1.06 m/s^2 residual. Double-integrated that is ~1900 m per
minute — matching the observed divergences exactly (1567 m, 2713 m, 4718 m).
MINS survives the same error because it fuses wheel odometry, which anchors
position; OpenVINS has only camera+IMU and nothing to catch it. Any MINS vs
VINS comparison is therefore invalid until this is corrected.

~accel_scale (default 1.1186 = 9.790 / 8.7525) rescales the accelerometer so
|a| at rest matches true local gravity, for BOTH estimators consistently (a
better fix than patching gravity_mag on the VINS side alone, which would
introduce a systematic 11% scale error on the trajectories themselves).

GYRO BIAS SUBTRACTION (2026-07-28, second and larger defect): with the robot
verified stationary, the gyro reads x=-2.407, y=+1.525, z=+2.328 deg/s, i.e.
|bias| = 3.679 deg/s. A healthy MEMS gyro sits at 0.01-0.1 deg/s, so this is
40-400x out of spec. This matters far more than the accel scale error, because
attitude is the integral of the gyro: 3.7 deg/s of drift means ~37 deg of
attitude error after 10 s, which mis-projects GRAVITY into the body frame and
injects a phantom horizontal acceleration of 9.79*sin(37deg) ~ 5.9 m/s^2 --
five times worse than the 1.06 m/s^2 accel residual. Double-integrated that is
~300 m in 10 s, matching the measured VINS divergence (501 m -> 1100 m in 12 s).
Correcting the accelerometer ALONE was measured to be insufficient.

AUTO-CALIBRATION rather than a frozen constant: a gyro bias drifts with
temperature, and this Pi runs at 86 deg C, so a hardcoded offset would go stale.
At startup the node therefore collects ~gyro_cal_samples and, IF the gyro
variance shows the robot is not rotating, adopts the measured mean as the bias.
Translation does not affect a gyro, so a stationary-or-straight-driving robot
gives a valid estimate; a rotating one is rejected (variance too high) and the
~gyro_bias_* parameter defaults are kept instead. Re-runs on every node
restart, so thermal drift is re-absorbed for free.

CAVEAT, same as the accel scale: this compensates a CONSTANT offset. It does not
fix in-run thermal drift, and openVINS is in principle supposed to estimate this
bias itself during static init -- why it fails to is NOT yet established (bias
too large for its initial guess, or the "static" window not actually static
when the operator jerks the robot). Set ~gyro_autocal:=false and
~gyro_bias_*:=0.0 to disable entirely.

KNOWN LIMITATION, stated deliberately: this factor comes from a measurement at
a SINGLE orientation, which cannot distinguish a scale error (sensor multiplies
everything by 0.894) from a per-axis bias (constant offset on Z). Both look
identical when the robot sits level. The rover operates essentially level, so
the correction is defensible in its working regime, but it remains empirical.
A 6-position calibration (robot tilted/inverted) is required to settle it.
Set ~accel_scale to 1.0 to disable and recover the raw behaviour.

ZUPT DETECTOR (2026-08-19): the gyro-recalibration guard above already answers
"is the robot really stopped?" from the wheels alone (_immobile_depuis()) —
this is that same question asked once more, this time so ANYONE downstream can
answer it too, without re-deriving it. ~zupt_topic publishes std_msgs/Bool,
latched, on every EDGE (not at IMU rate — a consumer that wants the current
state gets it immediately via latch; one that wants to know it changed gets an
edge). True requires BOTH, cumulatively, same spirit as evaluer_recal():
  1. wheels confirm standstill for >= ~wheel_still_time (the existing,
     already-tested signal — not re-implemented here, _immobile_depuis() is
     called directly);
  2. accelerometer std, on the corrected signal, stays under
     ~zupt_accel_std_max over the trailing ~zupt_window_s — a SECOND,
     independent witness. Wheels can be zero while the chassis is still being
     jostled (robot lifted with wheels off the ground spinning freely does NOT
     apply here since wheel encoders read the wheel, not the ground — but a
     robot rocked in place with wheels braked would fool wheels alone).
Consumers: is_stationary is a passive OBSERVER (a MATLAB/analysis script, a
dashboard, or the clamp below). See tools/README.md for what currently
consumes stillness.

CLAMP ZUPT (2026-08-19, ~zupt_clamp_enable, default false): openVINS ALREADY
runs its own native ZUPT (try_zupt, config/leo/estimator_config.yaml — on
since 2026-07-28), triggered by ITS OWN chi2/variance/disparity heuristic on
the IMU it receives. openVINS exposes no ROS topic to force a zupt from the
outside — there is no wire to attach a wheel-confirmed "stop now" signal to
inside its own filter without patching its C++ source. This clamp is the
alternative that needs no source patch: when is_stationary is true (wheel-
confirmed, not openVINS's own guess), /imu/data_clean publishes the trailing
window MEAN instead of the raw sample. The CORE2's own sensor noise (std
~0.02-0.05, measured 2026-07-28 for a different purpose, reused here) is
exactly what can keep openVINS's internal chi2 test from firing at a real
stop; smoothing it away during a period we independently KNOW is static does
not fabricate anything — the window only feeds the mean once its OWN std has
already cleared ~zupt_accel_std_max (see _update_zupt()).

TIMESTAMP-GAP FIX (2026-08-20): a live drive exposed a second bug in the accel
witness — with rospy.get_time() already fixed to sensor-timestamp indexing
(see above), IMU messages can still arrive with real >1s GAPS between sensor
stamps under CPU load (MINS+openVINS competing for the PC's CPU), which empty
the rolling window and used to force accel_ok=False — asserting "moving" from
an absence of data, not from evidence of motion. Fixed: when the window is too
thin to judge (span < 80% of ~zupt_window_s), the PREVIOUS accel verdict is
held instead. This is safe specifically because wheels_ok (the dominant, wheel
-based witness) is untouched by this and still drops immediately on any real
wheel motion — holding the accel verdict never overrides that.

RECAL ACCEL (2026-08-20, ~accel_recal_period, default 0.0 = OFF): the clean
10-min drive test that validated gyro recal + clamp (t^3.25 -> t^0.74, R^2
0.94) showed the exponent move to n~1, the signature of a VELOCITY error, not
a gyro one. Chief suspect: ~accel_scale is still a SINGLE constant (1.1186)
measured ONCE (2026-07-28) and never re-checked — exactly the gyro bias
problem this file already solved once, now on the other sensor.

WHY THIS CAN mirror gyro_recal_period, and why it CANNOT go further: |a| at
rest equals local gravity magnitude, and a VECTOR NORM IS ROTATION-INVARIANT
— same argument as "true gyro rate at rest is 0 in any orientation" (see
GYRO BIAS SUBTRACTION above). So periodically re-measuring |a| at a confirmed
stop and correcting the SCALE factor is exactly as sound as the existing gyro
recal, reusing the SAME is_stationary signal (wheels + accel-variance already
both confirmed by ZUPT DETECTOR above) rather than re-deriving stillness.

What this does NOT attempt: a full 3-axis ADDITIVE bias (a vector, not a
scalar) estimated from repeated stationary sampling. Unlike the gyro, the
accelerometer's TRUE value at rest is the gravity vector PROJECTED INTO BODY
FRAME, which depends on orientation — and this rover always rests at
essentially the SAME orientation (a ground vehicle, not a drone). Sampling
"what does it read at rest" repeatedly at the same tilt cannot distinguish:
  (a) a genuine per-axis electrical/scale defect in the sensor,
  (b) the IMU's MOUNTING misalignment relative to base_link,
  (c) the ground simply not being perfectly level under the robot right now.
This is the SAME confound the KNOWN LIMITATION paragraph already documents
for the existing single-orientation ~accel_scale measurement — attempting a
3-axis version from live is_stationary sampling would not fix that limitation,
it would just repeat it with more decimal places of false confidence. The
correct fix for a real per-axis bias is a MULTI-ORIENTATION calibration
(robot tilted/inverted, see tools/calib_terrain.py's pattern) — a deliberate
offline exercise, not something this node should attempt unsupervised.

~accel_recal_max_delta (default 0.03 = 3% relative — NOT measured, thermal
drift of a MEMS accel's scale over normal operating temperature swings is
expected to be small; conservative until a real multi-hour capture bounds it,
same honesty as ~gyro_recal_max_delta's own history).

Expected effect: the
existing periodic gyro-bias recalibration gets more reliable stops to work
with, and openVINS's own velocity-zeroing fires more consistently. It is NOT
expected to fix the t^3/t^4 drift by itself — see ~gyro_recal_period above for
the bias-recalibration mechanism this complements, not replaces.

PARAMS
  ~in_topic     (default /firmware/imu)
  ~out_topic    (default /imu/data_clean)
  ~frame_id     (default imu_frame — matches leo_bringup convention)
  ~gyro_limit   (default 35.0 rad/s)
  ~accel_limit  (default 160.0 m/s^2)
  ~accel_scale  (default 1.1186 — see ACCEL SCALE CORRECTION above; 1.0 = off)
  ~gyro_scale   (default 1.0 = off — never measured; use tools/calib_terrain.py)
  ~gyro_bias_x/_y/_z (defaults -0.04200/+0.02662/+0.04062 rad/s — measured)
  ~gyro_autocal        (default true — re-measure the bias at startup if still)
  ~gyro_cal_samples    (default 400 ~ 5 s at 80 Hz)
  ~gyro_cal_max_std    (default 0.01 rad/s — above this the robot is rotating,
                        the auto-calibration is rejected and the params stand)
  ~gyro_recal_period   (default 0.0 = OFF — seconds between periodic re-estimations
                        of the gyro bias while the robot is stopped. The startup
                        auto-calibration runs ONCE; a MEMS bias drifts with
                        temperature and the Pi heats up. Measured evidence, not
                        conjecture: openVINS drift on the 13/08 lap grows as
                        t^3.25 and t^4.26 over its two segments. An accelerometer
                        SCALE error would give t^2; t^3 is the signature of an
                        attitude error growing linearly, i.e. an uncompensated
                        constant gyro bias, and t^4 of a bias drifting on its own.
                        MINS on the same recording sits at t^-0.01 — its wheels
                        anchor it, so it never showed the problem.
                        OFF by default: this path feeds BOTH estimators and the
                        driving, and it could not be validated on the robot.
                        Enable it for a supervised run first.)
  ~gyro_recal_max_delta (default 0.02 rad/s — a bias that jumps more than this is
                        not drifting, it was measured during motion. Rejected.)
  ~accel_recal_period   (default 0.0 = OFF — seconds between periodic re-estimations
                        of ~accel_scale while is_stationary. See RECAL ACCEL above —
                        SCALE only, never a 3-axis bias.)
  ~accel_recal_max_delta (default 0.03 = 3% relative — see RECAL ACCEL above)
  ~gravite_locale       (default 9.790 m/s^2 — Melbourne FL, lat 28 deg; recompute
                        for a different site, see the ACCEL SCALE CORRECTION note)
  ~wheel_topic         (default /firmware/wheel_states — the stationarity signal.
                        Wheel encoders, not gyro variance alone: a biased gyro
                        turning at constant rate has LOW variance and would pass
                        a variance-only test.)
  ~wheel_still_eps     (default 0.01 — |wheel velocity| below this counts as stopped)
  ~wheel_still_time    (default 3.0 s of confirmed standstill before re-measuring)
  ~zupt_topic          (default /imu_sanitizer/is_stationary — std_msgs/Bool, latched)
  ~zupt_accel_std_max  (default 0.05 m/s^2 — deliberately conservative, NOT measured
                        for this specific purpose: the 0.02 std figure in the
                        accel-scale note above came from a 502-sample stationary
                        capture for a different measurement. Tune down once a
                        real at-rest capture is taken; see tools/calib_terrain.py
                        pattern for how prior thresholds here were established.)
  ~zupt_window_s       (default 1.0 s — trailing window for the accel std, matches
                        the "1 second consecutive" requirement as specified)
  ~zupt_clamp_enable   (default false — see CLAMP ZUPT above; publishes the
                        window MEAN instead of raw samples while is_stationary
                        is true. OFF until field-validated.)
  ~freeze_samples      (default 200 — consecutive identical samples => frozen)
  ~freeze_autoreset    (default true — call /firmware/reset_board on freeze)
  ~freeze_reset_cooldown (default 60.0 s between two auto-resets)

LAUNCH: rosrun leo_navigation imu_sanitizer.py   (or via pose_selector.launch)
"""
import threading
from collections import deque

import rospy
from leo_msgs.msg import Imu as LeoImu
from leo_msgs.msg import WheelStates
from sensor_msgs.msg import Imu as SensorImu
from std_msgs.msg import Bool
from std_srvs.srv import Trigger



def evaluer_recal(means, stds, bias_actuel, immobile_depuis,
                  max_std, max_delta, still_time):
    """Décide si un biais gyro fraîchement mesuré doit remplacer l'actuel.

    Fonction PURE : pas de ROS, pas d'état, pas d'horloge. Elle existe pour
    que le nœud ET les tests exercent EXACTEMENT le même code. Un test qui
    réimplémenterait ces règles ne prouverait rien : il vérifierait sa propre
    copie, et une divergence entre les deux passerait inaperçue.

    Renvoie (accepte, motif) — motif toujours renseigné, y compris en cas
    d'acceptation, pour que le journal dise pourquoi et pas seulement quoi.

    L'ordre des gardes n'est pas indifférent : on écarte d'abord ce qui est le
    moins coûteux à établir (le robot bougeait-il ?) avant ce qui demande une
    statistique.
    """
    if immobile_depuis is None:
        return False, "aucune donnée roue — l'immobilité n'est pas établie"
    if immobile_depuis < still_time:
        return False, ("robot arrêté depuis %.1f s seulement (%.1f s exigées)"
                       % (immobile_depuis, still_time))
    pire_std = max(stds)
    if pire_std > max_std:
        return False, ("écart-type %.4f > %.4f rad/s : ça tourne encore "
                       "(robot soulevé ?)" % (pire_std, max_std))
    delta = max(abs(means[i] - bias_actuel[i]) for i in range(3))
    if delta > max_delta:
        return False, ("écart %.4f > %.4f rad/s : une dérive thermique est "
                       "lente, ce saut vient d'une mesure prise en mouvement"
                       % (delta, max_delta))
    return True, "écart %.4f rad/s, dérive plausible" % delta


def evaluer_recal_accel_scale(norme_moyenne, echelle_actuelle, gravite_locale,
                              max_delta_relatif):
    """Décide si une échelle accéléro fraîchement mesurée doit remplacer
    l'actuelle. Miroir de evaluer_recal() ci-dessus, mêmes principes : fonction
    PURE (le nœud et les tests appellent EXACTEMENT ce code), et un motif
    toujours renseigné.

    Une seule garde ici, pas trois : l'appelant ne se sert de cette fonction
    QU'après que is_stationary soit passé à vrai, qui a DÉJÀ validé roues +
    variance accéléro (voir _update_zupt). Les répéter ici serait de la
    duplication, pas de la prudence — la même leçon que evaluer_recal()
    documente pour le gyro s'applique à l'inverse : ne pas retester ce
    qu'un autre code, déjà appelé, a déjà établi.

    norme_moyenne : |a| moyen BRUT (avant échelle) sur la fenêtre d'arrêt.
    echelle_actuelle : ~accel_scale en cours.
    Renvoie (accepte, motif, nouvelle_echelle).
    """
    if norme_moyenne <= 0:
        return False, "norme non-positive — mesure aberrante", echelle_actuelle
    nouvelle_echelle = gravite_locale / norme_moyenne
    delta_rel = abs(nouvelle_echelle - echelle_actuelle) / echelle_actuelle
    if delta_rel > max_delta_relatif:
        return False, ("écart %.2f%% > %.2f%% : une dérive thermique d'échelle "
                       "est lente, ce saut vient d'une mesure prise pendant un "
                       "mouvement (ou d'un capteur soulevé)"
                       % (100.0 * delta_rel, 100.0 * max_delta_relatif)), echelle_actuelle
    return True, "écart %.2f%%, dérive plausible" % (100.0 * delta_rel), nouvelle_echelle


class ImuSanitizer(object):
    def __init__(self):
        self.gyro_limit = rospy.get_param("~gyro_limit", 35.0)
        self.accel_limit = rospy.get_param("~accel_limit", 160.0)
        # 9.790 (gravité locale réelle) / 8.7525 (mesuré brut) — voir en-tête.
        self.accel_scale = float(rospy.get_param("~accel_scale", 1.1186))
        # ── BRANCHE VINS SÉPARÉE (2026-08-25) ────────────────────────────
        # POURQUOI DEUX ÉCHELLES ET DEUX TOPICS.
        # /imu/data_clean est consommé par MINS, openVINS, sqrtVINS ET le
        # backend — vérifié par `rostopic info` le 25/08. Une seule échelle
        # pour tous crée un conflit insoluble :
        #   * l'échelle DÉRIVE (8.7525 -> 8.6664 m/s^2 de brut mesuré entre
        #     deux campagnes), et un résidu de 0.0958 m/s^2 doublement intégré
        #     vaut ~17 km sur 10 minutes — fatal pour un VIO pur ;
        #   * MINS est ancré par les roues, survit à ce résidu, est validé,
        #     et toute modification de son entrée est interdite.
        # Corriger /imu/data_clean réparerait les VINS en cassant MINS ; ne
        # rien faire laisse les VINS inexploitables. La sortie est de séparer
        # les flux : /imu/data_clean reste identique au octet près pour MINS,
        # et les VINS lisent /imu/data_vins avec leur propre échelle,
        # recalibrable sans conséquence sur MINS.
        #
        # 0.0 = INACTIF : /imu/data_vins reste alors une copie exacte de
        # /imu/data_clean. Le topic est publié dans TOUS les cas, pour qu'un
        # estimateur configuré dessus ne se retrouve jamais sans IMU du tout
        # parce que le paramètre n'a pas été passé.
        self.accel_scale_vins = float(rospy.get_param("~accel_scale_vins", 0.0))
        # ── GYRO SCALE (ajouté 2026-07-29) ───────────────────────────────
        # Défaut 1.0 = AUCUNE correction, volontairement : ce facteur n'a
        # jamais été mesuré. La calibration du 28/07 s'est faite à UNE SEULE
        # orientation, ce qui donne le biais mais ne peut pas distinguer une
        # erreur d'échelle (cf. GYRO BIAS ci-dessus, même limite). Le mesurer
        # demande une vérité terrain : tools/calib_terrain.py rotation, qui
        # compare un 90 deg réel au sol à ce que le gyro intègre. Tant que
        # cette mesure n'est pas faite, 1.0 est le seul choix honnête — un
        # facteur inventé fausserait tous les caps, donc toutes les
        # trajectoires, pour les DEUX estimateurs à la fois.
        self.gyro_scale = float(rospy.get_param("~gyro_scale", 1.0))
        # Biais gyro mesuré robot immobile le 2026-07-28 (297 échantillons).
        self.gyro_bias = [float(rospy.get_param("~gyro_bias_x", -0.04200)),
                          float(rospy.get_param("~gyro_bias_y", +0.02662)),
                          float(rospy.get_param("~gyro_bias_z", +0.04062))]
        self.gyro_autocal = bool(rospy.get_param("~gyro_autocal", True))
        self.gyro_cal_samples = int(rospy.get_param("~gyro_cal_samples", 400))
        self.gyro_cal_max_std = float(rospy.get_param("~gyro_cal_max_std", 0.01))
        # ── Recalibration PÉRIODIQUE du biais gyro (2026-08-18) ──────────────
        # Pourquoi : l'auto-calibration ci-dessus ne tourne QU'UNE FOIS, au
        # démarrage (_cal_done passe à True et n'en redescend jamais). Or un
        # biais de gyro MEMS dérive avec la température, et le Pi chauffe.
        # L'analyse des trajectoires enregistrées le confirme plutôt qu'elle ne
        # le suppose : la dérive d'openVINS croît en t^3,25 et t^4,26 sur les
        # deux segments du roulage du 13/08. Une erreur d'ÉCHELLE accéléromètre
        # donnerait t^2 (accélération constante doublement intégrée) ; t^3
        # est la signature d'une erreur d'attitude qui croît linéairement,
        # c'est-à-dire d'un biais gyro constant non compensé, et t^4 celle d'un
        # biais qui dérive lui-même. MINS, sur le même enregistrement, reste à
        # t^-0,01 : ses roues l'ancrent, il ne voit pas le problème.
        #
        # DÉSACTIVÉ PAR DÉFAUT (période = 0). Ce chemin alimente les DEUX
        # estimateurs et le pilotage ; il n'a pas pu être validé sur le robot,
        # injoignable au moment de l'écriture. L'activer demande un essai
        # supervisé — voir l'en-tête du module.
        self.gyro_recal_period = float(rospy.get_param("~gyro_recal_period", 0.0))
        # Garde-fou : un biais qui « sauterait » de plus de ça n'est pas un
        # biais qui dérive, c'est une mesure prise pendant un mouvement.
        self.gyro_recal_max_delta = float(rospy.get_param("~gyro_recal_max_delta", 0.02))
        # Immobilité EXIGÉE par les roues, pas seulement par la variance gyro :
        # un gyro biaisé tournant à vitesse constante a une variance faible et
        # passerait le test de variance seul. Les roues, elles, ne mentent pas
        # sur l'immobilité.
        self.wheel_topic = rospy.get_param("~wheel_topic", "/firmware/wheel_states")
        self.wheel_still_eps = float(rospy.get_param("~wheel_still_eps", 0.01))
        self.wheel_still_time = float(rospy.get_param("~wheel_still_time", 3.0))
        self._last_motion_t = None      # None = aucune donnée roue reçue
        self._next_recal_t = None
        self._recal_buf = []
        # ── ZUPT DETECTOR (2026-08-19) — voir ZUPT DETECTOR en en-tête ───────
        # Publie is_stationary = roues immobiles ET variance accéléro basse.
        # Le premier signal est _immobile_depuis(), déjà écrit et déjà testé
        # pour la recalibration ci-dessus — réutilisé tel quel, pas réécrit.
        self.zupt_accel_std_max = float(rospy.get_param("~zupt_accel_std_max", 0.05))
        self.zupt_window_s = float(rospy.get_param("~zupt_window_s", 1.0))
        zupt_topic = rospy.get_param("~zupt_topic", "/imu_sanitizer/is_stationary")
        self._accel_buf = deque()       # [(t, gx,gy,gz, ax,ay,az)], fenêtre glissante
        self._is_stationary = False     # dernier état publié (pour ne publier que sur front)
        self._imu_mean = None           # moyenne fenêtre courante, voir _update_zupt()
        self._accel_ok_prev = False     # dernier verdict accel connu (2026-08-19, trous d'horodatage —
                                         # voir _update_zupt() ; False au démarrage, sans mesure encore)
        # ── CLAMP ZUPT (2026-08-19) — voir CLAMP ZUPT en en-tête ─────────────
        # DÉSACTIVÉ PAR DÉFAUT, même discipline que gyro_recal_period : ce
        # chemin change ce que /imu/data_clean publie à CHAQUE arrêt confirmé,
        # pour les deux estimateurs et pour la conduite. Pas encore validé sur
        # le robot.
        self.zupt_clamp_enable = bool(rospy.get_param("~zupt_clamp_enable", False))
        # ── RECALIBRATION PÉRIODIQUE DE L'ÉCHELLE ACCÉLÉRO (2026-08-20) ──────
        # Voir RECAL ACCEL EN-TÊTE pour la justification physique complète.
        # Résumé : |a| au repos = gravité locale, INVARIANTE PAR ROTATION —
        # même argument que le gyro (vrai taux = 0, quelle que soit
        # l'orientation). C'est pour ça que ~accel_scale (un seul facteur
        # scalaire) peut être réestimé à l'arrêt, périodiquement, EXACTEMENT
        # comme le biais gyro. Ce qui NE PEUT PAS être fait de la même façon :
        # un biais ADDITIF par axe — la valeur vraie au repos dépend alors de
        # l'orientation, que ce rover ne change jamais vraiment (roule à
        # plat). Voir RECAL ACCEL en-tête.
        self.accel_recal_period = float(rospy.get_param("~accel_recal_period", 0.0))
        # Une dérive thermique d'échelle MEMS est lente ; borne relative, pas
        # absolue, car le facteur lui-même est proche de 1.12 et non de 0.
        self.accel_recal_max_delta = float(rospy.get_param("~accel_recal_max_delta", 0.03))
        self.gravite_locale = float(rospy.get_param("~gravite_locale", 9.790))
        self._next_accel_recal_t = None
        self._accel_recal_buf = []   # normes |a| BRUTES accumulées pendant l'arrêt confirmé
        if self.accel_recal_period > 0.0:
            rospy.logwarn("[imu_sanitizer] recalibration ECHELLE ACCELERO PERIODIQUE "
                          "active (%.0f s, immobilité confirmée par is_stationary)",
                          self.accel_recal_period)
        self._cal_buf = []          # échantillons gyro bruts pour l'auto-calib
        self._cal_done = not self.gyro_autocal
        self.frame_id = rospy.get_param("~frame_id", "imu_frame")
        out_topic = rospy.get_param("~out_topic", "/imu/data_clean")
        in_topic = rospy.get_param("~in_topic", "/firmware/imu")

        self._last_good = None       # (gx,gy,gz,ax,ay,az)
        self._n = 0
        self._n_glitch = 0
        self._last_report = rospy.Time.now()

        # monotonicity guard: rosserial desync can replay stamps up to ~0.4s
        # BACKWARD (observed live 2026-07-06); a single backward stamp aborts
        # MINS on SystemManager.cpp:395 `clone_time >= state->time`. Any
        # sample whose stamp <= the last forwarded stamp is dropped entirely
        # (no ZOH republish — a duplicate stamp is as fatal as a backward one).
        self._last_stamp = None
        self._n_nonmono = 0
        # Compteurs de la garde de déverrouillage (2026-09-04, cf. _cb) :
        # _suite compte les rejets CONSÉCUTIFS (remis à zéro dès qu'une trame
        # passe), _deverrou compte les réamorçages, publié dans le diagnostic
        # pour qu'un verrouillage récurrent soit visible et non deviné.
        self._n_nonmono_suite = 0
        self._n_deverrou = 0
        # Une référence datée de plus de STAMP_FUTUR_MAX_S dans le futur ne
        # peut pas venir d'un capteur sain : 60 s couvre largement une
        # désynchronisation NTP tolérable, et exclut la trame à +50 ans qui a
        # verrouillé le nœud le 2026-09-04.
        self.stamp_futur_max_s = float(rospy.get_param("~stamp_futur_max_s", 60.0))
        # Filet générique : ~3 s de flux à 85 Hz. Assez long pour ne pas se
        # déclencher sur une rafale de trames désordonnées, assez court pour
        # qu'un blocage ne survive jamais à un essai.
        self.stamp_rejets_max = int(rospy.get_param("~stamp_rejets_max", 250))

        # freeze guard (see module docstring)
        self.freeze_samples = int(rospy.get_param("~freeze_samples", 200))
        self.freeze_autoreset = bool(rospy.get_param("~freeze_autoreset", True))
        self.freeze_cooldown = float(rospy.get_param("~freeze_reset_cooldown", 60.0))
        self._prev_vals = None
        self._identical_count = 0
        self._last_reset = rospy.Time(0)
        self._reset_in_flight = False

        self.pub = rospy.Publisher(out_topic, SensorImu, queue_size=20)
        vins_topic = rospy.get_param("~vins_topic", "/imu/data_vins")
        self.pub_vins = rospy.Publisher(vins_topic, SensorImu, queue_size=20)
        # Latché : un abonné qui arrive en cours de route (dashboard, script
        # d'analyse) obtient l'état courant tout de suite, pas au prochain front.
        self.zupt_pub = rospy.Publisher(zupt_topic, Bool, queue_size=5, latch=True)
        rospy.Subscriber(in_topic, LeoImu, self._cb, queue_size=50, tcp_nodelay=True)
        # Toujours abonné aux roues, plus seulement si la recalibration gyro est
        # active : le détecteur ZUPT (is_stationary) est un service indépendant,
        # utile même quand gyro_recal_period=0.
        rospy.Subscriber(self.wheel_topic, WheelStates, self._cb_wheels,
                         queue_size=10, tcp_nodelay=True)
        rospy.loginfo("[imu_sanitizer] détecteur ZUPT actif -> %s (roues immobiles "
                      ">= %.1fs ET écart-type accéléro <= %.3f m/s^2 sur %.1fs)",
                      zupt_topic, self.wheel_still_time, self.zupt_accel_std_max,
                      self.zupt_window_s)
        if self.gyro_recal_period > 0.0:
            rospy.logwarn("[imu_sanitizer] recalibration gyro PÉRIODIQUE active "
                          "(%.0f s, immobilité confirmée par %s)",
                          self.gyro_recal_period, self.wheel_topic)
        rospy.loginfo("[imu_sanitizer] %s (leo_msgs/Imu) -> %s (sensor_msgs/Imu), "
                       "gyro_limit=%.1f accel_limit=%.1f accel_scale=%.4f",
                       in_topic, out_topic, self.gyro_limit, self.accel_limit,
                       self.accel_scale)
        import math as _m
        _b = _m.degrees(_m.sqrt(sum(v * v for v in self.gyro_bias)))
        if _b > 1e-6:
            rospy.logwarn("[imu_sanitizer] SOUSTRACTION BIAIS GYRO ACTIVE : "
                          "[%.5f %.5f %.5f] rad/s (|b|=%.3f deg/s ; un MEMS sain "
                          "est à 0.01-0.1). Sans elle, l'attitude dérive et la "
                          "gravité est mal projetée -> ~5.9 m/s^2 fantômes. "
                          "auto-calib au démarrage : %s",
                          self.gyro_bias[0], self.gyro_bias[1], self.gyro_bias[2],
                          _b, "ON" if self.gyro_autocal else "OFF")
        if abs(self.accel_scale - 1.0) > 1e-6:
            rospy.logwarn("[imu_sanitizer] CORRECTION ACCELEROMETRE ACTIVE "
                          "(x%.4f) : accéléro CORE2 mesuré 10.6%% sous la "
                          "gravité locale (8.7525 vs 9.790 m/s^2). Sans elle "
                          "openVINS diverge (~1900 m/min). Correction empirique "
                          "sur UNE orientation — calibration 6 positions requise "
                          "pour la valider. ~accel_scale:=1.0 pour désactiver.",
                          self.accel_scale)

    def _is_glitch(self, m):
        g, a = self.gyro_limit, self.accel_limit
        return (abs(m.gyro_x) > g or abs(m.gyro_y) > g or abs(m.gyro_z) > g or
                abs(m.accel_x) > a or abs(m.accel_y) > a or abs(m.accel_z) > a)

    def _try_autocal(self, gx, gy, gz):
        """Auto-calibration du biais gyro au démarrage. Accumule des
        échantillons BRUTS ; quand il y en a assez, n'adopte la moyenne comme
        biais QUE si l'écart-type est faible sur les trois axes — sinon le
        robot tournait pendant la mesure et la moyenne ne serait pas un biais
        mais une vraie vitesse de rotation. La translation n'affecte pas un
        gyro : robot immobile OU en ligne droite donnent tous deux une mesure
        valide."""
        self._cal_buf.append((gx, gy, gz))
        if len(self._cal_buf) < self.gyro_cal_samples:
            return
        self._cal_done = True
        n = float(len(self._cal_buf))
        means = [sum(s[i] for s in self._cal_buf) / n for i in range(3)]
        stds = [ (sum((s[i] - means[i]) ** 2 for s in self._cal_buf) / n) ** 0.5
                 for i in range(3) ]
        self._cal_buf = []
        if max(stds) > self.gyro_cal_max_std:
            rospy.logwarn("[imu_sanitizer] auto-calib gyro REJETÉE (écart-type "
                          "%.4f > %.4f rad/s : le robot tournait). Biais des "
                          "paramètres conservé : [%.5f %.5f %.5f] rad/s",
                          max(stds), self.gyro_cal_max_std, *self.gyro_bias)
            return
        old = list(self.gyro_bias)
        self.gyro_bias = means
        import math as _m
        rospy.logwarn("[imu_sanitizer] auto-calib gyro OK sur %d échantillons : "
                      "biais [%.5f %.5f %.5f] rad/s (|b|=%.3f deg/s) — "
                      "remplace [%.5f %.5f %.5f]",
                      int(n), means[0], means[1], means[2],
                      _m.degrees(_m.sqrt(sum(v * v for v in means))), *old)

    def _cb_wheels(self, m):
        """Horodate le dernier instant où une roue tournait. C'est tout ce dont
        la recalibration a besoin : savoir depuis combien de temps le robot est
        VRAIMENT arrêté. Volontairement passif — ce rappel ne calibre rien, il
        ne fait qu'observer, pour que la calibration reste pilotée par le flux
        IMU et par lui seul."""
        try:
            bouge = any(abs(v) > self.wheel_still_eps for v in m.velocity)
        except Exception:
            return
        now = rospy.get_time()
        if bouge or self._last_motion_t is None:
            self._last_motion_t = now

    def _immobile_depuis(self):
        """Durée d'immobilité confirmée par les roues, ou None si on ne sait
        pas. Ne JAMAIS renvoyer « immobile » faute de données : sans message
        roue, on ignore l'état, et ignorer n'est pas savoir."""
        if self._last_motion_t is None:
            return None
        return rospy.get_time() - self._last_motion_t

    def _update_zupt(self, t_capteur, gx, gy, gz, ax, ay, az):
        """Détecteur ZUPT : publie is_stationary sur std_msgs/Bool, sur front
        seulement (latché, donc un abonné tardif obtient quand même l'état
        courant). Appelée sur le signal CORRIGÉ (post accel_scale/gyro_bias) —
        un glitch a déjà été écarté par _is_glitch() avant que _cb() n'arrive
        ici, donc le tampon ne voit jamais un pic de 1e16.

        Deux témoins, aucun ne suffit seul :
          - roues : _immobile_depuis(), déjà écrite et déjà testée pour la
            recalibration gyro — réutilisée telle quelle ;
          - accéléromètre : écart-type sur la fenêtre glissante ~zupt_window_s.

        Tient aussi à jour self._imu_mean = moyenne (gx,gy,gz,ax,ay,az) sur la
        fenêtre — c'est la valeur que _cb() publie à la place du brut quand le
        clamp (~zupt_clamp_enable) est actif et le robot confirmé immobile.
        Calculée ici plutôt que recalculée dans _cb() : un seul passage sur le
        tampon pour la décision ET pour la moyenne.
        """
        # Base de temps = HORODATAGE DU CAPTEUR, pas rospy.get_time().
        # Mesuré le 2026-08-19 : avec l'heure d'ARRIVÉE, le détecteur basculait
        # 8 fois par minute robot immobile. Ce n'était pas le bruit du capteur
        # (100 s de mesure : 0 dépassement sur 7714 fenêtres) mais la famine du
        # rappel — MINS et openVINS saturent le CPU du PC, le rappel saute plus
        # d'une seconde, le tampon se vide et len<2 fait retomber accel_ok.
        # L'horodatage capteur ne bouge pas quand le traitement prend du retard.
        self._accel_buf.append((t_capteur, gx, gy, gz, ax, ay, az))
        cutoff = t_capteur - self.zupt_window_s
        while self._accel_buf and self._accel_buf[0][0] < cutoff:
            self._accel_buf.popleft()

        immo = self._immobile_depuis()
        wheels_ok = immo is not None and immo >= self.wheel_still_time

        # Fenêtre insuffisante (len<2 OU span<80% de zupt_window_s) : ce n'est
        # PAS une preuve de mouvement, seulement une absence de mesure. Mesuré
        # en roulage le 2026-08-19 : les horodatages IMU présentent des trous
        # de >1s sous charge CPU (MINS+openVINS), qui vidaient le tampon et
        # faisaient retomber accel_ok à False à chaque trou — même roues
        # confirmées immobiles. Conserver la dernière décision accel connue
        # (self._accel_ok_prev) plutôt que d'affirmer « ça bouge » sur une
        # absence de données : les roues (wheels_ok) restent le témoin
        # dominant et retombent, elles, dès qu'un VRAI message de mouvement
        # arrive — cette conservation ne les contourne jamais.
        pire_std = float('nan')
        span = 0.0
        fenetre_ok = wheels_ok and len(self._accel_buf) >= 2
        if fenetre_ok:
            span = self._accel_buf[-1][0] - self._accel_buf[0][0]
            fenetre_ok = span >= self.zupt_window_s * 0.8

        if fenetre_ok:
            n = float(len(self._accel_buf))
            means = [sum(s[i] for s in self._accel_buf) / n for i in range(1, 7)]
            self._imu_mean = tuple(means)
            a_stds = [(sum((s[i] - means[i - 1]) ** 2 for s in self._accel_buf) / n) ** 0.5
                      for i in range(4, 7)]
            pire_std = max(a_stds)
            accel_ok = pire_std <= self.zupt_accel_std_max
            self._accel_ok_prev = accel_ok
        elif wheels_ok:
            accel_ok = self._accel_ok_prev   # trou de données : on garde le dernier verdict
        else:
            accel_ok = False                 # roues en mouvement : pas d'ambiguïté à lever

        stationary = wheels_ok and accel_ok
        if stationary != self._is_stationary:
            self._is_stationary = stationary
            self.zupt_pub.publish(Bool(data=stationary))
            # Journaliser l'écart-type, la garniture de la fenêtre ET si la
            # décision vient d'une mesure fraîche ou d'un maintien : sans ça,
            # un « accel=False » ne dit pas SI c'est le capteur qui bouge ou
            # la fenêtre qui est trop courte. Distinguer ces deux causes a
            # coûté une heure le 2026-08-19.
            rospy.loginfo("[imu_sanitizer] ZUPT is_stationary -> %s (roues=%s "
                          "accel=%s std=%.4f seuil=%.4f fenetre=%.2fs n=%d frais=%s)",
                          stationary, wheels_ok, accel_ok, pire_std,
                          self.zupt_accel_std_max, span, len(self._accel_buf), fenetre_ok)

    def _try_recal(self, gx, gy, gz):
        """Recalibration périodique du biais gyro, à l'arrêt.

        Trois conditions cumulatives, et aucune n'est redondante :
          1. les ROUES confirment l'arrêt depuis ~wheel_still_time — un gyro
             biaisé tournant à vitesse constante aurait une variance faible et
             tromperait le seul test de variance ;
          2. l'écart-type gyro reste sous le même seuil qu'au démarrage — les
             roues peuvent être à l'arrêt pendant qu'on soulève le robot ;
          3. le nouveau biais ne s'écarte pas de plus de ~gyro_recal_max_delta
             de l'actuel — une dérive thermique est lente ; un saut brutal
             signale une mesure prise pendant un mouvement, pas une dérive.
        Si l'une échoue, l'ancien biais est CONSERVÉ. Le défaut de cette
        fonction doit être de ne rien faire, jamais d'adopter une valeur
        douteuse : elle alimente les deux estimateurs et le pilotage.
        """
        now = rospy.get_time()
        if self._next_recal_t is None:
            self._next_recal_t = now + self.gyro_recal_period
            return
        if now < self._next_recal_t:
            return
        immo = self._immobile_depuis()
        if immo is None or immo < self.wheel_still_time:
            self._recal_buf = []          # le robot bouge : on repart de zéro
            return

        self._recal_buf.append((gx, gy, gz))
        if len(self._recal_buf) < self.gyro_cal_samples:
            return

        n = float(len(self._recal_buf))
        means = [sum(s[i] for s in self._recal_buf) / n for i in range(3)]
        stds = [(sum((s[i] - means[i]) ** 2 for s in self._recal_buf) / n) ** 0.5
                for i in range(3)]
        self._recal_buf = []
        self._next_recal_t = now + self.gyro_recal_period

        # Décision déléguée à evaluer_recal() : le nœud et les tests doivent
        # exercer le MÊME code, sinon un test ne valide que sa propre copie.
        accepte, motif = evaluer_recal(
            means, stds, self.gyro_bias, immo,
            self.gyro_cal_max_std, self.gyro_recal_max_delta,
            self.wheel_still_time)
        if not accepte:
            rospy.logwarn("[imu_sanitizer] recalib gyro REFUSÉE — %s. "
                          "Biais conservé.", motif)
            return
        import math as _m
        old = list(self.gyro_bias)
        self.gyro_bias = means
        rospy.logwarn("[imu_sanitizer] recalib gyro APPLIQUÉE après %.0f s "
                      "d'arrêt (%s) : [%.5f %.5f %.5f] (|b|=%.3f deg/s) — "
                      "remplace [%.5f %.5f %.5f]",
                      immo, motif, means[0], means[1], means[2],
                      _m.degrees(_m.sqrt(sum(v * v for v in means))), *old)

    def _try_accel_recal(self, ax, ay, az):
        """Recalibration périodique de l'ÉCHELLE accéléro, à l'arrêt. Voir
        RECAL ACCEL en en-tête pour la justification physique (norme
        invariante par rotation) et pourquoi ceci ne devient PAS un biais
        3 axes.

        Une seule garde : self._is_stationary. Contrairement à _try_recal()
        ci-dessus (gyro), qui a sa PROPRE garde en trois temps parce qu'elle
        tourne indépendamment, celle-ci s'appelle APRÈS _update_zupt() dans
        _cb() et réutilise directement son verdict — roues ET variance
        accéléro y sont déjà vérifiées. Retester ici serait la même variance
        recalculée deux fois pour la même conclusion.
        """
        now = rospy.get_time()
        if self._next_accel_recal_t is None:
            self._next_accel_recal_t = now + self.accel_recal_period
            return
        if now < self._next_accel_recal_t:
            return
        if not self._is_stationary:
            self._accel_recal_buf = []   # pas immobile : on repart de zéro
            return

        norme = (ax * ax + ay * ay + az * az) ** 0.5
        self._accel_recal_buf.append(norme)
        if len(self._accel_recal_buf) < self.gyro_cal_samples:
            # Réutilise gyro_cal_samples : même raisonnement « combien
            # d'échantillons pour une moyenne stable », pas la peine d'un
            # paramètre dédié rien que pour ce nombre.
            return

        n = float(len(self._accel_recal_buf))
        norme_moy = sum(self._accel_recal_buf) / n
        self._accel_recal_buf = []
        self._next_accel_recal_t = now + self.accel_recal_period

        # La recalibration vise la branche VINS dès qu'elle est active, et
        # JAMAIS /imu/data_clean dans ce cas : c'est précisément ce qui rend
        # la correction de dérive compatible avec le sanctuaire MINS. Sans
        # branche VINS, comportement d'origine inchangé.
        vins = self.accel_scale_vins > 0.0
        courante = self.accel_scale_vins if vins else self.accel_scale
        quoi = "VINS" if vins else "commune"

        accepte, motif, nouvelle = evaluer_recal_accel_scale(
            norme_moy, courante, self.gravite_locale,
            self.accel_recal_max_delta)
        if not accepte:
            rospy.logwarn("[imu_sanitizer] recalib échelle accéléro (%s) REFUSÉE "
                          "— %s. Échelle conservée (%.4f).", quoi, motif, courante)
            return
        if vins:
            self.accel_scale_vins = nouvelle
        else:
            self.accel_scale = nouvelle
        rospy.logwarn("[imu_sanitizer] recalib échelle accéléro (%s) APPLIQUÉE "
                      "(%s) : x%.4f — remplace x%.4f (norme brute mesurée "
                      "%.4f m/s^2 sur %d échantillons)%s",
                      quoi, motif, nouvelle, courante, norme_moy, int(n),
                      "" if vins else " — /imu/data_clean AFFECTÉ (MINS inclus)")

    def _check_freeze(self, raw_vals):
        if raw_vals == self._prev_vals:
            self._identical_count += 1
        else:
            self._identical_count = 0
            self._prev_vals = raw_vals
        if self._identical_count < self.freeze_samples:
            return
        # only warn once per freeze_samples window, not on every sample past it
        if self._identical_count % self.freeze_samples:
            return
        rospy.logerr("[imu_sanitizer] IMU FROZEN: %d consecutive bit-identical "
                     "samples %s — firmware stuck after serial desync",
                     self._identical_count, str(raw_vals))
        now = rospy.Time.now()
        if (not self.freeze_autoreset or self._reset_in_flight
                or (now - self._last_reset).to_sec() < self.freeze_cooldown):
            return
        self._last_reset = now
        self._reset_in_flight = True
        threading.Thread(target=self._reset_board, daemon=True).start()

    def _reset_board(self):
        # own thread: a service call must never block the IMU callback
        try:
            rospy.wait_for_service("/firmware/reset_board", timeout=5.0)
            resp = rospy.ServiceProxy("/firmware/reset_board", Trigger)()
            rospy.logwarn("[imu_sanitizer] auto reset_board: success=%s %s",
                          resp.success, resp.message)
        except Exception as e:
            rospy.logerr("[imu_sanitizer] auto reset_board FAILED: %s", e)
        finally:
            self._reset_in_flight = False

    def _cb(self, m):
        self._n += 1
        if self._last_stamp is not None and m.stamp <= self._last_stamp:
            self._n_nonmono += 1
            self._n_nonmono_suite += 1

            # ── GARDE DE DÉVERROUILLAGE (2026-09-04) ────────────────────────
            # Sans elle, cette garde de monotonicité n'a AUCUNE porte de
            # sortie : une seule mesure corrompue mémorisée comme référence
            # rejette ensuite TOUTES les mesures légitimes, indéfiniment.
            # Observé en vrai le 2026-09-04 : une trame datée du 2076-11-10
            # (50,2 ans dans le futur) a verrouillé le nœud et provoqué
            # 520 451 rejets d'affilée. /imu/data_clean est resté muet, MINS
            # n'a plus rien propagé, et le robot a disparu de la carte — sans
            # qu'aucun processus ne meure ni qu'aucune erreur ne s'affiche.
            #
            # Deux conditions de déverrouillage, volontairement distinctes :
            #   (a) la référence est absurde par rapport à l'horloge ROS
            #       (elle vient d'une trame corrompue) ;
            #   (b) on rejette en continu depuis trop longtemps, quelle que
            #       soit la raison (filet de sécurité générique).
            # Dans les deux cas on repart de la mesure courante plutôt que de
            # rester bloqué : perdre l'ordre strict sur une trame est très
            # préférable à perdre le flux entier.
            maintenant = rospy.Time.now()
            ref_absurde = (self._last_stamp - maintenant).to_sec() > self.stamp_futur_max_s
            trop_long = self._n_nonmono_suite >= self.stamp_rejets_max

            if ref_absurde or trop_long:
                rospy.logerr(
                    "[imu_sanitizer] référence d'horodatage DÉVERROUILLÉE "
                    "(%s) : ref=%.3f, courant=%.3f, horloge=%.3f, "
                    "%d rejets consécutifs — on repart de la mesure courante",
                    "référence dans le futur" if ref_absurde else "rejets en série",
                    self._last_stamp.to_sec(), m.stamp.to_sec(),
                    maintenant.to_sec(), self._n_nonmono_suite)
                self._n_deverrou += 1
                self._n_nonmono_suite = 0
                self._last_stamp = m.stamp        # on réamorce, on ne bloque plus
            else:
                if self._n_nonmono % 50 == 1:
                    rospy.logwarn("[imu_sanitizer] stamp non monotone rejeté "
                                  "(%.4f <= %.4f, %d au total)",
                                  m.stamp.to_sec(), self._last_stamp.to_sec(),
                                  self._n_nonmono)
                return
        else:
            self._n_nonmono_suite = 0
        self._last_stamp = m.stamp
        self._check_freeze((m.gyro_x, m.gyro_y, m.gyro_z,
                            m.accel_x, m.accel_y, m.accel_z))
        # _is_glitch() est évalué sur les valeurs BRUTES, avant mise à
        # l'échelle : les seuils (~accel_limit) gardent ainsi exactement le
        # sens qu'ils avaient quand ils ont été calibrés sur les bags.
        if self._is_glitch(m):
            self._n_glitch += 1
            vals = self._last_good
            if vals is None:
                # Repli déjà exprimé en unités CORRIGÉES (gravité locale) :
                # il ne doit surtout pas repasser par accel_scale.
                vals = (0.0, 0.0, 0.0, 0.0, 0.0, 9.790)
        else:
            # Auto-calib AVANT correction : elle doit voir le gyro BRUT, sinon
            # elle mesurerait le biais d'un signal déjà débiaisé (≈ 0) et
            # l'écraserait à zéro au redémarrage suivant.
            if not self._cal_done:
                self._try_autocal(m.gyro_x, m.gyro_y, m.gyro_z)
            elif self.gyro_recal_period > 0.0:
                # Même remarque que pour l'auto-calibration ci-dessus : on
                # accumule les valeurs BRUTES, avant retrait du biais. Mesurer
                # sur le signal déjà débiaisé donnerait ≈ 0 et écraserait le
                # biais au lieu de le corriger.
                self._try_recal(m.gyro_x, m.gyro_y, m.gyro_z)
            s = self.accel_scale
            b = self.gyro_bias
            g = self.gyro_scale
            # Ordre imposé par le modèle capteur : mesure = echelle * vrai +
            # biais. On retire donc le biais AVANT de diviser par l'échelle
            # (ici g = 1/echelle, déjà inversé). L'auto-calib du biais reste
            # correcte : elle tourne à l'arrêt, où le vrai taux est nul, donc
            # l'échelle n'a aucun effet sur ce qu'elle mesure.
            vals = ((m.gyro_x - b[0]) * g, (m.gyro_y - b[1]) * g,
                    (m.gyro_z - b[2]) * g,
                    m.accel_x * s, m.accel_y * s, m.accel_z * s)
            self._last_good = vals
            # Sur le signal frais uniquement (pas sur un maintien de glitch,
            # ligne 557 : un maintien a une variance nulle par construction et
            # ferait croire à une immobilité qui n'existe pas).
            self._update_zupt(m.stamp.to_sec(),
                              vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])
            # Après _update_zupt, PAS avant : is_stationary de CE cycle doit
            # être frais, c'est justement ce que _try_accel_recal réutilise
            # au lieu de rejuger l'immobilité une deuxième fois.
            if self.accel_recal_period > 0.0:
                self._try_accel_recal(m.accel_x, m.accel_y, m.accel_z)
            # CLAMP (désactivé par défaut, ~zupt_clamp_enable) : à l'arrêt
            # CONFIRMÉ par les roues (vérité terrain, pas une heuristique
            # openVINS), publier la moyenne fenêtre plutôt que le brut. Le
            # bruit CORE2 mesuré (écart-type ~0.02-0.05) suffit à empêcher le
            # test chi² interne d'openVINS (try_zupt, déjà actif dans
            # config/leo/estimator_config.yaml) de se déclencher à chaque
            # arrêt réel. Ceci ne fabrique rien : la fenêtre ne contribue à
            # self._imu_mean que quand accel_ok a déjà validé sa variance
            # (_update_zupt ci-dessus), donc la moyenne clampée est déjà
            # connue être proche du signal réel. _last_good (ligne au-dessus)
            # reste le signal BRUT non clampé, pour que le repli anti-glitch
            # ne compose jamais un clamp sur un clamp.
            if self.zupt_clamp_enable and self._is_stationary and self._imu_mean is not None:
                vals = self._imu_mean

        out = SensorImu()
        out.header.stamp = m.stamp
        out.header.frame_id = self.frame_id
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = vals[0:3]
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = vals[3:6]
        out.orientation_covariance[0] = -1.0  # no orientation estimate provided
        self.pub.publish(out)

        # ── Copie pour les VINS, avec LEUR échelle accéléro ──────────────
        # Republication plutôt que réécriture : /imu/data_clean vient d'être
        # publié inchangé, donc MINS ne voit strictement aucune différence.
        #
        # Le rapport (accel_scale_vins / accel_scale) est appliqué au signal
        # DÉJÀ mis à l'échelle, ce qui est exact : vals[3:6] valent brut *
        # accel_scale, donc le produit vaut brut * accel_scale_vins. Ça vaut
        # aussi pour le repli anti-glitch (_last_good, stocké en unités
        # corrigées) et pour le clamp ZUPT (moyenne fenêtre, mêmes unités).
        # SEULE exception : le repli en dur (0,0,9.790) utilisé quand un
        # glitch survient avant tout échantillon sain — il est déjà exprimé
        # en gravité locale et le rapport l'écarte de ~1 %. Cas rare, borné,
        # et sans commune mesure avec le biais systématique qu'on retire.
        if self.accel_scale_vins > 0.0:
            r = self.accel_scale_vins / self.accel_scale
            out_v = SensorImu()
            out_v.header.stamp = out.header.stamp
            out_v.header.frame_id = out.header.frame_id
            out_v.angular_velocity = out.angular_velocity
            out_v.linear_acceleration.x = vals[3] * r
            out_v.linear_acceleration.y = vals[4] * r
            out_v.linear_acceleration.z = vals[5] * r
            out_v.orientation_covariance[0] = -1.0
            self.pub_vins.publish(out_v)
        else:
            # Inactif : copie exacte, pour que le topic existe toujours et
            # qu'un estimateur configuré dessus ne se retrouve jamais muet.
            self.pub_vins.publish(out)

        # Periodic health line — a rising glitch rate is a real hardware signal
        # (loose IMU connector, failing sensor), worth surfacing not hiding.
        now = rospy.Time.now()
        if (now - self._last_report).to_sec() >= 30.0:
            if self._n_glitch:
                rospy.logwarn("[imu_sanitizer] %d/%d samples rejected as glitches "
                               "in last window (%.3f%%)",
                               self._n_glitch, self._n, 100.0 * self._n_glitch / self._n)
            self._n = 0
            self._n_glitch = 0
            self._last_report = now


if __name__ == "__main__":
    rospy.init_node("imu_sanitizer")
    ImuSanitizer()
    rospy.spin()
