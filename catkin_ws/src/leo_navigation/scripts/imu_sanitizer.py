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
SUBSCRIBES ~in_topic (default /firmware/imu)  leo_msgs/Imu

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
  ~freeze_samples      (default 200 — consecutive identical samples => frozen)
  ~freeze_autoreset    (default true — call /firmware/reset_board on freeze)
  ~freeze_reset_cooldown (default 60.0 s between two auto-resets)

LAUNCH: rosrun leo_navigation imu_sanitizer.py   (or via pose_selector.launch)
"""
import threading

import rospy
from leo_msgs.msg import Imu as LeoImu
from sensor_msgs.msg import Imu as SensorImu
from std_srvs.srv import Trigger


class ImuSanitizer(object):
    def __init__(self):
        self.gyro_limit = rospy.get_param("~gyro_limit", 35.0)
        self.accel_limit = rospy.get_param("~accel_limit", 160.0)
        # 9.790 (gravité locale réelle) / 8.7525 (mesuré brut) — voir en-tête.
        self.accel_scale = float(rospy.get_param("~accel_scale", 1.1186))
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

        # freeze guard (see module docstring)
        self.freeze_samples = int(rospy.get_param("~freeze_samples", 200))
        self.freeze_autoreset = bool(rospy.get_param("~freeze_autoreset", True))
        self.freeze_cooldown = float(rospy.get_param("~freeze_reset_cooldown", 60.0))
        self._prev_vals = None
        self._identical_count = 0
        self._last_reset = rospy.Time(0)
        self._reset_in_flight = False

        self.pub = rospy.Publisher(out_topic, SensorImu, queue_size=20)
        rospy.Subscriber(in_topic, LeoImu, self._cb, queue_size=50, tcp_nodelay=True)
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
            if self._n_nonmono % 50 == 1:
                rospy.logwarn("[imu_sanitizer] stamp non monotone rejeté "
                              "(%.4f <= %.4f, %d au total)",
                              m.stamp.to_sec(), self._last_stamp.to_sec(),
                              self._n_nonmono)
            return
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

        out = SensorImu()
        out.header.stamp = m.stamp
        out.header.frame_id = self.frame_id
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = vals[0:3]
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = vals[3:6]
        out.orientation_covariance[0] = -1.0  # no orientation estimate provided
        self.pub.publish(out)

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
