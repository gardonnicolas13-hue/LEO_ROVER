#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — Pose Selector
==========================
Republishes a single continuous pose on /robot_pose_fused, sourced from
either VINS (OpenVINS ov_msckf) or MINS, switchable at runtime without a
discontinuity in the output.

  WHY A SEPARATE FUSED TOPIC INSTEAD OF POINTING CONSUMERS AT ov_msckf/mins
  DIRECTLY: downstream consumers (nav stack, the web cockpit, logging) need
  one topic that keeps working across a VINS<->MINS swap. Both estimators
  run their own independent global/odom frame with their own drift, so a
  raw switch would jump the output by the accumulated difference between
  the two estimates at that instant. This node applies a one-time rigid
  (SE3) correction the moment it switches, computed so the fused output is
  bit-for-bit identical immediately before and after the switch (see
  _switch_to()). It does NOT keep correcting after that: the two estimators
  are independent and will naturally drift apart again over time, which is
  expected and cannot be hidden — only the switch instant itself is smoothed.

  SUBSCRIBES
    ~vins_topic  (default /ov_msckf/odomimu)  nav_msgs/Odometry
    ~mins_topic  (default /mins/imu/odom)     nav_msgs/Odometry

  PUBLISHES
    /robot_pose_fused              nav_msgs/Odometry   (continuous, active source)
    /leo_navigation/pose_source    std_msgs/String      (latched, "VINS"|"MINS")
    /leo_navigation/pose_source_pending  std_msgs/String  (latched, "VINS"|"MINS"|"")
                                      armed-but-not-yet-applied target, "" when
                                      nothing is armed — see ~set_source below
    TF  ~odom_frame -> ~base_frame  (broadcast on every fused message; composed
                                      through the static ~base_frame <-> ~imu_frame
                                      transform if it's on the tf tree yet, else
                                      falls back to ~odom_frame -> ~imu_frame
                                      directly with a one-time warning)

  SERVICES
    ~set_source   std_srvs/SetBool   (data=false -> VINS, data=true -> MINS)
                  If the requested source has already published, switches
                  immediately. If not, ARMS it (see _switch_to(),
                  2026-07-24): the switch is applied automatically the
                  instant that source's first message arrives, no second
                  click needed. Calling again with the same still-pending
                  source cancels the arm. Reintroduced after being tried
                  and reverted on 2026-07-23 for being hard to read from
                  the operator's seat — this time both the log message and
                  the dedicated pending topic above always name the exact
                  state (armed for X / cancelled / switched), on the
                  explicit operator request that not switching at all until
                  the target has data was the wrong trade-off, not the
                  concept of arming itself.

  NOTE on frame naming: this robot's existing TF convention (see
  leo_bringup/config/firmware_message_converter.yaml) uses "imu_frame" for
  the firmware IMU and "base_link"/"odom" for the chassis. When authoring
  OpenVINS'/MINS' own config YAMLs, set their child_frame_id to match
  ~imu_frame below so this node's static-tf lookup actually resolves.

  LAUNCH
    roslaunch leo_navigation pose_selector.launch
"""

import math
from collections import deque

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import TransformStamped

SOURCES = ("VINS", "MINS")

# Garde de plausibilité sur la PREMIÈRE mesure d'une source (2026-07-23,
# audit — item Bloquant #2) : un estimateur frais publie censément près de
# sa propre origine (0,0,0) — observé en direct le 23/07, VINS a rejoint
# avec (27.5, -25.7, -4.6) m dès son premier message, une divergence
# d'initialisation manifeste dans une pièce de quelques mètres. La garde de
# vraisemblance existante (_on_odom, "2026-07-13") ne couvre que la VITESSE
# entre deux échantillons consécutifs de la source déjà ACTIVE — elle ne
# protège pas contre une source qui n'a jamais servi et dont le tout premier
# échantillon est déjà aberrant. Complémentaire, pas redondante : celle-ci
# agit avant même que la source devienne sélectionnable.
MAX_INITIAL_JUMP_M = 5.0


def _mat_from_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0:3, 3] = [p.x, p.y, p.z]
    return T


def _odom_from_mat(T, stamp, frame_id, child_frame_id, template):
    out = Odometry()
    out.header.stamp = stamp
    out.header.frame_id = frame_id
    out.child_frame_id = child_frame_id
    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = T[0:3, 3]
    q = tft.quaternion_from_matrix(T)
    (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
     out.pose.pose.orientation.z, out.pose.pose.orientation.w) = q
    out.pose.covariance = template.pose.covariance
    # Twist is expressed in the child (body) frame per REP-103/nav_msgs/Odometry
    # convention — the SE3 correction only ever re-anchors the parent/world
    # frame, so body-frame velocities pass through unchanged.
    out.twist = template.twist
    return out


class PoseSelector(object):
    def __init__(self):
        self.active = rospy.get_param("~default_source", "VINS").upper()
        if self.active not in SOURCES:
            self.active = "VINS"

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.imu_frame = rospy.get_param("~imu_frame", "imu_frame")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        # garde de vraisemblance : vitesse max physiquement possible du rover
        # (0.4 m/s commandés ; 1.5 laisse la marge des corrections normales)
        self.max_plausible_speed = float(rospy.get_param("~max_plausible_speed", 1.5))
        # Nombre d'échantillons CONSÉCUTIFS au-dessus du seuil avant de
        # déclarer la source folle (2026-08-05). Avant : 1 seul suffisait, donc
        # un pic isolé verrouillait l'état et, comme le gel ne publiait rien,
        # /robot_pose_fused tombait à ~0 Hz — cockpit figé alors que la source
        # était saine 99 % du temps. Un pic isolé est maintenant simplement
        # ÉCARTÉ (l'échantillon n'entre pas dans la trace) sans verrouiller.
        # 3 à ~90 Hz = ~33 ms de rejet continu avant de considérer que ce
        # n'est plus un pic mais une vraie excursion.
        self.spike_tolerance = int(rospy.get_param("~spike_tolerance", 3))
        # Pendant un gel, on RÉÉMET la dernière pose saine au lieu de ne rien
        # publier. Ce n'est PAS une pose inventée : c'est la dernière mesure
        # valide, exactement ce que tout consommateur gardait déjà en mémoire
        # faute de mieux. La différence est qu'ils continuent de recevoir, donc
        # l'IHM reste vivante et un contrôle de débit distingue enfin "source
        # gelée" (topic vivant, covariance énorme) de "nœud mort" (topic muet).
        self.hold_during_freeze = bool(rospy.get_param("~hold_during_freeze", True))
        # Covariance annoncée sur une pose tenue : volontairement énorme, pour
        # qu'un consommateur rigoureux (analyse hors ligne, RTB) puisse la
        # rejeter sans ambiguïté au lieu de la prendre pour une mesure fraîche.
        self.hold_covariance = float(rospy.get_param("~hold_covariance", 1e6))
        # Base de temps d'évaluation de la vitesse (2026-08-05, MESURÉ).
        # Comparer deux échantillons CONSÉCUTIFS (~11 ms) était le vrai défaut :
        # un MSCKF applique ses corrections de façon DISCRÈTE au moment des
        # mises à jour visuelles, donc un bond de 2-4 cm en 9 ms est le filtre
        # qui se corrige, pas qui diverge — mais ça calcule 4.7 m/s et la garde
        # rejetait 8.3 % des échantillons en conduite. Mesuré sur 2020 messages
        # en roulant : max 4.7 m/s sur 11 ms, et 0.42 m/s pour le MÊME
        # déplacement évalué sur 100 ms. Une vraie divergence, elle, est
        # SOUTENUE : à 6 m/s elle parcourt 60 cm en 100 ms et reste détectée.
        self.guard_baseline_s = float(rospy.get_param("~guard_baseline_s", 0.10))
        self._sane_hist = deque()  # (t, x, y) des poses fused ACCEPTÉES
        self._sane_prev = None
        self._sane_prev_t = 0.0
        self._insane = False
        self._insane_t0 = 0.0
        self._spike_run = 0        # rejets consécutifs en cours
        self._held_count = 0       # poses tenues republiées (diagnostic)

        # Most recent raw message seen from each source (always kept fresh,
        # even for the source that isn't currently active) so a switch can
        # compute the correction instantly instead of waiting on a new msg.
        self._last_raw = {"VINS": None, "MINS": None}
        # SE3 correction applied to each source's raw pose before publishing.
        # Identity until the first switch away from that source.
        self._correction = {"VINS": np.eye(4), "MINS": np.eye(4)}
        self._last_fused_mat = None

        # Source armée (2026-07-24) : demandée mais pas encore publiée au
        # moment du clic. Appliquée automatiquement dès son premier message
        # dans _on_odom(). Toujours None ou l'UNE des deux sources non
        # actives — jamais égale à self.active (voir _switch_to()).
        self._pending = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._base_imu_static = None      # cached base_frame<-imu_frame, looked up lazily
        self._warned_no_static_tf = False

        self.pub_fused = rospy.Publisher("/robot_pose_fused", Odometry, queue_size=5)
        self.pub_source = rospy.Publisher("/leo_navigation/pose_source", String,
                                           queue_size=1, latch=True)
        self.pub_pending = rospy.Publisher("/leo_navigation/pose_source_pending", String,
                                            queue_size=1, latch=True)

        vins_topic = rospy.get_param("~vins_topic", "/ov_msckf/odomimu")
        mins_topic = rospy.get_param("~mins_topic", "/mins/imu/odom")
        rospy.Subscriber(vins_topic, Odometry, self._on_odom, callback_args="VINS",
                          queue_size=5)
        rospy.Subscriber(mins_topic, Odometry, self._on_odom, callback_args="MINS",
                          queue_size=5)

        rospy.Service("~set_source", SetBool, self._on_set_source)

        self._publish_source_status()
        self._publish_pending()
        rospy.loginfo("[pose_selector] active=%s  vins_topic=%s  mins_topic=%s",
                       self.active, vins_topic, mins_topic)

    # ── Source switch ────────────────────────────────────────────────────
    def _on_set_source(self, req):
        new_source = "MINS" if req.data else "VINS"
        ok, msg = self._switch_to(new_source)
        return SetBoolResponse(success=ok, message=msg)

    def _switch_to(self, new_source):
        # Armement (2026-07-24, ré-introduit sur demande opérateur explicite
        # après l'essai annulé du 23/07 — le refus pur et simple, gardé
        # depuis, était le vrai problème pour l'usage réel : re-cliquer à
        # l'aveugle jusqu'à ce que la source ait publié). Le correctif à
        # l'ambiguïté relevée le 23/07 n'est pas de supprimer l'état
        # intermédiaire mais de le rendre impossible à mal lire : un seul
        # topic dédié (pose_source_pending, jamais en désaccord avec ce que
        # dit ce message), et un message qui nomme toujours explicitement
        # l'état exact (ARMÉ / annulé / basculé / déjà actif).
        if new_source == self.active:
            if self._pending is not None:
                cancelled = self._pending
                self._pending = None
                self._publish_pending()
                rospy.loginfo("[pose_selector] pending switch to %s cancelled: "
                               "%s already active", cancelled, new_source)
            return True, "already on %s" % new_source

        new_raw = self._last_raw[new_source]
        if new_raw is not None:
            if self._pending is not None:
                self._pending = None
                self._publish_pending()
            self._apply_switch(new_source, new_raw)
            return True, "switched to %s" % new_source

        if self._pending == new_source:
            self._pending = None
            self._publish_pending()
            rospy.loginfo("[pose_selector] armed switch to %s cancelled by operator",
                           new_source)
            return True, "armed switch to %s cancelled" % new_source

        self._pending = new_source
        self._publish_pending()
        rospy.logwarn("[pose_selector] ARMED switch to %s: no message received on it "
                       "yet; will apply automatically the instant one arrives — click "
                       "again to cancel", new_source)
        return True, ("ARMED: will switch to %s as soon as it publishes "
                       "(click again to cancel)" % new_source)

    def _apply_switch(self, new_source, new_raw):
        if self._last_fused_mat is not None:
            # Correction such that applying it to the new source's raw pose
            # reproduces exactly the last fused pose -> zero jump at t_switch.
            T_new_raw = _mat_from_odom(new_raw)
            self._correction[new_source] = self._last_fused_mat @ np.linalg.inv(T_new_raw)
        # else: no fused pose published yet at all — nothing to match against.

        self.active = new_source
        self._publish_source_status()
        rospy.loginfo("[pose_selector] switched active source -> %s", new_source)

    def _publish_source_status(self):
        self.pub_source.publish(String(data=self.active))

    def _publish_pending(self):
        self.pub_pending.publish(String(data=self._pending or ""))

    # ── Pose passthrough ─────────────────────────────────────────────────
    def _on_odom(self, msg, source):
        if self._last_raw[source] is None:
            # Première mesure jamais vue de cette source — voir
            # MAX_INITIAL_JUMP_M. Rejetée (pas mise en cache) si déjà
            # aberrante : la source reste "jamais vue" pour _switch_to(),
            # donc pas sélectionnable, jusqu'à un premier échantillon sain.
            p0 = msg.pose.pose.position
            jump = math.hypot(p0.x, p0.y, p0.z)
            if jump > MAX_INITIAL_JUMP_M:
                rospy.logerr("[pose_selector] %s premier échantillon REJETÉ : "
                             "(%.1f, %.1f, %.1f) m = %.1f m de son origine, "
                             "> MAX_INITIAL_JUMP_M=%.1f m — divergence "
                             "d'initialisation probable, source non "
                             "sélectionnable tant qu'un échantillon sain "
                             "n'arrive pas", source, p0.x, p0.y, p0.z, jump,
                             MAX_INITIAL_JUMP_M)
                return
        self._last_raw[source] = msg

        if self._pending == source:
            # Source armée qui vient de publier pour la première fois —
            # bascule automatique (2026-07-24, voir _switch_to()). self.active
            # devient `source` ici, donc le message courant continue tout de
            # suite vers la publication fused normale ci-dessous, sans
            # attendre le message suivant.
            self._pending = None
            self._publish_pending()
            rospy.loginfo("[pose_selector] %s just published its first message — "
                           "applying armed switch", source)
            self._apply_switch(source, msg)

        if source != self.active:
            return

        T_raw = _mat_from_odom(msg)
        T_fused = self._correction[source] @ T_raw

        # ── Garde de vraisemblance (2026-07-13) ──────────────────────────
        # Pendant une coupure caméra, l'estimateur dérive en inertie pure
        # (biais accéléro intégré 2x) : bonds de plusieurs mètres/seconde sur
        # un rover qui plafonne à 0.4 m/s. AUCUN bond physiquement impossible
        # ne doit atteindre la trace, la carte ou le Return-to-Base. Politique:
        # on GÈLE la pose servie tant que la source déraille ; on ré-ancre
        # quand elle redevient calme (l'offset de l'excursion est absorbé
        # dans la correction SE3, la continuité du repère fused est préservée).
        now_t = msg.header.stamp.to_sec()
        if self._sane_prev is not None:
            # Référence = la plus ANCIENNE pose saine encore dans la fenêtre
            # guard_baseline_s (et non la précédente immédiate) : c'est ce qui
            # absorbe les corrections discrètes du filtre sans masquer une
            # divergence, laquelle persiste sur toute la fenêtre. Tant que
            # l'historique est plus court que la fenêtre (démarrage), on
            # retombe naturellement sur la pose la plus ancienne disponible.
            while len(self._sane_hist) > 1 and \
                    now_t - self._sane_hist[0][0] > self.guard_baseline_s:
                self._sane_hist.popleft()
            if self._sane_hist:
                ref_t, ref_x, ref_y = self._sane_hist[0]
            else:
                ref_t = self._sane_prev_t
                ref_x = float(self._sane_prev[0, 3])
                ref_y = float(self._sane_prev[1, 3])
            dt = max(1e-3, now_t - ref_t)
            dx = float(T_fused[0, 3]) - ref_x
            dy = float(T_fused[1, 3]) - ref_y
            speed = (dx * dx + dy * dy) ** 0.5 / dt
            if speed > self.max_plausible_speed:
                self._spike_run += 1
                self._insane_last_T = T_fused
                self._insane_last_t = now_t
                if self._spike_run >= self.spike_tolerance and not self._insane:
                    # Rejet SOUTENU : ce n'est plus un pic isolé, c'est une
                    # excursion. On verrouille (et on le dit une seule fois).
                    self._insane = True
                    self._insane_t0 = now_t
                    rospy.logwarn("[pose_selector] pose %s invraisemblable "
                                  "(%.1f m/s, %d echantillons consecutifs) — "
                                  "source GELEE, derniere pose saine tenue",
                                  source, speed, self._spike_run)
                # Dans TOUS les cas l'échantillon aberrant est écarté : il
                # n'entre ni dans la trace, ni dans _sane_prev, ni dans la
                # correction SE3. Aucune trajectoire fausse n'est injectée.
                # Mais on continue de SERVIR la dernière pose saine, pour ne
                # pas assécher le topic (voir hold_during_freeze).
                if self.hold_during_freeze and self._last_fused_mat is not None:
                    held = _odom_from_mat(self._last_fused_mat, msg.header.stamp,
                                          self.odom_frame, self.base_frame,
                                          template=msg)
                    # covariance saturée = "pose tenue, pas une mesure fraîche"
                    cov = list(held.pose.covariance)
                    for i in (0, 7, 14, 21, 28, 35):
                        cov[i] = self.hold_covariance
                    held.pose.covariance = cov
                    self.pub_fused.publish(held)
                    self._held_count += 1
                    if self._held_count % 200 == 0:
                        rospy.logwarn("[pose_selector] %d poses tenues republiees "
                                      "depuis le debut (source %s instable)",
                                      self._held_count, source)
                return
            # Échantillon accepté : la série de pics est rompue.
            self._spike_run = 0
            if self._insane:
                # la source est redevenue calme -> ré-ancrage : la correction
                # absorbe l'offset accumulé pendant l'excursion, la pose servie
                # repart de la dernière pose SAINE (continuité du repère).
                T_target = self._last_fused_mat if self._last_fused_mat is not None else T_fused
                self._correction[source] = T_target @ np.linalg.inv(T_raw)
                T_fused = self._correction[source] @ T_raw
                rospy.logwarn("[pose_selector] source %s calmée après %.1f s "
                              "d'excursion — ré-ancrée sur la dernière pose saine",
                              source, now_t - self._insane_t0)
                self._insane = False
                # Le ré-ancrage change le repère : l'historique d'avant
                # l'excursion n'est plus comparable au nouveau, il le ferait
                # rejeter immédiatement. On repart d'une fenêtre vide.
                self._sane_hist.clear()
        self._sane_prev = T_fused
        self._sane_prev_t = now_t
        self._sane_hist.append((now_t, float(T_fused[0, 3]), float(T_fused[1, 3])))

        self._last_fused_mat = T_fused

        fused = _odom_from_mat(T_fused, msg.header.stamp, self.odom_frame,
                                self.base_frame, template=msg)
        self.pub_fused.publish(fused)

        if self.publish_tf:
            self._broadcast_tf(T_fused, msg.header.stamp)

    # ── TF ────────────────────────────────────────────────────────────────
    def _broadcast_tf(self, T_odom_imu, stamp):
        """Broadcasts odom_frame -> base_frame, composing through the static
        base_frame <- imu_frame transform (from the URDF / Kalibr extrinsics)
        if it's on the tf tree; falls back to odom_frame -> imu_frame directly
        otherwise (logged once, not every frame)."""
        T_out = T_odom_imu
        child = self.imu_frame
        if self._base_imu_static is None:
            try:
                tr = self._tf_buffer.lookup_transform(
                    self.base_frame, self.imu_frame, rospy.Time(0), rospy.Duration(0.2))
                q = tr.transform.rotation
                M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
                M[0:3, 3] = [tr.transform.translation.x,
                             tr.transform.translation.y,
                             tr.transform.translation.z]
                self._base_imu_static = M  # T_base_imu (static)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                if not self._warned_no_static_tf:
                    rospy.logwarn("[pose_selector] static tf %s -> %s not available "
                                   "yet; broadcasting %s -> %s directly until it is "
                                   "(see tf_static_extrinsics.launch)",
                                   self.base_frame, self.imu_frame,
                                   self.odom_frame, self.imu_frame)
                    self._warned_no_static_tf = True

        if self._base_imu_static is not None:
            T_out = T_odom_imu @ np.linalg.inv(self._base_imu_static)
            child = self.base_frame

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = child
        (t.transform.translation.x, t.transform.translation.y,
         t.transform.translation.z) = T_out[0:3, 3]
        q = tft.quaternion_from_matrix(T_out)
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self._tf_broadcaster.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("pose_selector")
    PoseSelector()
    rospy.spin()
