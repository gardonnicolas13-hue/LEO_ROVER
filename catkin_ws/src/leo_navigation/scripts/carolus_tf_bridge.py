#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
carolus_tf_bridge.py — ancre les trajectoires dans un repere SOL fixe.

=============================================================================
  NON LANCE AUTOMATIQUEMENT. Ce noeud n'est declare dans AUCUN .launch, et
  c'est deliberé : il PUBLIE DES TF, et cette plateforme a deja connu un
  conflit de TF (MINS volait le frame `imu` de l'URDF, d'où son isolement sur
  /tf_mins). Lancez-le a la main, arbre TF sous les yeux :
      rosrun leo_navigation carolus_tf_bridge.py
      rosrun tf view_frames && evince frames.pdf
=============================================================================

POURQUOI CE NOEUD (2026-07-30)
------------------------------
Toute la comparaison MINS/openVINS de ce projet souffre d'un defaut de fond :
les deux estimateurs partent d'origines independantes, donc leur ecart brut ne
mesure rien. On le contourne aujourd'hui par un recalage rigide (Kabsch) ou de
similitude (Umeyama) — mais un recalage AJUSTE les donnees avant de les
comparer, ce qui interdit de parler d'erreur absolue.

La balise Carolus est le seul point FIXE du laboratoire. En ancrant l'arbre TF
dessus, les deux trajectoires deviennent nativement exprimees dans un meme
repere physique : le residu redevient une ERREUR MESUREE, plus un ajustement.

CE QUI EXISTE DEJA, ET QU'IL NE FAUT SURTOUT PAS REPUBLIER
----------------------------------------------------------
Le document de reference (Turki Yassin, §13.1) publie `odom -> base_footprint`
PUIS `base_footprint -> base_link`. Sur CETTE plateforme, l'arbre contient
deja (verifie le 30/07 par ecoute de /tf pendant 10 s) :

    odom            -> base_link        (dynamique, ~84 Hz)
    base_footprint  -> base_link        (statique)
    base_link       -> camera_frame     (statique)

Republier la chaine du document donnerait DEUX PARENTS a `base_link` et
casserait l'arbre. Ce noeud ne publie donc QUE les deux transformations
reellement manquantes :

    camera_frame -> carolus_raw     (dynamique : la balise vue par la camera)
    beacon_link  -> odom            (statique, figee au PREMIER fix)

Le garde-fou `~publish_body_chain` (defaut False) existe pour rendre ce choix
explicite : le passer a True republierait la chaine du document et casserait
l'arbre de cette plateforme. Ne le faites que sur un robot dont l'arbre est
different, et apres verification.

SORTIES
-------
    /odom_in_beacon     PoseStamped — le robot selon l'ODOMETRIE, en repere balise
    /carolus_in_beacon  PoseStamped — le robot selon CAROLUS, en repere balise
Les DEUX portent la meme grandeur (la pose du robot) mesuree par deux chemins
independants : leur ecart instantane EST l'erreur absolue.
Ces deux topics sont EXACTEMENT ceux que lisent deja trois scripts MATLAB du
depot (Matlab/carolus_odom.m, MATLAB-Drive/Ltest.m, jpp2.m) : la chaine
d'analyse existe, seul ce producteur manquait.

CONVENTION D'AXES
-----------------
Carolus publie en repere CAMERA. La correspondance des lettres est etablie
(doc §10.2.5) et verifiee : X_leo = -Z_c, Y_leo = -X_c, Z_leo = -Y_c. Les
SIGNES du quaternion restent a determiner parmi 16 combinaisons (doc §13.2.2) :
`~quat_signs` permet de les balayer sans toucher au code.

CE NOEUD NE PUBLIE JAMAIS sur /mins/external_ref/carolus (creneau prive du
bridge MINS, regle d'exclusivite du 08/07).

PARAMS
  ~pose_topic          (defaut /pose)            sortie de carolus_astrobee
  ~camera_frame        (defaut camera_frame)
  ~odom_frame          (defaut odom)
  ~body_frame          (defaut base_link)        cf. arbre reel, PAS base_footprint
  ~beacon_frame        (defaut beacon_link)
  ~raw_frame           (defaut carolus_raw)
  ~quat_signs          (defaut "-+++")           16 combinaisons possibles
  ~publish_body_chain  (defaut False)            DANGER — voir ci-dessus
  ~lookup_timeout      (defaut 0.1 s)
"""
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Empty, EmptyResponse


class CarolusTfBridge(object):
    def __init__(self):
        rospy.init_node("carolus_tf_bridge")

        self.pose_topic = rospy.get_param("~pose_topic", "/pose")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_frame")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.body_frame = rospy.get_param("~body_frame", "base_link")
        self.beacon_frame = rospy.get_param("~beacon_frame", "beacon_link")
        self.raw_frame = rospy.get_param("~raw_frame", "carolus_raw")
        self.lookup_timeout = float(rospy.get_param("~lookup_timeout", 0.1))
        self.publish_body_chain = bool(
            rospy.get_param("~publish_body_chain", False))

        signs = str(rospy.get_param("~quat_signs", "-+++"))
        if len(signs) != 4 or any(c not in "+-" for c in signs):
            rospy.logwarn("quat_signs invalide (%r) — repli sur '-+++'", signs)
            signs = "-+++"
        self.signs = [(-1.0 if c == "-" else 1.0) for c in signs]
        self.signs_str = signs

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.dyn_br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.anchored = False
        self.n_fix = 0

        self.pub_odom = rospy.Publisher("/odom_in_beacon", PoseStamped,
                                        queue_size=10)
        self.pub_caro = rospy.Publisher("/carolus_in_beacon", PoseStamped,
                                        queue_size=10)
        rospy.Service("~reanchor", Empty, self._srv_reanchor)
        rospy.Subscriber(self.pose_topic, PoseStamped, self._on_pose,
                         queue_size=5)

        if self.publish_body_chain:
            rospy.logerr(
                "publish_body_chain=True : ce noeud va republier "
                "odom->base_footprint->base_link. Sur CETTE plateforme ces "
                "transformations existent deja et l'arbre TF sera CASSE. "
                "Verifiez `rosrun tf view_frames` immediatement.")

        rospy.loginfo("carolus_tf_bridge : %s -> %s/%s | signes %s | "
                      "corps=%s (chaine corps NON publiee=%s)",
                      self.pose_topic, "/odom_in_beacon", "/carolus_in_beacon",
                      self.signs_str, self.body_frame,
                      not self.publish_body_chain)
        rospy.loginfo("En attente du PREMIER fix pour figer %s...",
                      self.beacon_frame)

    # ------------------------------------------------------------------ #
    def _srv_reanchor(self, _req):
        """Re-ancre beacon_link au prochain fix. Utile si la balise a ete
        deplacee, ou si le premier ancrage s'est fait sur une detection
        douteuse — sans avoir a relancer le noeud."""
        self.anchored = False
        rospy.logwarn("Re-ancrage demande : %s sera refige au prochain fix.",
                      self.beacon_frame)
        return EmptyResponse()

    def _on_pose(self, msg):
        self.n_fix += 1
        # 1) camera_frame -> carolus_raw, avec la permutation d'axes.
        t = TransformStamped()
        t.header.stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 \
            else rospy.Time.now()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.raw_frame
        p, o = msg.pose.position, msg.pose.orientation
        t.transform.translation.x = -p.z
        t.transform.translation.y = -p.x
        t.transform.translation.z = -p.y
        sx, sy, sz, sw = self.signs
        t.transform.rotation.x = sx * o.z
        t.transform.rotation.y = sy * o.x
        t.transform.rotation.z = sz * o.y
        t.transform.rotation.w = sw * o.w
        self.dyn_br.sendTransform(t)

        if self.publish_body_chain:
            self._publish_body_chain(t.header.stamp)

        # 2) Premier fix : figer beacon_link comme origine au sol.
        if not self.anchored:
            self._try_anchor()
            return

        # 3) Republier LA MEME GRANDEUR par deux chemins independants :
        #    la pose du ROBOT, vue par l'odometrie d'une part, par Carolus de
        #    l'autre. C'est leur ECART qui est l'erreur absolue recherchee.
        #
        #    Piege evite ici : publier `carolus_raw` dans `beacon_link` aurait
        #    donne ~(0,0,0) en permanence — la balise dans son propre repere —
        #    et l'ecart n'aurait rien mesure. Le chemin Carolus doit donc
        #    exprimer le CORPS relativement a `carolus_raw` (doc §13.1, qui
        #    fait lookup_transform("carolus_raw", "base_footprint")).
        self._emit(self.beacon_frame, self.body_frame, self.pub_odom)
        self._emit(self.raw_frame,    self.body_frame, self.pub_caro)

    def _try_anchor(self):
        """Fige beacon_link -> odom d'apres la premiere detection.

        On ancre sur `raw_frame -> odom_frame` : la balise devient l'origine
        (0,0,0) et TOUT le reste (odometrie comprise) s'exprime par rapport a
        elle. Une seule emission statique : le repere ne doit plus bouger,
        sinon les trajectoires enregistrees ne seraient pas comparables entre
        elles.
        """
        try:
            raw_to_odom = self.tf_buffer.lookup_transform(
                self.raw_frame, self.odom_frame, rospy.Time(0),
                rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn_throttle(
                2.0, "Ancrage pas encore possible (%s). L'arbre doit relier "
                     "%s a %s — verifiez que l'odometrie publie.",
                e, self.raw_frame, self.odom_frame)
            return
        anchor = TransformStamped()
        anchor.header.stamp = rospy.Time.now()
        anchor.header.frame_id = self.beacon_frame
        anchor.child_frame_id = self.odom_frame
        anchor.transform = raw_to_odom.transform
        self.static_br.sendTransform(anchor)
        self.anchored = True
        rospy.loginfo("%s FIGE comme origine absolue (apres %d fix). "
                      "Les trajectoires sont maintenant dans un repere sol "
                      "commun — plus besoin de recalage.",
                      self.beacon_frame, self.n_fix)

    def _emit(self, parent, child, pub):
        """Publie la pose de `child` vue depuis `parent`.

        L'entete porte TOUJOURS `beacon_frame` : les deux sorties sont ainsi
        directement comparables et tracables dans le meme repere. Quand
        l'ancrage est exact, `carolus_raw` et `beacon_link` coincident — la
        difference residuelle entre les deux topics EST l'erreur de mesure.
        """
        try:
            tr = self.tf_buffer.lookup_transform(
                parent, child, rospy.Time(0),
                rospy.Duration(self.lookup_timeout))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "lookup %s -> %s : %s",
                                   parent, child, e)
            return
        m = PoseStamped()
        m.header.stamp = tr.header.stamp
        m.header.frame_id = self.beacon_frame
        m.pose.position.x = tr.transform.translation.x
        m.pose.position.y = tr.transform.translation.y
        m.pose.position.z = tr.transform.translation.z
        m.pose.orientation = tr.transform.rotation
        pub.publish(m)

    def _publish_body_chain(self, stamp):
        """Chaine du document §13.1 — DESACTIVEE par defaut, voir l'en-tete.
        Conservee pour qu'un successeur sur une plateforme au TF different
        puisse l'activer en connaissance de cause."""
        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = self.odom_frame
        t1.child_frame_id = "base_footprint"
        t1.transform.rotation.w = 1.0
        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = "base_footprint"
        t2.child_frame_id = self.body_frame
        t2.transform.translation.z = 0.198      # hauteur chassis Leo
        t2.transform.rotation.w = 1.0
        self.dyn_br.sendTransform([t1, t2])


if __name__ == "__main__":
    try:
        CarolusTfBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
