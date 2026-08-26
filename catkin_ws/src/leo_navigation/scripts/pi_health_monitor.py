#!/usr/bin/env python3
"""Surveille la sante des flux capteurs du robot pendant une campagne d'essais.

POURQUOI CE NOEUD, ET POURQUOI IL TOURNE SUR LE PC
---------------------------------------------------
sqrtVINS tourne desormais nativement sur le Raspberry Pi, a cote du pilote
camera et de serial_node (SCHED_FIFO 25). Un VIO complet y consomme du CPU, et
ce robot a deja un mode de panne documente dans ce cas : /firmware/wheel_states
ralentit, MINS perd son ancrage roue, et RIEN ne le signale — l'estimateur
continue de publier des poses, simplement fausses.

Ce noeud tourne sur le PC, PAS sur le Pi. C'est deliberé : un moniteur destine
a detecter une famine CPU ajouterait a la charge qu'il mesure. Tous les topics
sont visibles depuis le PC via le meme roscore, donc rien n'est perdu.

LES SEUILS SONT DERIVES DU NOMINAL MESURE, PAS D'UNE VALEUR RONDE
------------------------------------------------------------------
Mesure du 2026-08-24 sur ce robot, 30 s, pile complete en marche :

    /firmware/wheel_states           18,38 Hz
    /imu/data_clean                  83,94 Hz
    /pc/camera/infra1/image_rect_raw 14,98 Hz

Le flux roue publie donc a 18,4 Hz, PAS a 20. Un seuil pose a 20 Hz — la
valeur « nominale » que tout le monde cite — se declencherait en permanence,
et un moniteur qui crie au loup en continu est pire que pas de moniteur : on
apprend a l'ignorer, et il se tait le jour ou il faut l'ecouter. Les seuils
sont donc exprimes en FRACTION du nominal reellement observe.

DEUX GRANDEURS, PAS UNE
-----------------------
  1. CADENCE, sur fenetre glissante. Elle chute quand le noeud producteur
     n'a plus assez de CPU.
  2. LATENCE (header.stamp -> instant de reception). Elle monte AVANT que la
     cadence ne chute : un noeud sature commence par prendre du retard, puis
     seulement perd des messages. C'est le signal precoce.

Un topic qui garde sa cadence mais dont la latence grimpe est deja en train
de decrocher — cas qu'un simple compteur de Hz ne verrait pas.

USAGE
-----
    rosrun leo_navigation pi_health_monitor.py
    rosrun leo_navigation pi_health_monitor.py _fenetre:=5.0 _fraction_alerte:=0.7

Parametres :
    ~fenetre           duree de la fenetre glissante, s (defaut 5.0)
    ~fraction_alerte   fraction du nominal sous laquelle on alerte (defaut 0.70)
    ~periode           periode d'evaluation, s (defaut 2.0)
    ~latence_max       latence au-dela de laquelle on alerte, s (defaut 0.5)
"""
import collections

import rospy
from leo_msgs.msg import WheelStates
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry

# Nominal MESURE le 2026-08-24, pas une valeur de specification. Si le materiel
# ou la configuration change, remesurer plutot que d'ajuster a l'aveugle.
SURVEILLES = [
    # (nom court, topic, type, Hz nominal mesure, critique ?)
    ('roues',    '/firmware/wheel_states',            WheelStates, 18.4, True),
    ('imu',      '/imu/data_clean',                   Imu,         83.9, True),
    ('cam',      '/pc/camera/infra1/image_rect_raw',  Image,       15.0, True),
    ('sqrtVINS_pi', '/ov_srvins/odomimu',             Odometry,    84.0, False),
]


class Flux(object):
    """Suit un topic : cadence sur fenetre glissante et latence de reception."""

    def __init__(self, nom, nominal, critique, fenetre):
        self.nom = nom
        self.nominal = nominal
        self.critique = critique
        self.fenetre = fenetre
        self.arrivees = collections.deque()
        self.latences = collections.deque()
        # Etat de sante courant, pour n'alerter qu'aux TRANSITIONS : sans cela
        # un flux degrade produirait une alerte toutes les `periode` secondes
        # et noierait le journal pendant tout un essai.
        self.sain = True
        self.vu = False

    def note(self, msg):
        maintenant = rospy.get_time()
        self.arrivees.append(maintenant)
        self.vu = True
        # Tous les messages n'ont pas de header (WheelStates porte `stamp`).
        t_capteur = None
        if hasattr(msg, 'header'):
            t_capteur = msg.header.stamp.to_sec()
        elif hasattr(msg, 'stamp'):
            t_capteur = msg.stamp.to_sec()
        if t_capteur:
            self.latences.append(maintenant - t_capteur)

    def purge(self):
        limite = rospy.get_time() - self.fenetre
        while self.arrivees and self.arrivees[0] < limite:
            self.arrivees.popleft()
        while len(self.latences) > len(self.arrivees):
            self.latences.popleft()

    def hz(self):
        self.purge()
        if len(self.arrivees) < 2:
            return 0.0
        span = self.arrivees[-1] - self.arrivees[0]
        return (len(self.arrivees) - 1) / span if span > 0 else 0.0

    def latence(self):
        return max(self.latences) if self.latences else 0.0


def main():
    rospy.init_node('pi_health_monitor')
    fenetre = float(rospy.get_param('~fenetre', 5.0))
    fraction = float(rospy.get_param('~fraction_alerte', 0.70))
    periode = float(rospy.get_param('~periode', 2.0))
    latence_max = float(rospy.get_param('~latence_max', 0.5))

    flux = {}
    for nom, topic, typ, nominal, critique in SURVEILLES:
        f = Flux(nom, nominal, critique, fenetre)
        flux[nom] = f
        rospy.Subscriber(topic, typ, (lambda ff: lambda m: ff.note(m))(f),
                         queue_size=50)

    rospy.loginfo('[pi_health] surveillance de %d flux, fenetre %.1f s, '
                  'alerte sous %.0f %% du nominal mesure',
                  len(flux), fenetre, fraction * 100)
    for nom, topic, _, nominal, critique in SURVEILLES:
        rospy.loginfo('[pi_health]   %-12s %-34s nominal %5.1f Hz -> seuil %5.1f Hz%s',
                      nom, topic, nominal, nominal * fraction,
                      '  (CRITIQUE)' if critique else '')

    # Laisser les fenetres se remplir avant de juger : sinon chaque demarrage
    # produit une fausse alerte sur tous les flux a la fois.
    rospy.sleep(fenetre)

    taux = rospy.Rate(1.0 / periode if periode > 0 else 0.5)
    while not rospy.is_shutdown():
        for f in flux.values():
            hz = f.hz()
            lat = f.latence()
            seuil = f.nominal * fraction

            if not f.vu:
                # Jamais rien recu : un topic absent n'est pas un topic lent.
                # On le dit une seule fois, sans repeter.
                if f.sain:
                    (rospy.logwarn if f.critique else rospy.loginfo)(
                        '[pi_health] %s : AUCUN message recu (topic absent ou '
                        'producteur arrete)', f.nom)
                    f.sain = False
                continue

            degrade = (hz < seuil) or (lat > latence_max)
            if degrade and f.sain:
                motif = []
                if hz < seuil:
                    motif.append('cadence %.1f Hz < %.1f Hz (%.0f %% du nominal)'
                                 % (hz, seuil, 100.0 * hz / f.nominal if f.nominal else 0))
                if lat > latence_max:
                    motif.append('latence %.2f s > %.2f s' % (lat, latence_max))
                msg = '[pi_health] %s DEGRADE : %s' % (f.nom, ' | '.join(motif))
                if f.critique:
                    # Le cas qui motive ce noeud : les roues ancrent MINS, et
                    # leur perte ne produit AUCUNE alarme ailleurs -- MINS
                    # continue de publier des poses, simplement fausses.
                    rospy.logerr(msg + '  <-- ancrage compromis, mesure a jeter')
                else:
                    rospy.logwarn(msg)
                f.sain = False
            elif not degrade and not f.sain:
                rospy.loginfo('[pi_health] %s retabli : %.1f Hz, latence %.2f s',
                              f.nom, hz, lat)
                f.sain = True
        taux.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
