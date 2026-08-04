#!/usr/bin/env bash
# ═════════════════════════════════════════════════════════════════════════════
# reset_essai.sh — nettoyage COMPLET d'un essai Test 1 / Test 2 bloqué,
#   SANS redémarrer le robot ni la stack de navigation.
#
#   À utiliser quand le bouton RESET du site ne débloque pas la situation
#   (symptôme du 2026-07-28 : "le reset reste bloqué, le redémarrage de
#   l'essai ne fonctionne pas"). Cause typique : un `rosbag record` fantôme
#   continue d'écrire alors que le backend a perdu sa trace (il mémorise le
#   PID du wrapper python, mais rosbag lance TROIS process en cascade
#   bash -> python -> binaire C++, et l'état en mémoire ne survit pas à un
#   redémarrage du backend).
#
#   Ce script ne touche NI à MINS, NI à VINS, NI à la caméra, NI au robot :
#   uniquement les enregistreurs et les fichiers d'état de l'essai.
#
#   Usage :
#     tools/reset_essai.sh            # nettoie (garde les .bag deja fermes)
#     tools/reset_essai.sh --purge    # + supprime les .bag.active incomplets
# ═════════════════════════════════════════════════════════════════════════════
set -u
DIR=/home/lab272/TOUT/data/trajectories
STATE="$DIR/.active_recording.json"
PURGE=0
[ "${1:-}" = "--purge" ] && PURGE=1

echo "── Nettoyage essai ──────────────────────────────────────────"

# 1) Arrêt PROPRE (SIGINT = comme un Ctrl-C : rosbag vide ses buffers et
#    ferme le .bag correctement). On ne cible QUE les enregistreurs qui
#    écrivent dans le dossier des essais.
mapfile -t PIDS < <(pgrep -af rosbag | grep -F "$DIR" | awk '{print $1}')
if [ ${#PIDS[@]} -eq 0 ]; then
  echo "  [=] aucun enregistreur en cours"
else
  echo "  [>] arrêt propre de ${#PIDS[@]} process : ${PIDS[*]}"
  kill -INT "${PIDS[@]}" 2>/dev/null
  for _ in $(seq 1 25); do                 # jusqu'à 5 s
    sleep 0.2
    pgrep -af rosbag | grep -qF "$DIR" || break
  done
  # 2) Insistance seulement si nécessaire (un .bag non fermé est préférable
  #    à un process qui bloque le prochain essai indéfiniment).
  if pgrep -af rosbag | grep -qF "$DIR"; then
    mapfile -t LEFT < <(pgrep -af rosbag | grep -F "$DIR" | awk '{print $1}')
    echo "  [!] ${#LEFT[@]} recalcitrant(s) -> SIGKILL : ${LEFT[*]}"
    kill -9 "${LEFT[@]}" 2>/dev/null
    sleep 1
  fi
  echo "  [ok] enregistreurs arrêtés"
fi

# 3) Fichier d'état : c'est LUI qui fait croire au backend qu'un essai tourne.
if [ -f "$STATE" ]; then
  echo "  [>] suppression de l'état : $(cat "$STATE")"
  rm -f "$STATE"
else
  echo "  [=] aucun état résiduel"
fi

# 4) .bag.active = enregistrement jamais refermé (inexploitable tel quel).
ACTIVE=$(find "$DIR" -maxdepth 1 -name '*.bag.active' 2>/dev/null)
if [ -n "$ACTIVE" ]; then
  echo "  [!] .bag.active incomplets détectés :"
  echo "$ACTIVE" | while read -r f; do
    printf "        %s  (%s)\n" "$(basename "$f")" "$(du -h "$f" | cut -f1)"
  done
  if [ "$PURGE" = "1" ]; then
    echo "$ACTIVE" | xargs -r rm -f
    echo "  [ok] supprimés (--purge)"
  else
    echo "        -> conservés. 'tools/reset_essai.sh --purge' pour les supprimer,"
    echo "           ou 'rosbag reindex <fichier>' pour tenter de les récupérer."
  fi
fi

echo "── Terminé. Le site peut redémarrer un essai (Test 1 / Test 2). ──"
