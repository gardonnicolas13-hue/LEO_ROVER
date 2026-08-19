#!/usr/bin/env bash
# =============================================================================
# dry_run_pipeline.sh — répétition à blanc de TOUTE la chaîne d'analyse,
#                       hors ligne, sans robot et sans ROS en marche.
#
# USAGE
#     bash tools/dry_run_pipeline.sh            # sortie 0 si tout passe
#     bash tools/dry_run_pipeline.sh --keep     # conserve le dossier temporaire
#
# À QUOI ÇA SERT
#     La chaîne d'analyse ne tourne pour de vrai qu'après un roulage : c'est le
#     pire moment pour découvrir qu'un maillon est cassé, parce que le robot est
#     là, la batterie descend, et personne n'a envie de déboguer du MATLAB. Ce
#     script exerce les cinq maillons sur des données FABRIQUÉES, à tout moment,
#     en une commande. À lancer après toute modification de la chaîne.
#
# CE QU'IL EXERCE
#     1. make_mock_srv.py        fabrique une 3e série synthétique
#     2. drift_exponent.py       exposants depuis les CSV
#     3. drift_exponent.py       --json  (le format que MATLAB consomme)
#     4. compare_estimators.m    figure + exports LaTeX/JSON, MATLAB réel
#     5. vérifications           les fichiers existent, sont non vides, et
#                                portent bien le marquage [MOCK]
#
# CE QU'IL N'EST PAS
#     Ce n'est pas une validation du ROBOT ni des estimateurs : les données sont
#     fausses par construction. Il prouve que la TUYAUTERIE fonctionne, rien de
#     plus. Un dry-run vert avec un robot en panne reste un robot en panne.
#
# WORKFLOW COMPLET AU RETOUR DU ROBOT — voir tools/README.md
# =============================================================================
set -u

RACINE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$RACINE/data/trajectories/sqrtvins_compare_20260813_175957"
GARDER=0
[ "${1:-}" = "--keep" ] && GARDER=1

TMP="$(mktemp -d -t leo_dryrun_XXXXXX)"
nettoyer() { [ "$GARDER" -eq 1 ] && echo "  (dossier conservé : $TMP)" || rm -rf "$TMP"; }
trap nettoyer EXIT

ETAPES=0
ECHECS=0
ok()   { ETAPES=$((ETAPES+1)); echo "  [ok]    $1"; }
ko()   { ETAPES=$((ETAPES+1)); ECHECS=$((ECHECS+1)); echo "  [ECHEC] $1"; }

echo "════════════════════════════════════════════════════════════"
echo "  Répétition à blanc de la chaîne d'analyse (données fausses)"
echo "  travail dans $TMP"
echo "════════════════════════════════════════════════════════════"

# ── 0. Prérequis ─────────────────────────────────────────────────────────────
if [ ! -f "${SRC}_mins.csv" ]; then
  echo "  [ECHEC] capture de référence absente : ${SRC}_mins.csv"
  echo "          (le dry-run emprunte ses horodatages à une vraie capture)"
  exit 1
fi
ok "capture de référence trouvée"

# ── 1. Série synthétique ─────────────────────────────────────────────────────
if python3 "$RACINE/tools/make_mock_srv.py" "$SRC" -o "$TMP" > "$TMP/mock.log" 2>&1; then
  ok "make_mock_srv.py"
else
  ko "make_mock_srv.py"; sed 's/^/          /' "$TMP/mock.log"
fi

BASE="$TMP/MOCK_$(basename "$SRC")"
# Les deux vraies séries rejoignent le lot pour que la comparaison ait trois
# entrées. Elles sont copiées SOUS le préfixe MOCK_ : tout le lot est ainsi
# marqué synthétique, ce qui est voulu — un jeu de test n'est pas une mesure.
for s in mins vins; do cp "${SRC}_${s}.csv" "${BASE}_${s}.csv"; done
[ -f "${BASE}_srv.csv" ] && ok "3 séries en place" || ko "série _srv.csv absente"

# ── 2. Exposants, sortie lisible ─────────────────────────────────────────────
if python3 "$RACINE/tools/drift_exponent.py" "$BASE" > "$TMP/expo.log" 2>&1; then
  ok "drift_exponent.py (texte)"
  grep -E "n = " "$TMP/expo.log" | sed 's/^/          /'
else
  ko "drift_exponent.py (texte)"; sed 's/^/          /' "$TMP/expo.log"
fi

# ── 3. Exposants, sortie machine ─────────────────────────────────────────────
if python3 "$RACINE/tools/drift_exponent.py" "$BASE" --json "$TMP/expo.json" \
     > /dev/null 2>&1 && [ -s "$TMP/expo.json" ]; then
  ok "drift_exponent.py --json"
else
  ko "drift_exponent.py --json"
fi

# ── 4. Tests des gardes gyro (indépendants des données) ──────────────────────
if python3 "$RACINE/tools/test_gyro_recal_guards.py" > "$TMP/guards.log" 2>&1; then
  ok "test_gyro_recal_guards.py — $(tail -1 "$TMP/guards.log" | tr -s ' ')"
else
  ko "test_gyro_recal_guards.py"; tail -5 "$TMP/guards.log" | sed 's/^/          /'
fi

# ── 5. MATLAB : figure + exports ─────────────────────────────────────────────
MB="$(command -v matlab || ls /usr/local/MATLAB/*/bin/matlab 2>/dev/null | head -1)"
if [ -z "$MB" ]; then
  echo "  [saut]  MATLAB absent — étapes 5 et 6 non exercées"
else
  if timeout 600 "$MB" -batch \
       "addpath('$RACINE/tools'); compare_estimators('$BASE','$TMP')" \
       > "$TMP/matlab.log" 2>&1; then
    ok "compare_estimators.m"
  else
    ko "compare_estimators.m"; tail -12 "$TMP/matlab.log" | sed 's/^/          /'
  fi

  # ── 6. Les sorties existent, sont non vides, et disent la vérité ───────────
  for f in estimators_comparison.png report_metrics.tex metrics_summary.json; do
    [ -s "$TMP/$f" ] && ok "produit : $f" || ko "manquant ou vide : $f"
  done

  if grep -q 'MOCK' "$TMP/report_metrics.tex" 2>/dev/null; then
    ok "marquage [MOCK] présent dans le tableau LaTeX"
  else
    ko "marquage [MOCK] ABSENT — des données fausses passeraient pour mesurées"
  fi

  if python3 - "$TMP/metrics_summary.json" <<'PY' 2>/dev/null; then
import json, sys
d = json.load(open(sys.argv[1]))
assert d.get('any_synthetic') is True, "any_synthetic devrait etre true"
assert d['estimators'], "aucun estimateur"
assert all('drift_exponent' in e for e in d['estimators']), "exposant absent"
PY
    ok "JSON valide : synthétique signalé, exposants présents"
  else
    ko "JSON invalide ou incomplet"
  fi
fi

echo "════════════════════════════════════════════════════════════"
if [ "$ECHECS" -eq 0 ]; then
  echo "  $ETAPES/$ETAPES étapes OK — la chaîne est saine."
  echo "  Rappel : données FAUSSES. Ceci valide la tuyauterie, pas le robot."
  exit 0
fi
echo "  $ECHECS échec(s) sur $ETAPES étapes."
exit 1
