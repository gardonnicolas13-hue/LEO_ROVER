#!/usr/bin/env bash
# ═════════════════════════════════════════════════════════════════════════════
# update_report.sh — recompile le rapport LaTeX et le publie sur le site web.
# À lancer après chaque avancée du projet (ou laisser Claude le faire).
#   1. pdflatex ×2 + biber (TeXLive 2026 locale)
#   2. déploie vers l'UNIQUE emplacement public : la carte de téléchargement
#      du Journal de bord (logbook.html) —
#      /reports/LEO_Rover_Mission_Control_Report_Nicolas_Gardon.pdf
#      (choix utilisateur 2026-07-08 : aucun autre lien vers le rapport)
# ═════════════════════════════════════════════════════════════════════════════
set -e
export PATH=/home/lab272/texlive/2026/bin/x86_64-linux:$PATH
cd /home/lab272/TOUT/report_latex

echo "[1/3] compilation..."
pdflatex -interaction=nonstopmode main.tex > /tmp/report_build.log 2>&1 || true
biber main >> /tmp/report_build.log 2>&1 || true
pdflatex -interaction=nonstopmode main.tex >> /tmp/report_build.log 2>&1 || true
pdflatex -interaction=nonstopmode main.tex >> /tmp/report_build.log 2>&1 || true

# `grep -a` OBLIGATOIRE : main.log contient des octets non-ASCII (noms de
# fichiers, messages de police), donc grep le classe « binaire » et SUPPRIME
# sa sortie sans le dire. Sans -a ce garde-fou ne voyait aucune erreur, quelle
# qu'elle soit — 13 erreurs TikZ sont passees inapercues le 29/07 pendant que
# le script affichait un succes.
errs=$(grep -ac "^!" /tmp/report_build.log || true)
[ -n "$errs" ] || errs=0
# Le nombre de pages est extrait APRES suppression des retours a la ligne :
# pdflatex coupe ses lignes de log a ~79 colonnes, donc la phrase "Output
# written on main.pdf (N pages" se retrouve scindee a un endroit qui depend de
# la longueur du document. Le grep echouait alors, le pipeline renvoyait
# non-zero, et `set -e` tuait le script AVANT le deploiement (constate le
# 29/07 en allongeant l'annexe D : compilation reussie, PDF correct, mais
# jamais publie). Le `|| true` garantit qu'un simple defaut d'affichage ne
# bloque plus jamais une publication.
pages=$(tr -d '\n' < main.log | grep -oE "Output written on main\.pdf \([0-9]+ pages" \
        | grep -oE "[0-9]+" | tail -1 || true)
[ -n "$pages" ] || pages="?"
[ "$errs" = "0" ] || { echo "ERREUR LaTeX ($errs) — voir /tmp/report_build.log"; exit 1; }

echo "[2/3] déploiement sur le site (carte du Journal de bord)..."
mkdir -p /home/lab272/TOUT/web/reports
cp main.pdf "/home/lab272/TOUT/web/reports/LEO_Rover_Mission_Control_Report_Nicolas_Gardon.pdf"

# ── ESTAMPILLE DE VERSION SUR LE LIEN (2026-07-29) ──────────────────────────
# Sans elle, Cloudflare sert une copie figée : le 29/07 le public recevait un
# rapport de 300 pages (cf-cache-status HIT, age 5256 s) alors que le disque
# en portait 309. Le tunnel IGNORE le `Cache-Control: no-cache` de l'origine
# (web/serve.py l'envoie pourtant, vérifié) — une règle du tableau de bord
# Cloudflare le remplace par max-age=14400, et elle n'est pas modifiable
# depuis cette machine. La query string, elle, fait partie de la clé de cache
# par défaut : changer ?v= à chaque publication force donc un MISS et livre
# la version fraîche immédiatement. Un rechargement forcé du navigateur
# n'aurait servi à rien, le cache étant au bord du réseau et pas chez le
# client.
VER=$(date +%s)
PDF="reports/LEO_Rover_Mission_Control_Report_Nicolas_Gardon.pdf"
python3 - "$VER" "$PDF" <<'PYEOF'
import re, sys
ver, pdf = sys.argv[1], sys.argv[2]
p = "/home/lab272/TOUT/web/logbook.html"
s = open(p, encoding="utf-8").read()
new = re.sub(r'href="' + re.escape(pdf) + r'(\?v=\d+)?"',
             'href="%s?v=%s"' % (pdf, ver), s)
if new != s:
    open(p, "w", encoding="utf-8").write(new)
    print("      lien du Journal de bord estampillé v=%s" % ver)
else:
    print("      ATTENTION : lien introuvable dans logbook.html — à vérifier")
PYEOF

echo "[3/3] OK — $pages pages, $(stat -c%s main.pdf) octets"
echo "  local  : http://localhost:8000/reports/LEO_Rover_Mission_Control_Report_Nicolas_Gardon.pdf"
echo "  public : https://cockpit.leo-rover-gardon.dev/$PDF?v=$VER"
echo "           (le ?v= est obligatoire : sans lui Cloudflare sert une copie périmée)"
