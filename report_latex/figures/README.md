# figures/

Emplacement des graphiques générés hors-ligne (repris par \includegraphics
dans le rapport, avec repli \IfFileExists tant que le PNG n'existe pas).

- `traj_comparison.png` — sortie de `tools/plot_trajectories.m`
  (superposition MINS vs openVINS + divergence, MÊME roulage). Généré par :
    tools/record_trajectories.sh  ->  tools/bag_to_csv.py  ->
    plot_trajectories('<base>')   (sauve <base>_comparison.png)
  puis copier/renommer en  report_latex/figures/traj_comparison.png

- `test_comparison.png` / `.pdf` — sortie de `tools/compare_tests.m`
  (Test 1/MINS vs Test 2/VINS, DEUX roulages séparés autour du labo).
  Généré automatiquement dans CE dossier (pas de copie manuelle) par :
    cockpit onglet Trajectory : sélecteur Test 1 -> roulage -> Export
                                  sélecteur Test 2 -> roulage -> Export
    tools/compare_tests.m   (lit web/exports/test1.mat + test2.mat)

  NOTE (2026-07-27) : les deux scripts appliquent 'Marker','.' sur les
  lignes de trajectoire — sans ça, une ligne '-' de plusieurs centaines de
  points ne se rasterise pas correctement dans ce MATLAB en contexte
  headless/batch (bug trouvé en direct, indépendant de Renderer/
  GraphicsSmoothing/LineJoin). Si un futur script de tracé est ajouté,
  reprendre ce même correctif.

- `divergence_par_essai.png` — bilan de TOUS les enregistrements appariés
  (§ sec:divsurvey de Results.tex). Généré le 2026-09-08 par un script
  matplotlib ponctuel, PAS par MATLAB, et c'est délibéré : les autres
  figures viennent de scripts .m, mais le piège de rastérisation headless
  documenté plus haut (Marker '.') n'a aucun équivalent ici, et matplotlib
  se vérifie plus simplement en lot. Toute régénération doit refaire la
  chaîne complète, qui n'est pas un simple tracé :

    1. extraire /firmware/wheel_odom des .bag (référence de mouvement
       INDÉPENDANTE des deux estimateurs) ;
    2. écarter les enregistrements dont l'odométrie roues est elle-même
       corrompue — deux le sont, avec des chemins de l'ordre de 1e25 m
       (test2_20260728_142421, test2_20260805_165507) ;
    3. classer chaque essai en « roule » (chemin roues > 1 m) ou
       « immobile », car à l'arrêt tout déplacement estimé est une erreur
       pure et c'est le seul test qui ne demande aucune vérité terrain ;
    4. tracer déplacement estimé (barres, échelle log) pour les immobiles
       et excursion max vs chemin roues (nuage log-log) pour les autres.

  POURQUOI PAS L'ERREUR FINALE : trois openVINS finissent à 0,00 m, ce qui
  se lit comme un suivi parfait. Vérification faite, deux ne sont jamais
  sortis d'une boule de 6 cm et un troisième a fait 249 m avant de revenir
  à l'origine par coïncidence. L'erreur finale ne distingue pas un
  estimateur juste d'un estimateur bloqué ; l'excursion maximale, si.
