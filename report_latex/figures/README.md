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
