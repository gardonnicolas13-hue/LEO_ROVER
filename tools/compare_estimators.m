function compare_estimators(prefix, out_dir)
% COMPARE_ESTIMATORS  Les trois estimateurs, sur UN SEUL enregistrement.
%
%   compare_estimators('data/trajectories/sqrtvins_compare_20260813_175957')
%   compare_estimators(prefix, out_dir)
%
% CE QUE CETTE FONCTION EST, ET CE QU'ELLE N'EST PAS
% ---------------------------------------------------
% `compare_tests.m` compare DEUX ROULAGES (Test 1 sous MINS, Test 2 sous
% openVINS) : deux trajectoires différentes, deux moments différents. C'est
% utile, mais ça ne permet pas de dire qu'un estimateur est meilleur qu'un
% autre — les conditions ne sont pas les mêmes.
%
% Celle-ci fait l'inverse : UN enregistrement, TROIS estimateurs qui ont vu
% exactement les mêmes mouvements, les mêmes images et la même IMU aux mêmes
% instants. C'est la seule forme de comparaison qui soit recevable, et c'est
% la raison d'être de ce fichier. Ne pas fusionner les deux.
%
% ENTRÉES
%   prefix   préfixe des CSV produits par tools/bag_to_csv.py, SANS le
%            suffixe : '<...>/sqrtvins_compare_20260813_175957'. La fonction
%            cherche <prefix>_mins.csv, <prefix>_vins.csv, <prefix>_srv.csv.
%   out_dir  dossier des figures (défaut : report_latex/figures).
%
% SÉRIES ABSENTES
%   Une série manquante n'est PAS une erreur : elle est annoncée puis omise.
%   Les captures antérieures au 2026-08-18 n'ont aucun _srv.csv, parce que
%   record_trajectories.sh n'enregistrait pas /ov_srvins/odomimu — corrigé
%   depuis. Le rapport attribuait cette absence au réseau ; la relecture du
%   bag (rosbag info : cinq topics, celui-ci absent) a montré que le topic
%   n'avait tout simplement jamais été demandé.
%
% ÉCHELLE
%   Sur l'essai du 13/08, MINS ferme la boucle à 1,02 m et sqrtVINS rapporte
%   43 742 m. Tracés sur des axes linéaires communs, MINS devient un point.
%   Le panneau de droite est donc en Y logarithmique, et le panneau de gauche
%   est cadré sur les séries BORNÉES, les divergentes étant laissées sortir du
%   cadre plutôt que d'écraser tout le reste.

  if nargin < 1 || isempty(prefix)
    error('compare_estimators:prefix', ...
          'Donner le préfixe des CSV (sans _mins/_vins/_srv).');
  end
  if nargin < 2 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'report_latex', 'figures');
  end
  if ~exist(out_dir, 'dir'); mkdir(out_dir); end

  % Ordre volontaire : MINS d'abord, c'est la référence de ce projet.
  % 4e colonne : le STYLE DE TRAIT, et il n'est pas décoratif. L'orange de MINS
  % et l'ambre de sqrtVINS sont proches en teinte (constaté en relisant la
  % figure à trois séries) : à l'œil ils se confondent par endroits, et en
  % niveaux de gris — un rapport imprimé — ils deviennent identiques. Un style
  % distinct rend la figure lisible sans dépendre de la couleur.
  spec = { 'MINS',     'mins', [0.95 0.42 0.21], '-'      % orange — convention cockpit
           'openVINS', 'vins', [0.13 0.83 0.93], '-'      % cyan   — convention cockpit
           'sqrtVINS', 'srv',  [0.92 0.70 0.03], '--' };  % ambre  — « testé, non déployé »

  S = struct('name', {}, 'color', {}, 'style', {}, 'd', {}, ...
             'mock', {}, 'path', {});
  fprintf('\n=== compare_estimators — %s ===\n', prefix);
  for k = 1:size(spec, 1)
    path = sprintf('%s_%s.csv', prefix, spec{k,2});
    [d, why] = local_read_pose(path);
    if isempty(d)
      fprintf('  %-9s : INDISPONIBLE — %s\n', spec{k,1}, why);
      fprintf('  %-9s   %s\n', '', path);
      continue;
    end
    mock = local_is_mock(path);
    S(end+1) = struct('name', spec{k,1}, 'color', spec{k,3}, ...
                      'style', spec{k,4}, 'd', d, 'mock', mock, ...
                      'path', path); %#ok<AGROW>
    if mock
      fprintf('  %-9s : %6d échantillons   [SYNTHÉTIQUE — ne mesure rien]\n', ...
              spec{k,1}, numel(d.t));
    else
      fprintf('  %-9s : %6d échantillons\n', spec{k,1}, numel(d.t));
    end
  end
  if isempty(S)
    error('compare_estimators:vide', 'Aucune série trouvée pour ce préfixe.');
  end

  % ---- Cohérence temporelle entre séries -----------------------------------
  % Les trois estimateurs doivent avoir vu LA MÊME fenêtre de temps ; c'est la
  % prémisse qui rend la comparaison recevable. Si deux séries se recouvrent
  % mal, les métriques restent calculables mais ne comparent plus la même
  % chose — le dire ici plutôt que de laisser conclure sur des chiffres
  % silencieusement incomparables.
  recouv = NaN;   % % de recouvrement temporel, NaN si une seule série
  if numel(S) > 1
    t0 = arrayfun(@(x) x.d.t(1),   S);
    t1 = arrayfun(@(x) x.d.t(end), S);
    inter = min(t1) - max(t0);              % durée du recouvrement commun
    union = max(t1) - min(t0);
    if union > 0; recouv = 100 * max(inter, 0) / union; end
    if inter <= 0
      fprintf(['\n  ATTENTION : les séries ne se recouvrent PAS dans le temps.\n' ...
               '              Les comparer n''a pas de sens — vérifier qu''elles\n' ...
               '              viennent bien du même enregistrement.\n']);
    elseif union > 0 && inter / union < 0.9
      fprintf(['\n  ATTENTION : recouvrement temporel partiel (%.0f %% de la\n' ...
               '              fenêtre totale). Les métriques portent sur des\n' ...
               '              portions differentes du trajet.\n'], 100 * inter / union);
      for k = 1:numel(S)
        fprintf('              %-9s t = %8.1f .. %8.1f s\n', S(k).name, t0(k), t1(k));
      end
    end
  end

  % ---- Métriques -----------------------------------------------------------
  % Fermeture de boucle = distance entre le premier et le dernier point. Même
  % définition que partout ailleurs dans ce projet (compare_tests.m, rapport),
  % pour que les chiffres restent comparables entre outils.
  fprintf('\n  %-9s %14s %14s %12s\n', 'estimateur', 'fermeture (m)', 'chemin (m)', 'ratio');
  for k = 1:numel(S)
    d = S(k).d;
    S(k).close = hypot(d.x(end)-d.x(1), d.y(end)-d.y(1));
    S(k).len   = local_pathlen(d.x, d.y);
    ratio = S(k).close / max(S(k).len, eps);
    fprintf('  %-9s %14.2f %14.1f %12.4f\n', S(k).name, S(k).close, S(k).len, ratio);
  end

  % Une série est « bornée » si son chemin reste du même ordre que la scène.
  % Seuil à 200 m : un tour de labo fait ~20 m, openVINS a dérivé à 541 m
  % (borné au sens où il reste du même ordre que la trajectoire), sqrtVINS a
  % rapporté 43 742 m. Le seuil sépare donc « dérive » de « emballement ».
  borne = arrayfun(@(s) s.len < 200, S);
  fprintf('\n  séries bornées (< 200 m de chemin) : %d / %d\n', sum(borne), numel(S));

  % ---- Figure --------------------------------------------------------------
  fig = figure('Color', 'w', 'Name', 'Trois estimateurs, un enregistrement', ...
               'Position', [100 100 1180 520]);

  ax1 = subplot(1, 2, 1); hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
  axis(ax1, 'equal');
  h = []; lab = {};
  for k = 1:numel(S)
    d = S(k).d; c = S(k).color;
    % 'Marker','.' OBLIGATOIRE : sans marqueur, une ligne '-' de plusieurs
    % centaines de points ne se rasterise pas correctement dans ce MATLAB
    % headless — les sections plates disparaissent en silence (bug trouvé le
    % 2026-07-27, présent dans plot_trajectories.m depuis le début).
    h(end+1) = plot(ax1, d.x, d.y, S(k).style, 'Color', c, 'LineWidth', 1.8, ...
                    'Marker', '.', 'MarkerSize', 3); %#ok<AGROW>
    plot(ax1, d.x(1),   d.y(1),   'o', 'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 7);
    plot(ax1, d.x(end), d.y(end), 's', 'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 8);
    lab{end+1} = S(k).name; %#ok<AGROW>
  end
  % Cadrage sur les séries bornées uniquement (voir en-tête).
  if any(borne)
    xs = []; ys = [];
    for k = find(borne(:)')
      xs = [xs; S(k).d.x]; ys = [ys; S(k).d.y]; %#ok<AGROW>
    end
    % max-min plutot que range() : range() appartient a la Statistics
    % Toolbox, absente de cette installation (verifie le 2026-08-18,
    % « Undefined function 'range' »). Cette fonction doit tourner sur un
    % MATLAB de base.
    mx = max(1, 0.1 * max(max(xs)-min(xs), max(ys)-min(ys)));
    xlim(ax1, [min(xs)-mx, max(xs)+mx]);
    ylim(ax1, [min(ys)-mx, max(ys)+mx]);
    if ~all(borne)
      title(ax1, {'Trajectoires (o = départ, carré = arrivée)', ...
                  'cadre ajusté aux séries bornées — les divergentes sortent du cadre'}, ...
            'Interpreter', 'none');
    else
      title(ax1, 'Trajectoires (o = départ, carré = arrivée)', 'Interpreter', 'none');
    end
  end
  xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)');
  % 'southeast' et non 'best' : 'best' plaçait la légende AU-DESSUS du cadre,
  % où elle chevauchait le titre sur deux lignes.
  legend(ax1, h, lab, 'Location', 'southeast');

  ax2 = subplot(1, 2, 2); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
  for k = 1:numel(S)
    d = S(k).d;
    dist = hypot(d.x - d.x(1), d.y - d.y(1));
    % Échelle log : 1 m et 40 km sur le même axe linéaire rendraient MINS
    % indiscernable de zéro.
    % Le plancher est à 1 mm et NON à eps : au premier échantillon la distance
    % au départ vaut exactement 0, et un plancher à eps (2e-16) étirait l'axe
    % sur seize décades dont treize ne portaient que du bruit numérique — la
    % zone utile, entre 1 m et 40 km, se retrouvait écrasée en haut du cadre.
    % Un millimètre est la plus petite distance qui ait un sens ici.
    plot(ax2, d.t - d.t(1), max(dist, 1e-3), S(k).style, 'Color', S(k).color, ...
         'LineWidth', 1.6, 'Marker', '.', 'MarkerSize', 3);
  end
  set(ax2, 'YScale', 'log');
  xlabel(ax2, 'temps depuis le début (s)');
  ylabel(ax2, 'distance au point de départ (m, échelle log)');
  title(ax2, 'Éloignement — échelle log, l''emballement se voit comme une pente');
  legend(ax2, {S.name}, 'Location', 'best');

  out = fullfile(out_dir, 'estimators_comparison.png');
  try
    exportgraphics(fig, out, 'Resolution', 150);
  catch
    print(fig, out, '-dpng', '-r150');   % MATLAB < R2020a
  end
  fprintf('\n  figure : %s\n', out);

  % Export des métriques : la figure se regarde, le tableau se cite.
  local_export(S, prefix, recouv, out_dir);
end


function tf = local_is_mock(path)
% Une serie est-elle SYNTHETIQUE ? Deux marqueurs INDEPENDANTS, et il en
% suffit d'un. La redondance est voulue : un fichier renomme perd le prefixe
% mais garde son temoin, un fichier deplace seul perd son temoin mais garde le
% prefixe.
%
% ASYMETRIE ASSUMEE. Le test est volontairement LARGE : un lot de test dont le
% dossier porte un temoin voit TOUTES ses series marquees [MOCK], y compris
% celles qui sont des copies de vraies mesures. C'est le bon sens de l'erreur.
% Un [MOCK] de trop coute une seconde de perplexite ; un [MOCK] manquant met
% un chiffre fabrique dans un rapport, presente comme une mesure. Ne pas
% « corriger » ce comportement en le rendant plus permissif.
  [dossier, nom] = fileparts(path);
  tf = ~isempty(regexp(nom, '^MOCK[_-]', 'once'));          % marqueur 1 : le nom
  if tf; return; end
  base = regexprep(nom, '_(mins|vins|srv|fused|carolus|source)$', '');
  temoin = fullfile(dossier, [base '.SYNTHETIC.txt']);      % marqueur 2 : le temoin
  tf = exist(temoin, 'file') == 2;
end


function local_export(S, prefix, recouv, out_dir)
% Ecrit les metriques dans deux formats, a cote de la figure :
%   report_metrics.tex    tableau booktabs, \input-able depuis le rapport
%   metrics_summary.json  memes chiffres, pour tout autre consommateur
%
% Deux formats et non un : le .tex sert a citer, le .json sert a relire. Un
% tableau LaTeX est penible a reparser ; un JSON ne se compose pas.
%
% Une serie SYNTHETIQUE est marquee [MOCK] dans la colonne Source du tableau
% ET par "synthetic": true dans le JSON. Un chiffre fabrique qui arriverait
% dans le rapport sans etiquette serait pire que pas de chiffre du tout.
  if isempty(S); return; end
  horod = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
  aucun_mock = ~any([S.mock]);

  % ---- LaTeX ---------------------------------------------------------------
  ftex = fullfile(out_dir, 'report_metrics.tex');
  fid = fopen(ftex, 'w');
  if fid < 0
    fprintf('  (export LaTeX impossible : %s non inscriptible)\n', ftex);
  else
    fprintf(fid, '%% GENERE AUTOMATIQUEMENT par tools/compare_estimators.m — NE PAS EDITER.\n');
    fprintf(fid, '%% Toute modification sera ecrasee au prochain lancement.\n');
    fprintf(fid, '%% Source  : %s\n', local_tex(prefix));
    fprintf(fid, '%% Genere  : %s\n', horod);
    fprintf(fid, '%% Requiert booktabs et siunitx dans le preambule.\n');
    if ~aucun_mock
      fprintf(fid, '%%\n%% ATTENTION : au moins une serie est SYNTHETIQUE (colonne Source).\n');
    end
    fprintf(fid, '\\begin{tabular}{@{}lS[table-format=6.2]S[table-format=6.1]S[table-format=1.4]rl@{}}\n');
    fprintf(fid, '\\toprule\n');
    fprintf(fid, 'Estimator & {Loop closure (\\si{\\meter})} & {Path length (\\si{\\meter})} & {Ratio} & {Samples} & Source \\\\\n');
    fprintf(fid, '\\midrule\n');
    for k = 1:numel(S)
      if S(k).mock
        src = '\textbf{[MOCK]} synthetic';
      else
        src = 'measured';
      end
      fprintf(fid, '%s & %.2f & %.1f & %.4f & %d & %s \\\\\n', ...
              S(k).name, S(k).close, S(k).len, S(k).close / max(S(k).len, eps), ...
              numel(S(k).d.t), src);
    end
    fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
    if ~isnan(recouv)
      fprintf(fid, '\n%% Recouvrement temporel entre series : %.1f %%\n', recouv);
    end
    fclose(fid);
    fprintf('  metriques LaTeX : %s\n', ftex);
  end

  % ---- JSON ----------------------------------------------------------------
  m = struct();
  m.generated = horod;
  m.source_prefix = prefix;
  m.temporal_overlap_pct = recouv;      % NaN si une seule serie
  m.any_synthetic = ~aucun_mock;
  serie = struct('name', {}, 'loop_closure_m', {}, 'path_length_m', {}, ...
                 'divergence_ratio', {}, 'samples', {}, 'synthetic', {}, 'file', {});
  for k = 1:numel(S)
    serie(end+1) = struct( ...
      'name',             S(k).name, ...
      'loop_closure_m',   S(k).close, ...
      'path_length_m',    S(k).len, ...
      'divergence_ratio', S(k).close / max(S(k).len, eps), ...
      'samples',          numel(S(k).d.t), ...
      'synthetic',        logical(S(k).mock), ...
      'file',             S(k).path); %#ok<AGROW>
  end
  m.estimators = serie;
  fjson = fullfile(out_dir, 'metrics_summary.json');
  fid = fopen(fjson, 'w');
  if fid < 0
    fprintf('  (export JSON impossible : %s non inscriptible)\n', fjson);
  else
    fprintf(fid, '%s\n', jsonencode(m, 'PrettyPrint', true));
    fclose(fid);
    fprintf('  metriques JSON  : %s\n', fjson);
  end
  if ~aucun_mock
    fprintf(['  ATTENTION : au moins une serie est SYNTHETIQUE, marquee [MOCK]\n' ...
             '              dans le tableau. Ne pas citer ces chiffres.\n']);
  end
  fprintf('\n');
end


function t = local_tex(str)
% Echappe ce qui casserait une compilation LaTeX dans un commentaire de chemin.
  t = strrep(str, '\', '/');
  t = strrep(t, '_', '\_');
  t = strrep(t, '%', '\%');
  t = strrep(t, '#', '\#');
  t = strrep(t, '&', '\&');
end


function [s, why] = local_read_pose(path)
% Lit un CSV de pose bag_to_csv.py. Renvoie s=[] et une RAISON explicite en
% cas d'echec.
%
% Pourquoi une raison et pas juste [] : jusqu'au 2026-08-18 tout echec etait
% rapporte « ABSENT », y compris un fichier present mais tronque ou dont
% l'en-tete avait change. Un fichier corrompu et un fichier inexistant
% demandent des gestes opposes — refaire la conversion dans un cas, refaire la
% capture dans l'autre — et les confondre fait perdre du temps.
%
% Format attendu : 11 colonnes, en-tete sur 1 ligne
%   t,x,y,z,qx,qy,qz,qw,roll_rad,pitch_rad,yaw_rad
  s = []; why = '';
  if exist(path, 'file') ~= 2
    why = 'fichier absent'; return;
  end
  fid = fopen(path, 'r');
  if fid < 0
    why = 'illisible (droits ?)'; return;
  end
  entete = fgetl(fid);
  if ~ischar(entete)
    fclose(fid); why = 'fichier vide'; return;
  end
  cols = strsplit(strtrim(entete), ',');
  attendu = {'t','x','y','z','qx','qy','qz','qw','roll_rad','pitch_rad','yaw_rad'};
  if numel(cols) ~= numel(attendu)
    fclose(fid);
    why = sprintf('en-tete a %d colonnes, 11 attendues', numel(cols)); return;
  end
  manquants = setdiff(attendu, cols);
  if ~isempty(manquants)
    fclose(fid);
    why = sprintf('champs manquants dans l''en-tete : %s', strjoin(manquants, ', '));
    return;
  end
  C = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f', 'Delimiter', ',', ...
               'CollectOutput', true, 'EmptyValue', NaN);
  fclose(fid);
  if isempty(C) || isempty(C{1})
    why = 'en-tete seul, aucune donnee'; return;
  end
  M = C{1};
  if size(M, 2) < 11
    why = sprintf('%d colonnes de donnees, 11 attendues', size(M, 2)); return;
  end
  if size(M, 1) < 2
    why = sprintf('%d ligne(s) de donnees, 2 minimum', size(M, 1)); return;
  end
  bons = all(isfinite(M(:, 1:3)), 2);      % t, x, y exploitables
  if ~any(bons)
    why = 'aucune ligne exploitable (t/x/y non finis)'; return;
  end
  if ~all(bons)
    fprintf('    (%d ligne(s) non finies ecartees)\n', sum(~bons));
    M = M(bons, :);
  end
  s.t = M(:,1); s.x = M(:,2); s.y = M(:,3); s.z = M(:,4);
  s.qx = M(:,5); s.qy = M(:,6); s.qz = M(:,7); s.qw = M(:,8);
  s.roll = M(:,9); s.pitch = M(:,10); s.yaw = M(:,11);
end


function L = local_pathlen(x, y)
  L = sum(hypot(diff(x), diff(y)));
end
