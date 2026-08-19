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
  spec = { 'MINS',     'mins', [0.95 0.42 0.21]     % orange — convention cockpit
           'openVINS', 'vins', [0.13 0.83 0.93]     % cyan   — convention cockpit
           'sqrtVINS', 'srv',  [0.92 0.70 0.03] };  % ambre  — « testé, non déployé »

  S = struct('name', {}, 'color', {}, 'd', {});
  fprintf('\n=== compare_estimators — %s ===\n', prefix);
  for k = 1:size(spec, 1)
    path = sprintf('%s_%s.csv', prefix, spec{k,2});
    d = local_read_pose(path);
    if isempty(d)
      fprintf('  %-9s : ABSENT (%s)\n', spec{k,1}, path);
      continue;
    end
    S(end+1) = struct('name', spec{k,1}, 'color', spec{k,3}, 'd', d); %#ok<AGROW>
    fprintf('  %-9s : %6d échantillons\n', spec{k,1}, numel(d.t));
  end
  if isempty(S)
    error('compare_estimators:vide', 'Aucune série trouvée pour ce préfixe.');
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
    h(end+1) = plot(ax1, d.x, d.y, '-', 'Color', c, 'LineWidth', 1.8, ...
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
    plot(ax2, d.t - d.t(1), max(dist, 1e-3), '-', 'Color', S(k).color, ...
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
  fprintf('\n  figure : %s\n\n', out);
end


function s = local_read_pose(path)
% Lit un CSV de pose bag_to_csv.py (11 colonnes, en-tête sur 1 ligne) :
% t,x,y,z,qx,qy,qz,qw,roll_rad,pitch_rad,yaw_rad. Renvoie [] si absent/vide —
% une série manquante doit être annoncée par l'appelant, pas planter ici.
  s = [];
  if exist(path, 'file') ~= 2; return; end
  fid = fopen(path, 'r');
  if fid < 0; return; end
  fgetl(fid);
  C = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f', 'Delimiter', ',', ...
               'CollectOutput', true, 'EmptyValue', NaN);
  fclose(fid);
  if isempty(C) || isempty(C{1}); return; end
  M = C{1};
  if size(M, 1) < 2 || size(M, 2) < 11; return; end
  s.t = M(:,1); s.x = M(:,2); s.y = M(:,3); s.z = M(:,4);
  s.qx = M(:,5); s.qy = M(:,6); s.qz = M(:,7); s.qw = M(:,8);
  s.roll = M(:,9); s.pitch = M(:,10); s.yaw = M(:,11);
end


function L = local_pathlen(x, y)
  L = sum(hypot(diff(x), diff(y)));
end
