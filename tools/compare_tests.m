function compare_tests(src1, src2, out_dir)
% COMPARE_TESTS  Compare deux essais indépendants MINS vs openVINS (roulages
% séparés autour du labo).
%
%   compare_tests()
%       charge automatiquement web/exports/test1.mat (estimateur MINS) et
%       web/exports/test2.mat (estimateur VINS) — l'export du cockpit web.
%       PAS RECOMMANDÉ pour un essai qui compte (2026-07-27, retour terrain) :
%       ce chemin dépend du site web (rosbridge + buffer en mémoire dans
%       leo_backend.py, remis à zéro à chaque redémarrage du backend, fréquent
%       ce soir-là) — voir §sec:july27rosbag du rapport.
%
%   compare_tests(src1, src2)
%   compare_tests(src1, src2, out_dir)
%       chemin ROBUST recommandé : src1/src2 pointent chacun soit vers un
%       .mat (export cockpit), soit vers le PRÉFIXE d'un jeu de CSV produit
%       par tools/bag_to_csv.py (ex. 'data/trajectories/test1_20260728_030512'
%       pour test1_20260728_030512_mins.csv) — copier/coller le préfixe
%       affiché par bag_to_csv.py à la fin de sa conversion. Ce chemin
%       n'utilise NI le site web NI leo_backend.py : rosbag record écrit
%       directement sur disque au fil de l'eau, indépendamment de rosbridge,
%       du navigateur, ou d'un redémarrage du backend — la seule dépendance
%       est que MINS/VINS eux-mêmes tournent. C'est le chemin à utiliser pour
%       un essai qui doit être fiable à 100%.
%
%   Pour chaque essai (src1 = Test 1, attendu en mode MINS ; src2 = Test 2,
%   attendu en mode VINS) : struct MINS lue pour src1, struct VINS pour src2
%   (mode .mat) ; <préfixe>_mins.csv / <préfixe>_vins.csv respectivement
%   (mode CSV).
%
%   MÉTHODOLOGIE
%   ------------
%   Test 1 et Test 2 sont DEUX ROULAGES DISTINCTS (pas deux estimateurs sur
%   la même trajectoire) — donc pas de recalage Kabsch point-à-point comme
%   dans plot_trajectories.m (rien à recaler : les horodatages et les
%   trajectoires réelles diffèrent). Les métriques utilisées sont celles qui
%   ont un sens pour deux essais indépendants :
%     - Trajectoire (X,Y) superposée : forme du tour du labo.
%     - Longueur parcourue : cohérence grossière entre les deux essais
%       (si le même tour a été fait, les longueurs doivent être proches).
%     - Erreur de fermeture de boucle : distance entre le point de départ
%       et le point d'arrivée. Un "tour du labo" revient near son point de
%       départ — cette erreur est donc une PROXY DIRECTE de la dérive
%       accumulée par l'estimateur sur tout le roulage (en l'absence de
%       vérité-terrain absolue, cf. la même honnêteté méthodologique que
%       plot_trajectories.m).
%     - Cap final vs cap initial : dérive de lacet sur le roulage complet.
%
%   Compatible Matlab (R2016b+) et GNU Octave.

  if nargin < 1 || isempty(src1)
    src1 = fullfile(fileparts(mfilename('fullpath')), '..', 'web', 'exports', 'test1.mat');
  end
  if nargin < 2 || isempty(src2)
    src2 = fullfile(fileparts(mfilename('fullpath')), '..', 'web', 'exports', 'test2.mat');
  end
  if nargin < 3 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'report_latex', 'figures');
  end
  if ~exist(out_dir, 'dir'); mkdir(out_dir); end

  t1 = local_load_source(src1, 'MINS', 'mins');
  t2 = local_load_source(src2, 'VINS', 'vins');

  fig = figure('Color', 'w', 'Name', 'Test 1 (MINS) vs Test 2 (VINS)', ...
               'Position', [100 100 1180 520]);
  cT1 = [0.95 0.42 0.21];   % orange — convention cockpit MINS
  cT2 = [0.13 0.83 0.93];   % cyan   — convention cockpit VINS

  % ---- Panneau 1 : trajectoires superposées --------------------------------
  ax1 = subplot(1, 2, 1); hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
  axis(ax1, 'equal');
  h = []; lab = {};
  % Marker', '.' AJOUTÉ À LA LIGNE (2026-07-27, bug trouvé en direct) : sans
  % marqueur, une ligne '-' de plusieurs centaines de points ne se rasterise
  % PAS correctement dans ce contexte MATLAB headless/batch — seuls de courts
  % fragments près des zones de forte pente apparaissent, les sections plates
  % disparaissent silencieusement (confirmé par test isolé : plot(x,sin(x))
  % pur casse pareil, indépendamment de Renderer/GraphicsSmoothing/LineJoin ;
  % ajouter un Marker, même minuscule, contourne le chemin de rendu fautif).
  % Repéré alors que plot_trajectories.m avait EXACTEMENT le même défaut,
  % silencieux depuis le début de la session — corrigé ici aussi.
  h(end+1) = plot(ax1, t1.x, t1.y, '-', 'Color', cT1, 'LineWidth', 1.8, ...
                  'Marker', '.', 'MarkerSize', 3);
  plot(ax1, t1.x(1),   t1.y(1),   'o', 'Color', cT1, 'MarkerFaceColor', cT1, 'MarkerSize', 7);
  plot(ax1, t1.x(end), t1.y(end), 's', 'Color', cT1, 'MarkerFaceColor', cT1, 'MarkerSize', 8);
  lab{end+1} = 'Test 1 — MINS';
  h(end+1) = plot(ax1, t2.x, t2.y, '-', 'Color', cT2, 'LineWidth', 1.8, ...
                  'Marker', '.', 'MarkerSize', 3);
  plot(ax1, t2.x(1),   t2.y(1),   'o', 'Color', cT2, 'MarkerFaceColor', cT2, 'MarkerSize', 7);
  plot(ax1, t2.x(end), t2.y(end), 's', 'Color', cT2, 'MarkerFaceColor', cT2, 'MarkerSize', 8);
  lab{end+1} = 'Test 2 — openVINS';
  xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)');
  title(ax1, 'Tour du labo — deux roulages  (o = départ, carré = arrivée)', ...
        'Interpreter', 'none');
  legend(ax1, h, lab, 'Location', 'best');

  % ---- Panneau 2 : distance au point de départ, par essai ------------------
  ax2 = subplot(1, 2, 2); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
  d1 = hypot(t1.x - t1.x(1), t1.y - t1.y(1));
  d2 = hypot(t2.x - t2.x(1), t2.y - t2.y(1));
  plot(ax2, t1.t, d1, '-', 'Color', cT1, 'LineWidth', 1.6, 'Marker', '.', 'MarkerSize', 3);
  plot(ax2, t2.t, d2, '-', 'Color', cT2, 'LineWidth', 1.6, 'Marker', '.', 'MarkerSize', 3);
  xlabel(ax2, 'temps depuis le début du roulage (s)');
  ylabel(ax2, 'distance au point de départ (m)');
  title(ax2, 'Éloignement du point de départ');
  legend(ax2, {'Test 1 — MINS', 'Test 2 — openVINS'}, 'Location', 'best');

  % ---- Métriques -------------------------------------------------------
  close1 = hypot(t1.x(end)-t1.x(1), t1.y(end)-t1.y(1));
  close2 = hypot(t2.x(end)-t2.x(1), t2.y(end)-t2.y(1));
  L1 = local_pathlen(t1.x, t1.y);
  L2 = local_pathlen(t2.x, t2.y);
  yawdrift1 = rad2deg(atan2(sin(t1.yaw(end)-t1.yaw(1)), cos(t1.yaw(end)-t1.yaw(1))));
  yawdrift2 = rad2deg(atan2(sin(t2.yaw(end)-t2.yaw(1)), cos(t2.yaw(end)-t2.yaw(1))));
  dur1 = t1.t(end) - t1.t(1);
  dur2 = t2.t(end) - t2.t(1);

  txt = {};
  txt{end+1} = 'TEST 1 — MINS';
  txt{end+1} = sprintf('  durée %.1f s | longueur %.2f m', dur1, L1);
  txt{end+1} = sprintf('  fermeture de boucle %.3f m (%.1f%% de la longueur)', ...
                       close1, 100*close1/max(L1, eps));
  txt{end+1} = sprintf('  dérive de cap %.1f deg', yawdrift1);
  txt{end+1} = '';
  txt{end+1} = 'TEST 2 — openVINS';
  txt{end+1} = sprintf('  durée %.1f s | longueur %.2f m', dur2, L2);
  txt{end+1} = sprintf('  fermeture de boucle %.3f m (%.1f%% de la longueur)', ...
                       close2, 100*close2/max(L2, eps));
  txt{end+1} = sprintf('  dérive de cap %.1f deg', yawdrift2);
  txt{end+1} = '';
  txt{end+1} = '(pas de vérité-terrain absolue — la fermeture de boucle est';
  txt{end+1} = ' une PROXY de dérive, pas une erreur métrique certifiée)';

  annotation(fig, 'textbox', [0.06 0.005 0.42 0.30], 'String', txt, ...
             'FitBoxToText', 'on', 'BackgroundColor', [0.97 0.97 0.99], ...
             'EdgeColor', [0.7 0.7 0.75], 'FontName', 'FixedWidth', ...
             'FontSize', 8, 'Interpreter', 'tex', 'VerticalAlignment', 'bottom');

  % ---- Export PNG + PDF pour le rapport ------------------------------------
  base = fullfile(out_dir, 'test_comparison');
  try
    exportgraphics(fig, [base '.png'], 'Resolution', 150);   % R2020a+
    exportgraphics(fig, [base '.pdf']);
  catch
    print(fig, [base '.png'], '-dpng', '-r150');              % repli universel
    print(fig, [base '.pdf'], '-dpdf');
  end
  fprintf('  ✓ figures enregistrées : %s.png / .pdf\n', base);
  for i = 1:numel(txt); if ~isempty(txt{i}); fprintf('    %s\n', txt{i}); end; end
end

% ═══════════════════════════════════════════════════════════════════════════
% Fonctions locales (mêmes conventions que plot_trajectories.m)
% ═══════════════════════════════════════════════════════════════════════════
function s = local_load_source(src, matname, csvsuffix)
% Charge un essai depuis .mat (export cockpit) ou préfixe CSV (bag_to_csv.py)
% — détecté par l'extension, même convention que plot_trajectories.m.
  if numel(src) >= 4 && strcmpi(src(end-3:end), '.mat')
    if exist(src, 'file') ~= 2
      error('compare_tests:missing', 'Introuvable : %s', src);
    end
    S = load(src);
    s = local_from_matstruct(S, matname);
    if isempty(s)
      error('compare_tests:empty', ...
            '%s : struct %s vide (aucune donnée bufferisée pour cet essai)', src, matname);
    end
  else
    path = [src '_' csvsuffix '.csv'];
    s = local_read_pose(path);
    if isempty(s)
      error('compare_tests:missing', ...
            'Introuvable ou vide : %s (lancer bag_to_csv.py sur le bag de cet essai)', path);
    end
  end
end

function s = local_from_matstruct(S, name)
  s = [];
  if ~isfield(S, name); return; end
  m = S.(name);
  if ~isstruct(m) || ~isfield(m, 't') || isempty(m.t); return; end
  s.t = m.t(:); s.x = m.x(:); s.y = m.y(:); s.z = m.z(:);
  s.qx = m.qx(:); s.qy = m.qy(:); s.qz = m.qz(:); s.qw = m.qw(:);
  s.roll = m.roll(:); s.pitch = m.pitch(:); s.yaw = m.yaw(:);
end

function s = local_read_pose(path)
% Lit un CSV de pose bag_to_csv.py (11 colonnes numériques, en-tête sur 1
% ligne) : t,x,y,z,qx,qy,qz,qw,roll_rad,pitch_rad,yaw_rad. [] si absent/vide.
  s = [];
  if exist(path, 'file') ~= 2; return; end
  fid = fopen(path, 'r');
  if fid < 0; return; end
  fgetl(fid);                                   % saute l'en-tête
  C = textscan(fid, '%f%f%f%f%f%f%f%f%f%f%f', 'Delimiter', ',', ...
               'CollectOutput', true, 'EmptyValue', NaN);
  fclose(fid);
  if isempty(C) || isempty(C{1}); return; end
  M = C{1};
  if size(M, 1) < 1 || size(M, 2) < 11; return; end
  s.t = M(:,1); s.x = M(:,2); s.y = M(:,3); s.z = M(:,4);
  s.qx = M(:,5); s.qy = M(:,6); s.qz = M(:,7); s.qw = M(:,8);
  s.roll = M(:,9); s.pitch = M(:,10); s.yaw = M(:,11);
end

function L = local_pathlen(x, y)
  L = sum(hypot(diff(x), diff(y)));
end
