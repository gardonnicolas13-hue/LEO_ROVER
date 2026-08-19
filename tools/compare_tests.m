function compare_tests(src1, src2, src3, out_dir)
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
  % ATTENTION : out_dir est passe en 4e position depuis l'ajout de src3
  % (2026-08-19). Le garde portait sur nargin < 3 et aurait pris le TROISIEME
  % essai pour un dossier de sortie — bug silencieux, la figure serait partie
  % dans un dossier nomme « .../test3 ».
  if nargin < 3; src3 = ''; end
  if nargin < 4 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'report_latex', 'figures');
  end
  if ~exist(out_dir, 'dir'); mkdir(out_dir); end

  % ── Correspondance essai -> estimateur RETENU (2026-08-19) ────────────────
  % Chaque essai enregistre les TROIS estimateurs ; ce tableau dit lequel on
  % retient de chaque tour. La correspondance a ete INVERSEE ce jour sur demande
  % operateur : elle etait test1=MINS / test2=VINS, elle est desormais
  % test1=openVINS, test2=MINS, test3=sqrtVINS, pour coller aux boutons du
  % cockpit. Un decalage entre les deux tracerait la mauvaise courbe sous le bon
  % nom -- le pire des defauts, parce qu'il ne se voit pas sur la figure.
  SPEC = { 'openVINS', 'vins', [0.13 0.83 0.93], '-'
           'MINS',     'mins', [0.95 0.42 0.21], '-'
           'sqrtVINS', 'srv',  [0.92 0.70 0.03], '--' };

  srcs = {src1, src2};
  if nargin >= 3 && ~isempty(src3); srcs{end+1} = src3; end

  T = struct('nom', {}, 'lbl', {}, 'col', {}, 'sty', {}, 'd', {});
  for k = 1:numel(srcs)
    d = local_load_source(srcs{k}, SPEC{k,1}, SPEC{k,2});
    T(end+1) = struct('nom', SPEC{k,1}, ...
                      'lbl', sprintf('Test %d — %s', k, SPEC{k,1}), ...
                      'col', SPEC{k,3}, 'sty', SPEC{k,4}, 'd', d); %#ok<AGROW>
  end

  fig = figure('Color', 'w', 'Name', 'Comparaison des essais', ...
               'Position', [100 100 1180 520]);

  % ---- Panneau 1 : trajectoires superposees ---------------------------------
  ax1 = subplot(1, 2, 1); hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
  axis(ax1, 'equal');
  h = []; lab = {};
  for k = 1:numel(T)
    d = T(k).d; c = T(k).col;
    % 'Marker','.' OBLIGATOIRE : sans marqueur, une ligne '-' de plusieurs
    % centaines de points ne se rasterise pas correctement dans ce MATLAB
    % headless (bug du 2026-07-27 : les sections plates disparaissent).
    h(end+1) = plot(ax1, d.x, d.y, T(k).sty, 'Color', c, 'LineWidth', 1.8, ...
                    'Marker', '.', 'MarkerSize', 3); %#ok<AGROW>
    plot(ax1, d.x(1),   d.y(1),   'o', 'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 7);
    plot(ax1, d.x(end), d.y(end), 's', 'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 8);
    lab{end+1} = T(k).lbl; %#ok<AGROW>
  end
  % Cadrage sur les series BORNEES : tracee a l'echelle d'un estimateur qui
  % part a 170 km, une trajectoire de 20 m devient un point. Seuil a 200 m,
  % meme convention que compare_estimators.m.
  lens = arrayfun(@(x) local_pathlen(x.d.x, x.d.y), T);
  borne = lens < 200;
  if any(borne)
    xs = []; ys = [];
    for k = find(borne(:)')
      xs = [xs; T(k).d.x]; ys = [ys; T(k).d.y]; %#ok<AGROW>
    end
    mrg = max(1, 0.1 * max(max(xs)-min(xs), max(ys)-min(ys)));
    xlim(ax1, [min(xs)-mrg, max(xs)+mrg]);
    ylim(ax1, [min(ys)-mrg, max(ys)+mrg]);
  end
  xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)');
  if any(borne) && ~all(borne)
    title(ax1, {'Tours du labo (o = depart, carre = arrivee)', ...
                'cadre ajuste aux tours bornes — les divergents sortent du cadre'}, ...
          'Interpreter', 'none');
  else
    title(ax1, 'Tours du labo (o = depart, carre = arrivee)', 'Interpreter', 'none');
  end
  legend(ax1, h, lab, 'Location', 'best');

  % ---- Panneau 2 : distance au point de depart ------------------------------
  ax2 = subplot(1, 2, 2); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
  for k = 1:numel(T)
    d = T(k).d;
    % Plancher 1 mm et echelle log : 1 m et 170 km sur un axe lineaire rendent
    % les series saines indiscernables de zero. eps comme plancher etirerait
    % l'axe sur seize decades de bruit numerique.
    plot(ax2, d.t, max(hypot(d.x - d.x(1), d.y - d.y(1)), 1e-3), T(k).sty, ...
         'Color', T(k).col, 'LineWidth', 1.6, 'Marker', '.', 'MarkerSize', 3);
  end
  set(ax2, 'YScale', 'log');
  xlabel(ax2, 'temps depuis le debut du roulage (s)');
  ylabel(ax2, 'distance au point de depart (m, echelle log)');
  title(ax2, 'Eloignement du point de depart');
  legend(ax2, {T.lbl}, 'Location', 'best');

  % ---- Metriques ------------------------------------------------------------
  txt = {};
  for k = 1:numel(T)
    d = T(k).d;
    clo = hypot(d.x(end)-d.x(1), d.y(end)-d.y(1));
    L   = local_pathlen(d.x, d.y);
    yaw = rad2deg(atan2(sin(d.yaw(end)-d.yaw(1)), cos(d.yaw(end)-d.yaw(1))));
    dur = d.t(end) - d.t(1);
    txt{end+1} = upper(T(k).lbl); %#ok<AGROW>
    txt{end+1} = sprintf('  duree %.1f s | longueur %.2f m', dur, L); %#ok<AGROW>
    txt{end+1} = sprintf('  fermeture de boucle %.3f m (%.1f%% de la longueur)', ...
                         clo, 100*clo/max(L, eps)); %#ok<AGROW>
    txt{end+1} = sprintf('  derive de cap %.1f deg', yaw); %#ok<AGROW>
    txt{end+1} = ''; %#ok<AGROW>
  end
  txt{end+1} = '(pas de verite-terrain absolue — la fermeture de boucle est';
  txt{end+1} = ' une PROXY de derive, pas une erreur metrique certifiee)';
  txt{end+1} = '(tours DISTINCTS : conditions non identiques. Pour comparer';
  txt{end+1} = ' les trois sur UN meme tour, voir compare_estimators.m)';

  % Interpreter 'none' et NON 'tex' : en mode tex, un underscore passe le
  % caractere suivant en indice — « compare_estimators.m » s'affichait
  % « compare(e)stimators.m ». Defaut vu en RELISANT la figure produite.
  % La figure est agrandie et l'encadre place SOUS les axes plutot que
  % par-dessus : avec trois essais il recouvrait le panneau de gauche.
  set(fig, 'Position', [100 100 1180 700]);
  set(ax1, 'Position', [0.07 0.42 0.40 0.50]);
  set(ax2, 'Position', [0.56 0.42 0.40 0.50]);
  annotation(fig, 'textbox', [0.05 0.01 0.90 0.34], 'String', txt, ...
             'BackgroundColor', [0.97 0.97 0.99], ...
             'EdgeColor', [0.7 0.7 0.75], 'FontName', 'FixedWidth', ...
             'FontSize', 8, 'Interpreter', 'none', 'VerticalAlignment', 'top');

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
