function plot_rectangle_test(basepath, length_m, width_m, estimator)
% PLOT_RECTANGLE_TEST  Compare une trajectoire enregistree a un rectangle
% mesure au sol (protocole "Test" de l'annexe D, sections MINS et openVINS).
%
%   plot_rectangle_test('data/trajectories/rect_test_20260810_143000', ...
%                        2.00, 1.50, 'mins')
%       lit  <basepath>_<estimator>.csv  (produit par tools/bag_to_csv.py),
%       superpose la trajectoire au rectangle ideal (length_m x width_m,
%       aligne sur le cap initial du robot), calcule et affiche :
%         - la boite englobante de la trajectoire (dimensions atteintes)
%         - l'ecart en % sur chaque dimension vs la mesure au ruban
%         - l'erreur de fermeture de boucle (depart vs arrivee)
%       et enregistre <basepath>_<estimator>_rect.png (a inserer dans le
%       rapport).
%
%   estimator : 'mins' ou 'vins' (lit <basepath>_mins.csv ou _vins.csv) ;
%               defaut 'mins'.
%
%   METHODOLOGIE / HONNETETE (meme esprit que plot_trajectories.m) : la
%   comparaison utilise la boite englobante de la trajectoire (extension
%   max en X et en Y), pas une detection automatique des 4 coins. Une
%   detection de coins serait plus fine mais plus fragile (patinage,
%   virages jamais nets a 90 degres) ; la boite englobante reste honnete
%   sur ce qu'elle verifie reellement : "le robot a-t-il couvert les bonnes
%   dimensions hors-tout", pas "chaque angle fait-il exactement 90 degres".
%   Valable si le rectangle est a peu pres aligne aux axes apres recalage
%   sur le cap initial -- suffisant pour ce protocole.
%
%   Compatible Matlab (R2016b+) et GNU Octave.

  if nargin < 4 || isempty(estimator); estimator = 'mins'; end
  if nargin < 3
    error('plot_rectangle_test:args', ...
          'Usage : plot_rectangle_test(basepath, length_m, width_m, estimator)');
  end
  estimator = lower(estimator);
  if ~any(strcmp(estimator, {'mins', 'vins'}))
    error('plot_rectangle_test:estimator', ...
          'estimator doit etre ''mins'' ou ''vins'', recu : %s', estimator);
  end

  basepath = regexprep(basepath, ['_' estimator '\.csv$'], '');
  p = local_read_pose([basepath '_' estimator '.csv']);
  if isempty(p)
    error('plot_rectangle_test:noData', 'Aucune donnee dans %s_%s.csv', ...
          basepath, estimator);
  end

  % ── Rectangle ideal, aligne sur le cap initial ───────────────────────────
  yaw0 = p.yaw(1);
  Rm = [cos(yaw0) -sin(yaw0); sin(yaw0) cos(yaw0)];
  corners_local = [0 0; length_m 0; length_m width_m; 0 width_m; 0 0]';
  corners_world = Rm * corners_local + [p.x(1); p.y(1)];

  % ── Boite englobante de la trajectoire ────────────────────────────────────
  achieved_L = max(p.x) - min(p.x);
  achieved_W = max(p.y) - min(p.y);
  err_L_pct = 100 * (achieved_L - length_m) / length_m;
  err_W_pct = 100 * (achieved_W - width_m)  / width_m;
  closure_m = hypot(p.x(end) - p.x(1), p.y(end) - p.y(1));

  % ── Figure ─────────────────────────────────────────────────────────────
  cTRAJ = [0.95 0.42 0.21];   % orange, meme convention que MINS dans plot_trajectories.m
  if strcmp(estimator, 'vins'); cTRAJ = [0.13 0.70 0.85]; end   % cyan, convention VINS
  cREF  = [0.20 0.75 0.35];   % vert, meme convention "ancre absolue"

  fig = figure('Color', 'w', 'Name', sprintf('%s vs rectangle mesure', upper(estimator)), ...
               'Position', [40 40 900 820]);
  ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on'); axis(ax, 'equal');
  hRef  = plot(ax, corners_world(1,:), corners_world(2,:), '--', 'Color', cREF, 'LineWidth', 1.6);
  hTraj = plot(ax, p.x, p.y, '-', 'Color', cTRAJ, 'LineWidth', 1.6, ...
               'Marker', '.', 'MarkerSize', 3);   % Marker '.' : voir plot_trajectories.m,
                                                   % une ligne seule ne se rasterise pas
                                                   % correctement sur ce Matlab headless.
  plot(ax, p.x(1), p.y(1), 'o', 'Color', cTRAJ, 'MarkerFaceColor', cTRAJ, 'MarkerSize', 8);
  plot(ax, p.x(end), p.y(end), 's', 'Color', cTRAJ, 'MarkerFaceColor', cTRAJ, 'MarkerSize', 8);
  % ── Balise AprilTag, si elle a ete vue pendant l'essai ──────────────────
  % Deux informations distinctes, a ne pas confondre :
  %   * les croix    = ou etait le ROBOT quand il a vu la balise ;
  %   * le losange   = ou la balise se situe, deduit de la pose robot + de la
  %                    position de la balise dans le repere camera.
  % CAVEAT ASSUME : cette deduction utilise l'extrinseque camera documentee
  % (cam-z->+x, cam-x->-y, cam-y->-z, camera a 0.15/0/0.10 m), qui reste une
  % mesure au RUBAN, pas une calibration Kalibr. La position tracee porte donc
  % cette incertitude ; la DISPERSION entre losanges, elle, reste significative
  % (meme balise physique -> tout ecart est de la derive).
  hTag = []; labTag = {};
  tagf = [basepath '_tags.csv'];
  if exist(tagf, 'file') == 2
    T = local_read_tags(tagf);
    if ~isempty(T)
      tx = NaN(numel(T.t),1); ty = NaN(numel(T.t),1);
      bx = NaN(numel(T.t),1); by = NaN(numel(T.t),1);
      for k = 1:numel(T.t)
        [rx, ry, ryaw] = local_pose_at(p, T.t(k));
        if isnan(rx); continue; end
        tx(k) = rx; ty(k) = ry;
        % balise -> repere base_link, puis -> repere odom par le cap robot
        bxb = T.z(k) + 0.15;
        byb = -T.x(k);
        bx(k) = rx + cos(ryaw)*bxb - sin(ryaw)*byb;
        by(k) = ry + sin(ryaw)*bxb + cos(ryaw)*byb;
      end
      ok = ~isnan(tx);
      if any(ok)
        cTAG = [0.55 0.35 0.95];   % violet : ni trajectoire, ni reference
        hTag(end+1) = plot(ax, tx(ok), ty(ok), 'x', 'Color', cTAG, ...
                           'MarkerSize', 9, 'LineWidth', 1.4);
        labTag{end+1} = 'Robot au moment de la detection';
        hTag(end+1) = plot(ax, bx(ok), by(ok), 'd', 'Color', cTAG, ...
                           'MarkerFaceColor', cTAG, 'MarkerSize', 7);
        labTag{end+1} = 'Balise (deduite)';
        fprintf('Balise : %d detections exploitables, position deduite moyenne (%.2f, %.2f) m\n', ...
                sum(ok), mean(bx(ok)), mean(by(ok)));
        if sum(ok) > 1
          fprintf('         dispersion des positions deduites : %.3f m (= derive)\n', ...
                  max(hypot(bx(ok)-mean(bx(ok)), by(ok)-mean(by(ok)))));
        end
      end
    end
  end

  xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
  title(ax, sprintf('%s : trajectoire vs rectangle mesure (%.2f x %.2f m)', ...
                     upper(estimator), length_m, width_m), 'Interpreter', 'none');
  legend(ax, [hTraj hRef hTag], [{upper(estimator), 'Rectangle mesure (ruban)'}, labTag], ...
         'Location', 'best');

  txt = sprintf(['Dimensions mesurees (ruban)             : %.3f x %.3f m\n' ...
                 'Dimensions atteintes (boite englobante) : %.3f x %.3f m\n' ...
                 'Ecart longueur : %+.1f %%    Ecart largeur : %+.1f %%\n' ...
                 'Fermeture de boucle (depart -> arrivee) : %.3f m'], ...
                length_m, width_m, achieved_L, achieved_W, ...
                err_L_pct, err_W_pct, closure_m);
  fprintf('%s\n', txt);
  annotation(fig, 'textbox', [0.02 0.01 0.6 0.14], 'String', txt, ...
             'EdgeColor', 'none', 'FitBoxToText', 'off', 'FontSize', 9, ...
             'VerticalAlignment', 'bottom');

  outpng = sprintf('%s_%s_rect.png', basepath, estimator);
  print(fig, outpng, '-dpng', '-r150');
  fprintf('Enregistre : %s\n', outpng);
end

function s = local_read_pose(path)
% Lit un CSV de pose (11 colonnes numeriques, en-tete sur 1 ligne) :
%   t,x,y,z,qx,qy,qz,qw,roll_rad,pitch_rad,yaw_rad
% [] si absent ou vide. Meme format que plot_trajectories.m (copie locale
% du lecteur : une fonction locale a un fichier .m n'est pas appelable
% depuis un autre fichier .m en Matlab/Octave).
  s = [];
  if exist(path, 'file') ~= 2; return; end
  fid = fopen(path, 'r');
  if fid < 0; return; end
  fgetl(fid);                                   % saute l'en-tete
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

function T = local_read_tags(path)
% Lit un CSV de detections AprilTag (t,id,x,y,z,distance_m) produit par
% bag_to_csv.py. Colonnes en repere CAMERA. [] si absent/vide.
  T = [];
  if exist(path, 'file') ~= 2; return; end
  fid = fopen(path, 'r');
  if fid < 0; return; end
  fgetl(fid);                                   % en-tete
  C = textscan(fid, '%f%f%f%f%f%f', 'Delimiter', ',', ...
               'CollectOutput', true, 'EmptyValue', NaN);
  fclose(fid);
  if isempty(C) || isempty(C{1}); return; end
  M = C{1};
  if size(M,1) < 1 || size(M,2) < 6; return; end
  T.t = M(:,1); T.id = M(:,2); T.x = M(:,3); T.y = M(:,4); T.z = M(:,5);
end

function [x, y, yaw] = local_pose_at(p, t)
% Pose interpolee a l'instant t. NaN si hors de la plage enregistree —
% renvoyer NaN plutot que d'extrapoler : une detection hors fenetre de pose
% ne doit pas produire une position inventee.
  x = NaN; y = NaN; yaw = NaN;
  if isempty(p) || t < p.t(1) || t > p.t(end); return; end
  i = find(p.t <= t, 1, 'last');
  if isempty(i); return; end
  if i >= numel(p.t); x = p.x(end); y = p.y(end); yaw = p.yaw(end); return; end
  dt = p.t(i+1) - p.t(i);
  a = 0; if dt > 0; a = (t - p.t(i)) / dt; end
  x   = p.x(i)   + a*(p.x(i+1)   - p.x(i));
  y   = p.y(i)   + a*(p.y(i+1)   - p.y(i));
  % cap : interpoler sur l'ecart deroule, sinon saut a +-pi
  d = p.yaw(i+1) - p.yaw(i);
  d = atan2(sin(d), cos(d));
  yaw = p.yaw(i) + a*d;
end
