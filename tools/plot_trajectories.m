function plot_trajectories(basepath)
% PLOT_TRAJECTORIES  Superpose et compare les trajectoires MINS et openVINS.
%
%   plot_trajectories('data/trajectories/traj_20260727_141530')
%       lit  <base>_mins.csv, <base>_vins.csv  et (si présent) <base>_carolus.csv
%       produits par  tools/bag_to_csv.py, superpose les trajectoires (X,Y)
%       dans une figure, calcule les métriques de divergence/erreur, et
%       enregistre  <base>_comparison.png  (repris par le rapport LaTeX).
%
%   plot_trajectories()   sans argument : ouvre un sélecteur de fichier sur un
%                         CSV *_mins.csv et en déduit le préfixe.
%
%   Données attendues (colonnes CSV, en-tête sur la 1re ligne) :
%       t, x, y, z, qx, qy, qz, qw, yaw_rad     (t en s depuis le début du bag)
%
%   MÉTHODOLOGIE / HONNÊTETÉ DES MÉTRIQUES
%   --------------------------------------
%   Il n'y a PAS de vérité-terrain absolue embarquée sur ce robot. Deux
%   lectures distinctes sont donc calculées, sans les confondre :
%
%     (1) DIVERGENCE inter-estimateurs — toujours disponible. MINS et VINS
%         estiment la MÊME trajectoire réelle ; leur écart mutuel (après un
%         recalage rigide 2D optimal, algorithme de Kabsch) mesure leur
%         cohérence relative, PAS une erreur absolue.
%
%     (2) ERREUR vs Carolus — seulement si <base>_carolus.csv existe. Le fix
%         global Carolus (balise fixe au sol) est la seule ancre absolue,
%         sans dérive : il sert alors de quasi-vérité-terrain, et l'erreur de
%         chaque estimateur y est mesurée séparément. C'est la vraie mesure
%         de "précision".
%
%   Compatible Matlab (R2016b+) et GNU Octave.

  if nargin < 1 || isempty(basepath)
    [f, p] = uigetfile({'*.mat;*_mins.csv', ...
                        'Export trajectoire (.mat ou *_mins.csv)'}, ...
                       'Choisir l''export de l''essai');
    if isequal(f, 0); disp('annulé.'); return; end
    basepath = fullfile(p, f);
  end

  % Deux entrées acceptées (2026-07-27) : le .mat téléchargé depuis le cockpit
  % (structs MINS/VINS/CAROLUS) OU le préfixe des CSV (<base>_mins.csv ...).
  % Si on donne un .mat, on lit tout depuis lui ; sinon depuis les CSV.
  if numel(basepath) >= 4 && strcmpi(basepath(end-3:end), '.mat')
    S = load(basepath);
    mins = local_from_matstruct(S, 'MINS');
    vins = local_from_matstruct(S, 'VINS');
    caro = local_from_matstruct(S, 'CAROLUS');
    basepath = basepath(1:end-4);                 % pour nommer le PNG de sortie
  else
    basepath = regexprep(basepath, '_mins\.csv$', '');  % tolère le CSV lui-même
    mins = local_read_pose([basepath '_mins.csv']);
    vins = local_read_pose([basepath '_vins.csv']);
    caro = local_read_pose([basepath '_carolus.csv']);   % [] si absent
  end

  if isempty(mins) && isempty(vins)
    error('plot_trajectories:noData', ...
          'Ni MINS ni VINS trouvés pour le préfixe : %s', basepath);
  end

  % ── Figure ────────────────────────────────────────────────────────────────
  % NEUF PANNEAUX (2026-07-29, demande operateur). L'ancienne figure n'en
  % avait que deux, et surtout elle superposait les trajectoires BRUTES — or
  % MINS et VINS ne partagent pas le meme repere de depart : sur l'essai du
  % 29/07 elles etaient tournees de 177.9 deg l'une par rapport a l'autre, ce
  % qui donnait deux traits sans rapport visible alors que, recalees, elles se
  % superposaient a 0.563 m RMS pres. Le panneau 2 montre donc la
  % SUPERPOSITION APRES RECALAGE RIGIDE : c'est la seule vue ou l'oeil peut
  % juger si les deux estimateurs decrivent le meme mouvement.
  fig = figure('Color', 'w', 'Name', 'MINS vs openVINS', ...
               'Position', [40 40 1720 980]);

  cMINS = [0.95 0.42 0.21];   % orange (convention cockpit)
  cVINS = [0.13 0.70 0.85];   % cyan   (convention cockpit)
  cCARO = [0.20 0.75 0.35];   % vert   (ancre absolue)
  cGRIS = [0.55 0.55 0.58];

  % 'Marker','.' PARTOUT : une ligne '-' de plusieurs centaines de points ne se
  % rasterise PAS correctement dans ce MATLAB headless (bug trouve le
  % 2026-07-27) — les sections plates disparaissent silencieusement. Ajouter un
  % marqueur, meme minuscule, contourne le chemin de rendu fautif.
  LW = {'LineWidth', 1.6, 'Marker', '.', 'MarkerSize', 3};

  % ---- Panneau 1 : trajectoires BRUTES -------------------------------------
  ax1 = subplot(3, 3, 1); hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
  axis(ax1, 'equal');
  h = []; lab = {};
  if ~isempty(mins)
    h(end+1) = plot(ax1, mins.x, mins.y, '-', 'Color', cMINS, LW{:});
    plot(ax1, mins.x(1), mins.y(1), 'o', 'Color', cMINS, 'MarkerFaceColor', cMINS, 'MarkerSize', 7);
    plot(ax1, mins.x(end), mins.y(end), 's', 'Color', cMINS, 'MarkerFaceColor', cMINS, 'MarkerSize', 8);
    lab{end+1} = 'MINS';
  end
  if ~isempty(vins)
    h(end+1) = plot(ax1, vins.x, vins.y, '-', 'Color', cVINS, LW{:});
    plot(ax1, vins.x(1), vins.y(1), 'o', 'Color', cVINS, 'MarkerFaceColor', cVINS, 'MarkerSize', 7);
    plot(ax1, vins.x(end), vins.y(end), 's', 'Color', cVINS, 'MarkerFaceColor', cVINS, 'MarkerSize', 8);
    lab{end+1} = 'openVINS';
  end
  if ~isempty(caro)
    h(end+1) = plot(ax1, caro.x, caro.y, '.', 'Color', cCARO, 'MarkerSize', 9);
    lab{end+1} = 'Carolus';
  end
  xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)');
  title(ax1, '1 · Traces bruts (reperes differents)', 'Interpreter', 'none');
  % Handles de legende conserves : copyobj ne suit PAS une legende laissee
  % seule (son parent est la figure, pas les axes), il faut la copier dans le
  % meme appel que ses axes — voir l'export par panneau plus bas.
  L1 = [];
  if ~isempty(h); L1 = legend(ax1, h, lab, 'Location', 'best'); end

  % ---- Preparation des grandeurs comparees ---------------------------------
  ok2 = ~isempty(mins) && ~isempty(vins);
  txt = {};
  if ok2
    [tc, mxy, vxy] = local_resample_common(mins, vins);
    ok2 = numel(tc) >= 2;
  end

  if ok2
    d_raw = hypot(mxy(:,1)-vxy(:,1), mxy(:,2)-vxy(:,2));
    [vxy_a, R, ~] = local_kabsch2d(vxy, mxy);
    dxy = mxy - vxy_a;
    d_al = hypot(dxy(:,1), dxy(:,2));
    % Recalage de similitude : identique au precedent mais l'echelle de VINS
    % est ramenee a celle de MINS. Ce qui reste apres est une erreur de FORME
    % pure, debarrassee de l'erreur d'echelle.
    [vxy_s, ~, ~, sc] = local_umeyama2d(vxy, mxy);
    d_sc = hypot(mxy(:,1)-vxy_s(:,1), mxy(:,2)-vxy_s(:,2));
    % Longueurs mesurees sur la base REECHANTILLONNEE, celle qui est tracee au
    % panneau 4. Les calculer sur les traces bruts donnait un chiffre en
    % desaccord visible avec sa propre courbe (29/07 : titre "100 %" au-dessus
    % de deux pentes nettement differentes) : MINS echantillonne a ~86 Hz et
    % VINS a ~80 Hz, et le bruit par echantillon gonfle la longueur cumulee
    % d'autant plus que la cadence est haute. Comparer deux longueurs brutes de
    % cadences differentes, c'est comparer deux niveaux de bruit.
    L_m = local_pathlen(mxy(:,1), mxy(:,2));
    L_v = local_pathlen(vxy(:,1), vxy(:,2));
    L_m_brut = local_pathlen(mins.x, mins.y);
    L_v_brut = local_pathlen(vins.x, vins.y);

    % ---- Panneau 2 : SUPERPOSITION apres recalage rigide -------------------
    ax2 = subplot(3, 3, 2); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
    axis(ax2, 'equal');
    p1 = plot(ax2, mxy(:,1), mxy(:,2), '-', 'Color', cMINS, LW{:});
    % Hierarchie voulue : la trace MISE A L'ECHELLE est la courbe principale
    % (c'est la superposition demandee), la version rigide reste visible en
    % fin liseré gris pour qu'on voie de combien l'echelle a corrige. En
    % pointillés fins par-dessus, elle etait illisible.
    p3 = plot(ax2, vxy_a(:,1), vxy_a(:,2), '-', 'Color', [0.72 0.74 0.78], ...
              'LineWidth', 1.2);
    p2 = plot(ax2, vxy_s(:,1), vxy_s(:,2), '-', 'Color', cVINS, LW{:});
    plot(ax2, mxy(1,1), mxy(1,2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    xlabel(ax2, 'X (m)'); ylabel(ax2, 'Y (m)');
    title(ax2, sprintf('2 · SUPERPOSITION (rot %.1f deg, echelle x%.3f)', ...
                       atan2d(R(2,1), R(1,1)), sc), 'Interpreter', 'none');
    L2 = legend(ax2, [p1 p2 p3], {'MINS', 'openVINS mis a l''echelle', ...
           'openVINS avant mise a l''echelle'}, 'Location', 'best');

    % ---- Panneau 3 : ecart dans le temps -----------------------------------
    ax3 = subplot(3, 3, 3); hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
    q1 = plot(ax3, tc, d_raw, '-', 'Color', cGRIS, LW{:});
    q2 = plot(ax3, tc, d_al,  '-', 'Color', [0.10 0.20 0.55], LW{:});
    q3 = plot(ax3, tc, d_sc,  '-', 'Color', [0.85 0.45 0.10], LW{:});
    % Echelle log : l'ecart brut (~90 m) ecrasait la courbe recalee (~0.5 m)
    % sur une ligne plate au ras de l'axe, donc illisible — or c'est ELLE qui
    % porte l'information.
    set(ax3, 'YScale', 'log');
    xlabel(ax3, 'temps (s)'); ylabel(ax3, 'ecart (m, echelle log)');
    title(ax3, '3 · Ecart MINS - VINS', 'Interpreter', 'none');
    L3 = legend(ax3, [q1 q2 q3], {'brut', 'recale (rigide)', 'recale + echelle'}, ...
           'Location', 'best');

    % ---- Panneau 4 : distance cumulee (comparaison d'ECHELLE) --------------
    % Deux estimateurs peuvent avoir la meme FORME et des echelles
    % differentes : c'est invisible sur une trajectoire recalee, et
    % parfaitement lisible ici (deux pentes differentes = deux echelles).
    ax4 = subplot(3, 3, 4); hold(ax4, 'on'); grid(ax4, 'on'); box(ax4, 'on');
    cm = [0; cumsum(hypot(diff(mxy(:,1)), diff(mxy(:,2))))];
    cv = [0; cumsum(hypot(diff(vxy(:,1)), diff(vxy(:,2))))];
    r1 = plot(ax4, tc, cm, '-', 'Color', cMINS, LW{:});
    r2 = plot(ax4, tc, cv, '-', 'Color', cVINS, LW{:});
    xlabel(ax4, 'temps (s)'); ylabel(ax4, 'distance cumulee (m)');
    title(ax4, sprintf('4 · Distance parcourue (VINS = %.0f %% MINS)', ...
                       100*L_v/max(L_m, eps)), 'Interpreter', 'none');
    L4 = legend(ax4, [r1 r2], {'MINS', 'openVINS'}, 'Location', 'best');

    % ---- Panneau 5 : cap (yaw) --------------------------------------------
    ax5 = subplot(3, 3, 5); hold(ax5, 'on'); grid(ax5, 'on'); box(ax5, 'on');
    ym = interp1(mins.t, unwrap(mins.yaw), tc) * 180/pi;
    yv = interp1(vins.t, unwrap(vins.yaw), tc) * 180/pi;
    ym = ym - ym(1); yv = yv - yv(1);          % cap RELATIF au depart
    s1 = plot(ax5, tc, ym, '-', 'Color', cMINS, LW{:});
    s2 = plot(ax5, tc, yv, '-', 'Color', cVINS, LW{:});
    xlabel(ax5, 'temps (s)'); ylabel(ax5, 'cap relatif (deg)');
    title(ax5, '5 · Cap parcouru depuis le depart', 'Interpreter', 'none');
    L5 = legend(ax5, [s1 s2], {'MINS', 'openVINS'}, 'Location', 'best');

    % ---- Panneau 6 : ecart de cap -----------------------------------------
    ax6 = subplot(3, 3, 6); hold(ax6, 'on'); grid(ax6, 'on'); box(ax6, 'on');
    dyaw = ym - yv;
    plot(ax6, tc, dyaw, '-', 'Color', [0.55 0.15 0.55], LW{:});
    xlabel(ax6, 'temps (s)'); ylabel(ax6, 'ecart de cap (deg)');
    title(ax6, sprintf('6 · Ecart de cap (fin %.1f deg)', dyaw(end)), ...
          'Interpreter', 'none');

    % ---- Panneau 7 : ecart decompose X / Y ---------------------------------
    % Un ecart concentre sur un seul axe ne se corrige pas comme un ecart
    % isotrope : le decomposer evite de confondre biais directionnel et bruit.
    ax7 = subplot(3, 3, 7); hold(ax7, 'on'); grid(ax7, 'on'); box(ax7, 'on');
    u1 = plot(ax7, tc, dxy(:,1), '-', 'Color', [0.85 0.25 0.25], LW{:});
    u2 = plot(ax7, tc, dxy(:,2), '-', 'Color', [0.20 0.45 0.85], LW{:});
    xlabel(ax7, 'temps (s)'); ylabel(ax7, 'ecart (m)');
    title(ax7, '7 · Ecart decompose (apres recalage)', 'Interpreter', 'none');
    L7 = legend(ax7, [u1 u2], {'axe X', 'axe Y'}, 'Location', 'best');

    % ---- Panneau 8 : distribution de l'ecart -------------------------------
    ax8 = subplot(3, 3, 8); hold(ax8, 'on'); grid(ax8, 'on'); box(ax8, 'on');
    nb = 24;
    lo = min(d_al); hi = max(d_al);
    if hi <= lo; hi = lo + 1e-6; end
    edges = linspace(lo, hi, nb+1);
    ctr = 0.5*(edges(1:end-1) + edges(2:end));
    cnt = zeros(1, nb);
    for k = 1:nb
      if k < nb
        cnt(k) = sum(d_al >= edges(k) & d_al < edges(k+1));
      else
        cnt(k) = sum(d_al >= edges(k) & d_al <= edges(k+1));
      end
    end
    bar(ax8, ctr, cnt, 1.0, 'FaceColor', [0.45 0.55 0.75], 'EdgeColor', 'none');
    ds = sort(d_al); p50 = ds(max(1, round(0.50*numel(ds))));
    p95 = ds(max(1, round(0.95*numel(ds))));
    yl = get(ax8, 'YLim');
    plot(ax8, [p50 p50], yl, '-',  'Color', [0.1 0.6 0.2], 'LineWidth', 1.6);
    plot(ax8, [p95 p95], yl, '--', 'Color', [0.85 0.2 0.2], 'LineWidth', 1.6);
    xlabel(ax8, 'ecart apres recalage (m)'); ylabel(ax8, 'occurrences');
    title(ax8, sprintf('8 · Distribution (med %.3f | p95 %.3f m)', p50, p95), ...
          'Interpreter', 'none');

    txt{end+1} = 'DIVERGENCE MINS <-> VINS';
    txt{end+1} = sprintf('  brut    : RMS %.3f m | max %.3f m | fin %.3f m', ...
                         rms(d_raw), max(d_raw), d_raw(end));
    txt{end+1} = sprintf('  recale  : RMS %.3f m | max %.3f m  (Kabsch 2D)', ...
                         rms(d_al), max(d_al));
    txt{end+1} = sprintf('  mediane %.3f m | p95 %.3f m', p50, p95);
    txt{end+1} = sprintf('  + echelle : RMS %.3f m | max %.3f m  (Umeyama)', ...
                         rms(d_sc), max(d_sc));
    % Etendue = rayon RMS autour du centroide. Publie explicitement parce que
    % l'echelle Umeyama (basee sur l'ETENDUE) et le ratio de longueurs (base
    % sur le CHEMIN, bruit compris) donnent des chiffres differents pour de
    % bonnes raisons : un estimateur qui gigote allonge son chemin sans
    % agrandir sa boucle. Les laisser cote a cote sans explication ferait
    % croire a une incoherence du script.
    et_m = sqrt(mean(sum((mxy - mean(mxy,1)).^2, 2)));
    et_v = sqrt(mean(sum((vxy - mean(vxy,1)).^2, 2)));
    txt{end+1} = sprintf('  ECHELLE VINS/MINS : x%.4f  (%+.1f %%)', sc, 100*(sc-1));
    txt{end+1} = sprintf('  etendue : MINS %.2f | VINS %.2f m (%.0f %%)', ...
                         et_m, et_v, 100*et_v/max(et_m, eps));
    txt{end+1} = '  (etendue = taille de la boucle ; longueur = chemin,';
    txt{end+1} = '   bruit compris. Les deux ratios different normalement.)';
    txt{end+1} = sprintf('  rotation de recalage : %.1f deg', atan2d(R(2,1), R(1,1)));
    txt{end+1} = sprintf('  longueur (20 Hz) : MINS %.2f | VINS %.2f m (%.0f %%)', ...
                         L_m, L_v, 100*L_v/max(L_m, eps));
    txt{end+1} = sprintf('  longueur (brute) : MINS %.2f | VINS %.2f m', ...
                         L_m_brut, L_v_brut);
    txt{end+1} = sprintf('  ecart de cap final : %.1f deg', dyaw(end));
    txt{end+1} = '';
    % Lecture guidee : c'est la difference entre "meme forme" et "meme
    % trajectoire" qui se joue ici, et elle n'est pas evidente a l'oeil.
    part_ech = 1 - rms(d_sc) / max(rms(d_al), eps);
    if rms(d_al) < 0.25 * max(rms(d_raw), eps)
      txt{end+1} = 'LECTURE : formes quasi identiques, reperes differents.';
      txt{end+1} = '  L''ecart brut vient du recalage, pas d''une derive.';
    else
      txt{end+1} = 'LECTURE : les formes elles-memes different -';
      txt{end+1} = '  un recalage rigide ne les reconcilie pas.';
    end
    txt{end+1} = sprintf('  Corriger l''echelle absorbe %.0f %% du residu.', ...
                         100*part_ech);
    if abs(sc-1) > 0.03
      txt{end+1} = '  ATTENTION : echelle mise a celle de MINS, qui n''est PAS';
      txt{end+1} = '  une verite terrain. Concordance non probante.';
    end
  end

  % ---- Panneau 9 : encart metriques ---------------------------------------
  if ~isempty(caro)
    txt{end+1} = '';
    txt{end+1} = 'ERREUR vs Carolus (ancre absolue)';
    if ~isempty(mins)
      em = local_err_vs_ref(mins, caro);
      txt{end+1} = sprintf('  MINS : RMS %.3f m | max %.3f m (n=%d)', em.rms, em.max, em.n);
    end
    if ~isempty(vins)
      ev = local_err_vs_ref(vins, caro);
      txt{end+1} = sprintf('  VINS : RMS %.3f m | max %.3f m (n=%d)', ev.rms, ev.max, ev.n);
    end
  else
    txt{end+1} = '';
    txt{end+1} = '(pas de fix Carolus dans cet essai :';
    txt{end+1} = ' divergence RELATIVE seule, pas une erreur absolue)';
  end

  ax9 = subplot(3, 3, 9); axis(ax9, 'off');
  text(ax9, 0.0, 1.0, txt, 'FontName', 'FixedWidth', 'FontSize', 8.5, ...
       'VerticalAlignment', 'top', 'Interpreter', 'none');
  title(ax9, '9 · Metriques', 'Interpreter', 'none');

  % ── EXPORT PANNEAU PAR PANNEAU (2026-07-29, retour operateur) ───────────
  % La planche 3x3 est un plan d'ensemble : lisible a l'ecran, illisible une
  % fois reduite a la largeur d'une page imprimee. Chaque panneau est donc
  % RE-EXPORTE seul en 200 dpi, pour etre insere en pleine largeur dans le
  % rapport. copyobj plutot qu'un retrace : retracer dupliquerait toute la
  % logique de dessin, et deux chemins de dessin finissent toujours par
  % diverger. Les polices sont remontees puisque la figure n'est plus reduite.
  if ~exist('L2', 'var'); L2 = []; end
  if ~exist('L3', 'var'); L3 = []; end
  if ~exist('L4', 'var'); L4 = []; end
  if ~exist('L5', 'var'); L5 = []; end
  if ~exist('L7', 'var'); L7 = []; end
  pans = {ax1}; lgds = {L1};
  if ok2
    pans = {ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8};
    lgds = {L1,  L2,  L3,  L4,  L5,  [],  L7,  []};
  end
  % ── Export de la planche combinee : EN PREMIER ──────────────────────────
  % L'ordre compte. La boucle par panneau ci-dessous cree huit figures et huit
  % fichiers ; si elle echoue ou traine, la planche principale ne doit pas
  % etre perdue avec elle. Elle est donc ecrite AVANT, et la boucle est
  % enfermee dans un try/catch qui n'interrompt jamais la fonction.
  png = [basepath '_comparison.png'];
  try
    exportgraphics(fig, png, 'Resolution', 150);   % R2020a+
  catch
    print(fig, png, '-dpng', '-r150');              % repli universel
  end
  fprintf('  figure enregistree : %s\n', png);

  nok = 0;
  for ip = 1:numel(pans)
    a = pans{ip};
    if isempty(a) || ~isvalid(a); continue; end
    fpng = sprintf('%s_p%d.png', basepath, ip);
    try
      fp = figure('Color', 'w', 'Position', [80 80 900 620], 'Visible', 'off');
      lg = lgds{ip};
      if ~isempty(lg) && isvalid(lg)
        cp = copyobj([a, lg], fp);
      else
        cp = copyobj(a, fp);
      end
      set(cp(1), 'Position', [0.13 0.13 0.80 0.76], 'FontSize', 12);
      set(get(cp(1), 'Title'),  'FontSize', 14);
      set(get(cp(1), 'XLabel'), 'FontSize', 13);
      set(get(cp(1), 'YLabel'), 'FontSize', 13);
      exportgraphics(fp, fpng, 'Resolution', 200);
      close(fp);
      nok = nok + 1;
    catch err
      fprintf('  panneau %d NON exporte (%s)\n', ip, err.message);
      if exist('fp', 'var') && ~isempty(fp) && isvalid(fp); close(fp); end
    end
  end
  fprintf('  %d/%d panneaux exportes separement\n', nok, numel(pans));
  for i = 1:numel(txt); if ~isempty(txt{i}); fprintf('    %s\n', txt{i}); end; end
end

% ═══════════════════════════════════════════════════════════════════════════
% Fonctions locales
% ═══════════════════════════════════════════════════════════════════════════
function s = local_from_matstruct(S, name)
% Convertit un struct du .mat (S.MINS / S.VINS / S.CAROLUS, champs vecteurs
% colonne t,x,y,z,qx..qw,roll,pitch,yaw) vers le format interne. [] si le
% champ est absent ou vide (source non enregistrée, ex. VINS non initialisé).
  s = [];
  if ~isfield(S, name); return; end
  m = S.(name);
  if ~isstruct(m) || ~isfield(m, 't') || isempty(m.t); return; end
  s.t = m.t(:); s.x = m.x(:); s.y = m.y(:); s.z = m.z(:);
  s.qx = m.qx(:); s.qy = m.qy(:); s.qz = m.qz(:); s.qw = m.qw(:);
  s.roll = m.roll(:); s.pitch = m.pitch(:); s.yaw = m.yaw(:);
end

function s = local_read_pose(path)
% Lit un CSV de pose (11 colonnes numériques, en-tête sur 1 ligne) :
%   t,x,y,z,qx,qy,qz,qw,roll_rad,pitch_rad,yaw_rad
% [] si absent ou vide. Portable Matlab/Octave (fopen + textscan).
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

function [tc, aXY, bXY] = local_resample_common(a, b)
% Ré-échantillonne a et b sur une base de temps commune (recouvrement),
% par interpolation linéaire. Renvoie tc (col) et les positions interpolées.
  t_lo = max(a.t(1),   b.t(1));
  t_hi = min(a.t(end), b.t(end));
  if t_hi <= t_lo; tc = []; aXY = []; bXY = []; return; end
  n  = max(50, round((t_hi - t_lo) * 20));       % ~20 Hz
  tc = linspace(t_lo, t_hi, n).';
  aXY = [interp1(a.t, a.x, tc), interp1(a.t, a.y, tc)];
  bXY = [interp1(b.t, b.x, tc), interp1(b.t, b.y, tc)];
end

function [Bal, R, t, sc] = local_umeyama2d(B, A)
% Recalage de SIMILITUDE 2D : rotation + translation + ECHELLE (Umeyama 1991),
% amenant B sur A. C'est la methode standard d'evaluation de trajectoires
% quand l'echelle d'un estimateur n'est pas observable ou est suspecte.
%
% POURQUOI SEPAREMENT DU KABSCH RIGIDE, ET PAS A LA PLACE : l'echelle estimee
% ici EST un resultat de mesure (« VINS parcourt sc fois ce que parcourt
% MINS »), pas un detail d'affichage. La superposer en silence ferait
% coincider les deux estimateurs par construction et rendrait toute conclusion
% de concordance circulaire — d'autant que MINS n'est PAS une verite terrain.
% Les deux lectures sont donc calculees et publiees cote a cote : le rigide
% conserve l'erreur d'echelle visible, la similitude isole l'erreur de FORME.
  muA = mean(A, 1); muB = mean(B, 1);
  Ac = A - muA; Bc = B - muB;
  H = Bc.' * Ac;
  [U, ~, V] = svd(H);
  D = eye(2); D(2,2) = sign(det(V * U.'));
  R = V * D * U.';
  varB = sum(sum(Bc.^2)) / size(B, 1);
  if varB < eps
    sc = 1;
  else
    sc = trace(D * diag(svd(H))) / (size(B, 1) * varB);
  end
  t = muA.' - sc * R * muB.';
  Bal = (sc * R * B.' + t).';
end

function [Bal, R, t] = local_kabsch2d(B, A)
% Recalage rigide 2D optimal (rotation R + translation t) amenant B sur A,
% minimisant sum |A - (R*B + t)|^2 (Kabsch/Umeyama, sans échelle). Bal = B recalé.
  muA = mean(A, 1); muB = mean(B, 1);
  Ac = A - muA; Bc = B - muB;
  H = Bc.' * Ac;
  [U, ~, V] = svd(H);
  D = eye(2); D(2,2) = sign(det(V * U.'));       % pas de réflexion
  R = V * D * U.';
  t = muA.' - R * muB.';
  Bal = (R * B.' + t).';
end

function e = local_err_vs_ref(est, ref)
% Erreur de position de 'est' vs référence 'ref' (Carolus), interpolée aux
% instants de ref. Recalage rigide 2D d'abord (origines/repères distincts).
  t_lo = max(est.t(1), ref.t(1));
  t_hi = min(est.t(end), ref.t(end));
  m = ref.t >= t_lo & ref.t <= t_hi;
  e = struct('rms', NaN, 'max', NaN, 'n', 0);
  if nnz(m) < 3; return; end
  tr = ref.t(m);
  P  = [interp1(est.t, est.x, tr), interp1(est.t, est.y, tr)];
  Q  = [ref.x(m), ref.y(m)];
  Pa = local_kabsch2d(P, Q);
  d  = hypot(Q(:,1)-Pa(:,1), Q(:,2)-Pa(:,2));
  e.rms = rms(d); e.max = max(d); e.n = numel(d);
end

function L = local_pathlen(x, y)
  L = sum(hypot(diff(x), diff(y)));
end

function r = rms(v)
  v = v(~isnan(v));
  if isempty(v); r = NaN; else; r = sqrt(mean(v.^2)); end
end
