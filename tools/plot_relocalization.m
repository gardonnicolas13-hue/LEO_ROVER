function plot_relocalization(csv_base)
% plot_relocalization(csv_base) — MINS free-drift vs. beacon-corrected pose.
%
% Loads <csv_base>_mins.csv and <csv_base>_carolus.csv, both produced by
% tools/bag_to_csv.py (t,x,y,z,qx,qy,qz,qw,roll_rad,pitch_rad,yaw_rad).
% The first row of the *_carolus.csv file is taken as the relocalization
% instant t_reloc (the first VICON-slot fix MINS received from Carolus,
% /mins/external_ref/carolus). Plots MINS's trajectory split at t_reloc:
% free-drift phase (beacon out of view) in dashed red, post-correction
% phase in solid green, with a marker at the correction instant. Also
% reports the position jump magnitude at t_reloc.
%
% HONESTY NOTE (2026-08-12): this script is tooling, written ahead of the
% experiment it is meant to analyse. It has not been exercised against a
% real recorded run — see Fusion_Campaign.tex Table~openitems item 5 and
% appendix S:repro-mins-guide's VICON enable ladder (config_vicon.yaml,
% vicon.enabled is still false as of this writing). Do not cite a figure
% from this script as a measured result until a real bag has been through
% it end to end.
%
% Usage:
%   cd ~/TOUT/tools
%   matlab -batch "plot_relocalization('../data/trajectories/essai_A_<ts>')"
%
% Requires <csv_base>_mins.csv and <csv_base>_carolus.csv to both exist
% (bag_to_csv.py only writes the latter if /mins/external_ref/carolus
% carried at least one message in the bag — i.e. the beacon was actually
% seen during the recording).

  mins_path = [csv_base '_mins.csv'];
  car_path  = [csv_base '_carolus.csv'];

  if ~isfile(mins_path)
    error('plot_relocalization:missingFile', ...
          'MINS CSV not found: %s (run tools/bag_to_csv.py on the bag first)', ...
          mins_path);
  end
  if ~isfile(car_path)
    error('plot_relocalization:noBeaconFix', ...
          ['No %s — /mins/external_ref/carolus produced nothing in this bag, ' ...
           'so the beacon was never acquired during the recording. Nothing ' ...
           'to correct against.'], car_path);
  end

  M = readtable(mins_path);
  V = readtable(car_path);

  if height(M) < 2
    error('plot_relocalization:emptyMins', 'MINS CSV has fewer than 2 rows: %s', mins_path);
  end
  if height(V) < 1
    error('plot_relocalization:emptyCarolus', 'Carolus CSV is empty: %s', car_path);
  end

  t_reloc = V.t(1);   % first beacon fix = correction instant

  pre  = M.t <  t_reloc;
  post = M.t >= t_reloc;

  if ~any(pre)
    warning('plot_relocalization:noPreSample', ...
            'No MINS samples before t_reloc (%.3fs) — beacon was seen from the very first sample; nothing to compare against a free-drift phase.', t_reloc);
  end
  if ~any(post)
    warning('plot_relocalization:noPostSample', ...
            'No MINS samples at/after t_reloc (%.3fs) — beacon fix arrived after the recording ended.', t_reloc);
  end

  figure('Name', 'MINS relocalization', 'Color', 'w');
  hold on; grid on; axis equal;
  if any(pre)
    plot(M.x(pre),  M.y(pre),  'r--', 'DisplayName', 'Free drift (beacon OFF)');
  end
  if any(post)
    plot(M.x(post), M.y(post), 'g-',  'DisplayName', 'Post-correction');
    i0 = find(post, 1);
    plot(M.x(i0), M.y(i0), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'y', ...
         'DisplayName', sprintf('t_{reloc} = %.2fs', t_reloc));
  end
  legend('Location', 'best');
  xlabel('x (m)'); ylabel('y (m)');
  title('MINS trajectory: free drift vs. beacon-corrected');
  hold off;

  % Jump magnitude at the correction instant (needs at least one sample on
  % each side to be meaningful).
  i0 = find(post, 1);
  if ~isempty(i0) && i0 > 1
    dx = M.x(i0) - M.x(i0 - 1);
    dy = M.y(i0) - M.y(i0 - 1);
    fprintf('Correction jump at t_reloc = %.3fs: %.3f m\n', t_reloc, hypot(dx, dy));
  else
    fprintf('Correction jump: not computable (no sample pair straddling t_reloc).\n');
  end
end
