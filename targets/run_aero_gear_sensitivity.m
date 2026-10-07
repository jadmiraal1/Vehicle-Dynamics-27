function out = run_aero_gear_sensitivity(p)
% RUN_AERO_GEAR_SENSITIVITY  How the aero package moves the optimal final drive.
%
% Two aero states: A = the ClA/CdA in the active config, B = the target
% package in p.scenario. If the config already carries the target values
% the two states coincide and the study shows nothing new (it says so). Each curve is a penalty against its own optimum (absolute
% times differ between states). Writes plots/gear_aero_sensitivity.png.
%
%   out = run_aero_gear_sensitivity()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_aero_gear_sensitivity(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');
here = vd_root();

GR        = 2.8:0.1:5.0;    % final-drive sweep [-] (coarser than run_gear_targets: flat objective)
BAND_OK   = [3.3 4.0];      % chosen acceptable range (team decision, run_gear_targets)
BAND_PREF = [3.45 3.7];     % chosen preferred range
TOL       = 0.05;           % indifference threshold [s]: ratios within it are equivalent

S(1).name = sprintf('this car''s aero (ClA %.2f / CdA %.2f)', p.ClA, p.CdA);
S(1).ClA  = p.ClA;                   S(1).CdA = p.CdA;
S(2).name = sprintf('target aero (ClA %.2f / CdA %.2f)', ...
                    p.scenario.ClA_target, p.scenario.CdA_target);
S(2).ClA  = p.scenario.ClA_target;   S(2).CdA = p.scenario.CdA_target;

[se, ke] = load_track(fullfile(here, 'tracks', 'track_endurance.csv'));

nG    = numel(GR);
t_lap = nan(nG, 2);
t_acc = nan(nG, 2);
vmax  = nan(nG, 2);

for k = 1:2
    for i = 1:nG
        p2 = set_gear(p, GR(i), p.rpm_motor_max);
        p2.ClA = S(k).ClA;  p2.CdA = S(k).CdA;
        vmax(i,k)  = p2.v_max;
        [~, t_lap(i,k)] = lap_sim(p2, se, ke, [], true);
        t_acc(i,k) = accel_time(p2);
    end
end

pen_lap = t_lap - min(t_lap, [], 1);     % penalty vs each state's OWN optimum
pen_acc = t_acc - min(t_acc, [], 1);

fprintf('\nFinal drive vs aero package - %s  (endurance lap and 75 m)\n', p.car);
if abs(S(1).ClA - S(2).ClA) < 1e-9 && abs(S(1).CdA - S(2).CdA) < 0.05
    fprintf('  This car already has the target aero, so the two packages below are the same.\n');
end
for k = 1:2
    [~, ie] = min(t_lap(:,k));  [~, ia] = min(t_acc(:,k));
    be = band_edges(GR, pen_lap(:,k), TOL);
    ba = band_edges(GR, pen_acc(:,k), TOL);
    fprintf('\n  %s\n', S(k).name);
    vd_row('Best ratio, endurance', sprintf('%.2f', GR(ie)), ...
           sprintf('%.2f s; %.2f-%.2f within %.2f s', t_lap(ie,k), be, TOL));
    vd_row('Best ratio, 75 m', sprintf('%.2f', GR(ia)), ...
           sprintf('%.2f s; %.2f-%.2f within %.2f s', t_acc(ia,k), ba, TOL));
    vd_row(sprintf('Time lost at %.2f: endurance / 75 m', p.gear_ratio), ...
           sprintf('%+.2f / %+.2f s', interp1(GR, pen_lap(:,k), p.gear_ratio), ...
                   interp1(GR, pen_acc(:,k), p.gear_ratio)));
end
[~, i1] = min(t_lap(:,1));  [~, i2] = min(t_lap(:,2));
fprintf('\n');
vd_row('Best endurance ratio, this car -> target', sprintf('%.2f -> %.2f', GR(i1), GR(i2)), ...
       sprintf('chosen range %.2f-%.2f', BAND_OK));
fprintf(['Assumes: lap simulation on the axle models; rev limit %.0f rpm (provisional);\n' ...
         '         no mass added for the target aero. Lap time changes little near the best\n' ...
         '         ratio, so read the range, not the decimal.\n'], p.rpm_motor_max);

out = struct('GR', GR, 't_lap', t_lap, 't_acc', t_acc, 'vmax', vmax, ...
             'pen_lap', pen_lap, 'pen_acc', pen_acc, 'states', {{S.name}}, 'tol', TOL);

outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
if vd_plots()
try
    aero_gear_plot(p, GR, pen_lap, pen_acc, S, BAND_OK, BAND_PREF, TOL, outdir);
    fprintf('Saved plots/gear_aero_sensitivity.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end


function p = set_gear(p, gr, rpm)
% Same car at a different final drive and rev ceiling (k_rot, v_max follow).
p = vd_set(p, 'gear_ratio', gr, 'rpm_motor_max', rpm);
end


function e = band_edges(GR, pen, tol)
% Widest contiguous run of ratios within tol of the optimum, edges by linear
% interpolation. Returns [lo hi].
in = pen(:).' <= tol;
d  = diff([false in false]);
i0 = find(d == 1);  i1 = find(d == -1) - 1;
[~, b] = max(i1 - i0);  i0 = i0(b);  i1 = i1(b);
lo = GR(i0);  hi = GR(i1);
if i0 > 1
    lo = interp1(pen(i0-1:i0), GR(i0-1:i0), tol);
end
if i1 < numel(GR)
    hi = interp1(pen(i1:i1+1), GR(i1:i1+1), tol);
end
e = [lo hi];
end


function aero_gear_plot(p, GR, pen_lap, pen_acc, S, bok, bpref, tol, outdir)
% Two panels: penalty against each state's own optimum.
C = [0.15 0.39 0.92;      % config aero   (blue)
     0.82 0.41 0.12];     % target aero   (orange)
MK = {'-o', '-s'};

f = figure('Visible','off','Position',[80 80 1040 460],'Color','w');

panels = {pen_lap, pen_acc};
titles = {'Endurance lap', '75 m acceleration'};
for q = 1:2
    ax = subplot(1, 2, q, 'Parent', f); hold(ax,'on'); grid(ax,'on');
    set(ax, 'GridAlpha', 0.12, 'Layer', 'top', 'FontSize', 9);
    yl = [0 max(0.55, 1.05*max(panels{q}(:)))];

    patch(ax, bok([1 2 2 1]),   yl([1 1 2 2]), [0.94 0.92 0.85], ...
          'EdgeColor','none', 'HandleVisibility','off');
    patch(ax, bpref([1 2 2 1]), yl([1 1 2 2]), [0.89 0.85 0.74], ...
          'EdgeColor','none', 'HandleVisibility','off');

    yline(ax, tol, ':', sprintf('%.2f s indifference', tol), ...
          'Color',[0.45 0.45 0.45], 'FontSize',8, 'HandleVisibility','off', ...
          'LabelHorizontalAlignment','left');

    for k = 1:2
        plot(ax, GR, panels{q}(:,k), MK{k}, 'MarkerSize',3.5, 'LineWidth',1.9, ...
             'Color', C(k,:), 'MarkerFaceColor','w', 'DisplayName', S(k).name);
        [~, im] = min(panels{q}(:,k));
        plot(ax, GR(im), 0, 'v', 'MarkerSize',8, 'LineWidth',1.2, ...
             'Color', C(k,:), 'MarkerFaceColor', C(k,:), 'HandleVisibility','off');
        text(ax, GR(im), -0.045*yl(2), sprintf('%.2f', GR(im)), ...
             'Color', [0.25 0.25 0.25], 'FontSize',8, 'HorizontalAlignment','center');
    end

    xline(ax, p.gear_ratio, '--', 'this car', 'Color',[0.35 0.35 0.35], ...
          'FontSize',8, 'HandleVisibility','off');
    ylim(ax, yl); xlim(ax, [GR(1) GR(end)]);
    xlabel(ax, 'final-drive ratio [-]');
    if q == 1
        ylabel(ax, 'time penalty vs that state''s own optimum [s]');
        legend(ax, 'Location','north', 'FontSize',8, 'Box','off');
    end
    title(ax, titles{q}, 'FontWeight','normal');
end

annotation(f, 'textbox', [0.005 0.945 0.99 0.05], 'String', ...
  sprintf('Best final drive vs aero package - shaded: chosen range %.2f-%.2f (preferred %.2f-%.2f)', bok, bpref), ...
  'EdgeColor','none', 'FontSize',11, 'FontWeight','bold', ...
  'HorizontalAlignment','center', 'VerticalAlignment','middle');

saveas(f, fullfile(outdir, 'gear_aero_sensitivity.png')); close(f);
end
