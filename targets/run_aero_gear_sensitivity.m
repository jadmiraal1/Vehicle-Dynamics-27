function out = run_aero_gear_sensitivity(p)
% RUN_AERO_GEAR_SENSITIVITY  How the aero state moves the optimal final drive.
% Evidence figure for #41 x #36/#37: the freeze band has to serve the car being
% DESIGNED (target aero), not the aero the config currently carries.
%
% State A = the aero in the active car config. Note both cars/config_TR26.m and
% cars/config_TR27.m currently hold the SAME pair (ClA 1.301 / CdA 0.953, TR26
% measured) - TR27 has no aero of its own until the package lands, so "TR27 aero"
% and "TR26 aero" are the same numbers today. That is the point of this figure.
% State B = p.scenario.ClA_target / CdA_target, the issued TR27 target (#36/#37).
%
% Absolute lap times differ by seconds between the states, so only WITHIN-state
% penalties are comparable - each curve is referenced to its own optimum, the same
% convention run_gear_targets uses. Writes plots/gear_aero_sensitivity.png.
%
%   out = run_aero_gear_sensitivity()      the active car, from vd_car / cars/config_<CAR>.m
%   out = run_aero_gear_sensitivity(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = run_aero_gear_sensitivity(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)
here = vd_root();

GR        = 2.8:0.1:5.0;    % final-drive sweep [-] (coarser than run_gear_targets: flat objective)
BAND_OK   = [3.3 4.0];      % issued acceptable band (#41)
BAND_PREF = [3.45 3.7];     % issued preferred band (#41)
TOL       = 0.05;           % indifference threshold [s] - inside this, the gear is a toss-up

S(1).name = sprintf('config aero  (ClA %.2f / CdA %.2f)', p.ClA, p.CdA);
S(1).ClA  = p.ClA;                   S(1).CdA = p.CdA;
S(2).name = sprintf('TR27 target  (ClA %.2f / CdA %.2f)', ...
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
        t_acc(i,k) = accel_time(p2, 75);
    end
end

pen_lap = t_lap - min(t_lap, [], 1);     % penalty vs each state's OWN optimum
pen_acc = t_acc - min(t_acc, [], 1);

fprintf('\nAERO STATE vs OPTIMAL FINAL DRIVE  (#41 x #36/#37; endurance + 75 m)\n');
fprintf('Indifference threshold %.2f s. Issued band %.2f-%.2f (preferred %.2f-%.2f).\n', ...
        TOL, BAND_OK, BAND_PREF);
for k = 1:2
    [~, ie] = min(t_lap(:,k));  [~, ia] = min(t_acc(:,k));
    be = band_edges(GR, pen_lap(:,k), TOL);
    ba = band_edges(GR, pen_acc(:,k), TOL);
    fprintf('\n%s\n', S(k).name);
    fprintf('  endurance : best %.2f s at GR %.2f   flat within %.2f s over GR %.2f-%.2f\n', ...
            t_lap(ie,k), GR(ie), TOL, be);
    fprintf('  75 m accel: best %.2f s at GR %.2f   flat within %.2f s over GR %.2f-%.2f\n', ...
            t_acc(ia,k), GR(ia), TOL, ba);
    fprintf('  penalty at the current %.2f : endurance %+.2f s, accel %+.2f s\n', ...
            p.gear_ratio, interp1(GR, pen_lap(:,k), p.gear_ratio), ...
            interp1(GR, pen_acc(:,k), p.gear_ratio));
end
[~, i1] = min(t_lap(:,1));  [~, i2] = min(t_lap(:,2));
fprintf('\nEndurance optimum moves %+.2f (GR %.2f -> %.2f) when the target package lands.\n', ...
        GR(i2)-GR(i1), GR(i1), GR(i2));
fprintf(['Caveat: QSS lap sim, load-sensitive lateral + per-axle combined longitudinal;\n' ...
         '  rev cap %.0f rpm (voltage-governed, PROVISIONAL - datasheet load constant is\n' ...
         '  11-14 rpm/V); target-aero mass penalty NOT applied; objective is flat, so read\n' ...
         '  the band, not the decimal.\n'], p.rpm_motor_max);

out = struct('GR', GR, 't_lap', t_lap, 't_acc', t_acc, 'vmax', vmax, ...
             'pen_lap', pen_lap, 'pen_acc', pen_acc, 'states', {{S.name}}, 'tol', TOL);

outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
try
    aero_gear_plot(p, GR, pen_lap, pen_acc, S, BAND_OK, BAND_PREF, TOL, outdir);
    fprintf('Plot written: plots/gear_aero_sensitivity.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end


function p = set_gear(p, gr, rpm)
% Same car at a different final drive / rev ceiling (rebuild the derived pair).
% NOTE: duplicated from run_gear_targets.m - promote to lapsim/ if a third caller appears.
p = vd_set(p, 'gear_ratio', gr, 'rpm_motor_max', rpm);   % k_rot and v_max follow
end


function t = accel_time(p, dist)
% Forward integration in fixed VELOCITY steps - same scheme as run_gg_targets'
% accel_event, which is what reproduces the issued 4.60 s (#63).
% NOT the dx-stepped accel_time in run_gear_targets.m: that one advances time as
% dx/v_end, which overstates the mean speed over each step and reads ~0.25 s fast.
vg  = linspace(0, p.v_max, 60);
axf = arrayfun(@(vv) ax_limit(p, vv, 'accel'), vg);
v = 0; x = 0; t = 0; dv = 0.005;
while x < dist
    if v >= p.v_max                            % rev limiter: coast at v_max
        t = t + (dist - x) / p.v_max;
        return;
    end
    a = interp1(vg, axf, min(max(v,0), p.v_max)) * p.g;
    if a <= 0, return; end                     % drag/power ceiling
    dt = dv / a;
    x  = x + v*dt + 0.5*a*dt^2;
    t  = t + dt;
    v  = v + dv;
end
end


function e = band_edges(GR, pen, tol)
% Widest contiguous run of gear ratios within tol of the optimum, edges by
% linear interpolation. Returns [lo hi]; the honest form of "the optimum".
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
% Two panels, one axis each, penalty referenced to each state's own optimum.
% Colours match the repo's existing figures (blue / orange); validated as a
% categorical pair for CVD separation.
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

    xline(ax, p.gear_ratio, '--', 'current', 'Color',[0.35 0.35 0.35], ...
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
  'Optimal final drive vs aero state - shaded: issued acceptable 3.3-4.0 / preferred 3.45-3.7', ...
  'EdgeColor','none', 'FontSize',11, 'FontWeight','bold', ...
  'HorizontalAlignment','center', 'VerticalAlignment','middle');

saveas(f, fullfile(outdir, 'gear_aero_sensitivity.png')); close(f);
end
