function out = run_gear_targets()
% RUN_GEAR_TARGETS  Final-drive sweep -> lap times, 75 m, top speed (#41).
% Rev cap is voltage-governed (~4500 rpm at this pack), not the 5500 mechanical rating.

p = vehicle_params();

GR = 2.8:0.05:5.0;                                   % final-drive ratio sweep [-]
tracks = {'track_endurance.csv', 'track_autocross.csv'};
here = vd_root();

nG = numel(GR);
t_lap = nan(nG, numel(tracks));
t_acc = nan(nG, 1);
vmax  = nan(nG, 1);

for i = 1:nG
    p2 = set_gear(p, GR(i));
    vmax(i) = p2.v_max;
    for j = 1:numel(tracks)
        [s, kappa] = load_track(fullfile(here, 'tracks', tracks{j}));
        [~, t_lap(i,j)] = lap_sim(p2, s, kappa, [], true);
    end
    t_acc(i) = accel_time(p2, 75);
end

% Lap-optimal ratio per event (parabola-agnostic: just the grid min)
[~, ie] = min(t_lap(:,1));  gr_e = GR(ie);
[~, ia] = min(t_lap(:,2));  gr_a = GR(ia);
[~, ic] = min(t_acc);       gr_c = GR(ic);
[~, icur] = min(abs(GR - p.gear_ratio));

fprintf('\nFINAL-DRIVE RATIO STUDY  (#41; lap sim, voltage-limited rev cap)\n');
fprintf('Rev limit %.0f rpm (voltage-governed at this pack); motor %.0f Nm peak / %.1f kW.\n', ...
        p.rpm_motor_max, p.T_motor_max, p.P_max/1e3);
fprintf('%6s %9s %8s %10s %10s  %s\n','ratio','vmax m/s','75m s','endur s','autox s','note');
for i = 1:nG
    if mod(i,2)~=1 && GR(i)~=p.gear_ratio, continue; end   % print every other row for brevity
    tag = ''; if i==icur, tag = '<- current'; end
    fprintf('%6.2f %9.1f %8.2f %10.2f %10.2f  %s\n', ...
            GR(i), vmax(i), t_acc(i), t_lap(i,1), t_lap(i,2), tag);
end
fprintf('\nLap-optimal ratio:  endurance %.2f:1 | autocross %.2f:1 | 75 m accel %.2f:1\n', ...
        gr_e, gr_a, gr_c);
fprintf('Current %.2f:1 costs %+.2f s endurance, %+.2f s autocross vs the endurance optimum.\n', ...
        p.gear_ratio, t_lap(icur,1)-t_lap(ie,1), t_lap(icur,2)-t_lap(ia,2));
fprintf('Caveats: QSS sim (lateral load-sensitive, longitudinal point-mass); flat-cap\n');
fprintf('        motor model, no efficiency map; rev cap is the pack-voltage limit, so a\n');
fprintf('        higher-voltage pack (or field weakening) would shift the optimum shorter.\n');

out = struct('gr', GR, 't_lap', t_lap, 't_acc', t_acc, 'vmax', vmax, ...
             'gr_opt_endur', gr_e, 'gr_opt_autox', gr_a, 'gr_opt_accel', gr_c, ...
             'gr_current', p.gear_ratio, 'tracks', {tracks});

try
    make_plot(p, GR, t_lap, t_acc, vmax, gr_e);
    fprintf('Plot written: plots/gear_targets.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end

function p = set_gear(p, gr)
% Same car at a different final-drive ratio (rebuild the two derived quantities
% that depend on it: rotating-inertia factor and the rev-limited top speed).
p.gear_ratio = gr;
sumI    = 4*p.I_wheel + p.I_rotor * gr^2;
p.k_rot = 1 + sumI / (p.m * p.Re^2);
p.v_max = (p.rpm_motor_max / gr) * (2*pi/60) * p.Re;
end

function t = accel_time(p, dist)
% Forward-Euler 75 m standing-start accel time [s], longitudinal g-g force.
v = 0; x = 0; t = 0; dx = 0.5;
while x < dist
    G = gg_envelope(p, v);
    a = G.ax_accel * p.g;
    if a <= 0, t = inf; return; end
    v = sqrt(v^2 + 2*a*dx);
    t = t + dx / max(v, 0.1);
    x = x + dx;
end
end

function make_plot(p, GR, t_lap, t_acc, vmax, gr_opt)
f = figure('Visible','off','Position',[80 80 820 520],'Color','w');
ax1 = axes(f); hold(ax1,'on'); grid(ax1,'on');
plot(ax1, GR, t_lap(:,1)-min(t_lap(:,1)), '-o','MarkerSize',3,'LineWidth',1.5,'DisplayName','endurance');
plot(ax1, GR, t_lap(:,2)-min(t_lap(:,2)), '-s','MarkerSize',3,'LineWidth',1.5,'DisplayName','autocross');
plot(ax1, GR, t_acc-min(t_acc),           '-^','MarkerSize',3,'LineWidth',1.5,'DisplayName','75 m accel');
xline(ax1, p.gear_ratio, '--', 'current', 'HandleVisibility','off');
xline(ax1, gr_opt, ':', 'optimum', 'HandleVisibility','off');
xlabel(ax1,'final-drive ratio [-]'); ylabel(ax1,'lap-time penalty vs each event''s optimum [s]');
title(ax1,'Final-drive ratio: lap-time penalty by event (flat optimum near 3.4-3.5:1)');
legend(ax1,'Location','north','FontSize',9);
outdir = fullfile(vd_root(),'plots');
if ~exist(outdir,'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir,'gear_targets.png')); close(f);
end
