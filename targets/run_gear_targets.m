function out = run_gear_targets(p)
% RUN_GEAR_TARGETS  Final-drive sweep -> lap times, 75 m, top speed (#41).
% Rev cap is voltage-governed (~4500 rpm at this pack), not the 5500 mechanical rating.
% Writes the two freeze-evidence figures: plots/gear_freeze_penalty.png (the
% plateau, all events) and plots/gear_freeze_robustness.png (rev-ceiling bracket).
%
%   out = run_gear_targets()      the active car, from vd_car / cars/config_<CAR>.m
%   out = run_gear_targets(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = run_gear_targets(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)

GR     = 2.8:0.05:5.0;                               % final-drive ratio sweep [-]
tracks = {'track_endurance.csv', 'track_autocross.csv'};
here   = vd_root();

% Issued freeze recommendation (tracker #41). The band is the decision; the sweep
% below is the evidence. Chosen by minimax across {current, target-aero} x
% {combined, motor-limited} model states - the freeze must serve the car being
% DESIGNED (target aero), with downside cover if aero or grip underdeliver.
BAND_OK   = [3.3 4.0];     % acceptable across all states
BAND_PREF = [3.45 3.7];    % preferred: four-state minimax neighborhood
RPM_ALT   = [4200 4880];   % rev-ceiling bracket around p.rpm_motor_max (low / full-charge est.)

nG = numel(GR);
t_lap = nan(nG, numel(tracks));
t_acc = nan(nG, 1);
vmax  = nan(nG, 1);

for i = 1:nG
    p2 = set_gear(p, GR(i), p.rpm_motor_max);
    vmax(i) = p2.v_max;
    for j = 1:numel(tracks)
        [s, kappa] = load_track(fullfile(here, 'tracks', tracks{j}));
        [~, t_lap(i,j)] = lap_sim(p2, s, kappa, [], true);
    end
    t_acc(i) = accel_time(p2, 75);
end

% Robustness: endurance lap at the alternate rev ceilings AND at the TR27
% target aero package (coarser grid; nominal curve = the main sweep's column).
ir  = 1:2:nG;
GRr = GR(ir);
[se, ke] = load_track(fullfile(here, 'tracks', tracks{1}));
t_rob  = nan(numel(GRr), numel(RPM_ALT));
t_aero = nan(numel(GRr), 1);
for i = 1:numel(GRr)
    for j = 1:numel(RPM_ALT)
        p2 = set_gear(p, GRr(i), RPM_ALT(j));
        [~, t_rob(i,j)] = lap_sim(p2, se, ke, [], true);
    end
    p2 = set_gear(p, GRr(i), p.rpm_motor_max);
    p2.ClA = p.scenario.ClA_target;  p2.CdA = p.scenario.CdA_target;
    [~, t_aero(i)] = lap_sim(p2, se, ke, [], true);
end

% Optima per event and worst-case penalty across states inside the band.
% Each state column is penalized against its OWN optimum (levels differ by
% seconds between aero states; only within-state penalties are comparable).
[~, ie] = min(t_lap(:,1));  gr_e = GR(ie);
[~, ia] = min(t_lap(:,2));  gr_a = GR(ia);
[~, ic] = min(t_acc);       gr_c = GR(ic);
[~, icur] = min(abs(GR - p.gear_ratio));
inband = GRr >= BAND_OK(1) & GRr <= BAND_OK(2);
cols   = [t_lap(ir,1) t_rob t_aero];
pen    = cols - min(cols, [], 1);
wc     = max(pen, [], 2);                            % worst-case penalty per gear
spread = max(wc(inband));

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
fprintf('Current %.2f:1 costs %+.2f s endurance, %+.2f s autocross vs each optimum.\n', ...
        p.gear_ratio, t_lap(icur,1)-t_lap(ie,1), t_lap(icur,2)-t_lap(ia,2));
fprintf('Robustness: across rev ceilings (%.0f/%.0f/%.0f rpm) AND the target-aero state\n', ...
        RPM_ALT(1), p.rpm_motor_max, RPM_ALT(2));
fprintf('        (ClA %.1f), worst-case penalty inside the %.1f-%.1f band is %.3f s.\n', ...
        p.scenario.ClA_target, BAND_OK(1), BAND_OK(2), spread);
fprintf('Issued freeze (tracker #41): %.1f-%.1f preferred; %.1f-%.1f acceptable. Snap to\n', ...
        BAND_PREF(1), BAND_PREF(2), BAND_OK(1), BAND_OK(2));
fprintf('        buildable sprockets; packaging chooses within the band.\n');
fprintf('Caveats: QSS sim (lateral load-sensitive, longitudinal point-mass); flat-cap\n');
fprintf('        motor model, no efficiency map. Re-run on: real motor curve, aero package.\n');

out = struct('gr', GR, 't_lap', t_lap, 't_acc', t_acc, 'vmax', vmax, ...
             'gr_opt_endur', gr_e, 'gr_opt_autox', gr_a, 'gr_opt_accel', gr_c, ...
             'gr_current', p.gear_ratio, 'tracks', {tracks}, ...
             'gr_rob', GRr, 't_rob', t_rob, 'rpm_alt', RPM_ALT, ...
             'band_ok', BAND_OK, 'band_pref', BAND_PREF, 'rob_spread_s', spread);

outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
if vd_plots()
try
    penalty_plot(p, GR, t_lap, t_acc, BAND_OK, BAND_PREF, gr_e, outdir);
    fprintf('Plot written: plots/gear_freeze_penalty.png\n');
catch e
    fprintf('[penalty plot skipped: %s]\n', e.message);
end
end
if vd_plots()
try
    robustness_plot(p, GR, t_lap(:,1), GRr, t_rob, RPM_ALT, BAND_OK, BAND_PREF, outdir);
    fprintf('Plot written: plots/gear_freeze_robustness.png\n');
catch e
    fprintf('[robustness plot skipped: %s]\n', e.message);
end
end
end


function p = set_gear(p, gr, rpm)
% Same car at a different final drive / rev ceiling (rebuild the derived pair).
p = vd_set(p, 'gear_ratio', gr, 'rpm_motor_max', rpm);   % k_rot and v_max follow
end


function t = accel_time(p, dist)
% Forward-Euler 75 m standing-start accel time [s]. Launch via ax_limit
% (load-sensitive rear traction), through a coarse LUT since it's a solver.
vg  = linspace(0, p.v_max, 60);
axf = arrayfun(@(vv) ax_limit(p, vv, 'accel'), vg);
v = 0; x = 0; t = 0; dx = 0.5;
while x < dist
    if v >= p.v_max                            % rev limiter: coast at v_max
        t = t + (dist - x) / p.v_max;
        return;
    end
    a = interp1(vg, axf, min(max(v,0), p.v_max)) * p.g;
    if a <= 0, t = inf; return; end
    v = min(sqrt(v^2 + 2*a*dx), p.v_max);
    t = t + dx / max(v, 0.1);
    x = x + dx;
end
end


function penalty_plot(p, GR, t_lap, t_acc, bok, bpref, gr_opt, outdir)
% Figure 1: penalty vs each event's own optimum - the plateau argument.
f = figure('Visible','off','Position',[80 80 900 560],'Color','w');
ax = axes(f); hold(ax,'on'); grid(ax,'on');
yl = [0 1.05];
patch(ax, bok([1 2 2 1]),   yl([1 1 2 2]), [0.94 0.92 0.85], 'EdgeColor','none', ...
      'DisplayName', sprintf('acceptable band %.1f-%.1f', bok(1), bok(2)));
patch(ax, bpref([1 2 2 1]), yl([1 1 2 2]), [0.89 0.85 0.74], 'EdgeColor','none', ...
      'DisplayName', sprintf('preferred %.1f-%.1f', bpref(1), bpref(2)));
plot(ax, GR, t_lap(:,1)-min(t_lap(:,1)), '-o','MarkerSize',3.5,'LineWidth',1.8, ...
     'Color',[0.15 0.39 0.92], 'DisplayName','endurance lap');
plot(ax, GR, t_lap(:,2)-min(t_lap(:,2)), '-s','MarkerSize',3.5,'LineWidth',1.8, ...
     'Color',[0.82 0.41 0.12], 'DisplayName','autocross lap');
plot(ax, GR, t_acc-min(t_acc),           '-^','MarkerSize',3.5,'LineWidth',1.8, ...
     'Color',[0.35 0.49 0.35], 'DisplayName','75 m accel');
xline(ax, p.gear_ratio, '--', 'current', 'HandleVisibility','off');
ylim(ax, yl);
xlabel(ax,'final-drive ratio [-]');
ylabel(ax,'time penalty vs each event''s optimum [s]');
title(ax,'Event time penalty vs final-drive ratio');
legend(ax,'Location','northwest','FontSize',9);
saveas(f, fullfile(outdir,'gear_freeze_penalty.png')); close(f);
end


function robustness_plot(p, GR, t_en, GRr, t_rob, rpm_alt, bok, bpref, outdir)
% Figure 2: endurance lap under the rev-ceiling bracket - the robustness argument.
f = figure('Visible','off','Position',[80 80 900 560],'Color','w');
ax = axes(f); hold(ax,'on'); grid(ax,'on');
allt = [t_en(:); t_rob(:)];
yl = [min(allt)-0.1, max(allt)+0.15];
patch(ax, bok([1 2 2 1]),   yl([1 1 2 2]), [0.94 0.92 0.85], 'EdgeColor','none', 'HandleVisibility','off');
patch(ax, bpref([1 2 2 1]), yl([1 1 2 2]), [0.89 0.85 0.74], 'EdgeColor','none', 'HandleVisibility','off');
plot(ax, GRr, t_rob(:,1), '-', 'LineWidth',1.8, 'Color',[0.71 0.25 0.16], ...
     'DisplayName', sprintf('rev ceiling %.0f rpm', rpm_alt(1)));
plot(ax, GR,  t_en,       '-', 'LineWidth',1.8, 'Color',[0.15 0.39 0.92], ...
     'DisplayName', sprintf('rev ceiling %.0f rpm (nominal)', p.rpm_motor_max));
plot(ax, GRr, t_rob(:,2), '-', 'LineWidth',1.8, 'Color',[0.35 0.49 0.35], ...
     'DisplayName', sprintf('rev ceiling %.0f rpm', rpm_alt(2)));
xline(ax, p.gear_ratio, '--', 'current', 'HandleVisibility','off');
ylim(ax, yl);
xlabel(ax,'final-drive ratio [-]');
ylabel(ax,'endurance lap time [s]');
title(ax,'Endurance lap time vs final-drive ratio at three rev ceilings');
legend(ax,'Location','northwest','FontSize',9);
saveas(f, fullfile(outdir,'gear_freeze_robustness.png')); close(f);
end
