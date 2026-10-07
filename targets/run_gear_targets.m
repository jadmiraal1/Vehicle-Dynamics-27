function out = run_gear_targets(p)
% RUN_GEAR_TARGETS  Final-drive ratio sweep: lap times, 75 m and top speed.
% The rev cap is set by pack voltage (~4500 rpm at this pack), not the motor's
% 5500 rpm mechanical rating, so robustness is checked at two other rev caps
% and at the target aero package. Writes plots/gear_freeze_penalty.png and
% plots/gear_freeze_robustness.png.
%
%   out = run_gear_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_gear_targets(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

GR     = 2.8:0.05:5.0;                               % final-drive ratio sweep [-]
tracks = {'track_endurance.csv', 'track_autocross.csv'};
here   = vd_root();

% The chosen final-drive range. The range is a team decision; the sweep is
% the evidence. It was chosen by minimax over aero and model states so it
% holds if aero or grip underdeliver.
BAND_OK   = [3.3 4.0];     % acceptable across all states
BAND_PREF = [3.45 3.7];    % preferred: four-state minimax neighborhood
RPM_ALT   = [4200 4880];   % rev-ceiling bracket around p.rpm_motor_max (low / full-charge est.)

nG = numel(GR);
t_lap = nan(nG, numel(tracks));
t_acc = nan(nG, 1);
vmax  = nan(nG, 1);

T = cell(1, numel(tracks));
for j = 1:numel(tracks)
    [T{j}.s, T{j}.k, ~, ~, T{j}.prov] = load_track(fullfile(here, 'tracks', tracks{j}));
end
for i = 1:nG
    p2 = set_gear(p, GR(i), p.rpm_motor_max);
    vmax(i) = p2.v_max;
    for j = 1:numel(tracks)
        t_lap(i,j) = track_time(p2, T{j});
    end
    t_acc(i) = accel_time(p2);
end

% Robustness: endurance lap at the alternate rev ceilings and at the target
% aero package, on every other ratio.
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

% Optimum per event, and the worst-case penalty across states inside the band.
% Each state is penalised against its own optimum: absolute times differ by
% seconds between aero states, so only within-state penalties compare.
[~, ie] = min(t_lap(:,1));  gr_e = GR(ie);
[~, ia] = min(t_lap(:,2));  gr_a = GR(ia);
[~, ic] = min(t_acc);       gr_c = GR(ic);
[~, icur] = min(abs(GR - p.gear_ratio));
inband = GRr >= BAND_OK(1) & GRr <= BAND_OK(2);
cols   = [t_lap(ir,1) t_rob t_aero];
pen    = cols - min(cols, [], 1);
wc     = max(pen, [], 2);                            % worst-case penalty per gear
spread = max(wc(inband));

in_pref = p.gear_ratio >= BAND_PREF(1) - 1e-9 && p.gear_ratio <= BAND_PREF(2) + 1e-9;
in_ok   = p.gear_ratio >= BAND_OK(1)   - 1e-9 && p.gear_ratio <= BAND_OK(2)   + 1e-9;
if in_pref, st = 'OK'; elseif in_ok, st = 'ACCEPTABLE'; else, st = 'OUTSIDE'; end

fprintf('\nFinal drive - %s  (rev limit %.0f rpm, set by pack voltage; motor %.0f Nm; power limit %.1f kW)\n', ...
        p.car, p.rpm_motor_max, p.T_motor_max, p.P_max/1e3);
fprintf('    %-8s %-10s %-8s %-11s %s\n', 'ratio', 'top speed', '75 m', 'endurance', 'autocross');
fprintf('    %-8s %-10s %-8s %-11s %s\n', '', '[m/s]', '[s]', 'lap [s]', 'run [s]');
for i = 1:nG
    if mod(i,2)~=1 && i~=icur, continue; end   % every other row, plus this car's ratio
    tag = ''; if i==icur, tag = '   this car'; end
    fprintf('    %-8.2f %-10.1f %-8.2f %-11.2f %-9.2f%s\n', ...
            GR(i), vmax(i), t_acc(i), t_lap(i,1), t_lap(i,2), tag);
end
fprintf('\n');
vd_row('Best ratio: endurance / autocross / 75 m', sprintf('%.2f / %.2f / %.2f', gr_e, gr_a, gr_c));
vd_row(sprintf('Time lost at %.2f: endurance / autocross', p.gear_ratio), ...
       sprintf('%+.2f / %+.2f s', t_lap(icur,1)-t_lap(ie,1), t_lap(icur,2)-t_lap(ia,2)));
vd_row(sprintf('Worst-case loss within %.1f-%.1f', BAND_OK(1), BAND_OK(2)), sprintf('%.3f s', spread), ...
       sprintf('rev limits %.0f-%.0f rpm, ClA up to %.1f', RPM_ALT(1), RPM_ALT(2), p.scenario.ClA_target));
vd_row('Chosen range (team decision), preferred', sprintf('%.2f-%.2f', BAND_PREF(1), BAND_PREF(2)), ...
       sprintf('now %.2f', p.gear_ratio), st);
vd_row('Chosen range (team decision), acceptable', sprintf('%.2f-%.2f', BAND_OK(1), BAND_OK(2)));
fprintf(['Assumes: lap simulation on the axle models; motor modelled as a torque limit plus\n' ...
         '         constant power (no measured torque curve); autocross from rest; rev limit\n' ...
         '         provisional. Pick sprockets within the range to suit packaging.\n']);

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
    fprintf('Saved plots/gear_freeze_penalty.png\n');
catch e
    fprintf('Plot not saved (penalty): %s\n', e.message);
end
end
if vd_plots()
try
    robustness_plot(p, GR, t_lap(:,1), GRr, t_rob, RPM_ALT, BAND_OK, BAND_PREF, outdir);
    fprintf('Saved plots/gear_freeze_robustness.png\n');
catch e
    fprintf('Plot not saved (robustness): %s\n', e.message);
end
end
end


function p = set_gear(p, gr, rpm)
% Same car at a different final drive and rev ceiling (k_rot, v_max follow).
p = vd_set(p, 'gear_ratio', gr, 'rpm_motor_max', rpm);
end


function t = track_time(p, T)
% Closed track: flying lap. Open track: from rest at the start line.
if T.prov.closed
    [~, t] = lap_sim(p, T.s, T.k, [], true);
else
    [~, t] = lap_sim(p, T.s, T.k, 0, false);
end
end


function penalty_plot(p, GR, t_lap, t_acc, bok, bpref, gr_opt, outdir)
% Time penalty against each event's own optimum.
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
xline(ax, p.gear_ratio, '--', 'this car', 'HandleVisibility','off');
ylim(ax, yl);
xlabel(ax,'final-drive ratio [-]');
ylabel(ax,'time penalty vs each event''s optimum [s]');
title(ax,'Event time penalty vs final-drive ratio');
legend(ax,'Location','northwest','FontSize',9);
saveas(f, fullfile(outdir,'gear_freeze_penalty.png')); close(f);
end


function robustness_plot(p, GR, t_en, GRr, t_rob, rpm_alt, bok, bpref, outdir)
% Endurance lap time at the three rev ceilings.
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
xline(ax, p.gear_ratio, '--', 'this car', 'HandleVisibility','off');
ylim(ax, yl);
xlabel(ax,'final-drive ratio [-]');
ylabel(ax,'endurance lap time [s]');
title(ax,'Endurance lap time vs final-drive ratio at three rev ceilings');
legend(ax,'Location','northwest','FontSize',9);
saveas(f, fullfile(outdir,'gear_freeze_robustness.png')); close(f);
end
