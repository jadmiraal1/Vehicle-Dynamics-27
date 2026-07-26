function lap_report(track_csv)
% LAP_REPORT  Lap dashboard + endurance energy budget figures -> plots/.
% Prints axle vs point-mass lap times side by side.

if nargin < 1 || isempty(track_csv), track_csv = 'track_endurance.csv'; end
p    = vehicle_params();
% Scenario assumptions from p.scenario (vehicle_params) - single source, so these
% cannot silently drift apart from run_energy_strategy / run_aero_targets.
REGEN_CAPTURE = p.scenario.regen_capture;
REGEN_RT      = p.scenario.regen_rt;
PACK_USABLE_F = p.scenario.pack_usable_f;
ENDURANCE_M   = p.scenario.endurance_m;   % OFFICIAL rules distance (feasibility basis)
here = vd_root();
[~, name] = fileparts(track_csv);  name = strrep(name, 'track_', '');
[s, kappa, x, y] = load_track(fullfile(here, 'tracks', track_csv));
[v, t_lap, E] = lap_sim(p, s, kappa, [], true);

% Before/after: same car on the point-mass lateral limit (the old optimistic
% model) so the fidelity gain from the axle upgrade is quantified, not asserted.
p_pm = p;  p_pm.grip_model = 'pointmass';
[~, t_pm] = lap_sim(p_pm, s, kappa, [], true);
fprintf(['lap_report: %s lap  axle(realistic) %.2f s  vs  point-mass %.2f s' ...
         '  ->  point mass is %.2f s / %.1f%% optimistic\n'], ...
        name, t_lap, t_pm, t_lap - t_pm, 100*(t_lap - t_pm)/t_lap);

ds   = diff(s);
ax_g = [diff(v.^2)./(2*ds); 0] / p.g;
ay_g = v.^2 .* kappa / p.g;                    % curvature unsigned -> |ay|
outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end

% Dashboard: map / speed trace / achieved g-g
f = figure('Visible', 'off', 'Position', [50 50 1250 840], 'Color', 'w');

subplot(2,2,[1 2]);
scatter(x, -y, 10, v, 'filled'); hold on;
plot(x(1), -y(1), 'ks', 'MarkerSize', 9, 'MarkerFaceColor', 'k');
axis equal; grid on;
cb = colorbar; cb.Label.String = 'speed [m/s]';
xlabel('x [m]'); ylabel('y [m]');
title(sprintf('%s - %.0f m, %.1f s per lap (QSS, square = start/finish)', ...
      name, s(end), t_lap), 'FontWeight', 'bold');

subplot(2,2,3);
plot(s, v, '-', 'Color', [0.82 0.82 0.82], 'LineWidth', 0.8); hold on;
scatter(s, v, 6, ax_g, 'filled'); caxis([-1.8 1.8]); grid on;
cb = colorbar; cb.Label.String = 'a_x [g]';
xlabel('distance s [m]'); ylabel('speed [m/s]');
title('Speed trace, colored by longitudinal g', 'FontWeight', 'bold');

subplot(2,2,4); hold on; grid on;
theta = linspace(0, pi, 150);
for vk = [10 20]
    G = gg_envelope(p, vk);
    aymax = ay_limit(p, vk);            % lateral extent = the limit the sim used
    ex = cos(theta);
    ex(cos(theta)>=0) = G.ax_accel*cos(theta(cos(theta)>=0));
    ex(cos(theta)<0)  = G.ax_brake*cos(theta(cos(theta)<0));
    plot(ex, aymax*sin(theta), '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);
    text(-2.15, aymax - 0.09, sprintf('%d m/s', vk), 'FontSize', 8, 'Color', [0.5 0.5 0.5]);
end
scatter(ax_g, ay_g, 8, v, 'filled', 'MarkerFaceAlpha', 0.65);
xlim([-2.3 1.3]); ylim([0 2.15]);
xlabel('a_x [g]  (+accel / -brake)'); ylabel('|a_y| [g]');
title('Achieved g-g vs envelope (curvature unsigned)', 'FontWeight', 'bold');

saveas(f, fullfile(outdir, ['lap_dashboard_' name '.png'])); close(f);

% Energy budget (endurance projection from this lap)
laps    = ENDURANCE_M / s(end);
E_dem   = E.drive_acc_Wh * laps / 1000;                          % kWh
E_regen = E.brake_wheel_Wh * REGEN_CAPTURE * REGEN_RT * laps / 1000;
pack_n  = p.E_pack_Wh / 1000;
pack_u  = PACK_USABLE_F * pack_n;

f = figure('Visible', 'off', 'Position', [50 50 1050 460], 'Color', 'w');
vals = [E_dem, E_dem - E_regen, pack_u, pack_n];
labs = {'No-regen demand', sprintf('Demand w/ regen (%.0f%% capture, %.0f%% RT)', ...
        100*REGEN_CAPTURE, 100*REGEN_RT), ...
        sprintf('Pack usable (%.0f%% est.)', 100*PACK_USABLE_F), 'Pack nominal'};
cols = [0.84 0.37 0; 0.90 0.62 0; 0 0.45 0.70; 0.34 0.71 0.91];
b = barh(numel(vals):-1:1, vals, 0.62, 'FaceColor', 'flat');
b.CData = cols;
for i = 1:numel(vals)
    text(vals(i)+0.1, numel(vals)+1-i, sprintf('%.1f kWh', vals(i)), ...
         'FontWeight', 'bold', 'VerticalAlignment', 'middle');
end
set(gca, 'YTick', 1:numel(vals), 'YTickLabel', labs(end:-1:1));
xline(pack_u, ':', 'Color', [0 0.45 0.70]);
xlabel('energy for 22 km endurance [kWh]'); xlim([0 11]); grid on; box off;
title(sprintf(['Endurance energy budget (%s) - regen and/or power derating ' ...
      'required to finish'], name), 'FontWeight', 'bold');
saveas(f, fullfile(outdir, 'energy_budget.png')); close(f);

fprintf('lap_report: dashboard + energy budget written to plots/ (%s)\n', name);
end
