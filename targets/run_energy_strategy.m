function out = run_energy_strategy()
% RUN_ENERGY_STRATEGY  Endurance power-cap sweep: lap time vs energy, regen scenarios.

p = vehicle_params();

P_CAPS_KW     = [62.7 50 45 40 35 30 28 25 22 20];
% Scenario assumptions come from p.scenario (vehicle_params) - single source, so
% these cannot silently drift apart from lap_report / run_aero_targets.
REGEN_CAPTURE = p.scenario.regen_capture;
REGEN_RT      = p.scenario.regen_rt;
PACK_USABLE_F = p.scenario.pack_usable_f;
MARGIN        = p.scenario.margin;
ENDURANCE_M   = p.scenario.endurance_m;   % OFFICIAL rules distance (feasibility basis)

here = vd_root();
[s, kappa, ~, ~, prov] = load_track(fullfile(here, 'tracks', 'track_endurance.csv'));
laps   = ENDURANCE_M / s(end);
usable = PACK_USABLE_F * p.E_pack_Wh / 1000;    % [kWh]

n = numel(P_CAPS_KW);
t_lap = nan(1,n);  E_nr = nan(1,n);  E_rg = nan(1,n);
for i = 1:n
    p2 = p;  p2.P_max = min(p.P_max, P_CAPS_KW(i)*1e3);
    [~, t_lap(i), E] = lap_sim(p2, s, kappa, [], true);
    E_nr(i) = E.drive_acc_Wh * laps / 1000;
    E_rg(i) = (E.drive_acc_Wh - E.brake_wheel_Wh*REGEN_CAPTURE*REGEN_RT) * laps / 1000;
end

fprintf('\nENDURANCE ENERGY STRATEGY  (%s, %.0f m, %.1f laps for 22 km)\n', ...
        prov.source_png, s(end), laps);
fprintf('usable pack %.2f kWh (%.0f%% of %.2f nominal), margin target %.2f kWh\n', ...
        usable, 100*PACK_USABLE_F, p.E_pack_Wh/1000, MARGIN*usable);
fprintf('%7s %10s %6s | %10s %6s | %7s %8s\n', 'cap kW', 'no-regen', '', ...
        'w/ regen', '', 'lap [s]', '+s/lap');
for i = 1:n
    fprintf('%7.1f %7.2f kWh %-4s | %7.2f kWh %-4s | %7.1f %+8.1f\n', ...
        P_CAPS_KW(i), E_nr(i), feas(E_nr(i), usable), ...
        E_rg(i), feas(E_rg(i), usable), t_lap(i), t_lap(i)-t_lap(1));
end

cap_rg = max(P_CAPS_KW(E_rg <= MARGIN*usable), [], 'omitnan');
cap_nr = max(P_CAPS_KW(E_nr <= MARGIN*usable), [], 'omitnan');
fprintf('\nT-STRAT decisions\n');
fprintf('  1. Regen is required: without it the margin-safe cap is %.0f kW\n', cap_nr);
fprintf('     (+%.1f s/lap); with %.0f%%x%.0f%% regen the cap rises to %.0f kW.\n', ...
        interp1(P_CAPS_KW, t_lap, cap_nr) - t_lap(1), ...
        100*REGEN_CAPTURE, 100*REGEN_RT, cap_rg);
fprintf('  2. Endurance VCU power cap: %.0f kW (margin-safe with regen).\n', cap_rg);
fprintf('  3. BMS SOC window: table assumes %.0f%% usable - attach this table\n', ...
        100*PACK_USABLE_F);
fprintf('     to the window decision; each +5%% usable is ~+2-3 kW of cap.\n');
fprintf('  Consistency: at %.0f kW the mean pack current ~%.0f A vs %.0f A main fuse.\n', ...
        cap_rg, cap_rg*1e3/p.V_pack_nom * (t_lap(1)/interp1(P_CAPS_KW,t_lap,cap_rg)) * 0.8, ...
        p.I_fuse_main);
fprintf('  Caveat: QSS point-mass; regen scenario + usable window are assumptions;\n');
fprintf('          re-run after pack load test and regen implementation.\n');

out = struct('P_caps_kW', P_CAPS_KW, 'E_noregen_kWh', E_nr, 'E_regen_kWh', E_rg, ...
             't_lap', t_lap, 'cap_regen_kW', cap_rg, 'cap_noregen_kW', cap_nr, ...
             'usable_kWh', usable);

try
    make_plot(P_CAPS_KW, E_nr, E_rg, t_lap, usable, MARGIN, ...
              REGEN_CAPTURE, REGEN_RT, here);
    fprintf('Plot written: plots/energy_strategy.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end

function s = feas(E, usable)
if E <= usable, s = 'OK'; else, s = 'DNF'; end
end

function make_plot(caps, E_nr, E_rg, t_lap, usable, margin, rc, rt, here)
f = figure('Visible', 'off', 'Position', [80 80 980 520], 'Color', 'w');
yyaxis left; hold on;
plot(caps, E_nr, 'o-', 'LineWidth', 1.8, 'DisplayName', 'no regen');
plot(caps, E_rg, 's-', 'LineWidth', 1.8, ...
     'DisplayName', sprintf('regen %.0f%% x %.0f%%', 100*rc, 100*rt));
yline(usable, '-', sprintf('usable pack %.2f kWh', usable), 'LineWidth', 1.2);
yline(margin*usable, ':', 'with 10% margin', 'LineWidth', 1.2);
ylabel('22 km endurance energy [kWh]');
yyaxis right;
plot(caps, t_lap, '^-', 'LineWidth', 1.2, 'DisplayName', 'lap time');
ylabel('lap time [s]');
xlabel('endurance power cap [kW]');
legend('Location', 'northwest'); grid on;
title('Endurance feasibility frontier - power cap vs energy and pace', ...
      'FontWeight', 'bold');
outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'energy_strategy.png'));
close(f);
end
