function out = run_energy_strategy()
% RUN_ENERGY_STRATEGY  Endurance feasibility: power-cap x regen sweep.
% T-STRAT: the "how do we finish endurance" table (targets #43/#44/#65).
% Consumers: powertrain (deployment), controls (VCU torque map cap),
% electrical (BMS SOC window decision - attach this table).
%
% Method: run lap_sim on the endurance course with p.P_max overridden by
% each candidate cap; net 22 km energy = (drive - regen credit) x laps;
% feasible when it fits the usable pack WITH margin. Scenario constants
% below are assumptions until the regen implementation and pack load test
% pin them - re-run then.

p = vehicle_params();

P_CAPS_KW     = [62.7 50 45 40 35 30 28 25 22 20];
REGEN_CAPTURE = 0.50;    % fraction of braking energy through the rear motor
REGEN_RT      = 0.65;    % round-trip efficiency of recovered energy
PACK_USABLE_F = 0.90;    % usable fraction of nominal pack (BMS window TBD)
MARGIN        = 0.90;    % design margin on usable energy (heat/driver/cones)
ENDURANCE_M   = 22000;

here = fileparts(mfilename('fullpath'));
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
fprintf('\nT-STRAT DECISIONS\n');
fprintf('  1. REGEN IS REQUIRED: without it the margin-safe cap is %.0f kW\n', cap_nr);
fprintf('     (+%.1f s/lap); with %.0f%%x%.0f%% regen the cap rises to %.0f kW.\n', ...
        interp1(P_CAPS_KW, t_lap, cap_nr) - t_lap(1), ...
        100*REGEN_CAPTURE, 100*REGEN_RT, cap_rg);
fprintf('  2. Endurance VCU power cap: %.0f kW (margin-safe with regen).\n', cap_rg);
fprintf('  3. BMS SOC window: table assumes %.0f%% usable - attach this table\n', ...
        100*PACK_USABLE_F);
fprintf('     to the window decision; each +5%% usable is ~+2-3 kW of cap.\n');
fprintf('  CONSISTENCY: at %.0f kW the mean pack current ~%.0f A vs %.0f A main fuse.\n', ...
        cap_rg, cap_rg*1e3/p.V_pack_nom * (t_lap(1)/interp1(P_CAPS_KW,t_lap,cap_rg)) * 0.8, ...
        p.I_fuse_main);
fprintf('  CAVEAT: QSS point-mass; regen scenario + usable window are assumptions;\n');
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
