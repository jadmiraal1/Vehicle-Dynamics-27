function out = run_energy_strategy(p)
% RUN_ENERGY_STRATEGY  Endurance power-cap sweep: lap time vs energy, regen scenarios.
%
%   out = run_energy_strategy()      the active car, from vd_car / cars/config_<CAR>.m
%   out = run_energy_strategy(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = run_energy_strategy(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)

P_CAPS_KW     = [62.7 50 45 40 35 30 28 25 22 20 18 16 14 12];
% The floor was 20 kW, which sits ABOVE the margin-safe cap now that regen is
% off. The cap search then returned empty, the decision block printed blanks,
% and the fuse-consistency line crashed on a 1x0 divide. Check E_nr(end) is
% under MARGIN*usable before raising this floor again.
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
    fprintf('%7.1f %7.2f kWh %-5s | %7.2f kWh %-5s | %7.1f %+8.1f\n', ...
        P_CAPS_KW(i), E_nr(i), feas(E_nr(i), usable, MARGIN*usable), ...
        E_rg(i), feas(E_rg(i), usable, MARGIN*usable), t_lap(i), t_lap(i)-t_lap(1));
end
fprintf('  OK = inside the margin target | TIGHT = fits usable but eats the\n');
fprintf('  margin | DNF = does not fit the usable pack at all.\n');

cap_rg = max(P_CAPS_KW(E_rg <= MARGIN*usable), [], 'omitnan');
cap_nr = max(P_CAPS_KW(E_nr <= MARGIN*usable), [], 'omitnan');
has_regen = REGEN_CAPTURE > 0;

fprintf('\nT-STRAT decisions\n');
if isempty(cap_nr)
    fprintf('  1. NO SWEPT CAP IS MARGIN-SAFE without regen. The lowest cap tried\n');
    fprintf('     (%.0f kW) still needs %.2f kWh against a %.2f kWh target.\n', ...
            P_CAPS_KW(end), E_nr(end), MARGIN*usable);
    fprintf('     Extend P_CAPS_KW downward - do NOT extrapolate the curve by hand,\n');
    fprintf('     it steepens as the cap falls and a linear read is optimistic.\n');
else
    fprintf('  1. Margin-safe cap without regen: %.0f kW (+%.1f s/lap vs uncapped).\n', ...
            cap_nr, interp1(P_CAPS_KW, t_lap, cap_nr) - t_lap(1));
end

if has_regen
    if isempty(cap_rg)
        fprintf('  2. No margin-safe cap with %.0f%%x%.0f%% regen either.\n', ...
                100*REGEN_CAPTURE, 100*REGEN_RT);
    else
        fprintf('  2. Endurance VCU power cap: %.0f kW (margin-safe with %.0f%%x%.0f%% regen).\n', ...
                cap_rg, 100*REGEN_CAPTURE, 100*REGEN_RT);
    end
else
    fprintf('  2. Regen capture is 0 (no regen for TR27), so the two energy columns\n');
    fprintf('     above are identical by construction and the cap is column one.\n');
end

fprintf('  3. BMS SOC window: table assumes %.0f%% usable - attach this table\n', ...
        100*PACK_USABLE_F);
fprintf('     to the window decision; each +5%% usable is ~+2-3 kW of cap.\n');

cap_q = cap_rg;  if isempty(cap_q), cap_q = cap_nr; end
if ~isempty(cap_q)
    fprintf('  Consistency: at %.0f kW the mean pack current ~%.0f A vs %.0f A main fuse.\n', ...
            cap_q, cap_q*1e3/p.V_pack_nom * (t_lap(1)/interp1(P_CAPS_KW,t_lap,cap_q)) * 0.8, ...
            p.I_fuse_main);
end
fprintf('  Caveat: QSS point-mass; the usable window is an assumption;\n');
fprintf('          re-run after the pack load test.\n');

out = struct('P_caps_kW', P_CAPS_KW, 'E_noregen_kWh', E_nr, 'E_regen_kWh', E_rg, ...
             't_lap', t_lap, 'cap_regen_kW', cap_rg, 'cap_noregen_kW', cap_nr, ...
             'usable_kWh', usable);

if vd_plots()
try
    make_plot(P_CAPS_KW, E_nr, E_rg, t_lap, usable, MARGIN, ...
              REGEN_CAPTURE, REGEN_RT, here);
    fprintf('Plot written: plots/energy_strategy.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end
end

function s = feas(E, usable, target)
% Three states, not two. The old two-state version flagged OK against the raw
% usable pack while the cap search below tested against MARGIN*usable, so the
% table could read OK on a row the decision logic called infeasible.
if     E <= target,  s = 'OK';
elseif E <= usable,  s = 'TIGHT';
else,                s = 'DNF';
end
end

function make_plot(caps, E_nr, E_rg, t_lap, usable, margin, rc, rt, here)
f = figure('Visible', 'off', 'Position', [80 80 980 520], 'Color', 'w');
yyaxis left; hold on;
if rc > 0
    plot(caps, E_nr, 'o-', 'LineWidth', 1.8, 'DisplayName', 'no regen');
    plot(caps, E_rg, 's-', 'LineWidth', 1.8, ...
         'DisplayName', sprintf('regen %.0f%% x %.0f%%', 100*rc, 100*rt));
else
    % regen capture 0: E_rg is identical to E_nr, so a second line would just
    % draw over the first and imply a comparison that is not being made.
    plot(caps, E_nr, 'o-', 'LineWidth', 1.8, 'DisplayName', 'endurance energy');
end
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
