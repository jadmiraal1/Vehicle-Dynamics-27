function out = run_energy_strategy(p)
% RUN_ENERGY_STRATEGY  Endurance power-cap sweep: lap time and event energy
% against the usable pack, and the highest cap that finishes with margin.
%
%   out = run_energy_strategy()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_energy_strategy(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

% Caps to test [kW], uncapped first. Keep the low end low enough that at
% least one cap is margin-safe (the table says so if none is).
P_CAPS_KW     = [p.P_max/1e3, 50 45 40 35 30 28 25 22 20 18 16 14 12];
REGEN_CAPTURE = p.scenario.regen_capture;
REGEN_RT      = p.scenario.regen_rt;
PACK_USABLE_F = p.scenario.pack_usable_f;
MARGIN        = p.scenario.margin;
ENDURANCE_M   = p.scenario.endurance_m;   % rules distance

here = vd_root();
[s, kappa] = load_track(fullfile(here, 'tracks', 'track_endurance.csv'));
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

cap_rg = max(P_CAPS_KW(E_rg <= MARGIN*usable));   % [] if none fits
cap_nr = max(P_CAPS_KW(E_nr <= MARGIN*usable));
has_regen = REGEN_CAPTURE > 0;

cap_q = cap_rg;  if isempty(cap_q), cap_q = cap_nr; end
I_mean = NaN;  I_rms = NaN;
if ~isempty(cap_q)
    % Pack current over one lap at that cap, from the duty cycle.
    p2 = p;  p2.P_max = min(p.P_max, cap_q*1e3);
    v_q = lap_sim(p2, s, kappa, [], true);
    D = lap_duty_cycle(p2, v_q, s);
    w = D.dt / D.t_lap;
    I_mean = sum(D.I_pack_A .* w);
    I_rms  = sqrt(sum(D.I_pack_A.^2 .* w));
end

fprintf('\nEndurance energy - %s  (%.0f m lap, %.1f laps = %.0f km)\n', ...
        p.car, s(end), laps, ENDURANCE_M/1000);
vd_row('Usable pack energy', sprintf('%.2f kWh', usable), ...
       sprintf('%.0f %% of %.2f kWh', 100*PACK_USABLE_F, p.E_pack_Wh/1000));
vd_row(sprintf('Energy budget with %.0f %% margin', 100*(1-MARGIN)), sprintf('%.2f kWh', MARGIN*usable));
fprintf('\n');
if has_regen
    fprintf('    %-8s %-18s %-18s %-9s %s\n', 'cap', 'energy, no regen', 'energy, regen', 'lap', 'vs uncapped');
    fprintf('    %-8s %-18s %-18s %-9s %s\n', '[kW]', '[kWh]', '[kWh]', '[s]', '[s/lap]');
    for i = 1:n
        fprintf('    %-8.1f %-6.2f %-11s %-6.2f %-11s %-9.1f %+.1f\n', P_CAPS_KW(i), ...
                E_nr(i), feas(E_nr(i), usable, MARGIN*usable), ...
                E_rg(i), feas(E_rg(i), usable, MARGIN*usable), t_lap(i), t_lap(i)-t_lap(1));
    end
else
    fprintf('    %-8s %-18s %-9s %s\n', 'cap', 'endurance energy', 'lap', 'vs uncapped');
    fprintf('    %-8s %-18s %-9s %s\n', '[kW]', '[kWh]', '[s]', '[s/lap]');
    for i = 1:n
        fprintf('    %-8.1f %-6.2f %-11s %-9.1f %+.1f\n', P_CAPS_KW(i), ...
                E_nr(i), feas(E_nr(i), usable, MARGIN*usable), t_lap(i), t_lap(i)-t_lap(1));
    end
end
fprintf(['  OK = within the energy budget; TIGHT = fits the usable pack but uses the\n' ...
         '  margin; NO = needs more than the usable pack.\n\n']);

if isempty(cap_nr)
    vd_row('Highest cap that fits with margin, no regen', 'none', ...
           sprintf('%.0f kW needs %.2f kWh', P_CAPS_KW(end), E_nr(end)));
    fprintf('  Add lower caps to P_CAPS_KW in this file; the curve steepens as the cap falls.\n');
else
    vd_row('Highest cap that fits with margin, no regen', sprintf('%.0f kW', cap_nr), ...
           sprintf('%+.1f s/lap', interp1(P_CAPS_KW, t_lap, cap_nr) - t_lap(1)));
end
if has_regen
    if isempty(cap_rg)
        vd_row('Highest cap that fits with margin, regen', 'none');
    else
        vd_row('Highest cap that fits with margin, regen', sprintf('%.0f kW', cap_rg), ...
               sprintf('regen %.0f %% x %.0f %%', 100*REGEN_CAPTURE, 100*REGEN_RT));
    end
end
if ~isempty(cap_q)
    vd_row(sprintf('Pack current at %.0f kW, mean / RMS', cap_q), ...
           sprintf('%.0f / %.0f A', I_mean, I_rms), sprintf('main fuse %.0f A', p.I_fuse_main), ...
           ternary(I_rms < p.I_fuse_main, 'OK', 'OVER'));
end
fprintf(['Assumes: %.0f %% of nominal pack energy is usable (set by the BMS window) and a\n' ...
         '         %.0f %% energy margin; pack current at nominal voltage; %s.\n'], ...
        100*PACK_USABLE_F, 100*(1-MARGIN), ternary(has_regen, 'regen as configured', 'no regen'));

out = struct('P_caps_kW', P_CAPS_KW, 'E_noregen_kWh', E_nr, 'E_regen_kWh', E_rg, ...
             't_lap', t_lap, 'cap_regen_kW', cap_rg, 'cap_noregen_kW', cap_nr, ...
             'usable_kWh', usable, 'I_mean_A', I_mean, 'I_rms_A', I_rms);

if vd_plots()
try
    make_plot(P_CAPS_KW, E_nr, E_rg, t_lap, usable, MARGIN, ...
              REGEN_CAPTURE, REGEN_RT, here);
    fprintf('Saved plots/energy_strategy.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

function s = feas(E, usable, target)
% OK: within the energy budget; TIGHT: fits the usable pack but uses the
% margin; NO: needs more than the usable pack. Same thresholds as the cap search.
if     E <= target,  s = 'OK';
elseif E <= usable,  s = 'TIGHT';
else,                s = 'NO';
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
    % With no regen the two columns are identical; draw one line.
    plot(caps, E_nr, 'o-', 'LineWidth', 1.8, 'DisplayName', 'endurance energy');
end
yline(usable, '-', sprintf('usable pack %.2f kWh', usable), 'LineWidth', 1.2);
yline(margin*usable, ':', sprintf('with %.0f%% margin', 100*(1-margin)), 'LineWidth', 1.2);
ylabel('endurance energy [kWh]');
yyaxis right;
plot(caps, t_lap, '^-', 'LineWidth', 1.2, 'DisplayName', 'lap time');
ylabel('lap time [s]');
xlabel('endurance power cap [kW]');
legend('Location', 'northwest'); grid on;
title('Endurance power cap vs energy and lap time', 'FontWeight', 'bold');
outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'energy_strategy.png'));
close(f);
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
