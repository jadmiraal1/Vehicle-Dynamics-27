function out = run_cooling_targets(p, P_cap_kW)
% RUN_COOLING_TARGETS  Powertrain heat, pack current, and what the radiator + fan must do.
%
% Answers the cooling and powertrain questions from the same simulated
% endurance lap the energy strategy uses:
%   1. How much heat do the motor and inverter put into the coolant, on average
%      and in bursts, over an endurance stint?           (-> radiator, fan, pump)
%   2. What current does the accumulator see - mean, RMS, peak - and how much
%      I^2 R heat does it make?                (-> fuse, cables, pack air cooling)
%   3. How much AIRFLOW and how much radiator (UA) does that heat need on a hot
%      day, holding the coolant where the datasheets want it?
%
% Writes plots/cooling_duty_cycle.png and plots/cooling_sizing.png.
%
%   out = run_cooling_targets()            active car; endurance cap from run_energy_strategy
%   out = run_cooling_targets(p)           explicit params (build "what if?" cars with vd_set)
%   out = run_cooling_targets(p, cap_kW)   force the endurance power cap [kW]
%
% Method:
% (1) Duty cycle: the endurance lap at the endurance power cap (the headline)
%     and uncapped (the ceiling), turned into losses against time by
%     lap_duty_cycle. The stint mean is the lap mean; bursts are the worst
%     rolling windows.
% (2) Coolant budget: one series loop; coolant leaves the radiator at T_cold
%     and warms by Q/(m_dot*cp) through each component, which fixes the
%     radiator inlet temperature and the inlet temperature difference (ITD)
%     to ambient.
% (3) Air side: the fan's operating point on its curve at the core pressure
%     drop, plus ram air at mean lap speed times a capture fraction.
% (4) Effectiveness-NTU, crossflow with both streams unmixed:
%     Q = eps*C_min*ITD. Gives the airflow a practical core (eps = EPS_DESIGN)
%     needs and the UA the core must have at that airflow.
% (5) Transient check: a lumped heat capacity turns the worst rolling-window
%     heat above the mean into a coolant temperature rise, to judge whether
%     sizing to the mean is reasonable.
%
% Theory: Incropera & DeWitt ch. 11 (eps-NTU relations); EMRAX 228 technical
% data 4.5 and the PM100 datasheet in references/ for the temperature limits.

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');
here = vd_root();
c    = p.cooling;

EPS_DESIGN = 0.70;   % practical crossflow-core effectiveness [-] CHOICE
                     %   (compact automotive cores run ~0.6-0.85)
WINDOWS_S  = [10 60];   % rolling windows for the burst numbers [s]
T_COLD_SWEEP = 40:5:70; % coolant-out targets to tabulate [degC]

% ------------------------- (0) the endurance power cap --------------------
if nargin < 2 || isempty(P_cap_kW)
    evalc('es = run_energy_strategy(p);');
    vd_warn('reset');                            % its warnings were not shown
    P_cap_kW = es.cap_regen_kW;                  % equals cap_noregen_kW while regen is 0
    if isempty(P_cap_kW), P_cap_kW = es.cap_noregen_kW; end
    if isempty(P_cap_kW)
        warning('run_cooling_targets:noCap', ...
            'No endurance power cap fits the pack with margin (run_energy_strategy); showing the uncapped lap only.');
        P_cap_kW = NaN;
    end
end

% ------------------------- (1) duty cycles ---------------------------------
[s, kappa] = load_track(fullfile(here, 'tracks', 'track_endurance.csv'));
laps = p.scenario.endurance_m / s(end);

cases = {'capped', 'uncapped'};
S = struct();
for k = 1:numel(cases)
    pc = p;
    if strcmp(cases{k}, 'capped')
        if isnan(P_cap_kW), continue; end
        pc.P_max = min(p.P_max, P_cap_kW*1e3);
    end
    [v, t_lap, E] = lap_sim(pc, s, kappa, [], true);
    D = lap_duty_cycle(pc, v, s);
    S.(cases{k}) = summarise(D, E, t_lap, WINDOWS_S);
    S.(cases{k}).v = v;  S.(cases{k}).D = D;
end
if ~isfield(S, 'capped'), S.capped = S.uncapped; end   % no cap found (warned above)
H = S.capped;                                          % the headline case

% ------------------------- (2) coolant temperature budget -----------------
assert(strcmpi(c.coolant, 'water'), 'run_cooling_targets:coolant', ...
    'only water is modelled (cp, rho); config says ''%s''.', c.coolant);
CP_W = 4180;  RHO_W = 998;                              % water near 50 degC
C_cool = c.flow_Lpm/60/1000 * RHO_W * CP_W;             % coolant capacity rate [W/K]

Q_mot = H.Q_motor_mean_W;  Q_inv = H.Q_inv_mean_W;  Q_cool = H.Q_cool_mean_W;
T_cold = c.T_cold_target_C;
switch lower(c.loop_order)
    case 'inverter_first'
        T_inv_in = T_cold;
        T_mot_in = T_cold + Q_inv/C_cool;
    case 'motor_first'
        T_mot_in = T_cold;
        T_inv_in = T_cold + Q_mot/C_cool;
    otherwise
        error('run_cooling_targets:loopOrder', 'cooling.loop_order must be inverter_first or motor_first');
end
dT_loop = Q_cool / C_cool;                              % coolant rise round the loop [K]
T_hot   = T_cold + dT_loop;                             % radiator inlet [degC]

% ------------------------- (3) air side -----------------------------------
T_amb   = c.T_amb_design_C;
RHO_AIR = 101325 / (287.05 * (T_amb + 273.15));         % air density at T_amb, sea level
CP_AIR  = 1006;
M3S_PER_CFM = 0.3048^3 / 60;
V_fan_free = c.fan_m3h(1) / 3600;                               % free-air flow
V_fan      = interp1(c.fan_dp_mmH2O, c.fan_m3h, c.core_dp_mmH2O) / 3600;   % on the curve [m^3/s]
A_core     = c.rad_core_w_m * c.rad_core_h_m;                   % NaN until measured
v_lap_mean = s(end) / H.t_lap;
V_ram      = c.ram_capture * v_lap_mean * A_core;               % NaN until measured
V_total    = V_fan + V_ram;
C_air = @(V) RHO_AIR * V * CP_AIR;                              % air capacity rate [W/K]

% ------------------------- (4) eps-NTU sizing -----------------------------
% Required heat: FOS x stint mean (FOS = 1 also tabulated).
Q_req = c.FOS * Q_cool;

% Sweep the coolant-out target. Each row depends only on ITD, so the table
% also answers "what if the day is 5 degC hotter" (read the row 5 degC lower).
nT = numel(T_COLD_SWEEP);
tab = struct('T_cold', T_COLD_SWEEP(:), 'T_hot', nan(nT,1), 'ITD', nan(nT,1), ...
             'V_req_m3s', nan(nT,1), 'V_req_fos1_m3s', nan(nT,1), ...
             'eps_fan', nan(nT,1), 'eps_total', nan(nT,1), ...
             'UA_req_W_per_K', nan(nT,1), 'NTU_req', nan(nT,1));
for i = 1:nT
    Th  = T_COLD_SWEEP(i) + dT_loop;
    ITD = Th - T_amb;
    tab.T_hot(i) = Th;  tab.ITD(i) = ITD;
    if ITD <= 0, continue; end
    % airflow for a core of EPS_DESIGN to reject Q_req (air assumed C_min; checked below)
    Vr  = Q_req  / (EPS_DESIGN * ITD) / (RHO_AIR * CP_AIR);
    Vr1 = Q_cool / (EPS_DESIGN * ITD) / (RHO_AIR * CP_AIR);
    tab.V_req_m3s(i) = Vr;  tab.V_req_fos1_m3s(i) = Vr1;
    % effectiveness the actual airflow would need (>= 1 is impossible)
    tab.eps_fan(i)   = Q_req / (cmin(C_air(V_fan),   C_cool) * ITD);
    tab.eps_total(i) = Q_req / (cmin(C_air(V_total), C_cool) * ITD);   % NaN until the core is measured
    % UA the core needs at the required airflow
    [NTU, UA] = ntu_from_eps(EPS_DESIGN, C_air(Vr), C_cool);
    tab.NTU_req(i) = NTU;  tab.UA_req_W_per_K(i) = UA;
end
i_des = find(T_COLD_SWEEP == T_cold, 1);
if isempty(i_des)
    error('run_cooling_targets:sweep', 'T_cold_target_C = %g is not on the sweep grid %s', ...
          T_cold, mat2str(T_COLD_SWEEP));
end
ITD_des = tab.ITD(i_des);
if cmin(C_air(tab.V_req_m3s(i_des)), C_cool) == C_cool
    warning('run_cooling_targets:cminCoolant', ...
        'At the design point the coolant, not the air, limits heat transfer: raise the coolant flow or treat the UA figure with care.');
end

% ------------------------- (5) transient check ----------------------------
% Heat above the mean, held for the window, warms the lumped loop by dT.
% The radiator is assumed to be rejecting the mean throughout.
dT_burst = (H.Q_cool_win_W - Q_cool) .* WINDOWS_S / c.thermal_mass_J_per_K;

% ============================== report ===================================
V_des = tab.V_req_m3s(i_des);
cap_lbl = tern(isnan(P_cap_kW), 'no cap', sprintf('%.0f kW cap', P_cap_kW));
fprintf('\nCooling - %s  (endurance: %.0f m lap, %.1f laps; %.0f degC day; coolant out %.0f degC)\n', ...
        p.car, s(end), laps, T_amb, T_cold);
fprintf('  Motor %s; inverter %s\n', p.motor_id, p.inverter_id);
fprintf('  Radiator %s; fan %s\n', c.rad_id, c.fan_id);
fprintf('  Coolant %s at %.0f L/min, inverter %s; %s\n', c.coolant, c.flow_Lpm, ...
        tern(strcmpi(c.loop_order, 'inverter_first'), 'first', 'after the motor'), ...
        tern(p.scenario.regen_capture > 0, 'regen on.', 'no regen (braking heat goes to the discs).'));

fprintf('\n');
row2('', cap_lbl, 'uncapped');
row2('Lap time', sprintf('%.1f s', S.capped.t_lap), sprintf('%.1f s', S.uncapped.t_lap));
row2('Pack power, mean / peak', ...
     sprintf('%.1f / %.1f kW', S.capped.P_elec_mean_W/1e3, S.capped.P_elec_peak_W/1e3), ...
     sprintf('%.1f / %.1f kW', S.uncapped.P_elec_mean_W/1e3, S.uncapped.P_elec_peak_W/1e3));
row2('Pack current, mean / RMS / peak', [S.capped.I_line ' A'], [S.uncapped.I_line ' A']);
row2('Heat into coolant, mean', sprintf('%.0f W', S.capped.Q_cool_mean_W), ...
     sprintf('%.0f W', S.uncapped.Q_cool_mean_W));
row2('  from motor / inverter', ...
     sprintf('%.0f / %.0f W', S.capped.Q_motor_mean_W, S.capped.Q_inv_mean_W), ...
     sprintf('%.0f / %.0f W', S.uncapped.Q_motor_mean_W, S.uncapped.Q_inv_mean_W));
row2(sprintf('Heat into coolant, worst %d s / %d s', WINDOWS_S(1), WINDOWS_S(2)), ...
     sprintf('%.0f / %.0f W', S.capped.Q_cool_win1_W, S.capped.Q_cool_win2_W), ...
     sprintf('%.0f / %.0f W', S.uncapped.Q_cool_win1_W, S.uncapped.Q_cool_win2_W));
row2('Pack resistive heat (air-cooled)', sprintf('%.0f W', S.capped.Q_pack_mean_W), ...
     sprintf('%.0f W', S.uncapped.Q_pack_mean_W));
row2('Friction-brake heat, mean', sprintf('%.0f W', S.capped.Q_brake_mean_W), ...
     sprintf('%.0f W', S.uncapped.Q_brake_mean_W));

fprintf('\n');
vd_row(sprintf('Pack RMS current, %s', cap_lbl), sprintf('%.0f A', S.capped.I_rms_A), ...
       sprintf('main fuse %.0f A', p.I_fuse_main), tern(S.capped.I_rms_A <= p.I_fuse_main, 'OK', 'OVER'));
vd_row('Pack RMS current, uncapped', sprintf('%.0f A', S.uncapped.I_rms_A), ...
       sprintf('main fuse %.0f A', p.I_fuse_main), tern(S.uncapped.I_rms_A <= p.I_fuse_main, 'OK', 'OVER'));
vd_row('Pack peak current, uncapped', sprintf('%.0f A', S.uncapped.I_peak_A), ...
       sprintf('cell limit %.0f A', p.I_pack_max), tern(S.uncapped.I_peak_A <= p.I_pack_max, 'OK', 'OVER'));
vd_row('Coolant temperature rise around the loop', sprintf('%.1f degC', dT_loop), ...
       sprintf('radiator in %.1f', T_hot));
vd_row('Inverter coolant inlet', sprintf('%.1f degC', T_inv_in), ...
       sprintf('full current <%.0f', c.T_inv_full_C), tern(T_inv_in <= c.T_inv_full_C, 'OK', 'OVER'));
vd_row('Motor coolant inlet', sprintf('%.1f degC', T_mot_in), ...
       sprintf('rating basis %.0f', c.T_motor_rated_C), ...
       tern(T_mot_in <= c.T_motor_rated_C + 0.05, 'OK', 'ABOVE (ratings drop a little)'));
vd_row('Highest coolant-out keeping the inverter OK', ...
       sprintf('%.0f degC', c.T_inv_full_C - (T_inv_in - T_cold)));
vd_row(sprintf('Fan airflow at %.1f mm H2O core drop', c.core_dp_mmH2O), ...
       sprintf('%.0f CFM', V_fan/M3S_PER_CFM), sprintf('%.0f CFM free air', V_fan_free/M3S_PER_CFM));
if isnan(A_core)
    vd_row('Ram air', 'not included', 'radiator core size not measured');
else
    vd_row('Ram air at the mean lap speed', sprintf('%.0f CFM', V_ram/M3S_PER_CFM), ...
           sprintf('capture %.2f', c.ram_capture));
end
if isfinite(V_des)
    if tab.eps_fan(i_des) >= 1
        fan_st = 'FAN TOO SMALL';
    else
        fan_st = sprintf('FAN %.0f %% SHORT', 100*(1 - V_fan/V_des));
    end
    if V_fan >= V_des, fan_st = 'OK'; end
    vd_row(sprintf('Airflow needed (heat x %.1f safety factor)', c.FOS), ...
           sprintf('%.0f CFM', V_des/M3S_PER_CFM), ...
           sprintf('%.0f CFM at x1.0', tab.V_req_fos1_m3s(i_des)/M3S_PER_CFM), fan_st);
    if tab.eps_fan(i_des) >= 1
        fprintf(['    The fan alone cannot move enough air at any radiator size: air from the\n' ...
                 '    car''s motion or a bigger fan has to make up the rest.\n']);
    end
    if isfinite(c.rad_UA_W_per_K)
        vd_row('Radiator UA needed at that airflow', sprintf('%.0f W/K', tab.UA_req_W_per_K(i_des)), ...
               sprintf('measured %.0f', c.rad_UA_W_per_K), ...
               tern(c.rad_UA_W_per_K >= tab.UA_req_W_per_K(i_des), 'OK', 'TOO SMALL'));
    else
        vd_row('Radiator UA needed at that airflow', sprintf('%.0f W/K', tab.UA_req_W_per_K(i_des)), ...
               'radiator not measured');
    end
end
vd_row(sprintf('Coolant rise in a worst %d s / %d s burst', WINDOWS_S(1), WINDOWS_S(2)), ...
       sprintf('%.1f / %.1f degC', dT_burst(1), dT_burst(end)));

fprintf('\n  Coolant-out temperature trade (core effectiveness %.2f, heat x %.1f):\n', EPS_DESIGN, c.FOS);
fprintf('    %-12s %-12s %-16s %-16s %s\n', 'coolant out', 'radiator in', 'radiator in', 'airflow needed', 'radiator UA');
fprintf('    %-12s %-12s %-16s %-16s %s\n', '[degC]', '[degC]', 'minus air [degC]', '[CFM]', '[W/K]');
for i = 1:nT
    fprintf('    %-12.0f %-12.1f %-16.1f %-16s %s%s\n', tab.T_cold(i), tab.T_hot(i), tab.ITD(i), ...
            cfm(tab.V_req_m3s(i)/M3S_PER_CFM), ua(tab.UA_req_W_per_K(i)), ...
            tern(i == i_des, '   this design', ''));
end

fprintf(['Assumes: lap-average heat (no thermal state) with a lumped burst check; heat\n' ...
         '         x %.1f safety factor. Provisional: inverter efficiency %.2f, coolant flow,\n' ...
         '         fan curve and core pressure drop, ram air, thermal mass. The radiator''s\n' ...
         '         UA is not measured, so this is a requirement, not a verdict on it. Pack\n' ...
         '         current at nominal voltage (voltage sag adds about %.0f %%).\n'], ...
        c.FOS, p.eta_inv, 100*S.uncapped.I_peak_A*p.R_pack/p.V_pack_nom);

% ------------------------------- outputs ---------------------------------
out = struct('cap_kW', P_cap_kW, 'laps', laps, 'lap_m', s(end), ...
             'capped', strip(S.capped), 'uncapped', strip(S.uncapped), ...
             'C_cool_W_per_K', C_cool, 'dT_loop_K', dT_loop, 'T_hot_C', T_hot, ...
             'T_inv_in_C', T_inv_in, 'T_mot_in_C', T_mot_in, 'T_amb_C', T_amb, ...
             'ITD_design_K', ITD_des, 'Q_req_W', Q_req, ...
             'V_fan_m3s', V_fan, 'V_fan_free_m3s', V_fan_free, 'V_ram_m3s', V_ram, ...
             'A_core_m2', A_core, 'v_lap_mean', v_lap_mean, 'eps_design', EPS_DESIGN, ...
             'sweep', tab, 'V_req_design_m3s', tab.V_req_m3s(i_des), ...
             'UA_req_design_W_per_K', tab.UA_req_W_per_K(i_des), ...
             'NTU_req_design', tab.NTU_req(i_des), 'eps_fan_design', tab.eps_fan(i_des), ...
             'windows_s', WINDOWS_S, 'dT_burst_K', dT_burst);

if vd_plots()
try
    outdir = fullfile(here, 'plots');
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    plot_duty(S, P_cap_kW, outdir);
    plot_sizing(tab, T_cold, V_fan, V_total, M3S_PER_CFM, T_amb, c.FOS, outdir);
    fprintf('Saved plots/cooling_duty_cycle.png and plots/cooling_sizing.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

% =========================================================================
function R = summarise(D, E, t_lap, windows)
% Time-weighted statistics of one lap's duty cycle (segments differ in
% duration, so means are sum(x*dt)/sum(dt)).
w  = D.dt / D.t_lap;
tm = @(x) sum(x .* w);
R.t_lap          = t_lap;
R.Q_cool_mean_W  = tm(D.Q_coolant_W);
R.Q_motor_mean_W = tm(D.P_motor_loss_W);
R.Q_spin_off_W   = tm(D.P_motor_loss_W .* (D.F_thrust <= 0));   % heat while off throttle
R.Q_inv_mean_W   = tm(D.P_inv_loss_W);
R.Q_chain_mean_W = tm(D.P_chain_loss_W);
R.Q_pack_mean_W  = tm(D.P_pack_loss_W);
R.Q_brake_mean_W = tm(D.P_brake_W);
R.Q_cool_peak_W  = max(D.Q_coolant_W);
R.Q_cool_win_W   = arrayfun(@(W) worst_window(D.Q_coolant_W, D.dt, W), windows);
R.Q_cool_win1_W  = R.Q_cool_win_W(1);
R.Q_cool_win2_W  = R.Q_cool_win_W(end);
R.P_elec_mean_W  = tm(D.P_elec_W);
R.P_elec_peak_W  = max(D.P_elec_W);
R.I_mean_A       = tm(D.I_pack_A);
R.I_rms_A        = sqrt(tm(D.I_pack_A.^2));
R.I_peak_A       = max(D.I_pack_A);
R.I_line         = sprintf('%.0f / %.0f / %.0f', R.I_mean_A, R.I_rms_A, R.I_peak_A);
R.E_elec_Wh      = D.E_elec_Wh;
R.E_lapenergy_Wh = E.drive_acc_Wh;
end

function m = worst_window(x, dt, W)
% Largest time-average of x over any W-second window, the lap treated as
% periodic. Windows start at segment boundaries (~1 m spacing).
T = sum(dt);
if W >= T, m = sum(x .* dt) / T; return; end
x2 = [x(:); x(:)];  d2 = [dt(:); dt(:)];
tn = [0; cumsum(d2)];  En = [0; cumsum(x2 .* d2)];
n  = numel(dt);
m  = -Inf;
for i = 1:n
    E1 = interp1(tn, En, tn(i) + W);
    m  = max(m, (E1 - En(i)) / W);
end
end

function [NTU, UA] = ntu_from_eps(eps, C_a, C_c)
% Invert the crossflow (both unmixed) eps-NTU relation, Incropera Table 11.3:
%   eps = 1 - exp( (1/Cr) NTU^0.22 ( exp(-Cr NTU^0.78) - 1 ) ),  Cr = C_min/C_max
% Monotone in NTU, so a bisection is enough and cannot pick a wrong branch.
C_min = min(C_a, C_c);  Cr = C_min / max(C_a, C_c);
f = @(N) 1 - exp((1/Cr) * N^0.22 * (exp(-Cr * N^0.78) - 1));
lo = 1e-6;  hi = 50;
if f(hi) < eps, NTU = Inf;  UA = Inf;  return; end
for it = 1:80
    mid = 0.5*(lo + hi);
    if f(mid) < eps, lo = mid; else, hi = mid; end
end
NTU = 0.5*(lo + hi);
UA  = NTU * C_min;
end

function row2(label, a, b)
% One line of the capped / uncapped comparison.
line = sprintf('  %-44s %-18s %s', label, a, b);
fprintf('%s\n', deblank(line));
end

function R = strip(R)
% Numbers only for the golden file: drop the trace and the text line.
R = rmfield(R, {'v', 'D', 'I_line'});
end

function m = cmin(C_a, C_c)
% NaN-preserving min: an unmeasured airflow must stay unknown.
if ~isfinite(C_a) || ~isfinite(C_c), m = NaN; else, m = min(C_a, C_c); end
end

function s = cfm(x)
if ~isfinite(x), s = 'n/a'; else, s = sprintf('%.0f', x); end
end

function s = ua(x)
if ~isfinite(x), s = 'inf'; else, s = sprintf('%.0f', x); end
end

function v = tern(c, a, b)
if c, v = a; else, v = b; end
end

% =========================================================================
function plot_duty(S, cap_kW, outdir)
% One endurance lap, both cases: speed, coolant heat by source, pack current.
BLUE = [0.122 0.310 0.847];  ORANGE = [0.761 0.255 0.047];  GREY = [0.55 0.55 0.55];
f = figure('Visible', 'off', 'Position', [60 60 1100 820], 'Color', 'w');
names = {'capped', 'uncapped'};  cols = {BLUE, ORANGE};
labs  = {sprintf('%.0f kW cap', cap_kW), 'uncapped'};
ax1 = subplot(3,1,1); hold(ax1, 'on'); grid(ax1, 'on');
ax2 = subplot(3,1,2); hold(ax2, 'on'); grid(ax2, 'on');
ax3 = subplot(3,1,3); hold(ax3, 'on'); grid(ax3, 'on');
for k = 1:2
    D = S.(names{k}).D;
    plot(ax1, D.t, D.vm, '-', 'Color', cols{k}, 'LineWidth', 1.2, 'DisplayName', labs{k});
    plot(ax2, D.t, D.Q_coolant_W/1e3, '-', 'Color', cols{k}, 'LineWidth', 1.2, ...
         'DisplayName', [labs{k} ' motor+inverter']);
    plot(ax3, D.t, D.I_pack_A, '-', 'Color', cols{k}, 'LineWidth', 1.2, 'DisplayName', labs{k});
end
D = S.capped.D;
plot(ax2, D.t, D.P_motor_loss_W/1e3, ':', 'Color', GREY, 'LineWidth', 1.0, ...
     'DisplayName', 'capped, motor only (spin loss floor)');
yline(ax2, S.capped.Q_cool_mean_W/1e3, '--', 'Color', BLUE, 'LineWidth', 1.0, ...
      'DisplayName', 'capped mean');
ylabel(ax1, 'speed [m/s]');  title(ax1, 'Endurance lap duty cycle (QSS)', 'FontWeight', 'bold');
ylabel(ax2, 'heat to coolant [kW]');
ylabel(ax3, 'pack current [A]');  xlabel(ax3, 'lap time [s]');
legend(ax1, 'Location', 'best');  legend(ax2, 'Location', 'best');  legend(ax3, 'Location', 'best');
saveas(f, fullfile(outdir, 'cooling_duty_cycle.png'));  close(f);
end

function plot_sizing(tab, T_cold, V_fan, V_total, m3s_per_cfm, T_amb, FOS, outdir)
% Airflow the core needs vs coolant target, against what the fan (and ram air)
% deliver. Where the requirement sits above the supply line, the coolant runs
% hotter than the target - the gap is the design problem in one picture.
BLUE = [0.122 0.310 0.847];  ORANGE = [0.761 0.255 0.047];  GREY = [0.45 0.45 0.45];
f = figure('Visible', 'off', 'Position', [80 80 820 520], 'Color', 'w');
ax = axes(f); hold(ax, 'on'); grid(ax, 'on');
plot(ax, tab.T_cold, tab.V_req_m3s/m3s_per_cfm, 'o-', 'Color', BLUE, 'LineWidth', 1.8, ...
     'DisplayName', sprintf('required, FOS %.1f', FOS));
plot(ax, tab.T_cold, tab.V_req_fos1_m3s/m3s_per_cfm, 's--', 'Color', BLUE, 'LineWidth', 1.2, ...
     'DisplayName', 'required, FOS 1');
yline(ax, V_fan/m3s_per_cfm, '-', 'Color', ORANGE, 'LineWidth', 1.5, 'DisplayName', 'fan on its curve');
if isfinite(V_total)
    yline(ax, V_total/m3s_per_cfm, '--', 'Color', ORANGE, 'LineWidth', 1.5, 'DisplayName', 'fan + ram air (PROV.)');
end
xline(ax, T_cold, ':', 'Color', GREY, 'LineWidth', 1.2, 'DisplayName', 'coolant target');
xlabel(ax, 'coolant leaving radiator [degC]');  ylabel(ax, 'airflow through core [CFM]');
title(ax, sprintf('Airflow to hold the coolant target on a %.0f degC day', T_amb), 'FontWeight', 'bold');
legend(ax, 'Location', 'northeast');
ylim(ax, [0, max(1.2*max(tab.V_req_m3s(isfinite(tab.V_req_m3s)))/m3s_per_cfm, 1.5*V_fan/m3s_per_cfm)]);
saveas(f, fullfile(outdir, 'cooling_sizing.png'));  close(f);
end
