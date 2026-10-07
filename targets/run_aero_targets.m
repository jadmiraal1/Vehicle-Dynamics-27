function out = run_aero_targets(p)
% RUN_AERO_TARGETS  Downforce, drag and L/D targets in FSAE points.
% Sweeps added downforce along package L/D lines (drag and device mass grow
% with ClA), runs accel, skidpad, autocross and a pack-limited endurance for
% each, and scores them with fsae_points against the real 2026 results.
%
%   out = run_aero_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_aero_targets(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');
here = vd_root();

% Sweep and package assumptions
DCLA        = 0:0.75:3.75;   % added downforce [m^2]
LD_PKG      = [3 4 5];       % whole-package lift/drag lines [-]
DM_PER_CLA  = 4.0;           % aero package mass penalty [kg per m^2 ClA] PROVISIONAL
CLA_TARGET  = p.scenario.ClA_target;   % target downforce [m^2]
CLA_BAND    = [3.5 4.5];     % target range [m^2]
LD_TARGET   = 4.0;           % package L/D assumed at the target [-] CHOICE

% Scoring assumptions
HAIRCUT     = 0.08;   % sprint-event time inflation vs QSS, calibrated to 2026 results
EF_MAX      = 0.60;   % best real 2026 efficiency factor PROVISIONAL
E_MIN_KWH   = 2.696;  % lowest real 22-lap finisher energy, 2026 (Wisconsin) [kWh]
LAPS        = p.scenario.benchmark_laps;  % lap count the 2026 results were scored on
                      % (energy feasibility uses the rules distance instead)

% Endurance strategy assumptions (shared with run_energy_strategy via p.scenario)
REGEN_CAPTURE = p.scenario.regen_capture;  REGEN_RT      = p.scenario.regen_rt;
PACK_USABLE_F = p.scenario.pack_usable_f;  MARGIN        = p.scenario.margin;

bm = read_benchmarks(fullfile(here, 'organization', 'comp_benchmarks_2026.csv'));

[s_en, k_en] = load_track(fullfile(here, 'tracks', 'track_endurance.csv'));
[s_ax, k_ax] = load_track(fullfile(here, 'tracks', 'track_autocross.csv'));
ctx = struct('p', p, 's_en', s_en, 'k_en', k_en, 's_ax', s_ax, 'k_ax', k_ax, ...
             'bm', bm, 'haircut', HAIRCUT, 'ef_max', EF_MAX, 'e_min', E_MIN_KWH, ...
             'laps', LAPS, 'laps_feas', p.scenario.endurance_m / s_en(end), ...
             'rc', REGEN_CAPTURE, 'rt', REGEN_RT, ...
             'usable', PACK_USABLE_F * p.E_pack_Wh / 1000, 'margin', MARGIN);

% Baseline and sweep
[pts0, d0] = score_point(ctx, 0, inf, 0);
P = nan(numel(LD_PKG), numel(DCLA));
for i = 1:numel(LD_PKG)
    for j = 1:numel(DCLA)
        P(i,j) = score_point(ctx, DCLA(j), LD_PKG(i), DM_PER_CLA);
    end
end

% Exchange rates at the target (forward differences)
dc_t = CLA_TARGET - p.ClA;
[pts_t, d_t] = score_point(ctx, dc_t, LD_TARGET, DM_PER_CLA);
xr_cla = (score_point_abs(ctx, dc_t+0.4, d_t.CdA,     DM_PER_CLA*dc_t) - pts_t) / 0.4;
xr_cda = (score_point_abs(ctx, dc_t,     d_t.CdA+0.2, DM_PER_CLA*dc_t) - pts_t) / 0.2;
xr_m   = (score_point_abs(ctx, dc_t,     d_t.CdA,     DM_PER_CLA*dc_t+10) - pts_t) / 10;
ld_floor_drag = abs(xr_cda) / xr_cla;                       % drag-only
ld_floor_full = abs(xr_cda) / (xr_cla + xr_m*DM_PER_CLA);   % incl. device mass

fprintf('\nAero - %s  (competition points against the real 2026 results)\n', p.car);
vd_row('Points, this car', sprintf('%.0f', pts0), sprintf('ClA %.2f, CdA %.2f', p.ClA, p.CdA));
fprintf('    autocross %.1f s, endurance %.0f s using %.2f kWh at a %.1f kW power cap\n', ...
        d0.t_ax, d0.t_tot, d0.E22, d0.cap/1e3);

fprintf('\n  Points by added downforce (columns) and whole-package lift/drag (rows)\n');
fprintf('    %-12s', 'added ClA'); fprintf(' %7.2f', DCLA); fprintf('   [m^2]\n');
for i = 1:numel(LD_PKG)
    fprintf('    %-12s', sprintf('L/D %.0f', LD_PKG(i))); fprintf(' %7.0f', P(i,:)); fprintf('\n');
end
if all(diff(P(end,:)) > 0)
    fprintf('  Points still rise at the right-hand edge on the best L/D line.\n');
end
fprintf('\n');
vd_row('Downforce target', sprintf('ClA %.1f m^2', CLA_TARGET), ...
       sprintf('range %.1f-%.1f', CLA_BAND));
vd_row('Drag budget at the target', sprintf('CdA %.2f m^2', d_t.CdA), ...
       sprintf('package L/D %.1f', LD_TARGET));
vd_row('Points at the target', sprintf('%.0f', pts_t), sprintf('%+.0f vs this car', pts_t - pts0));
vd_row('  power cap / energy at the target', ...
       sprintf('%.1f kW', d_t.cap/1e3), sprintf('%.2f kWh', d_t.E22));
vd_row('Points per +1 m^2 ClA', sprintf('%+.1f', xr_cla));
vd_row('Points per +1 m^2 CdA', sprintf('%+.1f', xr_cda));
vd_row('Points per +1 kg', sprintf('%+.2f', xr_m));
vd_row('Lowest device L/D worth adding', sprintf('%.1f', ld_floor_full), ...
       sprintf('%.1f ignoring mass', ld_floor_drag));
fprintf(['Assumes: lap simulation on the axle models; downforce balance fixed (see\n' ...
         '         run_balance_targets). Sprint times inflated %.0f %% to match 2026 results;\n' ...
         '         assumed: best efficiency factor %.2f, device mass %.0f kg per m^2 ClA,\n' ...
         '         package L/D %.1f at the target.\n'], 100*HAIRCUT, EF_MAX, DM_PER_CLA, LD_TARGET);

out = struct('pts_baseline', pts0, 'pts_target', pts_t, 'dcla', DCLA, ...
             'ld_pkg', LD_PKG, 'points', P, 'ClA_target', CLA_TARGET, ...
             'CdA_budget', d_t.CdA, 'xr_cla', xr_cla, 'xr_cda', xr_cda, ...
             'xr_m', xr_m, 'ld_floor_full', ld_floor_full, ...
             'ld_floor_drag', ld_floor_drag, 'target_detail', d_t);

if vd_plots()
try
    make_plot(p, DCLA, LD_PKG, P, pts0, CLA_TARGET, CLA_BAND, pts_t, here);
    fprintf('Saved plots/aero_targets.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

function [pts, d] = score_point(ctx, dcla, ld_pkg, dm_per_cla)
% A point on a package line: drag and device mass follow the added downforce.
CdA = ctx.p.CdA + dcla / ld_pkg;            % inf -> baseline drag
[pts, d] = score_point_abs(ctx, dcla, CdA, dm_per_cla * dcla);
end

function [pts, d] = score_point_abs(ctx, dcla, CdA, dm)
% dm is aero hardware on the car, so it goes on m_car (vd_set re-derives the rest).
p2 = vd_set(ctx.p, 'ClA', ctx.p.ClA + dcla, 'CdA', CdA, 'm_car', ctx.p.m_car + dm);

% Sprint events at full power; raw QSS times (fsae_points applies the haircut).
R = fsae_rules();
t_ac_raw = accel_time(p2, R.accel_m);
t_sk_raw = 2*pi*R.skidpad_R_m / corner_speed(p2, 1/R.skidpad_R_m);

[~, t_ax_raw] = lap_sim(p2, ctx.s_ax, ctx.k_ax, 0, false);

% Endurance: the power cap that fits the pack with margin, and the lap at it
[t_en, E22, cap] = solve_endurance(ctx, p2);

ev = struct('accel', t_ac_raw, 'skidpad', t_sk_raw, 'autocross', t_ax_raw, ...
            'endurance_lap', t_en, 'laps', ctx.laps, 'energy_kWh', E22, ...
            'lap_km', ctx.s_en(end)/1000);
[pts, brk] = fsae_points(ev, ctx.bm, struct('haircut', ctx.haircut, ...
                         'ef_max', ctx.ef_max, 'e_min_kWh', ctx.e_min));

% Report the times that were scored (after the haircut).
t_ac  = brk.t_used.accel;      t_sk  = brk.t_used.skidpad;
t_ax  = brk.t_used.autocross;  t_tot = brk.t_used.endurance_total;

d = struct('t_ac', t_ac, 't_sk', t_sk, 't_ax', t_ax, 't_en', t_en, ...
           't_tot', t_tot, 'E22', E22, 'cap', cap, 'CdA', p2.CdA);
end

function [t_en, E22, cap] = solve_endurance(ctx, p2)
% Power cap at which the full rules distance (ctx.laps_feas laps) uses
% margin x usable pack energy. Event energy is smooth in the cap, so fit a
% quadratic through three caps and take its root. E22 is the energy over
% the scoring lap count (ctx.laps) at that cap.
caps = [20e3 32e3 46e3];
E3   = nan(1,3);
for i = 1:3
    p3 = p2;  p3.P_max = min(p2.P_max, caps(i));
    [~, ~, E] = lap_sim(p3, ctx.s_en, ctx.k_en, [], true);
    E3(i) = (E.drive_acc_Wh - E.brake_wheel_Wh * ctx.rc * ctx.rt) * ctx.laps_feas / 1000;
end
c = polyfit(caps, E3, 2);
r = roots([c(1) c(2) c(3) - ctx.margin*ctx.usable]);
r = r(imag(r) == 0 & r > 12e3 & r < p2.P_max);
if isempty(r), cap = p2.P_max; else, cap = min(max(r), p2.P_max); end

p3 = p2;  p3.P_max = cap;
[~, t_en, E] = lap_sim(p3, ctx.s_en, ctx.k_en, [], true);
E22 = (E.drive_acc_Wh - E.brake_wheel_Wh * ctx.rc * ctx.rt) * ctx.laps / 1000;
end

function make_plot(p, DCLA, LD_PKG, P, pts0, cla_t, xr, pts_t, here)
f = figure('Visible', 'off', 'Position', [80 80 900 520], 'Color', 'w');
hold on;
for i = 1:numel(LD_PKG)
    plot(p.ClA + DCLA, P(i,:), '-o', 'LineWidth', 1.8, ...
         'DisplayName', sprintf('package L/D = %.0f', LD_PKG(i)));
end
xline(p.ClA, ':', 'this car', 'LineWidth', 1.2, 'HandleVisibility', 'off');
fill([xr(1) xr(2) xr(2) xr(1)], [min(P(:))-5 min(P(:))-5 max(P(:))+5 max(P(:))+5], ...
     [0.13 0.40 0.67], 'FaceAlpha', 0.08, 'EdgeColor', 'none', 'DisplayName', 'target range');
plot(cla_t, pts_t, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [0.9 0.65 0], ...
     'MarkerEdgeColor', 'k', 'DisplayName', 'target');
yline(pts0, '--', 'baseline', 'LineWidth', 1.0, 'HandleVisibility', 'off');
xlabel('total ClA [m^2]');  ylabel('projected dynamic points');
title('Points vs downforce along package L/D lines (real 2026 results)', ...
      'FontWeight', 'bold');
legend('Location', 'southeast');  grid on;
outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'aero_targets.png'));
close(f);
end
