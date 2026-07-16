function out = run_aero_targets()
% RUN_AERO_TARGETS  Aero package targets from the points model.
% T-CLA downforce target | T-CDA drag budget | T-LDF device L/D floor |
% T-XR exchange rates (pts per m^2, pts per kg).
%
% Method: sweep added downforce along ACHIEVABLE PACKAGE LINES
% (CdA = CdA0 + dClA/LD_package, mass = m0 + dm*dClA), score each point
% against the REAL FSAE 2026 Michigan benchmarks (comp_benchmarks_2026.csv),
% with the endurance power cap re-solved per point so drag pays its true
% energy price. Theory + caveats: VD_physics_reference.md; consumers: aero.
%
% KEY FINDING (Jul 2026): points are monotonically increasing in ClA up to
% the sweep edge (ClA ~5) for every scenario tested (eta 0.84-0.92, haircut
% 4-12%, mass penalty 3-8 kg/m^2, package L/D 3-5). The binding constraint
% is the rules envelope and packaging, NOT lap physics. So the deliverable
% is a target BAND plus exchange rates and floors, not an interior optimum.
%
% Runtime: several minutes (each sweep point runs ~5 lap sims).

p = vehicle_params();
here = fileparts(mfilename('fullpath'));

% Sweep and package assumptions
DCLA        = 0:0.75:3.75;   % added downforce [m^2]
LD_PKG      = [3 4 5];       % whole-package lift/drag lines [-]
DM_PER_CLA  = 4.0;           % aero package mass penalty [kg per m^2 ClA] PROVISIONAL
CLA_TARGET  = 4.0;           % issued target (band 3.5-4.5) [m^2]
LD_TARGET   = 4.0;           % package L/D at the issued target [-]

% Scoring assumptions
HAIRCUT     = 0.08;   % sprint-event time inflation vs QSS (2026-calibrated)
EF_MAX      = 0.60;   % best real 2026 efficiency factor PROVISIONAL
E_MIN_KWH   = 2.696;  % lowest real 22-lap finisher energy (Wisconsin)
LAPS        = 22;     % real 2026 endurance (21.2 km on our 964 m course).
                      % NOTE: run_energy_strategy uses the RULES distance
                      % (22.0 km -> 22.8 laps), ~4% more energy. Deliberate:
                      % scoring vs real 2026 Tmins uses the real event basis;
                      % the deployment strategy plans for the rules distance.

% Endurance strategy (must match run_energy_strategy.m)
REGEN_CAPTURE = 0.50;  REGEN_RT = 0.65;  PACK_USABLE_F = 0.90;  MARGIN = 0.90;

bm = read_benchmarks(fullfile(here, 'organization', 'comp_benchmarks_2026.csv'));

[s_en, k_en] = load_track(fullfile(here, 'tracks', 'track_endurance.csv'));
[s_ax, k_ax] = load_track(fullfile(here, 'tracks', 'track_autocross.csv'));
ctx = struct('p', p, 's_en', s_en, 'k_en', k_en, 's_ax', s_ax, 'k_ax', k_ax, ...
             'bm', bm, 'haircut', HAIRCUT, 'ef_max', EF_MAX, 'e_min', E_MIN_KWH, ...
             'laps', LAPS, 'rc', REGEN_CAPTURE, 'rt', REGEN_RT, ...
             'usable', PACK_USABLE_F * p.E_pack_Wh / 1000, 'margin', MARGIN);

% Baseline and sweep
[pts0, d0] = score_point(ctx, 0, inf, 0);
P = nan(numel(LD_PKG), numel(DCLA));
for i = 1:numel(LD_PKG)
    for j = 1:numel(DCLA)
        P(i,j) = score_point(ctx, DCLA(j), LD_PKG(i), DM_PER_CLA);
    end
end

% Exchange rates at the issued target (central differences are overkill;
% forward diffs at sweep resolution match the python prototype)
dc_t = CLA_TARGET - p.ClA;
[pts_t, d_t] = score_point(ctx, dc_t, LD_TARGET, DM_PER_CLA);
xr_cla = (score_point_abs(ctx, dc_t+0.4, d_t.CdA,     DM_PER_CLA*dc_t) - pts_t) / 0.4;
xr_cda = (score_point_abs(ctx, dc_t,     d_t.CdA+0.2, DM_PER_CLA*dc_t) - pts_t) / 0.2;
xr_m   = (score_point_abs(ctx, dc_t,     d_t.CdA,     DM_PER_CLA*dc_t+10) - pts_t) / 10;
ld_floor_drag = abs(xr_cda) / xr_cla;                       % drag-only
ld_floor_full = abs(xr_cda) / (xr_cla + xr_m*DM_PER_CLA);   % incl. device mass

fprintf('\nAERO TARGETS  (points vs REAL 2026 benchmarks, %.0f%% haircut, adaptive endurance cap)\n', 100*HAIRCUT);
fprintf('baseline ClA %.2f / CdA %.2f : %.0f pts (autox %.1f s, endur %.0f s, %.2f kWh @ %.0f kW cap)\n', ...
        p.ClA, p.CdA, pts0, d0.t_ax, d0.t_tot, d0.E22, d0.cap/1e3);
fprintf('%12s'  , 'added ClA:');  fprintf(' %7.2f', DCLA);  fprintf('\n');
for i = 1:numel(LD_PKG)
    fprintf('pkg L/D %3.0f:', LD_PKG(i));  fprintf(' %7.0f', P(i,:));  fprintf('\n');
end
fprintf('\nT-CLA  downforce target : ClA = %.1f m^2  (band 3.5-4.5; points MONOTONIC in ClA\n', CLA_TARGET);
fprintf('       to the sweep edge -- package as much as rules + structure allow)\n');
fprintf('T-CDA  drag budget      : CdA <= %.2f m^2 at ClA %.1f  (package L/D >= %.1f)\n', ...
        d_t.CdA, CLA_TARGET, LD_TARGET);
fprintf('T-XR   exchange rates   : %+.1f pts/m^2 ClA | %+.1f pts/m^2 CdA | %+.2f pts/kg\n', ...
        xr_cla, xr_cda, xr_m);
fprintf('T-LDF  device L/D floor : %.1f incl. %.0f kg/m^2 mass (drag-only %.1f) --\n', ...
        ld_floor_full, DM_PER_CLA, ld_floor_drag);
fprintf('       no device below L/D ~2 with margin; floor RISES with total ClA\n');
fprintf('T-BAL  balance (prov.)  : front downforce fraction 35-45%% (CoP at/just aft\n');
fprintf('       of CG) -- PROVISIONAL until the bicycle+CoP model (target #38)\n');
fprintf('at target: %.0f pts (%+.0f vs baseline); endurance cap re-solves to %.0f kW, %.2f kWh\n', ...
        pts_t, pts_t - pts0, d_t.cap/1e3, d_t.E22);
fprintf('CAVEAT: QSS point-mass, scalar mu (no load sensitivity on added mass),\n');
fprintf('        haircut/EF_MAX/regen/mass-penalty are calibrated assumptions.\n');
fprintf('        Re-issue on: mid-tier balance model, lambda calibration, mass change.\n');

out = struct('pts_baseline', pts0, 'pts_target', pts_t, 'dcla', DCLA, ...
             'ld_pkg', LD_PKG, 'points', P, 'ClA_target', CLA_TARGET, ...
             'CdA_budget', d_t.CdA, 'xr_cla', xr_cla, 'xr_cda', xr_cda, ...
             'xr_m', xr_m, 'ld_floor_full', ld_floor_full, ...
             'ld_floor_drag', ld_floor_drag, 'target_detail', d_t);

try
    make_plot(p, DCLA, LD_PKG, P, pts0, CLA_TARGET, pts_t, here);
    fprintf('Plot written: plots/aero_targets.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end


function [pts, d] = score_point(ctx, dcla, ld_pkg, dm_per_cla)
% Package line: drag and mass follow added downforce
CdA = ctx.p.CdA + dcla / ld_pkg;            % inf -> baseline drag
[pts, d] = score_point_abs(ctx, dcla, CdA, dm_per_cla * dcla);
end


function [pts, d] = score_point_abs(ctx, dcla, CdA, dm)
p2      = ctx.p;
p2.ClA  = ctx.p.ClA + dcla;
p2.CdA  = CdA;
p2.m    = ctx.p.m + dm;
sumI    = 4*p2.I_wheel + p2.I_rotor * p2.gear_ratio^2;
p2.k_rot = 1 + sumI / (p2.m * p2.Re^2);
p2.Wf_static = p2.m * p2.g * p2.mass_dist_f;
p2.Wr_static = p2.m * p2.g * (1 - p2.mass_dist_f);

% Sprint events (full power) with haircut
s75 = (0:0.5:75)';
[~, t_ac] = lap_sim(p2, s75, zeros(size(s75)), 0, false);
t_ac = t_ac * (1 + ctx.haircut);

R_skid = 9.125;
t_sk = 2*pi*R_skid / corner_speed(p2, 1/R_skid) * (1 + ctx.haircut);

[~, t_ax] = lap_sim(p2, ctx.s_ax, ctx.k_ax, 0, false);
t_ax = t_ax * (1 + ctx.haircut);

% Endurance: solve the power cap that fits the pack with margin, run at it
[t_en, E22, cap] = solve_endurance(ctx, p2);
t_tot = t_en * ctx.laps * (1 + ctx.haircut*0.5);   % half haircut: pace is capped anyway

% FSAE points vs real 2026 Tmins
pts =       tscore(t_ac,  ctx.bm.accel_tmin_s,     1.50,  95.5,  4.5, false);
pts = pts + tscore(t_sk,  ctx.bm.skidpad_tmin_s,   1.25,  71.5,  3.5, true);
pts = pts + tscore(t_ax,  ctx.bm.autocross_tmin_s, 1.45, 118.5,  6.5, false);
pts = pts + tscore(t_tot, ctx.bm.endurance_tmin_s, 1.45, 250.0, 25.0, false);
ef  = (ctx.bm.endurance_tmin_s / t_tot) * (ctx.e_min / max(E22, ctx.e_min));
pts = pts + min(100, max(0, 100 * (ef - 0.1) / (ctx.ef_max - 0.1)));

d = struct('t_ac', t_ac, 't_sk', t_sk, 't_ax', t_ax, 't_en', t_en, ...
           't_tot', t_tot, 'E22', E22, 'cap', cap, 'CdA', p2.CdA);
end


function [t_en, E22, cap] = solve_endurance(ctx, p2)
% E_net(cap) is smooth; 3 points + quadratic root beats a bisection loop
caps = [20e3 32e3 46e3];
E3   = nan(1,3);
for i = 1:3
    p3 = p2;  p3.P_max = min(p2.P_max, caps(i));
    [~, ~, E] = lap_sim(p3, ctx.s_en, ctx.k_en, [], true);
    E3(i) = (E.drive_acc_Wh - E.brake_wheel_Wh * ctx.rc * ctx.rt) * ctx.laps / 1000;
end
c = polyfit(caps, E3, 2);
r = roots([c(1) c(2) c(3) - ctx.margin*ctx.usable]);
r = r(imag(r) == 0 & r > 12e3 & r < p2.P_max);
if isempty(r), cap = p2.P_max; else, cap = min(max(r), p2.P_max); end

p3 = p2;  p3.P_max = cap;
[~, t_en, E] = lap_sim(p3, ctx.s_en, ctx.k_en, [], true);
E22 = (E.drive_acc_Wh - E.brake_wheel_Wh * ctx.rc * ctx.rt) * ctx.laps / 1000;
end


function s = tscore(t, tmin, fmax, pvar, pmin, squared)
% FSAE event score. Tmin floors at OUR time (we'd set the benchmark).
tmin = min(t, tmin);
tmax = fmax * tmin;
t    = min(t, tmax);
if squared, ratio = (tmax/t)^2 - 1;  rmax = (tmax/tmin)^2 - 1;
else,       ratio =  tmax/t    - 1;  rmax =  tmax/tmin    - 1;
end
s = pvar * ratio / rmax + pmin;
end


function bm = read_benchmarks(fname)
fid = fopen(fname, 'r');
if fid < 0, error('run_aero_targets:noBenchmarks', 'missing %s', fname); end
bm = struct();
while true
    ln = fgetl(fid);
    if ~ischar(ln), break; end
    ln = strtrim(ln);
    if isempty(ln) || ln(1) == '#' || startsWith(ln, 'metric'), continue; end
    c = strsplit(ln, ',');
    v = str2double(c{2});
    if ~isnan(v), bm.(matlab.lang.makeValidName(c{1})) = v; end
end
fclose(fid);
end


function make_plot(p, DCLA, LD_PKG, P, pts0, cla_t, pts_t, here)
f = figure('Visible', 'off', 'Position', [80 80 900 520], 'Color', 'w');
hold on;
for i = 1:numel(LD_PKG)
    plot(p.ClA + DCLA, P(i,:), '-o', 'LineWidth', 1.8, ...
         'DisplayName', sprintf('package L/D = %.0f', LD_PKG(i)));
end
xline(p.ClA, ':', 'current', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xr = [3.5 4.5];
fill([xr(1) xr(2) xr(2) xr(1)], [min(P(:))-5 min(P(:))-5 max(P(:))+5 max(P(:))+5], ...
     [0.13 0.40 0.67], 'FaceAlpha', 0.08, 'EdgeColor', 'none', 'DisplayName', 'target band');
plot(cla_t, pts_t, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [0.9 0.65 0], ...
     'MarkerEdgeColor', 'k', 'DisplayName', 'issued target');
yline(pts0, '--', 'baseline', 'LineWidth', 1.0, 'HandleVisibility', 'off');
xlabel('total ClA [m^2]');  ylabel('projected dynamic points');
title('Aero targets: points vs downforce along achievable package lines (real 2026 benchmarks)', ...
      'FontWeight', 'bold');
legend('Location', 'southeast');  grid on;
outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'aero_targets.png'));
close(f);
end
