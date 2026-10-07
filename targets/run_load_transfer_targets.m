function out = run_load_transfer_targets(p)
% RUN_LOAD_TRANSFER_TARGETS  CG height, track width and brake bias targets from
% rigid-body quasi-static load transfer.
%
%   out = run_load_transfer_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_load_transfer_targets(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

out = struct();

% Design assumptions
cfg.mu          = p.mu_y;                 % design lateral grip (TTC-derated)
cfg.SF_rollover = 1.3;                    % rollover margin over grip
cfg.D_design    = p.mu_x;                 % design braking decel [g]
cfg.t_mean      = mean([p.t_f p.t_r]);    % mean track [m]

% CG height: the car must slide before it tips. It tips when the lateral
% acceleration reaches (t/2)/h_cg, so require (t/2)/h_cg >= SF*mu,
% i.e. h_cg <= (t/2)/(SF*mu).
h_cg_ceiling = (cfg.t_mean/2) / (cfg.SF_rollover * cfg.mu);
a_roll_now   = (cfg.t_mean/2) / p.h_cg;

% Track width: the same relation solved for track, track >= 2*h_cg*SF*mu.
track_floor = 2 * p.h_cg * cfg.SF_rollover * cfg.mu;

% Ideal brake bias: the front share of the axle loads at the design decel.
LT_braking = load_transfer(p, 0, -cfg.D_design);
bias_f     = LT_braking.Wf / (LT_braking.Wf + LT_braking.Wr);

% Driver weight range: the LIGHT driver is the rollover case. Less load
% means more mu (load sensitivity), so the car grips harder relative to its
% weight and tips sooner.
k           = vd_const();
m_light     = p.m_car + p.m_driver_min;
Fz_light    = m_light * p.g / 4 / k.N_PER_LBF;            % [lbf] per corner
mu_light    = mu_of_load(p, Fz_light);                    % derated mu at that load
marg_design = (cfg.t_mean/2) / (p.h_cg * cfg.mu);
marg_light  = (cfg.t_mean/2) / (p.h_cg * mu_light);
h_ceil_light = (cfg.t_mean/2) / (cfg.SF_rollover * mu_light);

% Transfer fractions do not depend on mass; absolute loads scale with it.
dWlat_per_kg = cfg.mu * p.g * p.h_cg / cfg.t_mean;   % [N/kg] at ay = mu

fprintf('\nLoad transfer - %s  (%.0f kg, CG height %.3f m, mean track %.3f m)\n', ...
        p.car, p.m, p.h_cg, cfg.t_mean);
vd_row('Max CG height (slides before it tips)', sprintf('%.3f m', h_cg_ceiling), ...
       sprintf('now %.3f m', p.h_cg), ternary(p.h_cg <= h_cg_ceiling, 'OK', 'TOO HIGH'));
vd_row('Min mean track width', sprintf('%.3f m', track_floor), ...
       sprintf('now %.3f m', cfg.t_mean), ternary(cfg.t_mean >= track_floor, 'OK', 'TOO NARROW'));
vd_row(sprintf('Ideal front brake bias at %.2f g', cfg.D_design), sprintf('%.1f %%', 100*bias_f));
vd_row(sprintf('Rollover margin, %.0f lb driver', p.m_driver/k.KG_PER_LB), ...
       sprintf('%.2f', marg_design), sprintf('need %.2f', cfg.SF_rollover), ...
       ternary(marg_design >= cfg.SF_rollover, 'OK', 'FAILS'));
vd_row(sprintf('Rollover margin, %.0f lb driver', p.m_driver_min/k.KG_PER_LB), ...
       sprintf('%.2f', marg_light), sprintf('need %.2f', cfg.SF_rollover), ...
       ternary(marg_light >= cfg.SF_rollover, 'OK', 'FAILS'));
vd_row(sprintf('Max CG height, %.0f lb driver', p.m_driver_min/k.KG_PER_LB), ...
       sprintf('%.3f m', h_ceil_light), sprintf('now %.3f m', p.h_cg), ...
       ternary(p.h_cg <= h_ceil_light, 'OK', 'TOO HIGH'));
fprintf(['Assumes: rigid body, static loads (no downforce), mean track width. Rollover\n' ...
         '         margin = tipping acceleration / tire grip (%.2f g); %.2f required.\n'], ...
        cfg.mu, cfg.SF_rollover);

% Outputs (read by vd_selftest and vd_golden)
out.mu_light        = mu_light;
out.marg_light      = marg_light;
out.marg_design     = marg_design;
out.h_cg_ceil_light = h_ceil_light;
out.h_cg_ceiling = h_cg_ceiling;
out.track_floor  = track_floor;
out.bias_f       = bias_f;
out.dWlat_per_kg = dWlat_per_kg;
out.a_roll_now   = a_roll_now;
out.cfg          = cfg;

if vd_plots()
try
    make_plots(p, cfg, h_cg_ceiling, track_floor);
    fprintf('Saved plots/load_transfer_targets.png\n');
catch err
    fprintf('Plot not saved: %s\n', err.message);
end
end
end

function make_plots(p, cfg, h_cg_ceiling, track_floor)
% One panel per result: the swept quantity, the limit line and this car.
h_cg_sweep  = linspace(0.20, 0.40, 80);
track_sweep = linspace(1.00, 1.50, 80);
decel_sweep = linspace(0, cfg.mu, 60);
mass_sweep  = linspace(200, 400, 60);

a_roll_vs_h     = (cfg.t_mean/2) ./ h_cg_sweep;
a_roll_vs_track = (track_sweep/2) / p.h_cg;
bias_vs_decel   = p.mass_dist_f + decel_sweep * p.h_cg / p.L;
dWlat_vs_mass   = cfg.mu * p.g * p.h_cg / cfg.t_mean .* mass_sweep;

fig = figure('Visible', 'off', 'Position', [100 100 980 760]);

subplot(2, 2, 1);
plot(h_cg_sweep, a_roll_vs_h, '-', 'LineWidth', 1.6); hold on;
plot(xlim, cfg.SF_rollover * cfg.mu * [1 1], '--');
plot(h_cg_ceiling * [1 1], ylim, ':');
plot(p.h_cg, (cfg.t_mean/2)/p.h_cg, 'o', 'MarkerSize', 7, 'LineWidth', 1.5);
xlabel('CG height [m]'); ylabel('tipping acceleration [g]');
title('CG height limit'); grid on;
legend('tipping acceleration', sprintf('required (%.2f x grip)', cfg.SF_rollover), ...
       'limit', 'this car', 'Location', 'northeast');

subplot(2, 2, 2);
plot(track_sweep, a_roll_vs_track, '-', 'LineWidth', 1.6); hold on;
plot(xlim, cfg.SF_rollover * cfg.mu * [1 1], '--');
plot(track_floor * [1 1], ylim, ':');
plot(cfg.t_mean, (cfg.t_mean/2)/p.h_cg, 'o', 'MarkerSize', 7, 'LineWidth', 1.5);
xlabel('mean track width [m]'); ylabel('tipping acceleration [g]');
title('Track width limit'); grid on;

subplot(2, 2, 3);
plot(decel_sweep, 100*bias_vs_decel, '-', 'LineWidth', 1.6); hold on;
plot(cfg.D_design, 100*(p.mass_dist_f + cfg.D_design*p.h_cg/p.L), ...
     'o', 'MarkerSize', 7, 'LineWidth', 1.5);
xlabel('braking deceleration [g]'); ylabel('ideal front brake bias [%]');
title('Ideal brake bias vs deceleration'); grid on;

subplot(2, 2, 4);
plot(mass_sweep, dWlat_vs_mass, '-', 'LineWidth', 1.6); hold on;
plot(p.m, cfg.mu*p.g*p.h_cg/cfg.t_mean*p.m, 'o', 'MarkerSize', 7, 'LineWidth', 1.5);
xlabel('vehicle mass [kg]'); ylabel('lateral load transfer at the limit [N]');
title('Load transfer scales with mass'); grid on;

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(fig, fullfile(outdir, 'load_transfer_targets.png'));
close(fig);
end

function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end
