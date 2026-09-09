function out = run_load_transfer_targets(p)
% RUN_LOAD_TRANSFER_TARGETS  CG/track/bias targets from quasi-static load transfer.
%
%   out = run_load_transfer_targets()      the active car, from vd_car / cars/config_<CAR>.m
%   out = run_load_transfer_targets(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = run_load_transfer_targets(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)

out = struct();   % populated below

% Design assumptions
cfg.mu          = p.mu_y;                 % design lateral grip (TTC-derated)
cfg.SF_rollover = 1.3;                    % rollover margin over grip
cfg.D_design    = p.mu_x;                 % design braking decel [g]
cfg.t_mean      = mean([p.t_f p.t_r]);    % mean track [m]

fprintf('\nLOAD-TRANSFER TARGETS  (mu=%.2f, SF=%.2f | m=%.0f kg, h_cg=%.3f m, track=%.3f m)\n', ...
        cfg.mu, cfg.SF_rollover, p.m, p.h_cg, cfg.t_mean);

% T-CGH : slide-before-tip  =>  h_cg <= (track/2)/(SF*mu)
h_cg_ceiling = (cfg.t_mean/2) / (cfg.SF_rollover * cfg.mu);
a_roll_now   = (cfg.t_mean/2) / p.h_cg;

fprintf('T-CGH  h_cg ceiling : <= %.3f m  (current %.3f m: rollover %.2f g vs grip %.2f g)\n', ...
        h_cg_ceiling, p.h_cg, a_roll_now, cfg.mu);

% T-TRK : same relation solved for track  =>  track >= 2*h_cg*SF*mu
track_floor = 2 * p.h_cg * cfg.SF_rollover * cfg.mu;

fprintf('T-TRK  track floor  : >= %.3f m  (current %.3f m: %s)\n', ...
        track_floor, cfg.t_mean, ternary(cfg.t_mean >= track_floor, 'OK', 'UNDER floor'));

% T-BB : ideal front bias = dynamic front load fraction at design decel
LT_braking = load_transfer(p, 0, -cfg.D_design);
bias_f     = LT_braking.Wf / (LT_braking.Wf + LT_braking.Wr);

fprintf('T-BB   front bias   : %.1f%% front  (at %.2f g braking: front %.0f of %.0f N)\n', ...
        100*bias_f, cfg.D_design, LT_braking.Wf, LT_braking.Wf + LT_braking.Wr);

% DRIVER ENVELOPE: rollover must be checked at the LIGHT driver, not the heavy one.
N_PER_LBF   = 4.44822;
m_light     = p.m_car + p.m_driver_min;
Fz_light    = m_light * p.g / 4 / N_PER_LBF;              % [lbf] per corner
mu_light    = polyval(p.mu_coef, Fz_light) * p.mu_derate; % mu at THAT load
marg_design = (cfg.t_mean/2) / (p.h_cg * cfg.mu);
marg_light  = (cfg.t_mean/2) / (p.h_cg * mu_light);
h_ceil_light = (cfg.t_mean/2) / (cfg.SF_rollover * mu_light);

fprintf('T-CGH  driver envelope: rollover margin %.2fx design (%.0f lb driver) -> %.2fx light (%.0f lb)\n', ...
        marg_design, p.m_driver/0.45359237, marg_light, p.m_driver_min/0.45359237);
fprintf('       binding case is the light driver (less load -> more mu -> tips sooner).\n');
fprintf('       ceiling at light driver: h_cg <= %.3f m  (%s)\n', h_ceil_light, ...
        ternary(p.h_cg <= h_ceil_light, 'OK', 'EXCEEDS - flag to packaging'));
if marg_light < cfg.SF_rollover
    fprintf(2, '       *** ROLLOVER MARGIN FAILS AT the light DRIVER (%.2f < %.2f) ***\n', ...
            marg_light, cfg.SF_rollover);
end

out.mu_light        = mu_light;
out.marg_light      = marg_light;
out.marg_design     = marg_design;
out.h_cg_ceil_light = h_ceil_light;

% T-MS : transfer fractions are mass-invariant; only absolute loads scale
dWlat_per_kg = cfg.mu * p.g * p.h_cg / cfg.t_mean;   % [N/kg] at ay = mu

fprintf('T-MS   mass target  : none from this model (fractions mass-invariant; %.2f N/kg abs.)\n', ...
        dWlat_per_kg);
fprintf('Caveat: rigid body, no aero, geometric lateral split; bias provisional.\n');

% Pack for programmatic use / vd_selftest
out.h_cg_ceiling = h_cg_ceiling;
out.track_floor  = track_floor;
out.bias_f       = bias_f;
out.dWlat_per_kg = dWlat_per_kg;
out.a_roll_now   = a_roll_now;
out.cfg          = cfg;

try
    make_plots(p, cfg, h_cg_ceiling, track_floor);
    fprintf('\nPlots written: load_transfer_targets.png\n\n');
catch err
    fprintf('\n[plot skipped: %s]\n\n', err.message);
end
end

function make_plots(p, cfg, h_cg_ceiling, track_floor)
% One panel per target: swept assumption, limit line, current design point.
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
xlabel('CG height h_{cg} [m]'); ylabel('rollover threshold [g]');
title('T-CGH: CG height ceiling'); grid on;
legend('rollover thr.', sprintf('SF*mu=%.2f', cfg.SF_rollover*cfg.mu), ...
       'ceiling', 'current', 'Location', 'northeast');

subplot(2, 2, 2);
plot(track_sweep, a_roll_vs_track, '-', 'LineWidth', 1.6); hold on;
plot(xlim, cfg.SF_rollover * cfg.mu * [1 1], '--');
plot(track_floor * [1 1], ylim, ':');
plot(cfg.t_mean, (cfg.t_mean/2)/p.h_cg, 'o', 'MarkerSize', 7, 'LineWidth', 1.5);
xlabel('mean track [m]'); ylabel('rollover threshold [g]');
title('T-TRK: track width floor'); grid on;

subplot(2, 2, 3);
plot(decel_sweep, 100*bias_vs_decel, '-', 'LineWidth', 1.6); hold on;
plot(cfg.D_design, 100*(p.mass_dist_f + cfg.D_design*p.h_cg/p.L), ...
     'o', 'MarkerSize', 7, 'LineWidth', 1.5);
xlabel('braking decel [g]'); ylabel('ideal front brake bias [%]');
title('T-BB: brake bias vs decel'); grid on;

subplot(2, 2, 4);
plot(mass_sweep, dWlat_vs_mass, '-', 'LineWidth', 1.6); hold on;
plot(p.m, cfg.mu*p.g*p.h_cg/cfg.t_mean*p.m, 'o', 'MarkerSize', 7, 'LineWidth', 1.5);
xlabel('vehicle mass [kg]'); ylabel('lateral transfer @ limit [N]');
title('T-MS: absolute transfer scales w/ mass (fraction does not)'); grid on;

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(fig, fullfile(outdir, 'load_transfer_targets.png'));
close(fig);
end

function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end
