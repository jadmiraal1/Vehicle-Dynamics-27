function out = run_load_transfer_targets()
% RUN_LOAD_TRANSFER_TARGETS  Four chassis targets from quasi-static load transfer:
% T-CGH CG-height ceiling | T-TRK track floor | T-BB brake bias | T-MS mass.
% Derivations and caveats: VD_physics_reference.md, section 3.
% Output field names are used by vd_selftest.m — keep.

p = vehicle_params();

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

% DRIVER ENVELOPE: rollover is now driven by TWO opposing driver-mass effects,
% so both ends are checked. Margin = (t/2)/(h*mu):
%   - tire mu RISES as load falls  -> LIGHT driver makes more grip, tips sooner
%   - combined CG h RISES with load -> HEAVY driver rides higher, tips sooner
% The mu effect wins (light driver still binds), but the CG effect now offsets
% most of the spread the mu-only story implied -- the ends are ~1% apart, not
% the old ~2%. Each end uses its OWN combined CG height (p.h_cg_light/heavy),
% not the design CG, so the numbers are self-consistent.
N_PER_LBF   = 4.44822;
m_light     = p.m_car + p.m_driver_min;
m_heavy     = p.m_car + p.m_driver_max;
Fz_light    = m_light * p.g / 4 / N_PER_LBF;              % [lbf] per corner
Fz_heavy    = m_heavy * p.g / 4 / N_PER_LBF;
mu_light    = polyval(p.mu_coef, Fz_light) * p.mu_derate; % mu at THAT load
mu_heavy    = polyval(p.mu_coef, Fz_heavy) * p.mu_derate;
marg_design = (cfg.t_mean/2) / (p.h_cg       * cfg.mu);
marg_light  = (cfg.t_mean/2) / (p.h_cg_light * mu_light);
marg_heavy  = (cfg.t_mean/2) / (p.h_cg_heavy * mu_heavy);
marg_bind   = min(marg_light, marg_heavy);
h_ceil_light = (cfg.t_mean/2) / (cfg.SF_rollover * mu_light);

fprintf('T-CGH  driver envelope: margin %.2fx design | %.2fx LIGHT (%.0f lb) | %.2fx HEAVY (%.0f lb)\n', ...
        marg_design, marg_light, p.m_driver_min/0.45359237, marg_heavy, p.m_driver_max/0.45359237);
fprintf('       BINDING case is the LIGHT driver (less load -> more mu -> tips sooner).\n');
fprintf('       ceiling at light driver: h_cg <= %.3f m  (current light h_cg %.3f m: %s)\n', ...
        h_ceil_light, p.h_cg_light, ...
        ternary(p.h_cg_light <= h_ceil_light, 'OK', 'EXCEEDS - flag to packaging'));
if marg_bind < cfg.SF_rollover
    fprintf(2, '       *** ROLLOVER MARGIN FAILS THE SF=%.2f TARGET (binding %.2f) ***\n', ...
            cfg.SF_rollover, marg_bind);
end

out.mu_light        = mu_light;
out.mu_heavy        = mu_heavy;
out.marg_light      = marg_light;
out.marg_heavy      = marg_heavy;
out.marg_design     = marg_design;
out.h_cg_ceil_light = h_ceil_light;

% T-MS : transfer fractions are mass-invariant; only absolute loads scale
dWlat_per_kg = cfg.mu * p.g * p.h_cg / cfg.t_mean;   % [N/kg] at ay = mu

fprintf('T-MS   mass target  : none from this model (fractions mass-invariant; %.2f N/kg abs.)\n', ...
        dWlat_per_kg);
fprintf('CAVEAT: rigid body, no aero, geometric lateral split; bias provisional.\n');

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

outdir = fullfile(fileparts(mfilename('fullpath')), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(fig, fullfile(outdir, 'load_transfer_targets.png'));
close(fig);
end


function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end
