function out = run_gg_targets()
% RUN_GG_TARGETS  Skidpad / accel / braking targets from the g-g-V envelope.

p = vehicle_params();

% Event definitions (FSAE geometry, not vehicle parameters)
cfg.R_skid  = 9.125;   % skidpad path radius [m]
cfg.s_accel = 75.0;    % acceleration event length [m]

fprintf('\ng-g-V TARGETS  (mu=%.2f, P=%.0f kW, %s)\n', p.mu_y, p.P_max/1e3, p.drive);

% T-SKID : v^2/R = mu_y*N(v)/(m); closed form since N is linear in v^2
v_skid_sq = p.mu_y*p.g / (1/cfg.R_skid - p.mu_y*0.5*p.rho*p.ClA/p.m);
v_skid    = sqrt(v_skid_sq);
ay_skid   = v_skid_sq / (cfg.R_skid*p.g);
t_skid    = 2*pi*cfg.R_skid / v_skid;

fprintf('T-SKID skidpad     : %.2f g, %.2f s  (R=%.2f m, v=%.1f m/s)\n', ...
        ay_skid, t_skid, cfg.R_skid, v_skid);

% T-ACC : forward integration (limit changes with speed, no closed form)
[t_acc, v_end] = accel_event(p, cfg.s_accel);
G_launch = gg_envelope(p, 0.3);
v_cross  = crossover_speed(p);

fprintf('T-ACC  75 m accel  : %.2f s  (v_end %.1f m/s = %.0f kph; launch %.2f g)\n', ...
        t_acc, v_end, v_end*3.6, G_launch.ax_accel);

% T-GG : envelope cardinal points, near-static and aero-loaded
gg_lo = gg_envelope(p, 0.5);
gg_hi = gg_envelope(p, 25);

fprintf('T-GG   low speed   : accel %.2f | brake %.2f | lateral %.2f g\n', ...
        gg_lo.ax_accel, gg_lo.ax_brake, gg_lo.ay);
fprintf('       at 25 m/s   : brake %.2f | lateral %.2f g (aero)\n', ...
        gg_hi.ax_brake, gg_hi.ay);

% T-PWR : grip-limited below crossover, power-limited above
fprintf('T-PWR  crossover   : power-limited above ~%.1f m/s (%.0f kW rules cap)\n', ...
        v_cross, p.P_max/1e3);

% T-MS2 : finite difference on the accel event (+10 kg)
p_heavier   = p;
p_heavier.m = p.m + 10;
[t_acc_heavier, ~] = accel_event(p_heavier, cfg.s_accel);
dt_per_kg = (t_acc_heavier - t_acc) / 10;

fprintf('T-MS2  accel mass  : %.1f ms/kg  (+10 kg finite diff; accel event only)\n', ...
        dt_per_kg*1000);
fprintf('Caveat: point mass, no combined-slip tire; k_trac/eta provisional.\n');

% Pack for programmatic use / vd_selftest
out.ay_skid    = ay_skid;
out.t_skid     = t_skid;
out.v_skid     = v_skid;
out.t_acc      = t_acc;
out.v_end      = v_end;
out.v_cross    = v_cross;
out.gg_lo      = gg_lo;
out.gg_hi      = gg_hi;
out.dtdm_accel = dt_per_kg;
out.cfg        = cfg;

try
    plot_gg(p);
    fprintf('\nPlot written: gg_envelope.png\n\n');
catch e
    fprintf('\n[plot skipped: %s]\n\n', e.message);
end
end

function [t, v] = accel_event(p, dist)
% Standing-start time by forward integration in fixed velocity steps.
v = 0; x = 0; t = 0;
dv = 0.005;                                    % [m/s]
while x < dist
    GG = gg_envelope(p, v);
    a  = GG.ax_accel * p.g;
    if a <= 0, break; end                      % drag/power ceiling
    dt = dv / a;
    x  = x + v*dt + 0.5*a*dt^2;
    t  = t + dt;
    v  = v + dv;
end
end

function vc = crossover_speed(p)
% Lowest speed where the motor enters constant power (base speed).
v_sweep = linspace(1, 45, 450);
G   = gg_envelope(p, v_sweep);
idx = find(G.power_limited, 1);
if isempty(idx), vc = v_sweep(end); else, vc = v_sweep(idx); end
end

function plot_gg(p)
% Left: capability edges vs speed. Right: g-g ellipses at a few speeds.
v_sweep = linspace(0.5, 35, 60);
G = gg_envelope(p, v_sweep);
f = figure('Visible', 'off', 'Position', [100 100 1000 420]);

subplot(1, 2, 1); hold on;
plot(v_sweep, G.ay,       '-', 'LineWidth', 1.6);
plot(v_sweep, G.ax_accel, '-', 'LineWidth', 1.6);
plot(v_sweep, G.ax_brake, '-', 'LineWidth', 1.6);
xlabel('speed [m/s]'); ylabel('capability [g]'); grid on;
legend('lateral a_y', 'accel a_x', 'brake a_x', 'Location', 'east');
title('g-g-V: capability vs speed');

subplot(1, 2, 2); hold on;
for vk = [5 15 25 35]
    Gk    = gg_envelope(p, vk);
    theta = linspace(0, 2*pi, 200);
    ay    = Gk.ay * sin(theta);
    ax    = zeros(size(theta));
    accel_half      = cos(theta) >= 0;
    ax(accel_half)  = Gk.ax_accel * cos(theta(accel_half));
    ax(~accel_half) = Gk.ax_brake * cos(theta(~accel_half));
    plot(ax, ay, '-', 'LineWidth', 1.4, 'DisplayName', sprintf('%d m/s', vk));
end
xlabel('longitudinal a_x [g]  (+accel / -brake)'); ylabel('lateral a_y [g]');
axis equal; grid on; legend('Location', 'eastoutside');
title('g-g ellipses vs speed');

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'gg_envelope.png'));
close(f);
end
