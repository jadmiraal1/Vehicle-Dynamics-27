function out = run_gg_targets(p)
% RUN_GG_TARGETS  Skidpad, 75 m and braking targets from the g-g-V envelope.
%
%   out = run_gg_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_gg_targets(p)     an explicit params struct, e.g. a "what if?":
%       out = run_gg_targets(vd_set(vehicle_params(), 'm_car', 240, 'ClA', 4.0));
%
% The skidpad and g-g figures use the point-mass envelope (constant mu, tier
% T0) and are optimistic; run_lap_targets and run_balance_targets give the
% load-sensitive skidpad. The 75 m time uses accel_time (load-sensitive axle
% model, motor-capped). Also draws the g-g-V surface of the per-axle
% combined model.

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

R = fsae_rules();
cfg.R_skid  = R.skidpad_R_m;   % skidpad path radius [m]
cfg.s_accel = R.accel_m;       % acceleration event length [m]

% Skidpad: v^2/R = mu_y*(m*g + 0.5*rho*ClA*v^2)/m, solved for v^2 (point mass)
v_skid_sq = p.mu_y*p.g / (1/cfg.R_skid - p.mu_y*0.5*p.rho*p.ClA/p.m);
v_skid    = sqrt(v_skid_sq);
ay_skid   = v_skid_sq / (cfg.R_skid*p.g);
t_skid    = 2*pi*cfg.R_skid / v_skid;

% 75 m acceleration: integrated in time (the limit changes with speed)
[t_acc, v_end] = accel_time(p, cfg.s_accel);
G_launch = gg_envelope(p, 0.3);
v_cross  = crossover_speed(p);

% Point-mass envelope at low speed and at 25 m/s
gg_lo = gg_envelope(p, 0.5);
gg_hi = gg_envelope(p, 25);

% Mass sensitivity: finite difference on the accel event (+10 kg on the car)
p_heavier = vd_set(p, 'm_car', p.m_car + 10);
[t_acc_heavier, ~] = accel_time(p_heavier, cfg.s_accel);
dt_per_kg = (t_acc_heavier - t_acc) / 10;

fprintf('\nGrip and acceleration envelope - %s  (tire grip %.2f, power limit %.1f kW, %s)\n', ...
        p.car, p.mu_y, p.P_max/1e3, p.drive);
vd_row('Skidpad lap time, constant-grip point mass', sprintf('%.2f s', t_skid), ...
       sprintf('%.2f g, %.1f m/s', ay_skid, v_skid));
vd_row(sprintf('%.0f m acceleration time', cfg.s_accel), sprintf('%.2f s', t_acc), ...
       sprintf('ends at %.0f km/h', v_end*3.6));
vd_row('Launch acceleration', sprintf('%.2f g', G_launch.ax_accel));
vd_row('Low speed: accel / brake / lateral', ...
       sprintf('%.2f / %.2f / %.2f g', gg_lo.ax_accel, gg_lo.ax_brake, gg_lo.ay));
vd_row('25 m/s: brake / lateral', sprintf('%.2f / %.2f g', gg_hi.ax_brake, gg_hi.ay));
vd_row('Power-limited above', sprintf('%.1f m/s', v_cross));
vd_row(sprintf('%.0f m time per added kg', cfg.s_accel), sprintf('%.1f ms/kg', dt_per_kg*1000));
fprintf(['Assumes: skidpad and grip figures use a constant-grip point mass (optimistic);\n' ...
         '         the %.0f m time uses the load-sensitive axle model from rest at the line.\n' ...
         '         Provisional: launch traction use (%.2f) and driveline efficiency (%.2f).\n'], ...
        cfg.s_accel, p.k_trac, p.eta_dt);

% Outputs (read by vd_selftest and vd_golden)
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

if vd_plots()
try
    plot_gg(p);
    fprintf('Saved plots/gg_envelope.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
if vd_plots()
try
    gg_surface(p);
    fprintf('Saved plots/gg_surface.png\n');
catch e
    fprintf('Plot not saved (g-g-V surface):\n%s\n', getReport(e));
end
end
end


function gg_surface(p)
% 3-D g-g-V envelope of the per-axle combined model: at each speed the
% (ay, ax) boundary from ay_limit + ax_combined, i.e. the constraint surface
% the lap sim runs on. Solid = the car as configured; wireframe = the car
% with the target aero package (p.scenario.ClA_target / CdA_target).
[LAT,  LON,  VV,  idx] = gg_shell(p);
pt = p;  pt.ClA = p.scenario.ClA_target;  pt.CdA = p.scenario.CdA_target;
[LATt, LONt, VVt] = gg_shell(pt);

% The shell must be mirror-symmetric about ay = 0; warn if it is not.
fold = @(M, sgn) max(max(abs(M + sgn*fliplr(M))));
res  = max([ fold(LAT(:,idx.accel), +1), fold(LON(:,idx.accel), -1), ...
             fold(LAT(:,idx.brake), +1), fold(LON(:,idx.brake), -1) ]);
if res > 1e-6
    warning('run_gg_targets:ggvAsymmetry', ...
        'The g-g-V surface is not symmetric left to right (mismatch %.1e m/s^2); check gg_shell.', res);
end

f  = figure('Position',[60 60 900 720],'Color','w');   % left open for rotating
ax = axes(f);
hs = surf(ax, LAT, LON, VV, VV, 'FaceAlpha', 1, ...
     'EdgeColor', [0.15 0.15 0.15], 'EdgeAlpha', 0.22, 'LineWidth', 0.3);
hold(ax,'on');
for i = 1:3:size(LAT,1)                          % bolder speed rings
    plot3(ax, LAT(i,:), LON(i,:), VV(i,:), 'k-', 'LineWidth', 0.6);
end
ht = mesh(ax, LATt, LONt, VVt, 'FaceAlpha', 0, ...
     'EdgeColor', [0.75 0.15 0.15], 'EdgeAlpha', 0.45, 'LineWidth', 0.4);
legend(ax, [hs ht], {sprintf('%s as configured (ClA %.2f)', p.car, p.ClA), ...
        sprintf('target package (ClA %.2f)', p.scenario.ClA_target)}, ...
       'Location','northeast', 'FontSize', 8, 'AutoUpdate','off');
colormap(ax, parula);
cb = colorbar(ax); cb.Label.String = 'speed [m/s]';
xlabel(ax,'lateral acceleration [m/s^2]'); ylabel(ax,'longitudinal acceleration [m/s^2]'); zlabel(ax,'speed [m/s]');
title(ax, sprintf('%s g-g-V envelope (per-axle combined model)', p.car));
xl = max(abs([LAT(:); LATt(:)])) * 1.08;
xlim(ax, [-xl xl]);                              % centered about lat = 0
daspect(ax, [1 1 0.6]);                          % equal lat/long scale
zlim(ax, [0 p.v_max*1.03]);
view(ax, 145, 20); grid(ax,'on');                % accel side toward the viewer
rotate3d(f, 'on');
outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'gg_surface.png'));
end


function [LAT, LON, VV, idx] = gg_shell(p)
% Closed (lat, long) boundary loop at each speed: accel side left -> right,
% then brake side right -> left. The ay = 0 points appear on both sides and
% the ay = +-max points are duplicated, so the surface closes cleanly.
vg = linspace(0, p.v_max, 18);
nr = 13;  r = linspace(0, 1, nr);
ncol = 4*nr - 2;
LAT = zeros(numel(vg), ncol+1);  LON = LAT;  VV = LAT;
for i = 1:numel(vg)
    ayl = ay_limit(p, vg(i));
    axa = arrayfun(@(rr) ax_combined(p, vg(i), rr*ayl, 'accel'), r);
    axb = arrayfun(@(rr) ax_combined(p, vg(i), rr*ayl, 'brake'), r);
    ay_loop = [-fliplr(r),   r(2:end),   fliplr(r),    -r(2:end)] * ayl * p.g;
    ax_loop = [ fliplr(axa), axa(2:end), -fliplr(axb), -axb(2:end)] * p.g;
    LAT(i,:) = [ay_loop, ay_loop(1)];
    LON(i,:) = [ax_loop, ax_loop(1)];
    VV(i,:)  = vg(i);
end
% Column indices for the symmetry check.
idx = struct('accel', 1:2*nr-1, 'brake', 2*nr:ncol, ...
             'apex_accel', nr, 'apex_brake', 3*nr-1);
end


function vc = crossover_speed(p)
% Lowest speed at which the motor is power-limited (its base speed).
v_sweep = linspace(1, 45, 450);
G   = gg_envelope(p, v_sweep);
idx = find(G.power_limited, 1);
if isempty(idx), vc = v_sweep(end); else, vc = v_sweep(idx); end
end

function plot_gg(p)
% Left: point-mass edges vs speed. Right: point-mass g-g at a few speeds.
v_sweep = linspace(0.5, 35, 60);
G = gg_envelope(p, v_sweep);
f = figure('Visible', 'off', 'Position', [100 100 1000 420]);

subplot(1, 2, 1); hold on;
plot(v_sweep, G.ay,       '-', 'LineWidth', 1.6);
plot(v_sweep, G.ax_accel, '-', 'LineWidth', 1.6);
plot(v_sweep, G.ax_brake, '-', 'LineWidth', 1.6);
xlabel('speed [m/s]'); ylabel('limit [g]'); grid on;
legend('lateral', 'acceleration', 'braking', 'Location', 'east');
title('Point-mass capability vs speed');

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
title('Point-mass g-g at several speeds');

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'gg_envelope.png'));
close(f);
end