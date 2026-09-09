function out = run_gg_targets(p)
% RUN_GG_TARGETS  Skidpad / accel / braking targets from the g-g-V envelope.
%
%   out = run_gg_targets()      the active car, from vd_car / cars/config_<CAR>.m
%   out = run_gg_targets(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = run_gg_targets(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)

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

if vd_plots()
try
    plot_gg(p);
    fprintf('\nPlot written: gg_envelope.png\n');
catch e
    fprintf('\n[plot skipped: %s]\n', e.message);
end
end
if vd_plots()
try
    gg_surface(p);
    fprintf('Plot written: gg_surface.png\n\n');
catch e
    fprintf('[g-g-V surface FAILED - full report follows]\n%s\n\n', getReport(e));
end
end
end


function gg_surface(p)
% 3D g-g-V envelope with aero (per-axle combined model). At each speed the
% (ay, ax) boundary comes from ay_limit + ax_combined - the bottle IS the
% constraint surface the lap sim runs on. Accel side faces the viewer.
% Solid = the CURRENT car (measured ClA); wireframe = the TR27 TARGET package
% (p.scenario.ClA_target) - the envelope the car being designed will face.
[LAT,  LON,  VV,  idx] = gg_shell(p);
pt = p;  pt.ClA = p.scenario.ClA_target;  pt.CdA = p.scenario.CdA_target;
[LATt, LONt, VVt] = gg_shell(pt);

% Silent tripwire: fold each nose about its ay = 0 apex (lat antisymmetric,
% long symmetric). Nonzero residual = real data asymmetry; only then speak up.
fold = @(M, sgn) max(max(abs(M + sgn*fliplr(M))));
res  = max([ fold(LAT(:,idx.accel), +1), fold(LON(:,idx.accel), -1), ...
             fold(LAT(:,idx.brake), +1), fold(LON(:,idx.brake), -1) ]);
if res > 1e-6
    warning('run_gg_targets:ggvAsymmetry', ...
        'g-g-V mirror residual %.3e m/s^2 - the shell DATA is asymmetric; inspect gg_shell.', res);
end

f  = figure('Visible','on','Position',[60 60 900 720],'Color','w');   % on-screen for inspection
ax = axes(f);
hs = surf(ax, LAT, LON, VV, VV, 'FaceAlpha', 1, ...
     'EdgeColor', [0.15 0.15 0.15], 'EdgeAlpha', 0.22, 'LineWidth', 0.3);
hold(ax,'on');
for i = 1:3:size(LAT,1)                          % bolder speed rings
    plot3(ax, LAT(i,:), LON(i,:), VV(i,:), 'k-', 'LineWidth', 0.6);
end
ht = mesh(ax, LATt, LONt, VVt, 'FaceAlpha', 0, ...
     'EdgeColor', [0.75 0.15 0.15], 'EdgeAlpha', 0.45, 'LineWidth', 0.4);
legend(ax, [hs ht], {sprintf('current car (ClA %.2f, measured)', p.ClA), ...
        sprintf('TR27 target package (ClA %.1f)', p.scenario.ClA_target)}, ...
       'Location','northeast', 'FontSize', 8, 'AutoUpdate','off');
colormap(ax, parula);
cb = colorbar(ax); cb.Label.String = 'speed [m/s]';
xlabel(ax,'lat acc [m/s^2]'); ylabel(ax,'long acc [m/s^2]'); zlabel(ax,'speed [m/s]');
title(ax, 'g-g-V envelope with aero (per-axle combined model)');
xl = max(abs([LAT(:); LATt(:)])) * 1.08;
xlim(ax, [-xl xl]);                              % centered about lat = 0
xticks(ax, -20:5:20);  yticks(ax, -25:5:10);     % unambiguous, fixed ticks
daspect(ax, [1 1 0.6]);                          % equal lat/long scale; speed scaled to shape
zlim(ax, [0 p.v_max*1.03]);                      % grounded: base sits on the floor
view(ax, 145, 20); grid(ax,'on');                % accel side toward the viewer
rotate3d(f, 'on');
outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'gg_surface.png'));   % figure stays open - rotate/inspect, close manually
end


function [LAT, LON, VV, idx] = gg_shell(p)
% Build the closed (lat, long) boundary loop at each speed for the g-g-V surface.
% Loop order: accel nose left -> centre -> right, then brake nose right -> centre
% -> left. r = 0 (the ay = 0 apex) sits on BOTH noses, so max accel and max brake
% are both ON the surface; the lat = +-1 nodes are duplicated on purpose - they
% are the vertical edges where the accel and brake sides meet.
vg = linspace(0, p.v_max, 18);       % from standstill: the base ring IS the static g-g
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
% Column bookkeeping for the symmetry check and the lat = 0 centrelines.
idx = struct('accel', 1:2*nr-1, 'brake', 2*nr:ncol, ...
             'apex_accel', nr, 'apex_brake', 3*nr-1);
end


function [t, v] = accel_event(p, dist)
% Standing-start time by forward integration in fixed velocity steps.
% Launch uses ax_limit (p.long_model): the loaded rear tire is priced by
% mu_of_load, not constant mu - slightly slower and honest.
%#ok<*DEFNU>
v = 0; x = 0; t = 0;
dv = 0.005;                                    % [m/s]
axf = [];                                      % coarse LUT: ax_limit is a solver
vg  = linspace(0, p.v_max, 60);
axf = arrayfun(@(vv) ax_limit(p, vv, 'accel'), vg);
while x < dist
    if v >= p.v_max                            % rev limiter: coast at v_max
        t = t + (dist - x) / p.v_max;
        v = p.v_max;
        break;
    end
    a = interp1(vg, axf, min(max(v,0), p.v_max)) * p.g;
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