function out = run_camber_targets(p)
% RUN_CAMBER_TARGETS  What camber is worth on this tire at this car's loads.
%
%   out = run_camber_targets()     the active car (vd_car / cars/config_<CAR>.m)
%   out = run_camber_targets(p)    an explicit params struct (build it with vd_set)
%
% Camber changes peak grip, cornering stiffness and camber thrust at once
% (tire/camber_fit.m), and the net effect depends on tire load, so it is
% swept through the axle model. The sweep applies camber as a suspension
% does: the OUTSIDE wheel gets the helpful lean (+gamma) and the INSIDE
% wheel the opposite (-gamma).

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

R_SKID   = fsae_rules().skidpad_R_m;   % [m] skidpad path radius
V_CORNER = 15;                    % [m/s] speed the sweep is evaluated at
GAM      = 0:0.5:4;               % [deg] camber magnitude on the outside wheel

if ~isfield(p, 'camber_status') || ~strcmp(p.camber_status, 'ok')
    fprintf(2, '\nCamber: the tire file has no camber fit. Run build_tire_coeffs, then re-run this.\n');
    out = struct('status', 'no camber data');
    return
end

% --- 1. the tire on its own, at the loads this car actually puts on it ----
G0        = axle_grip(vd_set(p, 'camber_deg', zeroc()), V_CORNER);
N_PER_LBF = vd_const().N_PER_LBF;
Fz_out_f  = G0.Fz.fo / N_PER_LBF;   % [lbf] outer front tire at the limit
Fz_out_r  = G0.Fz.ro / N_PER_LBF;
Fz_in_f   = G0.Fz.fi / N_PER_LBF;

fprintf('\nCamber - %s  (tire %s)\n', p.car, p.tire_id);
fprintf('  Tire loads at the cornering limit (%.0f m/s): outer front %.0f lbf, outer rear %.0f lbf,\n', ...
        V_CORNER, Fz_out_f, Fz_out_r);
fprintf('  inner front %.0f lbf. Camber data covers 0-%.0f deg and %.0f-%.0f lbf', ...
        Fz_in_f, p.camber_gamma_max_deg, p.camber_Fz_min_lbf, p.camber_Fz_max_lbf);
if max(Fz_out_f, Fz_out_r) > p.camber_Fz_max_lbf
    fprintf(';\n  above %.0f lbf the camber effect is held at its %.0f lbf value.\n', ...
            p.camber_Fz_max_lbf, p.camber_Fz_max_lbf);
else
    fprintf('.\n');
end

% The tire alone: +gamma is the outside wheel, -gamma the inside wheel on the
% same suspension setting, at the light edge of the fit and at the outer
% front tire's load.
fprintf('\n  Peak-grip factor from camber (1.000 = no change)\n');
fprintf('    %-8s %-21s %-21s %-10s %s\n', 'camber', 'outside wheel', 'inside wheel', 'stiffness', 'thrust');
fprintf('    %-8s %-10s %-10s %-10s %-10s %-10s %s\n', '[deg]', ...
        sprintf('%.0f lbf', p.camber_Fz_min_lbf), sprintf('%.0f lbf', Fz_out_f), ...
        sprintf('%.0f lbf', p.camber_Fz_min_lbf), sprintf('%.0f lbf', Fz_out_f), ...
        'factor', '[deg slip]');
for g = [1 2 3 4]
    fD_o_lo = tire_camber(p, p.camber_Fz_min_lbf, +g);
    fD_o_hi = tire_camber(p, Fz_out_f,            +g);
    fD_i_lo = tire_camber(p, p.camber_Fz_min_lbf, -g);
    fD_i_hi = tire_camber(p, Fz_out_f,            -g);
    [~, fC, dSH] = tire_camber(p, Fz_out_f, +g);
    fprintf('    %-8.1f %-10.3f %-10.3f %-10.3f %-10.3f %-10.3f %.2f\n', ...
            g, fD_o_lo, fD_o_hi, fD_i_lo, fD_i_hi, fC, dSH);
end
fprintf(['  On this tire camber helps a lightly loaded tire and costs a heavily loaded\n' ...
         '  one, and at the cornering limit most of the load is on the outside tire.\n']);

% --- 2. the car: lateral limit vs camber ---------------------------------
ay   = zeros(size(GAM));
ay_f = zeros(size(GAM));       % camber on the FRONT axle only
ay_r = zeros(size(GAM));       % ... and the REAR axle only
for i = 1:numel(GAM)
    g = GAM(i);
    ay(i)   = axle_grip(vd_set(p, 'camber_deg', bothc(g, g)), V_CORNER).ay_lim_g;
    ay_f(i) = axle_grip(vd_set(p, 'camber_deg', bothc(g, 0)), V_CORNER).ay_lim_g;
    ay_r(i) = axle_grip(vd_set(p, 'camber_deg', bothc(0, g)), V_CORNER).ay_lim_g;
end
[ay_best, ib] = max(ay);
gam_best      = GAM(ib);

fprintf('\n  Lateral limit at %.0f m/s vs camber (outside wheel leaning in, inside wheel out)\n', V_CORNER);
fprintf('    %-8s %-12s %s\n', '[deg]', 'lateral [g]', 'change');
for i = 1:2:numel(GAM)
    fprintf('    %-8.1f %-12.3f %+.2f %%\n', GAM(i), ay(i), 100*(ay(i)/ay(1) - 1));
end
fprintf('\n');
vd_row('Best camber, both axles', sprintf('%+.1f deg', gam_best), ...
       sprintf('%.3f g, %+.2f %%', ay_best, 100*(ay_best/ay(1) - 1)));
vd_row('Lateral limit change per deg near best', ...
       sprintf('%+.4f g/deg', local_slope(GAM, ay, gam_best)));

% --- 3. balance: camber is a balance knob, not only a grip knob ----------
% Front-only and rear-only camber shift the limit between axles; front and
% rear camber are set independently.
lim0 = axle_grip(vd_set(p, 'camber_deg', zeroc()), V_CORNER).limiting;
lim2 = axle_grip(vd_set(p, 'camber_deg', bothc(2, 0)), V_CORNER).limiting;
vd_row('Axle that saturates first, 0 / 2 deg front', sprintf('%s / %s', lim0, lim2));
vd_row('Lateral limit, 2 deg front only', ...
       sprintf('%+.2f %%', 100*(interp1(GAM, ay_f, 2)/ay(1) - 1)));
vd_row('Lateral limit, 2 deg rear only', ...
       sprintf('%+.2f %%', 100*(interp1(GAM, ay_r, 2)/ay(1) - 1)));

% --- 4. skidpad ----------------------------------------------------------
v_skid = @(a) sqrt(a * p.g * R_SKID);
t_skid = @(a) 2*pi*R_SKID / v_skid(a);
vd_row('Skidpad lap time at best camber', sprintf('%.3f s', t_skid(ay_best)), ...
       sprintf('%+.3f s vs 0 deg', t_skid(ay_best) - t_skid(ay(1))));

fprintf(['Assumes: steady-state axle model with camber as an input (no suspension\n' ...
         '         kinematics). Camber terms come from TTC data with no negative-inclination\n' ...
         '         sweep, so the tire is assumed symmetric, and camber is confounded with\n' ...
         '         test order. Not credited: camber thrust below the grip limit (turn-in\n' ...
         '         response), so read these as what camber costs at the limit.\n']);

out.gamma_deg     = GAM;
out.ay_g          = ay;
out.ay_front_only = ay_f;
out.ay_rear_only  = ay_r;
out.gamma_best    = gam_best;
out.ay_best       = ay_best;
out.ay_zero       = ay(1);
out.d_skidpad_s   = t_skid(ay_best) - t_skid(ay(1));
out.Fz_outer_f_lbf = Fz_out_f;
out.beyond_fit_box = max(Fz_out_f, Fz_out_r) > p.camber_Fz_max_lbf;

if vd_plots()
try
    make_plot(p, GAM, ay, ay_f, ay_r);
    fprintf('Saved plots/camber_targets.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

% =========================================================================
function c = zeroc()
c = struct('fo', 0, 'fi', 0, 'ro', 0, 'ri', 0);
end

function c = bothc(gf, gr)
% Outside wheel +gamma, inside wheel -gamma (one suspension setting).
c = struct('fo', +gf, 'fi', -gf, 'ro', +gr, 'ri', -gr);
end

function s = local_slope(x, y, x0)
% Central difference about x0, on the sweep grid.
[~, i] = min(abs(x - x0));
lo = max(1, i-1);  hi = min(numel(x), i+1);
if hi == lo, s = 0; return; end
s = (y(hi) - y(lo)) / (x(hi) - x(lo));
end

function make_plot(p, GAM, ay, ay_f, ay_r)
f = figure('Visible', 'off', 'Position', [100 100 1000 420]);

subplot(1,2,1); hold on;
plot(GAM, ay,   '-',  'LineWidth', 1.8);
plot(GAM, ay_f, '--', 'LineWidth', 1.2);
plot(GAM, ay_r, ':',  'LineWidth', 1.4);
xlabel('outside-wheel camber \gamma [deg, + = leaning into the corner]');
ylabel('lateral limit a_y [g]');
legend('both axles', 'front only', 'rear only', 'Location', 'best');
title(sprintf('%s - lateral limit vs camber', p.car)); grid on;

subplot(1,2,2); hold on;
Fz = linspace(p.camber_Fz_min_lbf, p.camber_Fz_max_lbf, 60);
for g = [1 2 3 4]
    plot(Fz, tire_camber(p, Fz, g), '-', 'LineWidth', 1.4);
end
yline(1, ':');
xlabel('tire vertical load F_z [lbf]');
ylabel('peak-grip factor f_D [-]');
legend('\gamma = 1', '\gamma = 2', '\gamma = 3', '\gamma = 4', 'Location', 'best');
title('Camber helps light tires, hurts loaded ones'); grid on;

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'camber_targets.png'));
close(f);
end
