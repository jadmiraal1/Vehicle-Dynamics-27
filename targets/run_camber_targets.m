function out = run_camber_targets(p)
% RUN_CAMBER_TARGETS  What camber is worth, on this tire, at this car's loads.
%
%   out = run_camber_targets()     the active car (vd_car / cars/config_<CAR>.m)
%   out = run_camber_targets(p)    an explicit params struct (build it with vd_set)
%
% Issues the camber targets the suspension subteam needs: how much camber the
% outside wheel should see at the cornering limit, and what it costs or buys.
%
% HOW TO READ THIS
% ---------------
% Camber does three things at once and they fight (tire/camber_fit.m explains
% each). The sweep below is the only honest way to ask which one wins, because
% the answer depends on load and this car's outer tire is heavily loaded.
%
% The sweep applies camber the way a real suspension does: the OUTSIDE wheel
% gets the helpful lean (+gamma) and the INSIDE wheel, on the same links, gets
% the opposite (-gamma). Reporting only the outside wheel would flatter camber.

if nargin < 1 || isempty(p), p = vehicle_params(); end

R_SKID   = 9.125;                 % [m] FSAE skidpad path radius
V_CORNER = 15;                    % [m/s] speed the sweep is evaluated at
GAM      = 0:0.5:4;               % [deg] camber magnitude on the outside wheel

if ~isfield(p, 'camber_status') || ~strcmp(p.camber_status, 'ok')
    fprintf(2, ['\nCAMBER TARGETS: the tire artifact has no camber fit ' ...
                '(%s).\nRun build_tire_coeffs, then re-run this.\n'], ...
            ternary_local(isfield(p,'camber_status'), '', 'field missing'));
    out = struct('status', 'no camber data');
    return
end

% --- 1. the tire on its own, at the loads this car actually puts on it ----
G0        = axle_grip(vd_set(p, 'camber_deg', zeroc()), V_CORNER);
Fz_out_f  = G0.Fz.fo / 4.44822;   % [lbf] outer front tire at the limit
Fz_out_r  = G0.Fz.ro / 4.44822;
Fz_in_f   = G0.Fz.fi / 4.44822;

fprintf('\nCAMBER TARGETS  (tire %s, %s)\n', p.tire_id, p.camber_basis);
fprintf('Camber fit box: |gamma| <= %.0f deg, %.0f-%.0f lbf.\n', ...
        p.camber_gamma_max_deg, p.camber_Fz_min_lbf, p.camber_Fz_max_lbf);
fprintf('At the limit this car loads its tires to %.0f lbf (front outer) / %.0f lbf (rear outer),\n', ...
        Fz_out_f, Fz_out_r);
fprintf('and unloads the inner front to %.0f lbf.%s\n', Fz_in_f, ...
        ternary_local(max(Fz_out_f, Fz_out_r) > p.camber_Fz_max_lbf, ...
        ' The outer tire is BEYOND the fitted load range - camber terms are clamped at the edge.', ''));

% The tire on its own, at both ends of the fitted box plus the load that
% actually matters. The sign convention is doing real work here: +gamma is the
% outside wheel (leaning into the corner), -gamma is the inside wheel on the
% same suspension setting.
fprintf('\n%-8s | %-21s | %-21s | %9s %10s\n', '', 'peak factor, OUTSIDE', 'peak factor, INSIDE', 'stiff x', 'thrust');
fprintf('%-8s | %10s %10s | %10s %10s | %9s %10s\n', 'gamma', ...
        sprintf('%.0f lbf', p.camber_Fz_min_lbf), sprintf('%.0f lbf', Fz_out_f), ...
        sprintf('%.0f lbf', p.camber_Fz_min_lbf), sprintf('%.0f lbf', Fz_out_f), ...
        '[-]', '[deg slip]');
for g = [1 2 3 4]
    fD_o_lo = tire_camber(p, p.camber_Fz_min_lbf, +g);
    fD_o_hi = tire_camber(p, Fz_out_f,            +g);
    fD_i_lo = tire_camber(p, p.camber_Fz_min_lbf, -g);
    fD_i_hi = tire_camber(p, Fz_out_f,            -g);
    [~, fC, dSH] = tire_camber(p, Fz_out_f, +g);
    fprintf('%-8.1f | %10.3f %10.3f | %10.3f %10.3f | %9.3f %10.2f\n', ...
            g, fD_o_lo, fD_o_hi, fD_i_lo, fD_i_hi, fC, dSH);
end
fprintf(['Read the two peak columns together: camber pays on a LIGHTLY loaded tire\n' ...
         'and charges on a heavily loaded one, and at the cornering limit this car\n' ...
         'puts almost all of its load on the tire that is being charged.\n']);

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

fprintf('\nLateral limit at %.0f m/s (outside +gamma, inside -gamma, both axles)\n', V_CORNER);
fprintf('%-8s %10s %10s\n', 'gamma', 'ay [g]', 'vs 0 deg');
for i = 1:2:numel(GAM)
    fprintf('%-8.1f %10.3f %+9.2f%%\n', GAM(i), ay(i), 100*(ay(i)/ay(1) - 1));
end

fprintf('\nT-CAM  best outside-wheel camber : %+.1f deg  (ay %.3f g, %+.2f%% vs zero camber)\n', ...
        gam_best, ay_best, 100*(ay_best/ay(1) - 1));
fprintf('T-CAMS camber sensitivity        : %+.4f g per deg near the optimum\n', ...
        local_slope(GAM, ay, gam_best));

% --- 3. balance: camber is a balance knob, not only a grip knob ----------
% Front-only and rear-only camber move the limiting axle. That is the lever the
% suspension team actually has, because front and rear camber gain are set
% independently.
lim0 = axle_grip(vd_set(p, 'camber_deg', zeroc()), V_CORNER).limiting;
lim2 = axle_grip(vd_set(p, 'camber_deg', bothc(2, 0)), V_CORNER).limiting;
fprintf('T-CAMB balance                   : limiting axle %s at 0 deg -> %s with 2 deg front only\n', ...
        lim0, lim2);
fprintf('       front-only 2 deg: ay %+.2f%%   rear-only 2 deg: ay %+.2f%%\n', ...
        100*(interp1(GAM, ay_f, 2)/ay(1) - 1), 100*(interp1(GAM, ay_r, 2)/ay(1) - 1));

% --- 4. skidpad ----------------------------------------------------------
v_skid = @(a) sqrt(a * p.g * R_SKID);
t_skid = @(a) 2*pi*R_SKID / v_skid(a);
fprintf('T-CAMK skidpad                   : %.3f s at 0 deg -> %.3f s at %+.1f deg (%+.3f s)\n', ...
        t_skid(ay(1)), t_skid(ay_best), gam_best, t_skid(ay_best) - t_skid(ay(1)));

% --- caveats -------------------------------------------------------------
fprintf(['\nWhat this study CANNOT see: the thrust column is a slip-angle offset, so\n' ...
         'camber still adds force BELOW the peak - that is turn-in response and\n' ...
         'steering feel, and a steady-state limit model gives it no credit. Read\n' ...
         'the ay numbers as "what camber costs at the limit", not "camber is bad".\n']);
fprintf(['Caveat: quasi-static axle model, camber is an INPUT (no suspension\n' ...
         '        kinematics yet - nothing here knows your camber gain). Camber\n' ...
         '        terms fitted from TTC belt data with NO negative-inclination\n' ...
         '        sweep: the adverse branch assumes a symmetric tire. Camber and\n' ...
         '        test order are collinear in the TTC run structure, so a slow\n' ...
         '        test-order drift would land inside these coefficients.\n' ...
         '        Directional until checked on track. lambda_Ca=%.2f, mu_derate=%.2f prov.\n'], ...
        p.lambda_Ca, p.mu_derate);

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

try
    make_plot(p, GAM, ay, ay_f, ay_r);
    fprintf('Plot written: camber_targets.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end

% =========================================================================
function c = zeroc()
c = struct('fo', 0, 'fi', 0, 'ro', 0, 'ri', 0);
end

function c = bothc(gf, gr)
% Outside wheel gets the helpful lean, inside wheel the opposite - one
% suspension setting, two different tires.
c = struct('fo', +gf, 'fi', -gf, 'ro', +gr, 'ri', -gr);
end

function s = local_slope(x, y, x0)
% Central difference about x0, on the sweep grid.
[~, i] = min(abs(x - x0));
lo = max(1, i-1);  hi = min(numel(x), i+1);
if hi == lo, s = 0; return; end
s = (y(hi) - y(lo)) / (x(hi) - x(lo));
end

function s = ternary_local(c, a, b)
if c, s = a; else, s = b; end
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
