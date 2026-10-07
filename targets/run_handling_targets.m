function out = run_handling_targets(p)
% RUN_HANDLING_TARGETS  Linear bicycle-model targets: understeer gradient,
% critical/characteristic speed, yaw-rate gain and yaw response time.
%
%   out = run_handling_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_handling_targets(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

k = vd_const();
LBF_DEG_TO_N_RAD = k.LBF_DEG_TO_N_RAD;

% Axle cornering stiffness at the static loads (understeer_at at ax = 0), so
% this study and run_stability_targets read the tire the same way.
[~, i0] = understeer_at(p, 0);
Ca_axle_f = i0.Ca_f;
Ca_axle_r = i0.Ca_r;

B = bicycle_model(p, Ca_axle_f, Ca_axle_r, linspace(1, p.v_max, 100));

gain_15   = interp1(B.v, B.yaw_gain, 15);
gain_vmax = B.yaw_gain(end);
[~, i15] = min(abs(B.v - 15));

fprintf('\nHandling - %s  (linear bicycle model, top speed %.1f m/s)\n', p.car, p.v_max);
vd_row('Understeer gradient', sprintf('%+.3f deg/g', B.K_deg), balance_word(B.K_deg));
if isfinite(B.v_crit)            % oversteer: speed where the car goes unstable
    vd_row('Critical speed (unstable above)', sprintf('%.0f m/s', B.v_crit), ...
           sprintf('%.1fx top speed', B.v_crit/p.v_max), ternary(B.v_crit > p.v_max, 'OK', 'TOO LOW'));
elseif isfinite(B.v_char)        % understeer: speed of maximum yaw-rate gain
    vd_row('Characteristic speed (max yaw gain)', sprintf('%.0f m/s', B.v_char));
else
    vd_row('Neutral steer', '-', 'no critical or characteristic speed');
end
vd_row('Yaw-rate gain at 15 m/s [(deg/s)/deg]', sprintf('%.2f', gain_15), ...
       sprintf('neutral car %.2f', 15/p.L));
vd_row('Yaw-rate gain at top speed [(deg/s)/deg]', sprintf('%.2f', gain_vmax), ...
       sprintf('neutral car %.2f', p.v_max/p.L));
vd_row('Yaw response time, 15 m/s / top speed', ...
       sprintf('%.0f / %.0f ms', 1000*B.tau_slow(i15), 1000*B.tau_slow(end)));
vd_row('Yaw damping ratio', sprintf('%.2f-%.2f', min(B.zeta_eq), max(B.zeta_eq)), ...
       ternary(min(B.zeta_eq) >= 1, 'no overshoot', 'overshoots'));
vd_row('Axle cornering stiffness, front / rear', ...
       sprintf('%.0f / %.0f lbf/deg', Ca_axle_f/LBF_DEG_TO_N_RAD, Ca_axle_r/LBF_DEG_TO_N_RAD));
fprintf(['Assumes: static axle loads, no downforce, linear tires (valid to about 0.4 g),\n' ...
         '         no lateral load transfer, roll or compliance steer. Provisional: yaw\n' ...
         '         inertia (dynamic index %.2f, %.0f kg*m^2) and tire stiffness scale %.2f.\n'], ...
        p.DI, p.Izz, p.lambda_Ca);
out.v_crit = B.v_crit;
out.v_char = B.v_char;

out.Ca_coef_source = 1;   % 1 = cornering stiffness read from the tire artifact
out.Ca_axle_f   = Ca_axle_f;
out.Ca_axle_r   = Ca_axle_r;
out.K_deg_per_g = B.K_deg;
out.yaw_gain_15 = gain_15;
out.tau_vmax    = B.tau_slow(end);
out.Izz         = p.Izz;

if vd_plots()
try
    make_plot(p, B);
    fprintf('Saved plots/handling_response.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

function make_plot(p, B)
% Left: steady-state yaw-rate gain vs a neutral car. Right: transient yaw response.
f = figure('Visible', 'off', 'Position', [100 100 1000 420]);

subplot(1,2,1); hold on;
plot(B.v, B.yaw_gain, '-', 'LineWidth', 1.8);
plot(B.v, B.v./p.L, '--', 'LineWidth', 1.2);
xline(p.v_max, ':');
xlabel('speed [m/s]'); ylabel('yaw-rate gain r/\delta [(deg/s)/deg]');
legend(sprintf('this car (K=%+.3f deg/g)', B.K_deg), 'neutral (v/L)', ...
       'v_{max}', 'Location', 'northwest');
title('Steady-state yaw-rate gain'); grid on;

subplot(1,2,2);
yyaxis left;
plot(B.v, 1000*B.tau_slow, '-', 'LineWidth', 1.8);
ylabel('slowest-pole time constant [ms]');
yyaxis right;
plot(B.v, B.zeta_eq, '--', 'LineWidth', 1.2);
ylabel('equivalent damping ratio \zeta [-]');
xlabel('speed [m/s]'); grid on;
title('Transient yaw response vs speed');

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'handling_response.png'));
close(f);
end

function s = balance_word(K)
if     K >  0.1, s = 'understeer';
elseif K < -0.1, s = 'oversteer';
else,            s = 'near neutral';
end
end

function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end
