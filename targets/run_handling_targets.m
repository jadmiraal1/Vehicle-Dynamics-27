function out = run_handling_targets(p)
% RUN_HANDLING_TARGETS  Bicycle-model targets: K, stability speed, yaw gain/response.
%
%   out = run_handling_targets()      the active car, from vd_car / cars/config_<CAR>.m
%   out = run_handling_targets(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = run_handling_targets(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)

% Cornering stiffness comes from the artifact (p.Ca_coef, loaded from
N_PER_LBF = 4.44822;
LBF_DEG_TO_N_RAD = N_PER_LBF * 180/pi;

% Axle stiffness at static loads, via understeer_at's shared evaluator (ax=0
% gives exactly the static axle loads), so this and the stability study agree.
[~, i0] = understeer_at(p, 0);
Ca_axle_f = i0.Ca_f;
Ca_axle_r = i0.Ca_r;

B = bicycle_model(p, Ca_axle_f, Ca_axle_r, linspace(1, p.v_max, 100));

fprintf('\nSTEADY-STATE HANDLING (bicycle model, static loads, linear tires)\n');
fprintf('Axle Ca: front %.0f N/rad (%.0f lbf/deg), rear %.0f N/rad (%.0f lbf/deg)\n', ...
        Ca_axle_f, Ca_axle_f/LBF_DEG_TO_N_RAD, Ca_axle_r, Ca_axle_r/LBF_DEG_TO_N_RAD);
fprintf('T-USG  understeer gradient : K = %+.3f deg/g  (%s)\n', ...
        B.K_deg, balance_word(B.K_deg));
if isfield(B, 'v_crit')
    fprintf('T-VCR  critical speed      : %.0f m/s (v_max %.1f -> margin %.1fx)\n', ...
            B.v_crit, p.v_max, B.v_crit/p.v_max);
    out.v_crit = B.v_crit;
else
    fprintf('T-VCH  characteristic speed: %.0f m/s\n', B.v_char);
    out.v_char = B.v_char;
end

gain_15   = interp1(B.v, B.yaw_gain, 15);
gain_vmax = B.yaw_gain(end);
fprintf('T-YRG  yaw-rate gain       : %.2f (deg/s)/deg @ 15 m/s (neutral %.2f), %.2f @ v_max (neutral %.2f)\n', ...
        gain_15, 15/p.L, gain_vmax, p.v_max/p.L);

[~, i15] = min(abs(B.v - 15));
fprintf('T-YAW  yaw response        : tau %.0f ms @ 15 m/s -> %.0f ms @ v_max, zeta %.2f-%.2f\n', ...
        1000*B.tau_slow(i15), 1000*B.tau_slow(end), min(B.zeta_eq), max(B.zeta_eq));
fprintf('       (zeta >= 1: overdamped, no yaw oscillation; Izz = %.0f kg*m^2, DI %.2f prov.)\n', ...
        p.Izz, p.DI);
fprintf('Caveat: static axle loads, linear tires (valid to ~0.4 g), no load\n');
fprintf('        transfer or roll stiffness (mid-tier moves K). lambda_Ca=%.2f prov.\n', ...
        p.lambda_Ca);

out.Ca_coef_source = 1;   % 1 = from tire_coeffs.mat artifact (not a live fit)
out.Ca_axle_f   = Ca_axle_f;
out.Ca_axle_r   = Ca_axle_r;
out.K_deg_per_g = B.K_deg;
out.yaw_gain_15 = gain_15;
out.tau_vmax    = B.tau_slow(end);
out.Izz         = p.Izz;

try
    make_plot(p, B);
    fprintf('Plot written: handling_response.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end

function make_plot(p, B)
% Left: steady-state gain vs neutral. Right: transient yaw response.
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
elseif K < -0.1, s = 'oversteer - check margin';
else,            s = 'near neutral';
end
end
