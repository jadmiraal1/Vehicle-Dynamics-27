function out = run_handling_targets()
% RUN_HANDLING_TARGETS  Handling targets from the bicycle model.
% T-USG understeer gradient | T-VCR/VCH stability speed | T-YRG yaw-rate
% gain | T-YAW transient yaw response. Model: bicycle_model.m; theory:
% VD_physics_reference.md, sec 10.

p = vehicle_params();

% Cornering stiffness comes from the ARTIFACT (p.Ca_coef, loaded from
% tire_coeffs.mat), NOT from a live pacejka_fit() call.
%
% This used to be `evalc('T = pacejka_fit();')` - a runtime re-fit. That
% quietly defeated the whole point of the generated-artifact pattern: edit
% pacejka_fit.m and the understeer gradient would change on the next run, with
% no rebuild, no re-issue, and no staleness warning. Grip was decoupled from
% the car but STIFFNESS was not. Now both come from the same promoted artifact,
% so the staleness gate covers K, yaw gain and yaw response too.
N_PER_LBF = 4.44822;
LBF_DEG_TO_N_RAD = N_PER_LBF * 180/pi;

% Axle stiffness: 2 x per-tire Ca at static per-tire load, belt->track scaled
Fz_tire_f = p.Wf_static / 2 / N_PER_LBF;
Fz_tire_r = p.Wr_static / 2 / N_PER_LBF;
Ca_axle_f = 2 * polyval(p.Ca_coef, Fz_tire_f) * LBF_DEG_TO_N_RAD * p.lambda_Ca;
Ca_axle_r = 2 * polyval(p.Ca_coef, Fz_tire_r) * LBF_DEG_TO_N_RAD * p.lambda_Ca;

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
fprintf('CAVEAT: static axle loads, linear tires (valid to ~0.4 g), no load\n');
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

outdir = fullfile(fileparts(mfilename('fullpath')), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'handling_response.png'));
close(f);
end


function s = balance_word(K)
if     K >  0.1, s = 'understeer';
elseif K < -0.1, s = 'OVERSTEER - check margin';
else,            s = 'near neutral';
end
end
