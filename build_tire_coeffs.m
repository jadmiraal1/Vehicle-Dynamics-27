function T = build_tire_coeffs()
% BUILD_TIRE_COEFFS  The tire -> car promotion step. Run DELIBERATELY.
%
%   build_tire_coeffs()      % re-fits the TTC data, rewrites tire_coeffs.mat
%
% WHY THIS EXISTS
% ---------------
% Grip used to reach the car by hand-copy: pacejka_fit / ttc_fit printed a
% number, you typed it into vehicle_params. Two consequences, both bad:
%
%   1. The same number lived in two places and could silently disagree.
%   2. mu_anisotropy = 1.008 had NO CODE BEHIND IT AT ALL. It was a hand
%      calculation recorded in a comment. Nothing could reproduce or check it.
%
% Now: this script is the ONLY producer of design grip. vehicle_params LOADS
% tire_coeffs.mat and never computes or hard-codes grip. But the load is
% decoupled from the fit, so re-fitting the tire does NOT silently change the
% car -- you have to run this, on purpose, and then re-issue the grip targets.
% vd_selftest hashes the inputs and FAILS if the artifact has gone stale, so
% you cannot forget.
%
% GRIP BASIS: PACEJKA CURVE (changed Jul 2026)
% --------------------------------------------
% Design grip is the Magic Formula curve peak, not ttc_fit's 99th-percentile
% envelope. The percentile reads the upper envelope of a noisy point cloud;
% the MF peak reads the fitted median curve. The percentile ran ~10% high in
% mu_y and ~16% high in mu_x. ttc_fit is retained as a TIRE-SCREENING tool
% (cross-tire comparison, camber/pressure windows) but is no longer a source
% of design values. See VD_physics_reference.md sec 8.
%
% CAVEAT (real, and it limits what this number is worth): the LC0's lateral
% peak is NOT reached inside the +/-12 deg TTC sweep at high load. The 250 lbf
% bin is excluded from the mu(Fz) trend for exactly this reason, and mu at the
% design load still rests on an MF extrapolation beyond the data. Treat
% mu_y_raw as good to a few percent, not better.
%
% Outputs -> tire_coeffs.mat, and returns the same struct.

here = fileparts(mfilename('fullpath'));
N_PER_LBF = 4.44822;

p = vehicle_params('bootstrap');   % car mass/geometry only - NO grip.
                                   % 'bootstrap' breaks the circular dependency
                                   % (vehicle_params loads what this produces).

fprintf('\n=============== BUILD_TIRE_COEFFS ===============\n');
R = pacejka_fit();                 % the fit itself (prints its own table)

Fz_design = p.m * p.g / 4 / N_PER_LBF;          % design corner load [lbf]

% --- Lateral: design tire, curve basis -----------------------------------
T.mu_y_raw          = polyval(R.mu_coef, Fz_design);
T.Ca_design_lbf_deg = polyval(R.Ca_coef, Fz_design);
T.mu_coef           = R.mu_coef;
T.Ca_coef           = R.Ca_coef;
T.Fz_design_lbf     = Fz_design;

% --- Anisotropy mu_x/mu_y: the cross-tire transfer, NOW COMPUTED ---------
% The 18in LC0 is the only tire with drive/brake data, but it never gets a
% full lateral sweep (SA is held at ~0/-3/-6 deg), so its lateral PEAK is not
% measured -- only its value at 6 deg. The design tire's MF curve supplies the
% shape correction: what fraction of peak does a curve of this shape reach at
% 6 deg, AT THE LOAD THE 18in LATERAL DATA WAS TAKEN AT?
%
% That last clause is the bit the old hand-calc got wrong. It used 0.88, the
% shape factor at the DESIGN load (183 lbf). But the 18in lateral data sits at
% ~245 lbf, where the load-matched factor is ~0.85. Tire curves get flatter
% with load; you must evaluate the correction where the data lives.
Fz_lat = R.long18.Fz_lat6;
alpha  = linspace(0, 25, 5000);
curve  = R.eval(alpha, Fz_lat);
T.shape6       = R.eval(6.0, Fz_lat) / max(curve);
T.mu_y_18_peak = R.long18.mu_y_at6 / T.shape6;
T.mu_x_18      = 0.5 * (R.long18.drive.mu_x + R.long18.brake.mu_x);
T.mu_anisotropy = T.mu_x_18 / T.mu_y_18_peak;

% --- Extrapolation guard -------------------------------------------------
% mu(Fz) is a LINE fitted over a limited load range. Evaluating it outside that
% range is not a measurement, it is a guess -- and for a tire with steep negative
% load sensitivity (LC0: -2.6 mu/1000 lbf) it is the guess most likely to be
% wrong. Say so, loudly, instead of quietly returning a number.
ok_bins = ~isnan(R.Fz_lbf) & R.peak_in_sweep;
if nnz(ok_bins) < 3, ok_bins = ~isnan(R.Fz_lbf); end
T.Fz_fit_min = min(R.Fz_lbf(ok_bins));
T.Fz_fit_max = max(R.Fz_lbf(ok_bins));

if Fz_design < T.Fz_fit_min || Fz_design > T.Fz_fit_max
    warning('build_tire_coeffs:extrapolated', ...
        ['DESIGN LOAD %.0f lbf IS OUTSIDE THE FITTED RANGE %.0f-%.0f lbf.\n' ...
         'mu_y_raw = %.3f is an extrapolation, not a measurement. Check the\n' ...
         'vehicle mass (a units slip here is the usual cause) or get tire data\n' ...
         'at the load this car actually runs.'], ...
        Fz_design, T.Fz_fit_min, T.Fz_fit_max, T.mu_y_raw);
end

% The load that actually sets the cornering limit is the OUTER tire, not the
% static average. Check that one too - it is always the binding case.
mu_y_der = T.mu_y_raw * p.mu_derate;
t_mean   = mean([p.t_f p.t_r]);
dW_lat   = p.m * p.g * mu_y_der * p.h_cg / t_mean;          % [N]
T.Fz_outer_limit_lbf = (p.m*p.g/2 + dW_lat) / 2 / N_PER_LBF;
fprintf('  outer tire at the limit %8.1f lbf  (fitted range %.0f-%.0f)\n', ...
        T.Fz_outer_limit_lbf, T.Fz_fit_min, T.Fz_fit_max);
if T.Fz_outer_limit_lbf > T.Fz_fit_max
    fprintf(2, ['  NOTE: the loaded outer tire runs %.0f%% beyond the fitted load range.\n' ...
                '        Load-sensitivity effects there are extrapolated. The mid-tier\n' ...
                '        axle-grip model inherits this limitation.\n'], ...
            100*(T.Fz_outer_limit_lbf/T.Fz_fit_max - 1));
end

T.mu_y_at6    = R.long18.mu_y_at6;
T.Fz_lat6     = Fz_lat;
T.mu_x_drive  = R.long18.drive.mu_x;
T.mu_x_brake  = R.long18.brake.mu_x;
T.n_envelope  = R.long18.drive.n_envelope;

% --- Provenance ----------------------------------------------------------
T.tire_id          = p.tire_id;
T.tire_data_prefix = p.tire_data_prefix;
T.basis            = 'pacejka-curve';
T.built_by         = 'build_tire_coeffs.m';
T.built_on         = datestr(now, 'yyyy-mm-dd HH:MM');
T.src_hash         = vd_hash(tire_src_files(here));

save(fullfile(here, 'tire_coeffs.mat'), '-struct', 'T');

fprintf('\n--- promoted to tire_coeffs.mat (basis: %s) ---\n', T.basis);
fprintf('  design corner load     %8.2f lbf\n', Fz_design);
fprintf('  mu_y_raw               %8.4f\n', T.mu_y_raw);
fprintf('  Ca @ design            %8.1f lbf/deg\n', T.Ca_design_lbf_deg);
fprintf('  MF shape factor @6deg  %8.4f  (at Fz %.0f lbf, load-matched)\n', ...
        T.shape6, Fz_lat);
fprintf('  mu_x/mu_y anisotropy   %8.4f\n', T.mu_anisotropy);
fprintf('  mu_x_raw               %8.4f\n', T.mu_y_raw * T.mu_anisotropy);
fprintf('  src hash               %s\n', T.src_hash);
fprintf('\n!! Grip changed -> RE-ISSUE the grip-derived targets:\n');
fprintf('   #8 tire, #12 scalings, #13 peak mu, #48 brake bias, #49 decel,\n');
fprintf('   #62 skidpad, #63 accel, #64 g-g, #61/#65 lap+energy.\n');
fprintf('   Then run vd_selftest.\n');
end


function files = tire_src_files(here)
% Everything the artifact depends on: the fit code, the PROMOTION MATH IN THIS
% FILE, and the TTC inputs. Change any of these and the artifact is stale.
% MUST match tests/vd_selftest.m/tire_src_files exactly.
%
% Including build_tire_coeffs.m itself matters: the design load, the
% load-matched shape correction and the anisotropy are computed HERE, not in
% pacejka_fit. Omitting it would let someone change the anisotropy math and
% still get a "fresh" verdict from the selftest.
files = {fullfile(here, 'pacejka_fit.m'), ...
         fullfile(here, 'ttc_fit.m'), ...
         fullfile(here, 'build_tire_coeffs.m')};
d = dir(fullfile(here, 'TTC_Data', '*.mat'));
for i = 1:numel(d)
    if contains(d(i).name, 'raw'), continue; end     % raw files are not read
    files{end+1} = fullfile(here, 'TTC_Data', d(i).name); %#ok<AGROW>
end
end
