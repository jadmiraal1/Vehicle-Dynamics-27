function T = build_tire_coeffs()
% BUILD_TIRE_COEFFS  Fit the tires and write the design tire's model to
% tire_coeffs_<CAR>.mat, the artifact vehicle_params loads.
%   build_tire_coeffs
%
% Needs TTC_Data/ and the Optimization Toolbox (lsqcurvefit): without the
% toolbox the fit falls back to fminsearch and gives a materially different
% tire, so do not rebuild without it. The artifact is stamped with a hash of
% its inputs (tire_src_files) so vd_selftest can detect a stale one.
% Re-run after any change to the tire fit code or the TTC data, or to the
% car's mass, then run vd_selftest and re-issue the grip-derived targets.

here = fileparts(mfilename('fullpath'));
N_PER_LBF = vd_const().N_PER_LBF;

p = vehicle_params('bootstrap');   % car only, no grip: vehicle_params
                                   % normally loads what this function writes

fprintf('\nBuilding the tire model for %s\n', p.car);
if ~exist('lsqcurvefit', 'file')
    error('build_tire_coeffs:noToolbox', ...
        ['The Optimization Toolbox (lsqcurvefit) is not available. The fminsearch\n' ...
         'fallback gives a different tire (cornering stiffness ~30%% low); do not\n' ...
         'rebuild the artifact on this machine.']);
end
evalc('R = pacejka_fit();');       % quiet; run pacejka_fit directly for the per-bin tables
fprintf('  Magic Formula fit done for the candidate tires.\n');

Fz_design = p.m * p.g / 4 / N_PER_LBF;          % design corner load [lbf]

% --- Lateral: design tire, curve basis -----------------------------------
T.mu_y_raw          = polyval(R.mu_coef, Fz_design); % peak lateral mu at the design load
T.Ca_design_lbf_deg = polyval(R.Ca_coef, Fz_design); % cornering stiffness at the design load
T.mu_coef           = R.mu_coef;
T.Ca_coef           = R.Ca_coef;
T.Fz_design_lbf     = Fz_design;

% --- Per-load-bin Magic Formula table ------------------------------------
% B, C and E per tested load, so tire_forces can build a whole Fy(alpha)
% curve. D is not stored: it comes from mu_coef, so the curve and mu_of_load
% always agree about the peak.
ok_mf              = ~isnan(R.Fz_lbf);
T.mf_bin_Fz_lbf    = R.Fz_lbf(ok_mf);
T.mf_bin_B         = R.B(ok_mf);
T.mf_bin_C         = R.C(ok_mf);
T.mf_bin_E         = R.E(ok_mf);

% --- Camber sensitivity ---------------------------------------------------
% Stored as flat fields; gamma > 0 = the helpful lean (models/tire_camber.m).
CB                     = R.camber;
T.camber_kD            = CB.kD;              % peak factor   1 + (kD1+kD2*dfz+kD3*dfz^2)*g + kD4*g^2
T.camber_kC            = CB.kC;              % stiffness     1 + kC1*g^2
T.camber_kS            = CB.kS;              % thrust        (kS1+kS2*dfz)*g  [deg of slip]
T.camber_Fz_ref_lbf    = CB.Fz_ref_lbf;      % dfz = (Fz - ref)/ref
T.camber_Fz_min_lbf    = CB.Fz_min_lbf;      % fitted load box - clamped, not extrapolated
T.camber_Fz_max_lbf    = CB.Fz_max_lbf;
T.camber_gamma_max_deg = CB.gamma_max_deg;   % fitted camber range
T.camber_rms           = [CB.rms_peak CB.rms_stiff CB.rms_offset];
T.camber_n             = [CB.n_peak  CB.n_stiff  CB.n_offset];
T.camber_status        = CB.status;
T.camber_basis         = CB.basis;

% --- Anisotropy mu_x/mu_y ------------------------------------------------
% Only the 18in LC0 has drive/brake data, and its lateral data stops at 6 deg
% of slip. Its lateral PEAK is estimated from the 6 deg value using the design
% tire's curve shape at that load, then mu_x/mu_y is carried to the design tire.
Fz_lat = R.long18.Fz_lat6;
alpha  = linspace(0, 25, 5000);
curve  = R.eval(alpha, Fz_lat);
T.shape6       = R.eval(6.0, Fz_lat) / max(curve);
T.mu_y_18_peak = R.long18.mu_y_at6 / T.shape6;
T.mu_x_18      = 0.5 * (R.long18.drive.mu_x + R.long18.brake.mu_x);
T.mu_anisotropy = T.mu_x_18 / T.mu_y_18_peak;

% --- Extrapolation guard -------------------------------------------------
ok_bins = ~isnan(R.Fz_lbf) & R.peak_in_sweep;   % bins whose peak lies inside the slip sweep
if nnz(ok_bins) < 3, ok_bins = ~isnan(R.Fz_lbf); end
T.Fz_fit_min = min(R.Fz_lbf(ok_bins));
T.Fz_fit_max = max(R.Fz_lbf(ok_bins));

% --- Donor-constrained high-load extrapolation ---------------------------
[T.mu_hiload_slope, T.hiload_cov_lbf, T.hiload_spread_pct, T.hiload_donors, T.hiload_n_spread] = ...
        donor_hiload_slope(R, T.Fz_fit_max, T.mu_coef);

if Fz_design < T.Fz_fit_min || Fz_design > T.Fz_fit_max
    warning('build_tire_coeffs:extrapolated', ...
        ['DESIGN LOAD %.0f lbf IS OUTSIDE THE FITTED RANGE %.0f-%.0f lbf.\n' ...
         'mu_y_raw = %.3f is an extrapolation, not a measurement. Check the\n' ...
         'vehicle mass (a units slip here is the usual cause) or get tire data\n' ...
         'at the load this car actually runs.'], ...
        Fz_design, T.Fz_fit_min, T.Fz_fit_max, T.mu_y_raw);
end

% The outer tire at the cornering limit carries far more than the average
% load; store mu there both ways so the extrapolation band is visible.
mu_y_der = T.mu_y_raw * p.mu_derate;
t_mean   = mean([p.t_f p.t_r]);
dW_lat   = p.m * p.g * mu_y_der * p.h_cg / t_mean;          % [N]
T.Fz_outer_limit_lbf = (p.m*p.g/2 + dW_lat) / 2 / N_PER_LBF;

pc = p; pc.Fz_fit_max = T.Fz_fit_max; pc.mu_coef = T.mu_coef;
pc.mu_hiload_slope = T.mu_hiload_slope; pc.hiload_cov_lbf = T.hiload_cov_lbf;
ws = warning('off', 'all');
pc.tire_hiload = 'central'; T.mu_outer_central = mu_of_load(pc, T.Fz_outer_limit_lbf);
pc.tire_hiload = 'low';     T.mu_outer_low     = mu_of_load(pc, T.Fz_outer_limit_lbf);
warning(ws);

T.mu_y_at6    = R.long18.mu_y_at6;
T.Fz_lat6     = Fz_lat;
T.mu_x_drive  = R.long18.drive.mu_x;
T.mu_x_brake  = R.long18.brake.mu_x;
T.n_env_drive  = R.long18.drive.n_envelope;   % combined-slip ellipse exponent, DRIVE
T.n_env_brake  = R.long18.brake.n_envelope;   % ... BRAKE (18in LC0 held-SA sweeps)

% --- Provenance ----------------------------------------------------------
T.tire_id          = p.tire_id;
T.tire_data_prefix = p.tire_data_prefix;
T.basis            = 'pacejka-curve';
% Bump schema_version (and SCHEMA_EXPECTED in vehicle_params) whenever a
% field is added, removed or redefined.
%   1 : (implicit) before Sep 2026
%   2 : + camber_* terms, + mf_bin_* table
T.schema_version   = 2;
T.built_by         = 'build_tire_coeffs.m';
T.built_on         = datestr(now, 'yyyy-mm-dd HH:MM');
T.src_hash         = vd_hash(tire_src_files());

T.car = p.car;                                   % which config this was built for
save(fullfile(vd_root(), ['tire_coeffs_' p.car '.mat']), '-struct', 'T');

% ---- compact console summary ----
fb   = abs(T.mu_hiload_slope - T.mu_coef(1)) < 1e-9;   % donor slope fell back to design
band = 100*(T.mu_outer_central/T.mu_outer_low - 1);
inside = Fz_design >= T.Fz_fit_min && Fz_design <= T.Fz_fit_max;

fprintf('\n  design tire %s at %.0f lbf per corner   (basis: %s)\n', T.tire_data_prefix, Fz_design, T.basis);
fprintf('    rig grip lateral %.3f, longitudinal %.3f (ratio %.3f); cornering stiffness %.0f lbf/deg\n', ...
        T.mu_y_raw, T.mu_y_raw*T.mu_anisotropy, T.mu_anisotropy, T.Ca_design_lbf_deg);
fprintf('    data covers %.0f-%.0f lbf; design load %s\n', ...
        T.Fz_fit_min, T.Fz_fit_max, ternary(inside, 'inside it', 'OUTSIDE it (see warning above)'));
fprintf('    outer tire at the limit %.0f lbf; other tires'' data reaches %.0f lbf%s\n', ...
        T.Fz_outer_limit_lbf, T.hiload_cov_lbf, ...
        ternary(T.Fz_outer_limit_lbf > T.hiload_cov_lbf, ' (so grip there is extrapolated)', ''));
fprintf('    grip at the outer tire: central %.3f, pessimistic %.3f (%+.0f %%)%s\n', ...
        T.mu_outer_central, T.mu_outer_low, band, ...
        ternary(fb, '; the two coincide (no donor data above the design tire''s)', ''));
fprintf('    camber fit: %s\n', T.camber_status);
if ~strcmp(T.camber_status, 'ok')
    fprintf(2, '    camber terms are all zero, so camber has no effect in the models.\n');
else
    for gshow = [2 4]
        fprintf(['    camber %+d deg: peak grip x%.3f at %3.0f lbf, x%.3f at %3.0f lbf;' ...
                 ' stiffness x%.3f; thrust %+.2f deg slip at %3.0f lbf\n'], gshow, ...
                cam_fD(T, T.camber_Fz_min_lbf, gshow), T.camber_Fz_min_lbf, ...
                cam_fD(T, T.camber_Fz_max_lbf, gshow), T.camber_Fz_max_lbf, ...
                1 + T.camber_kC(1)*gshow^2, ...
                (T.camber_kS(1) + T.camber_kS(2)*(T.camber_Fz_max_lbf - T.camber_Fz_ref_lbf)/T.camber_Fz_ref_lbf)*gshow, ...
                T.camber_Fz_max_lbf);
    end
    fprintf('    camber fit RMS error: peak %.3f, stiffness %.3f, thrust %.3f deg (n = %d/%d/%d)\n', ...
            T.camber_rms(1), T.camber_rms(2), T.camber_rms(3), T.camber_n(1), T.camber_n(2), T.camber_n(3));
end
fprintf('    wrote tire_coeffs_%s.mat (format v%d, inputs hash %s)\n', p.car, T.schema_version, T.src_hash(1:8));
fprintf('\n  Next: vd_selftest, then vd_golden to see which results moved.\n\n');

if nargout == 0, clear T; end   % don't auto-dump the struct when called as a command
end

function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end

function f = cam_fD(T, Fz_lbf, gamma_deg)
% Peak-factor preview for the console summary only (models use tire_camber).
dfz = (Fz_lbf - T.camber_Fz_ref_lbf) / T.camber_Fz_ref_lbf;
f = 1 + (T.camber_kD(1) + T.camber_kD(2)*dfz + T.camber_kD(3)*dfz^2)*gamma_deg ...
      + T.camber_kD(4)*gamma_deg^2;
end

function [slope_hi, cov_lbf, spread_pct, donors, n_spread] = donor_hiload_slope(R, edge, mu_coef_design)
% Slope of mu(Fz) above the design tire's data edge, from the shape of the
% other tested tires (donors) normalised at 150 lbf. Falls back to the design
% tire's own slope when the donors reach barely past its edge.
donor_candidates = {'LC0_16x75', 'R20_16x75', 'R20_18x60', 'GY_18x65'};
poolF = []; poolS = []; cov_lbf = edge; s200 = []; donors = {};
for i = 1:numel(donor_candidates)
    name = donor_candidates{i};
    if ~isfield(R, name), continue; end
    Td = R.(name);
    ok = ~isnan(Td.Fz_lbf) & Td.peak_in_sweep;
    if nnz(ok) < 2, continue; end
    Fz = Td.Fz_lbf(ok);  mu = Td.mu_peak(ok);
    % The 150 lbf reference must lie inside this donor's data.
    if 150 < min(Fz) || 150 > max(Fz)
        fprintf(2, '  donor %s excluded: does not bracket the 150 lbf reference load (range %.0f-%.0f)\n', ...
                name, min(Fz), max(Fz));
        continue
    end
    ref = interp1(Fz, mu, 150, 'linear');
    poolF = [poolF, Fz];  poolS = [poolS, mu/ref]; %#ok<AGROW>
    cov_lbf = max(cov_lbf, max(Fz));
    donors{end+1} = name; %#ok<AGROW>
    if max(Fz) >= 200, s200(end+1) = interp1(Fz, mu/ref, 200); end %#ok<AGROW>
end
% Donor spread at 200 lbf is meaningful only with two or more donors.
n_spread = numel(s200);
if n_spread >= 2
    spread_pct = 100 * std(s200) / mean(s200);
else
    spread_pct = NaN;
end

MIN_HEADROOM = 30;   % donor data needed above the edge [lbf] (> the 15 lbf offset below)
if cov_lbf < edge + MIN_HEADROOM
    % Donors barely extend past the design tire's own data: use its own slope.
    slope_hi = mu_coef_design(1);
    return
end

shape      = polyfit(poolF, poolS, 2);
mu_edge    = polyval(mu_coef_design, edge);           % raw, design tire at its edge
shape_edge = polyval(shape, edge);
% shape_edge is a denominator below; a near-zero value means a degenerate fit.
if shape_edge <= 0.05 * mean(poolS)
    warning('build_tire_coeffs:donorShapeDegenerate', ...
        ['donor shape fit is ~0 at the design edge (%.0f lbf) -- normalized\n' ...
         'donor curve is degenerate here. Falling back to the design tire''s\n' ...
         'own measured slope instead of an unstable donor-scaled estimate.'], edge);
    slope_hi = mu_coef_design(1);
    return
end
hiF     = linspace(edge + 15, cov_lbf, 4);         % edge+15 < cov by MIN_HEADROOM
hi_mu   = mu_edge * polyval(shape, hiF) ./ shape_edge;
c       = polyfit([edge, hiF], [mu_edge, hi_mu], 1);
slope_hi = c(1);
end
