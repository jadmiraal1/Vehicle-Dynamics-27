function T = build_tire_coeffs()
% BUILD_TIRE_COEFFS  Fit the tires and promote the design tire's grip into
% tire_coeffs.mat - the artifact vehicle_params loads, hash-stamped so vd_selftest
% catches staleness. Re-run after tire-fit/TTC changes, then re-issue grip targets.

here = fileparts(mfilename('fullpath'));
N_PER_LBF = 4.44822;

p = vehicle_params('bootstrap');   % car mass/geometry only - NO grip.
                                   % 'bootstrap' breaks the circular dependency
                                   % (vehicle_params loads what this produces).

fprintf('\n=============== BUILD_TIRE_COEFFS ===============\n');
evalc('R = pacejka_fit();');       % run quietly (per-bin tables: run pacejka_fit directly)
fprintf('  Pacejka fit: 4 candidate tires\n');

Fz_design = p.m * p.g / 4 / N_PER_LBF;          % design corner load [lbf]

% --- Lateral: design tire, curve basis -----------------------------------
T.mu_y_raw          = polyval(R.mu_coef, Fz_design); % fitted polynomial for peak grip vs vertical load
T.Ca_design_lbf_deg = polyval(R.Ca_coef, Fz_design); % fitted polynomial for cornering stiffness vs verticla load
T.mu_coef           = R.mu_coef;
T.Ca_coef           = R.Ca_coef;
T.Fz_design_lbf     = Fz_design;

% --- Per-load-bin Magic Formula table ------------------------------------
% B, C and E per tested load, so tire_forces can build a whole Fy(alpha) curve
% instead of only the peak. D is NOT stored: it is regenerated from mu_coef so
% that the curve and mu_of_load can never disagree about the peak.
ok_mf              = ~isnan(R.Fz_lbf);
T.mf_bin_Fz_lbf    = R.Fz_lbf(ok_mf);
T.mf_bin_B         = R.B(ok_mf);
T.mf_bin_C         = R.C(ok_mf);
T.mf_bin_E         = R.E(ok_mf);

% --- Camber sensitivity ---------------------------------------------------
% Flattened onto the artifact (not nested) because vehicle_params copies fields
% one by one and a nested struct there would be a second place to keep in sync.
% gamma > 0 = the helpful lean; see models/tire_camber.m.
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

% --- Anisotropy mu_x/mu_y: the cross-tire transfer, now computed ---------
Fz_lat = R.long18.Fz_lat6;
alpha  = linspace(0, 25, 5000);
curve  = R.eval(alpha, Fz_lat);
T.shape6       = R.eval(6.0, Fz_lat) / max(curve);
T.mu_y_18_peak = R.long18.mu_y_at6 / T.shape6; % LC0 18 in data only goes to 6 deg slip angle but is the only one with longitudinal data, so peak is obtained by MF shape
T.mu_x_18      = 0.5 * (R.long18.drive.mu_x + R.long18.brake.mu_x);
T.mu_anisotropy = T.mu_x_18 / T.mu_y_18_peak;

% --- Extrapolation guard -------------------------------------------------
ok_bins = ~isnan(R.Fz_lbf) & R.peak_in_sweep; % check if peak grip was reached at slip angle bin
if nnz(ok_bins) < 3, ok_bins = ~isnan(R.Fz_lbf); end
T.Fz_fit_min = min(R.Fz_lbf(ok_bins));
T.Fz_fit_max = max(R.Fz_lbf(ok_bins));

% --- Donor-constrained high-load extrapolation ---------------------------
[T.mu_hiload_slope, T.hiload_cov_lbf, T.hiload_spread_pct, T.hiload_donors, T.hiload_n_spread] = ...
        donor_hiload_slope(R, T.Fz_fit_max, T.mu_coef);

if Fz_design < T.Fz_fit_min || Fz_design > T.Fz_fit_max % chck if design load within data range
    warning('build_tire_coeffs:extrapolated', ...
        ['DESIGN LOAD %.0f lbf IS OUTSIDE THE FITTED RANGE %.0f-%.0f lbf.\n' ...
         'mu_y_raw = %.3f is an extrapolation, not a measurement. Check the\n' ...
         'vehicle mass (a units slip here is the usual cause) or get tire data\n' ...
         'at the load this car actually runs.'], ...
        Fz_design, T.Fz_fit_min, T.Fz_fit_max, T.mu_y_raw);
end

% The load that actually sets the cornering limit is the outer tire, not the
% static average. Check that one too - it is always the binding case.
mu_y_der = T.mu_y_raw * p.mu_derate;
t_mean   = mean([p.t_f p.t_r]);
dW_lat   = p.m * p.g * mu_y_der * p.h_cg / t_mean;          % [N]
T.Fz_outer_limit_lbf = (p.m*p.g/2 + dW_lat) / 2 / N_PER_LBF;

% mu at the outer-tire load, both ways, so the extrapolation band is stored.
% Suppress mu_of_load's beyond-donor warning here - the summary reports it cleanly.
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
% NOTE: field is n_env_drive (was n_envelope) so both exponents share one naming
% pattern and vehicle_params can copy the PAIR into p for lap_sim's ellipse_exp.

% --- Provenance ----------------------------------------------------------
T.tire_id          = p.tire_id;
T.tire_data_prefix = p.tire_data_prefix;
T.basis            = 'pacejka-curve';
% Bump SCHEMA_VERSION whenever a field is added, removed or redefined here.
% vehicle_params refuses an artifact it does not recognise and names the fix,
% so an old .mat can never be read as if it were a new one.
%   1 : (implicit) everything before Sep 2026
%   2 : + camber_* terms, + mf_bin_* table
T.schema_version   = 2;
T.built_by         = 'build_tire_coeffs.m';
T.built_on         = datestr(now, 'yyyy-mm-dd HH:MM');
T.src_hash         = vd_hash(tire_src_files(here));

T.car = p.car;                                   % which config this was built for
save(fullfile(vd_root(), ['tire_coeffs_' p.car '.mat']), '-struct', 'T');   % per-car artifact at repo root

% ---- compact console summary ----
fb   = abs(T.mu_hiload_slope - T.mu_coef(1)) < 1e-9;   % donor slope fell back to design
band = 100*(T.mu_outer_central/T.mu_outer_low - 1);
inside = Fz_design >= T.Fz_fit_min && Fz_design <= T.Fz_fit_max;

fprintf('\n  promoted %s @ %.0f lbf   (basis: %s)\n', T.tire_data_prefix, Fz_design, T.basis);
fprintf('    mu_y_raw %.3f    mu_x_raw %.3f  (anisotropy %.3f)    Ca %.0f lbf/deg\n', ...
        T.mu_y_raw, T.mu_y_raw*T.mu_anisotropy, T.mu_anisotropy, T.Ca_design_lbf_deg);
fprintf('    fit range %.0f-%.0f lbf  ->  design load %s\n', ...
        T.Fz_fit_min, T.Fz_fit_max, ternary(inside, 'INSIDE', 'OUTSIDE (see warning above)'));
fprintf('    outer tire %.0f lbf (donor cover %.0f)%s\n', ...
        T.Fz_outer_limit_lbf, T.hiload_cov_lbf, ...
        ternary(T.Fz_outer_limit_lbf > T.hiload_cov_lbf, ' - beyond donors, still extrapolated', ''));
fprintf('    band @ outer: central %.3f  vs  low %.3f  (%+.0f%%)%s\n', ...
        T.mu_outer_central, T.mu_outer_low, band, ...
        ternary(fb, '   [fallback: edge ~ donor ceiling, band ~ 0]', ''));
fprintf('    camber  %s\n', T.camber_status);
if ~strcmp(T.camber_status, 'ok')
    fprintf(2, '    ! camber terms are all zero - the model will behave as it did before.\n');
else
    for gshow = [2 4]
        fprintf(['    camber  gamma %+d deg: peak x%.3f @ %3.0f lbf, x%.3f @ %3.0f lbf' ...
                 '   stiffness x%.3f   thrust %+.2f deg slip @ %3.0f lbf\n'], gshow, ...
                cam_fD(T, T.camber_Fz_min_lbf, gshow), T.camber_Fz_min_lbf, ...
                cam_fD(T, T.camber_Fz_max_lbf, gshow), T.camber_Fz_max_lbf, ...
                1 + T.camber_kC(1)*gshow^2, ...
                (T.camber_kS(1) + T.camber_kS(2)*(T.camber_Fz_max_lbf - T.camber_Fz_ref_lbf)/T.camber_Fz_ref_lbf)*gshow, ...
                T.camber_Fz_max_lbf);
    end
    fprintf('    camber  fit RMS  peak %.3f  stiffness %.3f  thrust %.3f deg   (n = %d/%d/%d)\n', ...
            T.camber_rms(1), T.camber_rms(2), T.camber_rms(3), T.camber_n(1), T.camber_n(2), T.camber_n(3));
end
fprintf('    schema  v%d\n', T.schema_version);
fprintf('    wrote tire_coeffs_%s.mat   (hash %s)\n', p.car, T.src_hash(1:8));
fprintf('\n  re-issue grip targets #8 #12 #13 #48 #49 #61-65, then run vd_selftest.\n\n');

if nargout == 0, clear T; end   % don't auto-dump the struct when called as a command
end

function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end

function f = cam_fD(T, Fz_lbf, gamma_deg)
% Peak-factor preview for the console summary only. models/tire_camber.m is the
% single implementation that anything downstream is allowed to use.
dfz = (Fz_lbf - T.camber_Fz_ref_lbf) / T.camber_Fz_ref_lbf;
f = 1 + (T.camber_kD(1) + T.camber_kD(2)*dfz + T.camber_kD(3)*dfz^2)*gamma_deg ...
      + T.camber_kD(4)*gamma_deg^2;
end

function [slope_hi, cov_lbf, spread_pct, donors, n_spread] = donor_hiload_slope(R, edge, mu_coef_design)
% Pooled high-load slope for mu(Fz) above the design tire's data edge.
donor_candidates = {'LC0_16x75', 'R20_16x75', 'R20_18x60', 'GY_18x65'};
poolF = []; poolS = []; cov_lbf = edge; s200 = []; donors = {};
for i = 1:numel(donor_candidates)
    name = donor_candidates{i};
    if ~isfield(R, name), continue; end
    Td = R.(name);
    ok = ~isnan(Td.Fz_lbf) & Td.peak_in_sweep;
    if nnz(ok) < 2, continue; end
    Fz = Td.Fz_lbf(ok);  mu = Td.mu_peak(ok);
    % Reference point (150 lbf) must fall inside this donor's own data, or the
    % normalization anchor is itself an extrapolation. Skip donors that don't
    % bracket it rather than silently extrapolating the reference.
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
% spread_pct only means something with >=2 donors agreeing at 200 lbf.
% With 0 or 1, std()/mean() gives NaN or a false-confident 0% -- report
% honestly and surface the donor count alongside it (see caller fprintf).
n_spread = numel(s200);
if n_spread >= 2
    spread_pct = 100 * std(s200) / mean(s200);
else
    spread_pct = NaN;
end

% Need enough donor coverage ABOVE the design edge to define a slope. This
MIN_HEADROOM = 30;   % lbf of donor cover above the edge (> hiF offset of 15)
if cov_lbf < edge + MIN_HEADROOM
    % Design tire's OWN data now reaches nearly as high as the donors, so the
    slope_hi = mu_coef_design(1);
    return
end

shape      = polyfit(poolF, poolS, 2);
mu_edge    = polyval(mu_coef_design, edge);           % raw, design tire at its edge
shape_edge = polyval(shape, edge);
% Guard: shape_edge sits in a denominator next. The quadratic is fit on
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

function files = tire_src_files(here)
% Everything the artifact depends on: the fit code, the PROMOTION MATH IN THIS
% FILE, and the TTC inputs. Change any of these and the artifact is stale.
% must match tests/vd_selftest.m/tire_src_files exactly.
files = {fullfile(here, 'pacejka_fit.m'), ...
         fullfile(here, 'ttc_fit.m'), ...
         fullfile(here, 'camber_fit.m'), ...
         fullfile(here, 'build_tire_coeffs.m')};
d = dir(fullfile(vd_root(), 'TTC_Data', '*.mat'));
for i = 1:numel(d)
    if contains(d(i).name, 'raw'), continue; end     % raw files are not read
    files{end+1} = fullfile(vd_root(), 'TTC_Data', d(i).name); %#ok<AGROW>
end
end
