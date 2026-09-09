function pass = vd_selftest()
% VD_SELFTEST  Regression self-check: artifact staleness, formula wiring, data anchors.
% Run after any edit. addpath tests, run from repo root.

fprintf('\n================= VD SELF-TEST =================\n');

here  = fileparts(fileparts(mfilename('fullpath')));   % repo root (tests/ is below)
nfail = 0;

% ---------------------------------------------------------------- layer 0
% The staleness gate. The generated-artifact pattern is only safe if something
% screams when the artifact and its inputs diverge.
fprintf('\n-- artifact integrity --\n');
car = vd_car();
art = fullfile(here, ['tire_coeffs_' car '.mat']);
if ~exist(art, 'file')
    error('vd_selftest:noArtifact', ...
          '%s missing (car %s). Run: build_tire_coeffs', ['tire_coeffs_' car '.mat'], car);
end
T = load(art);
if isfield(T, 'car') && ~strcmp(T.car, car)
    error('vd_selftest:wrongCar', ...
          'Artifact belongs to car %s but vd_car() selects %s. Rebuild it.', T.car, car);
end
fprintf('%-32s %s\n', 'active car', car);

want = vd_hash(tire_src_files(here));
if ~isfield(T, 'src_hash'), stored = '(none)'; else, stored = T.src_hash; end
if ~strcmp(stored, want)
    fprintf(2, ['\n*** Stale tire artifact ***\n' ...
        'tire_coeffs.mat was built from different inputs than are on disk.\n' ...
        '  stored: %s\n  actual: %s\n' ...
        'The tire fit code, the promotion math, or the TTC data changed since\n' ...
        'the last build, so the car is running on grip that no longer matches.\n\n' ...
        'Run:  build_tire_coeffs\n' ...
        'Then re-issue the grip-derived targets (#8 #12 #13 #48 #49 #61-65)\n' ...
        'before handing any of them to another subteam.\n\n'], stored, want);
    error('vd_selftest:staleArtifact', 'tire_coeffs.mat is stale.');
end
fprintf('%-32s %s\n', 'tire_coeffs.mat fresh (hash)', 'PASS');

% The hash covers the fit code + TTC data, but not vehicle_params.m -- and
p0 = vehicle_params();
if ~isfield(T, 'Fz_design_lbf')
    fprintf(2, 'tire_coeffs.mat predates the Fz_design check. Run build_tire_coeffs.\n');
    error('vd_selftest:noFzDesign', 'artifact has no Fz_design_lbf.');
end
dFz = abs(p0.Fz_design_lbf - T.Fz_design_lbf);
if dFz > 1e-6 * max(1, T.Fz_design_lbf)
    fprintf(2, ['\n*** Design load moved ***\n' ...
        'The car mass changed since tire_coeffs.mat was built.\n' ...
        '  artifact built at : %.2f lbf/corner\n' ...
        '  params now give   : %.2f lbf/corner  (m = %.2f kg)\n' ...
        'Tire mu is read at the design load and falls with load, so the stored\n' ...
        'grip no longer describes this car.\n\n' ...
        'Run:  build_tire_coeffs      then re-issue the grip targets.\n\n'], ...
        T.Fz_design_lbf, p0.Fz_design_lbf, p0.m);
    error('vd_selftest:designLoadMoved', 'design corner load != artifact.');
end
fprintf('%-32s PASS  (%.1f lbf/corner)\n', 'design load matches artifact', T.Fz_design_lbf);

% Grip must be loaded, never a literal. Catches a future edit that re-introduces
% `p.mu_y_raw = 2.34;` into the parameter file.
src  = fileread(fullfile(here, 'vehicle_params.m'));
body = regexprep(src, '%[^\n]*', '');            % strip comments first
bad  = {};
for k = {'mu_y_raw', 'mu_anisotropy', 'mu_x_raw'}
    if ~isempty(regexp(body, ['p\.' k{1} '\s*=\s*[\d\.]'], 'once'))
        bad{end+1} = k{1}; %#ok<AGROW>
    end
end
ok = isempty(bad);
nfail = nfail + ~ok;
if ok
    fprintf('%-32s PASS\n', 'grip not hand-typed in params');
else
    fprintf('%-32s FAIL  <-- literal: %s\n', 'grip not hand-typed in params', ...
            strjoin(bad, ', '));
end
fprintf('%-32s INFO  (%s)\n', 'grip basis', T.basis);

% ---------------------------------------------------------------- layer 1
p = vehicle_params();

% Wiring guard (root cause of the silent-circle bug): the friction-ellipse
% exponents must actually reach p, or lap_sim's ellipse_exp falls back to n=2
% (a circle) with no error. Fail with a clear message if either is missing.
for fld = {'n_env_drive','n_env_brake'}
    if ~isfield(p, fld{1}) || ~isfinite(p.(fld{1}))
        error('vd_selftest:ellipseExpUnwired', ...
            ['p.%s is missing/NaN - vehicle_params did not load it, so lap_sim ' ...
             'silently uses the n=2 circle. Re-run build_tire_coeffs and check the ' ...
             'vehicle_params LOADED block.'], fld{1});
    end
end

evalc('out  = run_load_transfer_targets();');
evalc('R    = ttc_fit();');
evalc('out2 = run_gg_targets();');
evalc('H    = run_handling_targets();');
gg0 = gg_envelope(p, 0);
G12 = axle_grip(p, 12);
g12  = gg_envelope(p, 12);
p_pm = p;  p_pm.grip_model = 'pointmass';   % same car, point-mass lateral limit
p_lm = p;  p_lm.long_model = 'pointmass';   % same car, point-mass longitudinal edges

% --- camber: the reduction case and a live case --------------------------
% The reduction case is the important one. A term that is switched off must
% change NOTHING - not "almost nothing". If these drift apart, the camber
% factors have leaked into the zero-camber path and every previously issued
% grip target silently moved.
cam0  = struct('fo', 0, 'fi', 0, 'ro', 0, 'ri', 0);
cam2  = struct('fo', 2, 'fi', -2, 'ro', 2, 'ri', -2);
p_c0  = vd_set(p, 'camber_deg', cam0);
p_c2  = vd_set(p, 'camber_deg', cam2);
G12c0 = axle_grip(p_c0, 12);
G12c2 = axle_grip(p_c2, 12);
Fz_t  = 200;                                  % a load inside the fitted box
TF0   = tire_forces(p, Fz_t, [], 0);
a_swp = 0:0.1:25;
TFc   = tire_forces(p, Fz_t, a_swp, 0);

C = {
  'mu_y = raw * derate',       p.mu_y,            p.mu_y_raw * p.mu_derate
  'mu_x = raw * derate',       p.mu_x,            p.mu_x_raw * p.mu_derate
  'mu_x_raw = mu_y_raw * ani', p.mu_x_raw,        p.mu_y_raw * p.mu_anisotropy
  'params grip == artifact',   p.mu_y_raw,        T.mu_y_raw
  'CG ceiling = t/2/(SF mu)',  out.h_cg_ceiling,  (out.cfg.t_mean/2)/(out.cfg.SF_rollover*out.cfg.mu)
  'Track floor = 2 h SF mu',   out.track_floor,   2*p.h_cg*out.cfg.SF_rollover*out.cfg.mu
  'Brake bias = Xf + mu h/L',  out.bias_f,        p.mass_dist_f + out.cfg.D_design*p.h_cg/p.L
  'Mass sens = mu g h / t',    out.dWlat_per_kg,  out.cfg.mu*p.g*p.h_cg/out.cfg.t_mean
  'gg ay(v=0) = mu_y',         gg0.ay,            p.mu_y
  'gg brake(0) w/ driveline',  gg0.ax_brake,      (p.mu_x + p.Crr + p.Tc_driveline/(p.Re*p.m*p.g))/p.k_rot
  'skidpad ay = v^2/(R g)',    out2.ay_skid,      out2.v_skid^2/(out2.cfg.R_skid*p.g)
  'v_max = rev-limit formula', p.v_max,           (p.rpm_motor_max/p.gear_ratio)*(2*pi/60)*p.Re
  'corner_speed(0) = v_max',   corner_speed(p,0), p.v_max
  'K = Wf/Caf - Wr/Car',       H.K_deg_per_g,     (p.Wf_static/H.Ca_axle_f - p.Wr_static/H.Ca_axle_r)*180/pi
  'Ca_axle from ARTIFACT',     H.Ca_coef_source,  1
  'axle Fy demand = b/L',      G12.dem_f/(p.m*G12.ay_lim_g*p.g), p.b/p.L
  'grip_model default axle',   double(strcmp(p.grip_model,'axle')),  1
  'ay_limit axle = axle_grip', ay_limit(p, 12),    G12.ay_lim_g
  'ay_limit pmass = gg.ay',    ay_limit(p_pm, 12), g12.ay
  'n_env_drive loaded (p=art)',p.n_env_drive,      T.n_env_drive
  'n_env_brake loaded (p=art)',p.n_env_brake,      T.n_env_brake
  'long_model default combined', double(strcmp(p.long_model,'combined')), 1
  'ax_limit pmass = gg accel', ax_limit(p_lm, 12, 'accel'), g12.ax_accel
  'ax_limit pmass = gg brake', ax_limit(p_lm, 12, 'brake'), g12.ax_brake
  % --- camber reduction: gamma = 0 must reproduce the pre-camber model ---
  'mu_of_load 2arg == 3arg@0', mu_of_load(p, Fz_t),          mu_of_load(p, Fz_t, 0)
  'axle_grip camber0 == base', G12c0.ay_lim_g,               G12.ay_lim_g
  'tire_camber fD(0) == 1',    tire_camber(p, Fz_t, 0),      1
  'tire_camber fC(0) == 1',    second_out(@() tire_camber(p, Fz_t, 0)), 1
  'tire_forces mu == mu_of_load', TF0.mu_y,                  mu_of_load(p, Fz_t)
  'tire_forces Ca == artifact',TF0.Ca_lbf_deg,               polyval(p.Ca_coef, Fz_t)*p.lambda_Ca
  'tire_forces curve peak = D',max(TFc.Fy_lbf),              TF0.Fy_max_lbf
  'schema version == 2',       p.tire_schema_version,        2
};

fprintf('\n-- formula wiring --\n');
fprintf('%-28s %12s %12s   %s\n','check','script','recompute','result');
for i = 1:size(C,1)
    got = C{i,2};  wantv = C{i,3};
    ok  = abs(got-wantv) <= 1e-6*max(1,abs(wantv));
    nfail = nfail + ~ok;
    fprintf('%-28s %12.5f %12.5f   %s\n', C{i,1}, got, wantv, tern(ok,'PASS','FAIL'));
end

% ---------------------------------------------------------------- layer 2
fprintf('\n-- data anchors --\n');
% Two kinds of check here:
inr = @(x,lo,hi) double(x >= lo & x <= hi);
A = {
  % (1) raw-data anchor - fit-method-independent
  'ttc_fit LC0 pctile (data)', R.LC0_16x75.mu_y_raw, 2.602, 0.030
  % (2) physical invariants
  'mu_y_raw in [2.0,2.6]',     inr(p.mu_y_raw, 2.0, 2.6),        1, 0.5
  'anisotropy in [0.90,1.10]', inr(p.mu_anisotropy, 0.90, 1.10), 1, 0.5
  'axle ay in (0.85,1.0)*mu_y',inr(G12.ay_lim_g, 0.85*p.mu_y, p.mu_y), 1, 0.5
  'axle limit < point mass',   double(ay_limit(p,12) < ay_limit(p_pm,12)), 1, 0.5
  'ellipse exp drive not n=2', inr(p.n_env_drive, 1.5, 1.98), 1, 0.5
  'ellipse exp brake not n=2', inr(p.n_env_brake, 1.5, 1.98), 1, 0.5
  'ax accel axle < point mass',double(ax_limit(p,3,'accel') < ax_limit(p_lm,3,'accel')), 1, 0.5
  'ax brake axle < point mass',double(ax_limit(p,12,'brake') < ax_limit(p_lm,12,'brake')), 1, 0.5
  'combined(ay=0) = axle accel',ax_combined(p,12,0,'accel'), ax_limit(p,12,'accel'), 2e-3
  'combined(ay=0) = axle brake',ax_combined(p,12,0,'brake'), ax_limit(p,12,'brake'), 2e-3
  'combined falls with ay',    double(ax_combined(p,12,0.8*G12.ay_lim_g,'accel') < ax_combined(p,12,0,'accel')), 1, 0.5
  'motor map 96% island',      inr(motor_eff(2500,110), 0.945, 0.965), 1, 0.5
  'motor map high-torque band',inr(motor_eff(2500,220), 0.925, 0.945), 1, 0.5
  % wiring / regression locks (tautological against the artifact, tight)
  'mu_of_load @edge cont.',    mu_of_load(p, p.Fz_fit_max), polyval(p.mu_coef, p.Fz_fit_max)*p.mu_derate, 1e-9
  'mu_of_load outer=artifact', mu_of_load(p, T.Fz_outer_limit_lbf), T.mu_outer_central, 1e-6
  'hi-load band non-negative', double(T.mu_outer_central >= T.mu_outer_low - 1e-9), 1, 0.5
  % --- camber: is it fitted, wired, and pointing the right way? ---------
  'camber fit succeeded',      double(strcmp(p.camber_status, 'ok')), 1, 0.5
  'camber levels reached >=2deg', double(p.camber_gamma_max_deg >= 2), 1, 0.5
  'mf_bin table promoted',     double(numel(p.mf_bin_Fz_lbf) >= 3), 1, 0.5
  'camber COSTS peak @ high Fz', double(tire_camber(p, p.camber_Fz_max_lbf, 4) < 1), 1, 0.5
  'camber peak worse as Fz up', double(tire_camber(p, p.camber_Fz_max_lbf, 4) < tire_camber(p, p.camber_Fz_min_lbf, 4)), 1, 0.5
  'camber cuts stiffness',     double(second_out(@() tire_camber(p, Fz_t, 4)) < 1), 1, 0.5
  'camber thrust +ve, grows',  double(third_out(@() tire_camber(p, p.camber_Fz_max_lbf, 4)) > third_out(@() tire_camber(p, p.camber_Fz_min_lbf, 4))), 1, 0.5
  'camber clamped past box',   tire_camber(p, Fz_t, 3*p.camber_gamma_max_deg), tire_camber(p, Fz_t, p.camber_gamma_max_deg), 1e-12
  'axle_grip RESPONDS to camber', double(abs(G12c2.ay_lim_g - G12.ay_lim_g) > 1e-4), 1, 0.5
  'camber fit RMS < 8%',       double(max(T.camber_rms(1:2)) < 0.08), 1, 0.5
};
for i = 1:size(A,1)
    got = A{i,2}; wantv = A{i,3}; tol = A{i,4};
    ok = abs(got-wantv) <= tol;
    nfail = nfail + ~ok;
    fprintf('%-28s %12.5f %12s   %s\n', A{i,1}, got, ...
            sprintf('%.3f+/-%.3f', wantv, tol), tern(ok,'PASS','FAIL'));
end

pass    = (nfail == 0);
nchecks = size(C,1) + size(A,1) + 2;
fprintf('-----------------------------------------------\n');
fprintf('%s   (%d checks, %d failed)\n', ...
        tern(pass,'ALL PASS','*** FAILURES ***'), nchecks, nfail);
if ~pass
    error('vd_selftest:failed','%d self-test check(s) failed - see table above.', nfail);
end
end

function v = second_out(f)
% Grab the 2nd output of a multi-output call from inside a table literal.
[~, v] = f();
end

function v = third_out(f)
[~, ~, v] = f();
end

function files = tire_src_files(here)
% must match build_tire_coeffs/tire_src_files exactly, or every run reads
files = {fullfile(here, 'tire', 'pacejka_fit.m'), ...
         fullfile(here, 'tire', 'ttc_fit.m'), ...
         fullfile(here, 'tire', 'camber_fit.m'), ...
         fullfile(here, 'tire', 'build_tire_coeffs.m')};
d = dir(fullfile(here, 'TTC_Data', '*.mat'));
for i = 1:numel(d)
    if contains(d(i).name, 'raw'), continue; end     % raw files are not read
    files{end+1} = fullfile(here, 'TTC_Data', d(i).name); %#ok<AGROW>
end
end

function s = tern(c,a,b)
if c, s = a; else, s = b; end
end
