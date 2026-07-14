function pass = vd_selftest()
% VD_SELFTEST  Regression self-check for the VD concept-tier toolchain.
% Run after any edit. Prints PASS/FAIL; errors on failure.
%
% Add tests/ to the path first, or run from the repo root with `addpath tests`.
%
% Three layers:
%   0. ARTIFACT INTEGRITY - is tire_coeffs.mat stale? is grip hand-typed?
%   1. FORMULA WIRING     - does each script's output equal its own formula?
%   2. DATA ANCHORS       - do the tire fits still reproduce known numbers?

fprintf('\n================= VD SELF-TEST =================\n');

here  = fileparts(fileparts(mfilename('fullpath')));   % repo root (tests/ is below)
nfail = 0;

% ---------------------------------------------------------------- layer 0
% The staleness gate. The generated-artifact pattern is only safe if something
% screams when the artifact and its inputs diverge.
fprintf('\n-- artifact integrity --\n');
art = fullfile(here, 'tire_coeffs.mat');
if ~exist(art, 'file')
    error('vd_selftest:noArtifact', 'tire_coeffs.mat missing. Run: build_tire_coeffs');
end
T = load(art);

want = vd_hash(tire_src_files(here));
if ~isfield(T, 'src_hash'), stored = '(none)'; else, stored = T.src_hash; end
if ~strcmp(stored, want)
    fprintf(2, ['\n*** STALE TIRE ARTIFACT ***\n' ...
        'tire_coeffs.mat was built from different inputs than are on disk.\n' ...
        '  stored: %s\n  actual: %s\n' ...
        'The tire fit code, the promotion math, or the TTC data changed since\n' ...
        'the last build, so the car is running on grip that no longer matches.\n\n' ...
        'Run:  build_tire_coeffs\n' ...
        'Then RE-ISSUE the grip-derived targets (#8 #12 #13 #48 #49 #61-65)\n' ...
        'before handing any of them to another subteam.\n\n'], stored, want);
    error('vd_selftest:staleArtifact', 'tire_coeffs.mat is stale.');
end
fprintf('%-32s %s\n', 'tire_coeffs.mat fresh (hash)', 'PASS');

% The hash covers the fit code + TTC data, but NOT vehicle_params.m -- and
% Fz_design = m*g/4 comes from there. mu is read AT the design load, so a mass
% change silently invalidates the artifact's grip with a still-green hash.
% This is exactly how a units slip (546 lb typed into a kg field, Jul 2026)
% went undetected. Close the loop explicitly.
p0 = vehicle_params();
if ~isfield(T, 'Fz_design_lbf')
    fprintf(2, 'tire_coeffs.mat predates the Fz_design check. Run build_tire_coeffs.\n');
    error('vd_selftest:noFzDesign', 'artifact has no Fz_design_lbf.');
end
dFz = abs(p0.Fz_design_lbf - T.Fz_design_lbf);
if dFz > 1e-6 * max(1, T.Fz_design_lbf)
    fprintf(2, ['\n*** DESIGN LOAD MOVED ***\n' ...
        'The car mass changed since tire_coeffs.mat was built.\n' ...
        '  artifact built at : %.2f lbf/corner\n' ...
        '  params now give   : %.2f lbf/corner  (m = %.2f kg)\n' ...
        'Tire mu is read AT the design load and FALLS with load, so the stored\n' ...
        'grip no longer describes this car.\n\n' ...
        'Run:  build_tire_coeffs      then re-issue the grip targets.\n\n'], ...
        T.Fz_design_lbf, p0.Fz_design_lbf, p0.m);
    error('vd_selftest:designLoadMoved', 'design corner load != artifact.');
end
fprintf('%-32s PASS  (%.1f lbf/corner)\n', 'design load matches artifact', T.Fz_design_lbf);

% Grip must be LOADED, never literal. Catches a future edit that re-introduces
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
evalc('out  = run_load_transfer_targets();');
evalc('R    = ttc_fit();');
evalc('out2 = run_gg_targets();');
evalc('H    = run_handling_targets();');
gg0 = gg_envelope(p, 0);
G12 = axle_grip(p, 12);

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
A = {
  'ttc_fit LC0_16x75 pctile',R.LC0_16x75.mu_y_raw, 2.602, 0.030
  'curve mu_y_raw (design)', p.mu_y_raw,       2.336, 0.030
  'mu_x/mu_y anisotropy',    p.mu_anisotropy,  0.968, 0.020
  'axle_grip ay @12 m/s',    G12.ay_lim_g,     1.426, 0.020
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


function files = tire_src_files(here)
% MUST match build_tire_coeffs/tire_src_files exactly, or every run reads
% "stale". Includes build_tire_coeffs.m itself: it carries the promotion math
% (design load, the load-matched shape correction, the anisotropy), so editing
% it genuinely invalidates the artifact.
files = {fullfile(here, 'pacejka_fit.m'), ...
         fullfile(here, 'ttc_fit.m'), ...
         fullfile(here, 'build_tire_coeffs.m')};
d = dir(fullfile(here, 'TTC_Data', '*.mat'));
for i = 1:numel(d)
    if contains(d(i).name, 'raw'), continue; end
    files{end+1} = fullfile(here, 'TTC_Data', d(i).name); %#ok<AGROW>
end
end

function s = tern(c,a,b)
if c, s = a; else, s = b; end
end
