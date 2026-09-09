function R = pacejka_fit()
% PACEJKA_FIT  Magic Formula fit per load bin, all candidate tires.
% Returns mu(Fz) and Ca(Fz) coefficients + the combined-slip envelope exponents
% from the 18in LC0 held-SA sweeps. Theory: ref doc sec 8.

% THIS IS now THE SOURCE OF design GRIP (Jul 2026). Promoted into the car by
% build_tire_coeffs -> tire_coeffs_<CAR>.mat -> vehicle_params. Do not hand-copy
% anything out of this function's printout into vehicle_params.

p = vehicle_params('bootstrap');   % car mass only; must not require the
                                   % artifact that this fit produces

% Fit setup
TIRES         = {'LC0_16x75', 'R20_16x75', 'R20_18x60', 'GY_18x65'};
LOAD_BINS_LBF = [50 100 150 200 250]; % isolate data at discrete vertical loads to run fits at different load cases
LOAD_BAND     = 0.15;                 % +/-15% of data is accepted around each bin
SA_EDGES      = 0.25:0.5:12.25;      % |slip angle| bins [deg], discretizes the 'continuous' slip angle data into bins to deal with noisy raw data
MIN_BIN_N     = 40;                  % each slip angle bin req 40 samples to be trustworthy
MIN_TREND_PTS = 3;                   % min valid load bins needed for mu/Ca vs load trend fits
V_MIN_MPH     = 20;                  % ROLLING ONLY - see the note on the mask below

here     = vd_root();
data_dir = fullfile(here, 'TTC_Data');
N_PER_LBF        = 4.44822; % conversion rates
LBF_DEG_TO_N_RAD = N_PER_LBF * 180/pi;
Fz_design        = p.m * p.g / 4 / N_PER_LBF;   % design corner load [lbf], just assumes mean

fprintf('\nPACEJKA PURE-LATERAL FIT  (IA<1.5deg, P 9-13 psi, V>%d mph)\n', V_MIN_MPH);
fprintf('  + camber sensitivity from the TTC camber sweeps (tire/camber_fit.m)\n');

% Loading and filtering raw data
for t = 1:numel(TIRES)
    tire = TIRES{t};
    D = load_channels(data_dir, [tire '_*.mat']);
    Fz_mag = -D.FZ;

    % Pure lateral, near-zero camber, near-target pressure, ROLLING; drop the
    % 5-7 deg slip band where the sweep reverses.
    %
    % THE ROLLING FILTER (added Sep 2026, and it moved every grip number).
    % Each of the four candidate tires has a first TTC run that is ~99% below
    % 5 mph - 52 to 55% of the samples this mask would otherwise keep. A tire
    % that is not rolling has no steady-state cornering force to give: at 150
    % lbf and 4 deg of slip the static samples read Fy/Fz ~ 0.22 where the
    % rolling ones read ~1.93. Including them dragged the binned medians down
    % and the design tire's peak mu with them, by 4.1% at the design load
    % (2.383 -> 2.481).
    %
    % Note WHY this had to be removed rather than corrected for: the bias is
    % not uniform. Fitting on medians makes it partly self-limiting, so the
    % same 50%-static contamination moved LC0_16x75 by +4.1% and R20_16x75 by
    % -0.0%. An unpredictable bias cannot be calibrated away, only excluded.
    base = (abs(D.FX ./ D.FZ) < 0.10) & (Fz_mag > 30) & (abs(D.IA) < 1.5) ...
           & (D.P > 9) & (D.P < 13) & ~(abs(D.SA) > 5.0 & abs(D.SA) < 7.0) ...
           & (D.V > V_MIN_MPH);

    report_rolling_filter(tire, D, base, V_MIN_MPH);

    n = numel(LOAD_BINS_LBF);
    T = struct('Fz_lbf', nan(1,n), 'B', nan(1,n), 'C', nan(1,n), ...
               'D', nan(1,n), 'E', nan(1,n), 'R2', nan(1,n), ...
               'peak_in_sweep', false(1,n));
    T.curves = cell(1,n);

    fprintf('--- %s\n%6s %7s %6s %8s %7s %8s %7s %9s\n', tire, ...
            'Fz', 'B', 'C', 'D', 'E', 'R2', 'mu_pk', 'Ca[l/dg]');
    for k = 1:n
        sel = base & (abs(Fz_mag - LOAD_BINS_LBF(k)) < LOAD_BINS_LBF(k)*LOAD_BAND);
        if nnz(sel) < 3000, continue; end % min samples for a fit
        [alpha, fy] = binned_median_curve(D.SA(sel), -D.FY(sel), SA_EDGES, MIN_BIN_N); % takes median within each bin to represent SA to prevent skewed data (not actual line)
        T.curves{k} = [alpha, fy];

        prm = fit_mf(alpha, fy); % runs actual Pacejka/Magic Formula fit with least squares to determine coeffs.
        T.Fz_lbf(k) = mean(Fz_mag(sel));
        T.B(k) = prm(1);  T.C(k) = prm(2);  T.D(k) = prm(3);  T.E(k) = prm(4);
        res     = mf(prm, alpha) - fy;
        T.R2(k) = 1 - sum(res.^2)/sum((fy - mean(fy)).^2); % standard goodness of fit calculation
        % peak identifiable only if the curve stops rising inside the sweep
        T.peak_in_sweep(k) = mf(prm, 12.0)/mf(prm, 10.0) <= 1.02; % checks to see if curve peak is within slip angle sweep, otherwise D is extrapolated

        marker = ' ';  if ~T.peak_in_sweep(k), marker = '*'; end
        fprintf('%6.0f %7.3f %6.3f %8.1f %7.2f %8.4f %7.3f%s %8.1f\n', ...
                T.Fz_lbf(k), prm(1), prm(2), prm(3), prm(4), T.R2(k), ...
                prm(3)/T.Fz_lbf(k), marker, prm(1)*prm(2)*prm(3));
    end

    % Load dependence: Ca quadratic over all bins; mu LINEAR in load
    % (Pacejka pDy1/pDy2 form), fitted only over bins whose peak sits
    % inside the 12-deg sweep - D is unidentifiable when still rising
    ok  = ~isnan(T.Fz_lbf);
    n_ok = nnz(ok);
    if n_ok < MIN_TREND_PTS % not enough valid load bins to fit mu/Ca vs load at all
        error('pacejka_fit:insufficientData', ...
              ['%s: only %d of %d load bins produced a valid fit ' ...
               '(need >=%d for the mu/Ca vs load trend). Check TTC ' ...
               'coverage / the nnz(sel)<3000 threshold for this tire.'], ...
              tire, n_ok, n, MIN_TREND_PTS);
    end

    idp = ok & T.peak_in_sweep;
    used_fallback = nnz(idp) < MIN_TREND_PTS; % not enough in-sweep peaks -> fall back to all valid bins
    if used_fallback
        idp = ok;
    end
    T.mu_peak    = T.D ./ T.Fz_lbf; % peak mu per load
    T.Ca_lbf_deg = T.B .* T.C .* T.D; % cornering stiffness for every load
    T.mu_coef    = polyfit(T.Fz_lbf(idp), T.mu_peak(idp), 1); % fits peak grip as straight line vs load
    T.Ca_coef    = polyfit(T.Fz_lbf(ok), T.Ca_lbf_deg(ok), 2); % fits cornering stiffness as quadratic vs load
    T.mu_coef_used_extrapolated_peaks = used_fallback; % flag for downstream/diagnostics
    % Camber sensitivity. Separate file, separate data mask (rolling only),
    % separate normalisation - see tire/camber_fit.m for why. It cannot change
    % anything above it: the camber terms are RATIOS against the gamma = 0
    % curves fitted here, so at gamma = 0 they are exactly 1.
    T.camber = camber_fit(data_dir, tire, true);

    R.(tire) = T; % store in struct

    if used_fallback && any(ok & ~T.peak_in_sweep)
        fprintf(['       ! fewer than %d in-sweep peaks available - mu(Fz) trend ' ...
                 'FELL BACK to including extrapolated-peak bins\n'], MIN_TREND_PTS);
    elseif any(ok & ~T.peak_in_sweep)
        fprintf('       * peak beyond sweep - excluded from mu(Fz) trend\n');
    end
end

% Longitudinal MF fit, 18in LC0 drive/brake (torque sweeps exist at ~250 lbf
% only). SR needs a frozen free-rolling radius: the RE channel is defined as
% V/omega per sample, so instantaneous RE makes SR identically zero.
R.long18 = fit_longitudinal(data_dir);

% Mirror the design tire at top level (consumed by run_handling_targets)
Td = R.(p.tire_data_prefix);
for f = fieldnames(Td)'
    R.(f{1}) = Td.(f{1});
end
R.eval = @(alpha_deg, Fz_lbf) mf_at_load(Td, alpha_deg, Fz_lbf);

% Clamp the same way mf_at_load/R.eval does, so this printed summary can
ok_d       = ~isnan(Td.Fz_lbf);
Fz_clamped = min(max(Fz_design, min(Td.Fz_lbf(ok_d))), max(Td.Fz_lbf(ok_d)));
if Fz_clamped ~= Fz_design
    fprintf('\nNote: design load %.0f lbf is outside the tested range [%.0f, %.0f] lbf;\n', ...
            Fz_design, min(Td.Fz_lbf(ok_d)), max(Td.Fz_lbf(ok_d)));
    fprintf('      summary below evaluated at the clamped load %.0f lbf, matching R.eval.\n', ...
            Fz_clamped);
end
Ca_design = polyval(R.Ca_coef, Fz_clamped);
fprintf('\nDesign tire %s at %.0f lbf: mu_peak %.3f, Ca %.1f lbf/deg = %.0f N/rad\n', ...
        p.tire_data_prefix, Fz_clamped, polyval(R.mu_coef, Fz_clamped), ...
        Ca_design, Ca_design*LBF_DEG_TO_N_RAD);
fprintf('Note: MF peak reads the median curve; ttc_fit 99th percentile reads the\n');
fprintf('upper envelope. Figures: run tire_report (presentation layer).\n');
end

function report_rolling_filter(tire, D, base, v_min)
% Provenance: say how much the rolling filter removed. If this ever prints 0%
% for a tire that used to lose half its samples, the mask has been edited.
if ~isfield(D, 'V') || isempty(D.V), return; end
would = (abs(D.FX ./ D.FZ) < 0.10) & (-D.FZ > 30) & (abs(D.IA) < 1.5) ...
        & (D.P > 9) & (D.P < 13) & ~(abs(D.SA) > 5.0 & abs(D.SA) < 7.0);
dropped = nnz(would) - nnz(base);
fprintf('       rolling filter (V>%d mph): kept %d of %d samples (%.0f%% dropped as non-rolling)\n', ...
        v_min, nnz(base), nnz(would), 100*dropped/max(nnz(would),1));
end

function y = mf(p, alpha)
% Magic Formula, pure slip. p = [B C D E], alpha in deg.
Bx = p(1) .* alpha;
y  = p(3) .* sin(p(2) .* atan(Bx - p(4).*(Bx - atan(Bx)))); 
end

function prm = fit_mf(alpha, fy)
% Least-squares MF fit with bounds; falls back to fminsearch w/o toolbox.
% B = stiffness factor, C = shape factor, D = peak value, E = curvature
% factor, a = slip angle
D0   = max(fy); % peak value guess
BCD0 = fy(1) / alpha(1); % estimate initial slope
p0   = [BCD0/(1.4*D0), 1.4, D0, -0.5]; % initial guess for lsq
lb   = [0.01, 1.0, 0.5*D0, -3.0]; % well known bounds
ub   = [2.00, 2.0, 1.5*D0,  0.99];
% Least squares function
if exist('lsqcurvefit', 'file')
    opt = optimoptions('lsqcurvefit', 'Display', 'off');
    prm = lsqcurvefit(@mf, p0, alpha, fy, lb, ub, opt);
else
    sse = @(q) sum((mf(min(max(q,lb),ub), alpha) - fy).^2);
    prm = fminsearch(sse, p0, optimset('Display','off'));
    prm = min(max(prm, lb), ub);
end
end

function [x, y] = binned_median_curve(sa, fy, edges, min_n)
% Symmetrized |SA| median curve (robust to sweep artifacts/hysteresis).
fy_odd = fy;  fy_odd(sa < 0) = -fy_odd(sa < 0);
a = abs(sa);
x = []; y = [];
for i = 1:numel(edges)-1
    m = (a >= edges(i)) & (a < edges(i+1));
    if nnz(m) > min_n
        x(end+1,1) = mean(a(m));            %#ok<AGROW>
        y(end+1,1) = median(fy_odd(m));     %#ok<AGROW>
    end
end
end

function fy = mf_at_load(T, alpha_deg, Fz_lbf)
% Evaluate a tire's fit at arbitrary load: B,C,E interpolated (clamped),
% D from its load quadratic.
ok = ~isnan(T.Fz_lbf);
Fz = min(max(Fz_lbf, min(T.Fz_lbf(ok))), max(T.Fz_lbf(ok)));
prm = [interp1(T.Fz_lbf(ok), T.B(ok), Fz), ...
       interp1(T.Fz_lbf(ok), T.C(ok), Fz), ...
       polyval(T.mu_coef, Fz) * Fz, ...
       interp1(T.Fz_lbf(ok), T.E(ok), Fz)];
fy = mf(prm, alpha_deg);
end

function L = fit_longitudinal(data_dir)
% FX vs slip ratio at SA~0, 18in LC0, ~250 lbf; drive and brake fitted
% separately (the tire is measurably asymmetric). Also estimates the
% friction-envelope exponent n from the held-SA combined sweeps.
channels = {'SA','FX','FY','FZ','IA','V','N','RE'};   % V: rolling filter + slip ratio
files    = dir(fullfile(data_dir, 'LC0_18x60_*.mat'));
chunks   = cell(numel(files), numel(channels));
for i = 1:numel(files)
    S = load(fullfile(data_dir, files(i).name));
    for c = 1:numel(channels)
        chunks{i,c} = S.(channels{c})(:);
    end
end
for c = 1:numel(channels)
    D.(channels{c}) = vertcat(chunks{:,c});
end

Fz_mag = -D.FZ;
omega  = D.N * 2*pi/60; % needs speeds because grip is a function of slip ratio not slip angle
v_road = D.V * 0.44704;
% V > 20 mph is redundant for this tire today (LC0_18x60 has no near-static
% samples at all) but it is not decorative: slip ratio divides by road speed,
% so a future data set with a creep segment would produce infinities here
% rather than a visible error.
sel0   = (abs(D.SA) < 1.0) & (abs(D.IA) < 1.5) & (abs(Fz_mag - 250) < 35) ...
         & (D.V > 20);

% Frozen free-rolling effective radius, then slip ratio
free = sel0 & (abs(D.FX)./Fz_mag < 0.02); % define wheel radius from free roll and use as reference to calculate slip ratios
RE0  = median(D.RE(free)) * 0.0254;
SR   = (omega .* RE0 - v_road) ./ v_road; % slip ratio

fprintf('--- 18in LC0 longitudinal @ ~250 lbf (single-load; no torque sweeps elsewhere)\n');
fprintf('%6s %6s %6s %8s %7s %8s %7s %9s\n', 'side','B','C','D','E','R2','mu_x','Kx/Fz');
L = struct();
for sides = {{'drive', +1}, {'brake', -1}} % split + and - samples into drive and brake, otherwise same logic as lateral
    name = sides{1}{1};  sgn = sides{1}{2};
    m  = sel0 & (sgn*SR > 0) & (abs(SR) < 0.25); % data at low SR is noise-dominated so filter
    sr = abs(SR(m));  fx = sgn * D.FX(m);
    edges = 0.004:0.008:0.20;
    xs = []; ys = [];
    for i = 1:numel(edges)-1
        mm = (sr >= edges(i)) & (sr < edges(i+1));
        if nnz(mm) > 25
            xs(end+1,1) = mean(sr(mm));   %#ok<AGROW>
            ys(end+1,1) = median(fx(mm)); %#ok<AGROW>
        end
    end
    D0  = max(ys);
    p0  = [max(ys(1)/xs(1),1)/(1.4*D0), 1.4, D0, -0.5];
    lb  = [1, 1.0, 0.5*D0, -3.0];  ub = [500, 2.0, 1.5*D0, 0.99];
    if exist('lsqcurvefit','file')
        prm = lsqcurvefit(@mf, p0, xs, ys, lb, ub, ...
                          optimoptions('lsqcurvefit','Display','off'));
    else
        prm = fminsearch(@(q) sum((mf(min(max(q,lb),ub),xs)-ys).^2), p0, ...
                         optimset('Display','off'));
        prm = min(max(prm, lb), ub);
    end
    res = mf(prm, xs) - ys;
    fzm = mean(Fz_mag(sel0));
    L.(name) = struct('B',prm(1),'C',prm(2),'D',prm(3),'E',prm(4), ...
                      'R2', 1 - sum(res.^2)/sum((ys-mean(ys)).^2), ...
                      'Fz_lbf', fzm, 'mu_x', prm(3)/fzm, ...
                      'Kx_per_Fz', prm(1)*prm(2)*prm(3)/fzm, ...
                      'curve', [xs ys]);
    fprintf('%6s %6.1f %6.3f %8.1f %7.2f %8.4f %7.3f %9.1f\n', name, ...
            prm(1), prm(2), prm(3), prm(4), L.(name).R2, ...
            L.(name).mu_x, L.(name).Kx_per_Fz);
end

% Lateral reference for the mu_x/mu_y anisotropy transfer.
lat6 = (abs(D.IA) < 1.5) & (abs(Fz_mag - 250) < 35) & (abs(SR) < 0.005) ...
       & (abs(abs(D.SA) - 6) < 0.6);
L.mu_y_at6 = median(abs(D.FY(lat6)) ./ Fz_mag(lat6));
L.Fz_lat6  = median(Fz_mag(lat6));
fprintf('%6s lateral @6deg: mu_y %.3f at Fz %.0f lbf (peak not swept;', ...
        '18in', L.mu_y_at6, L.Fz_lat6);
fprintf(' shape-corrected in build_tire_coeffs)\n');

% Friction-envelope exponent from combined sweeps (held SA 3/6 deg):
% fit |fx|^n + |fy|^n = 1 to max-over-slices, normalized by pure values.
% LOWER-BOUND estimate (SA only tested to 6 deg); n=2 ellipse retained.
sel_c  = (abs(D.SA) > 2.4) & (abs(D.SA) < 6.6) & (abs(Fz_mag - 250) < 35) ...
         & (abs(D.IA) < 1.5) & (abs(SR) < 0.25);
fy_ref = median(abs(D.FY((abs(abs(D.SA)-6) < 0.6) & (abs(Fz_mag-250) < 35) ...
         & (abs(D.IA) < 1.5) & (abs(SR) < 0.005))));
for sides = {{'drive', +1}, {'brake', -1}}
    name = sides{1}{1};  sgn = sides{1}{2};
    m   = sel_c & (sgn*SR > 0.005);
    fxn = abs(D.FX(m)) / (L.(name).mu_x * median(Fz_mag(m)));
    fyn = abs(D.FY(m)) / fy_ref;
    edges = 0.05:0.075:1.0;
    xs = []; ys = [];
    for i = 1:numel(edges)-1
        mm = (fxn >= edges(i)) & (fxn < edges(i+1));
        if nnz(mm) > 40
            xs(end+1,1) = mean(fxn(mm));                 %#ok<AGROW>
            ys(end+1,1) = prctile_local(fyn(mm), 95);    %#ok<AGROW>
        end
    end
    n_fit = fminsearch(@(n) sum(((1 - min(xs,0.999).^abs(n)).^(1/abs(n)) - ys).^2), 2.0); % friction ellipse fitting
    L.(name).n_envelope = abs(n_fit);
    fprintf('%6s combined-slip envelope n = %.2f (lower bound; model keeps n=2)\n', ...
            name, abs(n_fit));
end
end

function v = prctile_local(x, q)
x = sort(x(~isnan(x)));
n = numel(x);
if n == 0, v = NaN; return; end
rank = q/100 * (n - 1);
lo   = floor(rank);
v    = x(lo+1) + (rank - lo) * (x(min(lo+2, n)) - x(lo+1));
end

function D = load_channels(data_dir, pattern)
channels = {'SA','FY','FX','FZ','IA','P','V'};
files    = dir(fullfile(data_dir, pattern));
chunks   = cell(numel(files), numel(channels));
for i = 1:numel(files)
    if contains(files(i).name, 'raw'), continue; end
    S = load(fullfile(data_dir, files(i).name));
    if ~isfield(S, 'FZ'), continue; end
    for c = 1:numel(channels)
        chunks{i,c} = S.(channels{c})(:);
    end
end
for c = 1:numel(channels)
    D.(channels{c}) = vertcat(chunks{:,c});
end
end