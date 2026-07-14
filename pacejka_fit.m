function R = pacejka_fit()
% PACEJKA_FIT  Magic Formula pure-lateral fit for ALL candidate tires.
% FY = D*sin(C*atan(B*a - E*(B*a - atan(B*a)))), a in deg, forces in lbf.
% Method, artifact handling, interpretation: VD_physics_reference.md, sec 8.
% R.(tire) holds each tire's fit; the design tire (p.tire_data_prefix) is
% mirrored at top level (Ca_coef, D_coef, eval, ...) for downstream code.

% THIS IS NOW THE SOURCE OF DESIGN GRIP (Jul 2026). Promoted into the car by
% build_tire_coeffs -> tire_coeffs.mat -> vehicle_params. Do not hand-copy
% anything out of this function's printout into vehicle_params.

p = vehicle_params('bootstrap');   % car mass only; must not require the
                                   % artifact that this fit produces

% Fit setup
TIRES         = {'LC0_16x75', 'R20_16x75', 'R20_18x60', 'GY_18x65'};
LOAD_BINS_LBF = [50 100 150 200 250];
LOAD_BAND     = 0.15;                 % +/-15% around each bin
SA_EDGES      = 0.25:0.5:12.25;      % |slip angle| bins [deg]
MIN_BIN_N     = 40;

here     = fileparts(mfilename('fullpath'));
data_dir = fullfile(here, 'TTC_Data');
N_PER_LBF        = 4.44822;
LBF_DEG_TO_N_RAD = N_PER_LBF * 180/pi;
Fz_design        = p.m * p.g / 4 / N_PER_LBF;   % design corner load [lbf]

fprintf('\nPACEJKA PURE-LATERAL FIT  (IA<1.5deg, P 9-13 psi)\n');

for t = 1:numel(TIRES)
    tire = TIRES{t};
    D = load_channels(data_dir, [tire '_*.mat']);
    Fz_mag = -D.FZ;

    % Pure lateral, near-zero camber, near-target pressure; drop the
    % +5..7 deg sweep-junction artifact
    base = (abs(D.FX ./ D.FZ) < 0.10) & (Fz_mag > 30) & (abs(D.IA) < 1.5) ...
           & (D.P > 9) & (D.P < 13) & ~((D.SA > 5.0) & (D.SA < 7.0));

    n = numel(LOAD_BINS_LBF);
    T = struct('Fz_lbf', nan(1,n), 'B', nan(1,n), 'C', nan(1,n), ...
               'D', nan(1,n), 'E', nan(1,n), 'R2', nan(1,n), ...
               'peak_in_sweep', false(1,n));
    T.curves = cell(1,n);

    fprintf('--- %s\n%6s %7s %6s %8s %7s %8s %7s %9s\n', tire, ...
            'Fz', 'B', 'C', 'D', 'E', 'R2', 'mu_pk', 'Ca[l/dg]');
    for k = 1:n
        sel = base & (abs(Fz_mag - LOAD_BINS_LBF(k)) < LOAD_BINS_LBF(k)*LOAD_BAND);
        if nnz(sel) < 3000, continue; end
        [alpha, fy] = binned_median_curve(D.SA(sel), -D.FY(sel), SA_EDGES, MIN_BIN_N);
        T.curves{k} = [alpha, fy];

        prm = fit_mf(alpha, fy);
        T.Fz_lbf(k) = mean(Fz_mag(sel));
        T.B(k) = prm(1);  T.C(k) = prm(2);  T.D(k) = prm(3);  T.E(k) = prm(4);
        res     = mf(prm, alpha) - fy;
        T.R2(k) = 1 - sum(res.^2)/sum((fy - mean(fy)).^2);
        % peak identifiable only if the curve stops rising inside the sweep
        T.peak_in_sweep(k) = mf(prm, 12.0)/mf(prm, 10.0) <= 1.02;

        marker = ' ';  if ~T.peak_in_sweep(k), marker = '*'; end
        fprintf('%6.0f %7.3f %6.3f %8.1f %7.2f %8.4f %7.3f%s %8.1f\n', ...
                T.Fz_lbf(k), prm(1), prm(2), prm(3), prm(4), T.R2(k), ...
                prm(3)/T.Fz_lbf(k), marker, prm(1)*prm(2)*prm(3));
    end

    % Load dependence: Ca quadratic over all bins; mu LINEAR in load
    % (Pacejka pDy1/pDy2 form), fitted ONLY over bins whose peak sits
    % inside the 12-deg sweep - D is unidentifiable when still rising
    ok  = ~isnan(T.Fz_lbf);
    idp = ok & T.peak_in_sweep;
    if nnz(idp) < 3, idp = ok; end
    T.mu_peak    = T.D ./ T.Fz_lbf;
    T.Ca_lbf_deg = T.B .* T.C .* T.D;
    T.mu_coef    = polyfit(T.Fz_lbf(idp), T.mu_peak(idp), 1);
    T.Ca_coef    = polyfit(T.Fz_lbf(ok), T.Ca_lbf_deg(ok), 2);
    R.(tire) = T;
    if any(ok & ~T.peak_in_sweep)
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

Ca_design = polyval(R.Ca_coef, Fz_design);
fprintf('\nDesign tire %s at %.0f lbf: mu_peak %.3f, Ca %.1f lbf/deg = %.0f N/rad\n', ...
        p.tire_data_prefix, Fz_design, polyval(R.mu_coef, Fz_design), ...
        Ca_design, Ca_design*LBF_DEG_TO_N_RAD);
fprintf('Note: MF peak reads the median curve; ttc_fit 99th percentile reads the\n');
fprintf('upper envelope. Figures: run tire_report (presentation layer).\n');
end


function y = mf(p, alpha)
% Magic Formula, pure slip. p = [B C D E], alpha in deg.
Bx = p(1) .* alpha;
y  = p(3) .* sin(p(2) .* atan(Bx - p(4).*(Bx - atan(Bx))));
end


function prm = fit_mf(alpha, fy)
% Least-squares MF fit with bounds; falls back to fminsearch w/o toolbox.
D0   = max(fy);
BCD0 = fy(1) / alpha(1);
p0   = [BCD0/(1.4*D0), 1.4, D0, -0.5];
lb   = [0.01, 1.0, 0.5*D0, -3.0];
ub   = [2.00, 2.0, 1.5*D0,  0.99];
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
channels = {'SA','FX','FY','FZ','IA','V','N','RE'};
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
omega  = D.N * 2*pi/60;
v_road = D.V * 0.44704;
sel0   = (abs(D.SA) < 1.0) & (abs(D.IA) < 1.5) & (abs(Fz_mag - 250) < 35);

% Frozen free-rolling effective radius, then slip ratio
free = sel0 & (abs(D.FX)./Fz_mag < 0.02);
RE0  = median(D.RE(free)) * 0.0254;
SR   = (omega .* RE0 - v_road) ./ v_road;

fprintf('--- 18in LC0 longitudinal @ ~250 lbf (single-load; no torque sweeps elsewhere)\n');
fprintf('%6s %6s %6s %8s %7s %8s %7s %9s\n', 'side','B','C','D','E','R2','mu_x','Kx/Fz');
L = struct();
for sides = {{'drive', +1}, {'brake', -1}}
    name = sides{1}{1};  sgn = sides{1}{2};
    m  = sel0 & (sgn*SR > 0) & (abs(SR) < 0.25);
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
% The 18in donor never gets a full lateral sweep (SA is HELD at ~0/-3/-6 deg),
% so its lateral PEAK is not measured - only its value at 6 deg. We export that
% value and the load it was taken at; build_tire_coeffs applies the shape
% correction using the design tire's MF curve AT THIS LOAD. Exporting Fz_lat6
% (rather than assuming the design load) is what lets the correction be
% load-matched -- the previous hand-calc used the 183 lbf shape factor on
% 245 lbf data, and tire curves flatten with load.
lat6 = (abs(D.IA) < 1.5) & (abs(Fz_mag - 250) < 35) & (abs(SR) < 0.005) ...
       & (abs(abs(D.SA) - 6) < 0.6);
L.mu_y_at6 = median(abs(D.FY(lat6)) ./ Fz_mag(lat6));
L.Fz_lat6  = median(Fz_mag(lat6));
fprintf('%6s lateral @6deg: mu_y %.3f at Fz %.0f lbf (peak NOT swept;', ...
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
    n_fit = fminsearch(@(n) sum(((1 - min(xs,0.999).^abs(n)).^(1/abs(n)) - ys).^2), 2.0);
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
channels = {'SA','FY','FX','FZ','IA','P'};
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
