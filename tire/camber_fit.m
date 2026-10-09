function C = camber_fit(data_dir, prefix, verbose)
% CAMBER_FIT  Camber sensitivity of one tire, from the TTC camber sweeps.
%   C = camber_fit(data_dir, prefix)          prints a one-line summary
%   C = camber_fit(data_dir, prefix, false)   no printing
%
% Called by pacejka_fit. Produces the three camber coefficient sets that
% tire_camber evaluates.
%
% WHAT CAMBER DOES TO A TIRE (three separate effects)
% 1. Camber thrust: a leaning tire makes lateral force at zero slip angle.
%    In the data it is a horizontal shift of the whole Fy(alpha) curve, so it
%    adds force at low slip without changing the peak.
% 2. Peak grip: leaning loads one shoulder harder, so peak friction changes -
%    a symmetric penalty (gamma^2) plus an asymmetric part (linear in gamma)
%    whose sign flips with load.
% 3. Cornering stiffness falls with camber magnitude (even in gamma).
% The best camber is where effect 1 stops paying for effect 2; it moves with
% load, so it is an output of the vehicle model, not a fitted parameter.
%
% SIGN CONVENTION
%   gamma > 0: the tire leans so that camber thrust ADDS to the cornering
%   force (top of the tire toward the corner centre). That is the loaded
%   outside wheel, i.e. NEGATIVE camber in the SAE chassis convention; the
%   inside wheel on the same suspension usually sits at gamma < 0.
% The TTC rig swept inclination only to +2 and +4 deg, which in its sign
% convention is the adverse direction (at IA = +4, Fz ~ 150 lbf the zero-slip
% force moves against the force from positive slip). The data are therefore
% mirrored (alpha -> -alpha, Fy -> -Fy) before fitting. With no negative
% inclination data, the adverse branch assumes a symmetric tire; the two
% branches of the zero-camber curve agree to about 2%.
%
% METHOD
% Three numbers per condition - peak, zero crossing, low-slip slope - are
% read off the binned median curve (a Magic Formula with shift terms
% cross-validated slightly worse: leave-one-load-out RMS 3.4% vs 2.8%).

if nargin < 3, verbose = true; end

% ---- fit setup ----------------------------------------------------------
C.Fz_ref_lbf = 150;        % normalising load for dfz = (Fz - Fz_ref)/Fz_ref.
                           % A MEASURED load bin, deliberately not the design
                           % load: the camber coefficients must not move when
                           % the car's mass changes.
LOAD_BINS_LBF = [50 100 150 200 250];
LOAD_BAND     = 0.15;      % +/-15% around each bin, same as pacejka_fit
SA_EDGES      = -12.25:0.5:12.25;   % SIGNED bins - see the warning below
MIN_BIN_N     = 10;        % samples needed to keep a slip-angle bin
MIN_CELL_N    = 300;       % samples needed for one (file, load, camber, P) cell
MIN_CURVE_PTS = 12;        % bins needed before a cell's curve is usable
V_MIN_MPH     = 20;        % ROLLING ONLY - see below
GAMMA_STEP    = 1.0;       % deg, resolution for finding the camber levels
P_STEP        = 2.0;       % psi, resolution for finding the pressure levels
MIN_LEVEL_N   = 2000;      % samples needed before a level counts as present

% ---- load and mask ------------------------------------------------------
D = load_camber_channels(data_dir, [prefix '_*.mat']);
if isempty(D.FZ)
    C = empty_result(C, 'no TTC files matched');
    return
end
Fz_lbf = -D.FZ;

% Rolling only: near-static samples (most of each tire's first run) carry
% almost no lateral force and no camber thrust.
rolling = (abs(D.FX ./ D.FZ) < 0.10) & (Fz_lbf > 30) & (D.V > V_MIN_MPH);

if nnz(rolling) < 5000
    C = empty_result(C, sprintf('only %d rolling pure-lateral samples', nnz(rolling)));
    return
end

gam_levels = discrete_levels(D.IA(rolling), GAMMA_STEP, MIN_LEVEL_N);
p_levels   = discrete_levels(D.P(rolling),  P_STEP,     MIN_LEVEL_N);
gam_levels = gam_levels(gam_levels >= 0);          % TTC swept one side only
if ~any(gam_levels == 0) || numel(gam_levels) < 2
    C = empty_result(C, 'no camber sweep found (need a zero level and at least one other)');
    return
end
C.gamma_levels_deg = gam_levels;
C.p_levels_psi     = p_levels;

% ---- build the per-cell response table ----------------------------------
% One cell = (file, load bin, camber level, pressure level). Everything is
% normalised to the gamma = 0 cell of the same file, load and pressure: the
% tire loses grip through a test session (~17% of peak between runs 2 and 3
% of the design tire), so only ratios within one run are comparable.
rowsD = zeros(0,3);   % [gamma, dfz, peak ratio]
rowsC = zeros(0,3);   % [gamma, dfz, cornering-stiffness ratio]
rowsS = zeros(0,3);   % [gamma, dfz, slip-angle offset, deg]
files = unique(D.file);

for fi = 1:numel(files)
  for k = 1:numel(LOAD_BINS_LBF)
    for pj = 1:numel(p_levels)
      cellsel = (D.file == files(fi)) & rolling ...
                & (abs(Fz_lbf - LOAD_BINS_LBF(k)) < LOAD_BINS_LBF(k)*LOAD_BAND) ...
                & (abs(D.P - p_levels(pj)) < P_STEP/2);

      M = cell(1, numel(gam_levels));
      Fz_cell = NaN;
      for gi = 1:numel(gam_levels)
          sel = cellsel & (abs(D.IA - gam_levels(gi)) < GAMMA_STEP/2);
          if nnz(sel) < MIN_CELL_N, break; end
          % Mirror into the model convention (see the header). Do not fold
          % |SA| as pacejka_fit does: that would cancel the camber thrust.
          [a, fy] = binned_signed_curve(-D.SA(sel), D.FY(sel), SA_EDGES, MIN_BIN_N);
          if numel(a) < MIN_CURVE_PTS, break; end
          M{gi} = curve_metrics(a, fy);
          if gi == 1, Fz_cell = mean(Fz_lbf(sel)); end
      end
      if any(cellfun(@isempty, M)), continue; end     % incomplete camber set

      base = M{1};                                    % gamma = 0
      dfz  = (Fz_cell - C.Fz_ref_lbf) / C.Fz_ref_lbf;
      for gi = 2:numel(gam_levels)
          g = gam_levels(gi);
          % Peak: +g reads the branch where thrust helps, -g the branch where
          % it hurts. One camber sweep therefore gives BOTH signs of gamma,
          % via the symmetry assumption stated in the header.
          if all(isfinite([M{gi}.pk_pos M{gi}.pk_neg base.pk_pos base.pk_neg]))
              rowsD(end+1,:) = [+g, dfz, M{gi}.pk_pos / base.pk_pos]; %#ok<AGROW>
              rowsD(end+1,:) = [-g, dfz, M{gi}.pk_neg / base.pk_neg]; %#ok<AGROW>
          end
          % Slip-angle offset: camber thrust as an equivalent slip angle.
          % Odd in gamma.
          if isfinite(M{gi}.a_zero) && isfinite(base.a_zero)
              d = -(M{gi}.a_zero - base.a_zero);
              rowsS(end+1,:) = [+g, dfz, +d]; %#ok<AGROW>
              rowsS(end+1,:) = [-g, dfz, -d]; %#ok<AGROW>
          end
          % Cornering stiffness: even in gamma.
          if isfinite(M{gi}.Ca) && isfinite(base.Ca)
              r = M{gi}.Ca / base.Ca;
              rowsC(end+1,:) = [+g, dfz, r]; %#ok<AGROW>
              rowsC(end+1,:) = [-g, dfz, r]; %#ok<AGROW>
          end
      end
    end
  end
end

if size(rowsD,1) < 8 || size(rowsS,1) < 8 || size(rowsC,1) < 4
    C = empty_result(C, sprintf(['too few complete camber cells (peak %d, ' ...
        'offset %d, stiffness %d rows)'], size(rowsD,1), size(rowsS,1), size(rowsC,1)));
    return
end

% ---- least squares ------------------------------------------------------
% Forms chosen by leave-one-load-bin-out and leave-one-file-out
% cross-validation.
%
%   peak factor        fD  = 1 + (kD1 + kD2*dfz + kD3*dfz^2)*gamma + kD4*gamma^2
%   stiffness factor   fC  = 1 + kC1*gamma^2
%   slip-angle offset  dSH = (kS1 + kS2*dfz)*gamma            [deg]
%
% The peak factor splits cleanly into two physical pieces, which is why it has
% the shape it does. Decomposing the measured ratios into their EVEN and ODD
% parts in gamma:
%
%   EVEN part (same whichever way the tire leans): the contact-patch
%   penalty, -3.4% to -4.3% at gamma = 4 in every load bin, i.e. flat in
%   load -> the kD4*gamma^2 term.
%
%   ODD part (helps one way, hurts the other): +2.9% at 50 lbf and -2.9% at
%   250 lbf at gamma = 2 - it changes sign with load -> the
%   (kD1 + kD2*dfz + kD3*dfz^2)*gamma term. The dfz^2 term captures its
%   flattening above ~200 lbf (leave-one-load-out RMS 0.023 -> 0.014).
%
% The stiffness drop needed no load dependence.
g = rowsD(:,1);  z = rowsD(:,2);
A = [g, g.*z, g.*z.^2, g.^2];  C.kD = (A \ (rowsD(:,3) - 1)).';
C.rms_peak   = rms_of(A*C.kD.' - (rowsD(:,3) - 1));

g = rowsC(:,1);
A = g.^2;                   C.kC = (A \ (rowsC(:,3) - 1)).';
C.rms_stiff  = rms_of(A*C.kC.' - (rowsC(:,3) - 1));

g = rowsS(:,1);  z = rowsS(:,2);
A = [g, g.*z];              C.kS = (A \ rowsS(:,3)).';
C.rms_offset = rms_of(A*C.kS.' - rowsS(:,3));

% ---- provenance and guard rails -----------------------------------------
C.n_peak    = size(rowsD,1);
C.n_stiff   = size(rowsC,1);
C.n_offset  = size(rowsS,1);
C.gamma_max_deg = max(gam_levels);          % tire_camber clamps camber to this
C.Fz_min_lbf    = min(LOAD_BINS_LBF);       % ... and load to this range
C.Fz_max_lbf    = max(LOAD_BINS_LBF);
C.status  = 'ok';
C.basis   = 'ttc-camber-sweep, rolling only, within-file normalised';

if verbose
    fprintf('--- %s camber (%d/%d/%d rows; levels %s deg; %s psi)\n', prefix, ...
            C.n_peak, C.n_stiff, C.n_offset, mat2str(gam_levels), mat2str(p_levels));
    fprintf('      peak   fD  = 1 + (%+.5f %+.5f*dfz %+.5f*dfz^2)*g %+.6f*g^2  (RMS %.4f)\n', ...
            C.kD(1), C.kD(2), C.kD(3), C.kD(4), C.rms_peak);
    fprintf('      stiff  fC  = 1 %+.6f*g^2                          (RMS %.4f)\n', ...
            C.kC(1), C.rms_stiff);
    fprintf('      thrust dSH = (%+.5f %+.5f*dfz)*g deg              (RMS %.4f)\n', ...
            C.kS(1), C.kS(2), C.rms_offset);
end
end

% =========================================================================
function C = empty_result(C, why)
% No usable camber sweep: coefficients exactly zero (camber changes
% nothing), never NaN.
C.kD = [0 0 0 0];  C.kC = 0;  C.kS = [0 0];
C.rms_peak = NaN; C.rms_stiff = NaN; C.rms_offset = NaN;
C.n_peak = 0; C.n_stiff = 0; C.n_offset = 0;
C.gamma_levels_deg = 0;  C.p_levels_psi = [];
C.gamma_max_deg = 0;  C.Fz_min_lbf = NaN;  C.Fz_max_lbf = NaN;
C.status = ['none: ' why];
C.basis  = 'no camber data - camber terms disabled';
end

% =========================================================================
function lv = discrete_levels(x, step, min_n)
% Set points the rig held (recorded channels wander around them): round to
% the grid and keep levels with enough data.
r  = round(x(:) / step) * step;
u  = unique(r);
lv = u(arrayfun(@(v) nnz(r == v) >= min_n, u)).';
end

% =========================================================================
function [x, y] = binned_signed_curve(sa, fy, edges, min_n)
% Median force in each SIGNED slip-angle bin (folding would destroy the
% left/right asymmetry camber creates; medians resist sweep hysteresis).
x = zeros(0,1); y = zeros(0,1);
for i = 1:numel(edges)-1
    m = (sa >= edges(i)) & (sa < edges(i+1));
    if nnz(m) > min_n
        x(end+1,1) = mean(sa(m));    %#ok<AGROW>
        y(end+1,1) = median(fy(m));  %#ok<AGROW>
    end
end
end

% =========================================================================
function m = curve_metrics(a, fy)
% Read the three numbers we need straight off one binned Fy(alpha) curve.
%   a  : signed slip angle [deg], model convention (positive alpha -> positive Fy)
%   fy : lateral force [lbf]
[a, k] = sort(a(:));  fy = fy(k);
pos = a >= 0;  neg = a <= 0;
m.pk_pos = local_peak(a(pos),  fy(pos));    % peak with camber thrust helping
m.pk_neg = local_peak(a(neg), -fy(neg));    % ... and with it hurting
m.a_zero = zero_crossing(a, fy);            % slip angle where Fy = 0
m.Ca     = low_slip_slope(a, fy, 1.5);      % [lbf/deg] near alpha = 0
end

function pk = local_peak(x, y)
% Largest sample, refined by a parabola through it and its neighbours.
if numel(y) < 3, pk = max(y); if isempty(pk), pk = NaN; end, return; end
[~, i] = max(y);
lo = max(1, i-2);  hi = min(numel(y), i+2);
if hi - lo < 2, pk = y(i); return; end
c = polyfit(x(lo:hi), y(lo:hi), 2);
pk = y(i);
if c(1) < 0
    xv = -c(2) / (2*c(1));
    if xv >= min(x(lo:hi)) && xv <= max(x(lo:hi)), pk = polyval(c, xv); end
end
end

function a0 = zero_crossing(a, fy)
% Slip angle at zero force. Its shift with camber is the camber thrust
% expressed as a slip angle (the small offset at zero camber is ply steer
% and conicity).
a0 = NaN;
s = sign(fy);  s(s == 0) = 1;
k = find(diff(s) ~= 0);
if isempty(k), return; end
[~, j] = min(abs(a(k)));          % the crossing nearest zero slip
i = k(j);
den = fy(i+1) - fy(i);
if den == 0, return; end
a0 = a(i) - fy(i) * (a(i+1) - a(i)) / den;
end

function Ca = low_slip_slope(a, fy, halfwidth_deg)
% Cornering stiffness: straight line through the bins within +/-1.5 deg.
m = abs(a) <= halfwidth_deg;
if nnz(m) < 4, Ca = NaN; return; end
c  = polyfit(a(m), fy(m), 1);
Ca = c(1);
end

function r = rms_of(e)
r = sqrt(mean(e.^2));
end

% =========================================================================
function D = load_camber_channels(data_dir, pattern)
% The channels camber_fit needs, plus a file index so ratios can be taken
% inside one run. A file missing any channel is skipped whole.
channels = {'SA','FY','FX','FZ','IA','P','V'};
files    = dir(fullfile(data_dir, pattern));
chunks   = cell(numel(files), numel(channels));
tags     = cell(numel(files), 1);
for i = 1:numel(files)
    if contains(files(i).name, 'raw'), continue; end
    S = load(fullfile(data_dir, files(i).name));
    if ~all(isfield(S, channels)), continue; end
    for c = 1:numel(channels)
        chunks{i,c} = S.(channels{c})(:);
    end
    tags{i} = i * ones(numel(S.FZ), 1);
end
for c = 1:numel(channels)
    D.(channels{c}) = vertcat(chunks{:,c});
end
D.file = vertcat(tags{:});
end
