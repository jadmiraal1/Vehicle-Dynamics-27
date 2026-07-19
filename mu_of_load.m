function mu = mu_of_load(p, Fz_lbf)
% MU_OF_LOAD  Load-sensitive peak lateral mu of the DESIGN tire, DERATED.
% The single evaluator for mu(Fz) - build_tire_coeffs, axle_grip and the tire
% report all call this so they cannot disagree. Vectorized in Fz_lbf.
%
%   mu = mu_of_load(p, Fz_lbf)
%
% WHY THIS IS NOT JUST polyval(p.mu_coef, Fz)
% -------------------------------------------
% mu_coef is a straight line fitted to the design tire (LC0_16x75) over the
% loads it was actually tested at: ~48-197 lbf. But at the cornering limit the
% loaded OUTER tire runs ~319 lbf - 62% past that data. A straight line is the
% WORST thing to extrapolate with: it keeps dropping, eventually goes negative,
% and it is steepest exactly where there is no data. The design tire has steep
% low-load sensitivity (-2.6 mu/1000 lbf), so blind-linear says mu ~ 1.33 at
% 319 lbf - almost certainly too pessimistic.
%
% Real tires FLATTEN with load, and we can prove it from data we already own:
% the three 18in tires (incl. the same LC0 compound) carry lateral data to
% ~322-360 lbf, and their peak-in-sweep mu(Fz) is nearly flat above 200 lbf
% (GY: 2.462 @200 -> 2.413 @264). Crucially the NORMALIZED load-sensitivity
% shape mu(Fz)/mu(150) agrees across all four tires to ~1-1.5% (tighter at low
% load, ~1.5% at 200 lbf, LC0-driven), so that shape is a tire-property we can
% transfer to the design tire even though absolute mu is not. build_tire_coeffs
% extracts a gentle high-load slope from that pooled shape -> p.mu_hiload_slope.
%
% PIECEWISE LAW (continuous at the edge):
%   Fz <= p.Fz_fit_max :  measured line          polyval(mu_coef, Fz)
%   Fz >  p.Fz_fit_max :  mu(edge) + slope_hi*(Fz - edge)     [donor-informed]
%
% It is still extrapolation above the ~264 lbf donor ceiling - just far better
% supported than a straight line. BAND / bracketing: set p.tire_hiload = 'low'
% to force the pessimistic blind-linear extension instead; the gap between the
% two IS the honest extrapolation uncertainty.

% --- Input validation -----------------------------------------------------
required = {'Fz_fit_max', 'mu_coef', 'mu_derate'};
for k = 1:numel(required)
    if ~isfield(p, required{k})
        error('mu_of_load:missingField', ...
            'p is missing required field "%s" - was it loaded from tire_coeffs.mat?', required{k});
    end
end

mode = 'central';
if isfield(p, 'tire_hiload') && ~isempty(p.tire_hiload), mode = p.tire_hiload; end
mode = lower(mode);
if ~ismember(mode, {'central', 'low'})
    error('mu_of_load:badMode', ...
        'p.tire_hiload = ''%s'' is not recognized - use ''central'' or ''low''.', mode);
end
if strcmp(mode, 'central') && ~isfield(p, 'mu_hiload_slope')
    error('mu_of_load:missingField', ...
        'p.tire_hiload = ''central'' requires p.mu_hiload_slope (from donor_hiload_slope).');
end

Fz = max(Fz_lbf(:).', 25);          % row, floor at 25 lbf (below any data)
if any(Fz_lbf(:) < 25)
    warning('mu_of_load:belowFloor', ...
        'Fz_lbf contains values below 25 lbf (min %.1f) - clamped to 25. Check for a units or sign error.', ...
        min(Fz_lbf(:)));
end
edge = p.Fz_fit_max;

if strcmp(mode, 'low')
    mu_raw = polyval(p.mu_coef, Fz);                 % blind linear (pessimistic)
else
    mu_raw = polyval(p.mu_coef, min(Fz, edge));      % measured, in range
    hi     = Fz > edge;
    mu_raw(hi) = polyval(p.mu_coef, edge) + p.mu_hiload_slope .* (Fz(hi) - edge);

    % Surface the "beyond even donor coverage" warning here, not just in
    % build_tire_coeffs, so axle_grip / the tire report get it too.
    if isfield(p, 'hiload_cov_lbf') && any(Fz(hi) > p.hiload_cov_lbf)
        n_beyond = nnz(Fz(hi) > p.hiload_cov_lbf);
        warning('mu_of_load:beyondDonorCoverage', ...
            ['%d of %d requested load(s) exceed even the donor coverage (%.0f lbf).\n' ...
             'mu there rests on the donor slope extended past its own support - ' ...
             'run with p.tire_hiload = ''low'' too and report the band.'], ...
            n_beyond, numel(Fz), p.hiload_cov_lbf);
    end
end

% Physical floor: a straight-line extrapolation (especially 'low' mode) can run
% to zero or negative at a high enough load. mu <= 0 is not tire behavior, it is
% the extrapolation failing - clamp and say so rather than passing it downstream.
MU_FLOOR = 0.1;
below = mu_raw < MU_FLOOR;
if any(below)
    warning('mu_of_load:floored', ...
        '%d of %d mu value(s) fell below the physical floor (%.2f) and were clamped - extrapolation has likely gone too far.', ...
        nnz(below), numel(mu_raw), MU_FLOOR);
    mu_raw(below) = MU_FLOOR;
end

mu = reshape(mu_raw * p.mu_derate, size(Fz_lbf));
end
