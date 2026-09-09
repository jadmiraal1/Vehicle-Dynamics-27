function mu = mu_of_load(p, Fz_lbf, gamma_deg)
% MU_OF_LOAD  Load-sensitive peak lateral mu of the design tire, derated. Vectorized.
%   mu = mu_of_load(p, Fz_lbf)              zero camber - unchanged behaviour
%   mu = mu_of_load(p, Fz_lbf, gamma_deg)   with camber
%
% Measured fit to Fz_fit_max, donor-informed slope above; p.tire_hiload='low' brackets.
% Camber multiplies the result by tire_camber's peak factor fD; gamma > 0 is the
% helpful lean (see models/tire_camber.m for the sign). Calling this with two
% arguments is EXACTLY the old function - fD(0) = 1 identically, not to within a
% tolerance - which is what tests/vd_selftest.m asserts.
% Theory: references/VD_physics_reference.md sec 13 (load), sec 8b (camber).

% --- Input validation -----------------------------------------------------
% HOT PATH. One g-g-V surface calls this ~120,000 times, so the checks below
% are written fast-first: a cheap test that almost always passes, and the slow,
% helpful version only when something is actually wrong.
%
% This is not premature optimisation, it is a measured one. The previous form
% used ismember(mode, {'central','low'}) and a loop of isfield over a cell
% array of names. Measured at 123 us and 19 us per call respectively - so the
% ARGUMENT CHECKING cost more than the physics, and at 120k calls it was
% roughly 17 seconds of a single vd_selftest run. Same checks, same error
% messages, ~40x cheaper.
if ~(isfield(p, 'Fz_fit_max') && isfield(p, 'mu_coef') && isfield(p, 'mu_derate'))
    explain_missing(p, {'Fz_fit_max', 'mu_coef', 'mu_derate'});   % errors
end

mode = 'central';
if isfield(p, 'tire_hiload') && ~isempty(p.tire_hiload), mode = p.tire_hiload; end
if ~(strcmp(mode, 'central') || strcmp(mode, 'low'))
    mode = lower(mode);                       % only pay for lower() off the happy path
    if ~(strcmp(mode, 'central') || strcmp(mode, 'low'))
        error('mu_of_load:badMode', ...
            'p.tire_hiload = ''%s'' is not recognized - use ''central'' or ''low''.', mode);
    end
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

% --- camber ---------------------------------------------------------------
% Separated deliberately: the load curve above is fitted from the near-zero
% camber data, and the camber factor is a RATIO measured against that same
% zero-camber condition. Multiplying is therefore the correct composition, and
% it is also what makes gamma = 0 reduce to the old answer bit for bit.
if nargin >= 3 && ~isempty(gamma_deg) && any(gamma_deg(:) ~= 0)
    fD = tire_camber(p, reshape(Fz, size(Fz_lbf)), gamma_deg);
    mu = mu .* fD;
end
end

function explain_missing(p, required)
% Off the hot path: work out WHICH field is missing and say so.
for k = 1:numel(required)
    if ~isfield(p, required{k})
        error('mu_of_load:missingField', ...
            ['p is missing required field "%s" - was it loaded from ' ...
             'tire_coeffs_<CAR>.mat?'], required{k});
    end
end
end
