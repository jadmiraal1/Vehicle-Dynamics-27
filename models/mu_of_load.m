function mu = mu_of_load(p, Fz_lbf, gamma_deg)
% MU_OF_LOAD  Load-sensitive peak lateral mu of the design tire, derated. Vectorized.
%   mu = mu_of_load(p, Fz_lbf)              zero camber
%   mu = mu_of_load(p, Fz_lbf, gamma_deg)   with camber (gamma > 0 = helpful lean)
%
% Up to the edge of the tire data (p.Fz_fit_max) mu follows the fitted line
% p.mu_coef. Above it, p.tire_hiload selects the extrapolation:
%   'central'  continue with p.mu_hiload_slope (donor-tire informed)
%   'low'      continue the fitted line (pessimistic bracket)
% Camber multiplies the result by tire_camber's peak factor; with gamma = 0
% that factor is exactly 1, so the two-argument call is unchanged.
% Theory: VD_physics_reference.md sec 13 (load) and 8b (camber).
%
% Hot path (~1e5 calls per g-g-V surface): the argument checks are written
% cheapest-first and only do expensive work when something is wrong.

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
    if any(Fz_lbf(:) < 0)
        vd_warn('mu_of_load:negativeLoad', ...
            'A negative tire load (%.1f lbf) reached mu_of_load - check units and sign (it expects lbf).', ...
            min(Fz_lbf(:)));
    else
        % A nearly lifted inside wheel: legitimate, and its force is small.
        vd_warn('mu_of_load:belowFloor', ...
            'Tire loads under 25 lbf (a nearly lifted inside wheel) are evaluated at 25 lbf.');
    end
end
edge = p.Fz_fit_max;

if strcmp(mode, 'low')
    mu_raw = polyval(p.mu_coef, Fz);                 % blind linear (pessimistic)
else
    mu_raw = polyval(p.mu_coef, min(Fz, edge));      % measured, in range
    hi     = Fz > edge;
    mu_raw(hi) = polyval(p.mu_coef, edge) + p.mu_hiload_slope .* (Fz(hi) - edge);

    % Past every tire's data: say so once per run.
    if isfield(p, 'hiload_cov_lbf') && any(Fz(hi) > p.hiload_cov_lbf)
        vd_warn('mu_of_load:beyondDonorCoverage', ...
            ['Tire loads above %.0f lbf, where the tire data ends: grip there is ' ...
             'extrapolated (see docs/STATUS.md, Tire model).'], p.hiload_cov_lbf);
    end
end

% A straight-line extrapolation can reach zero at a high enough load. That is
% the extrapolation failing, not tire behaviour: clamp and warn.
MU_FLOOR = 0.1;
below = mu_raw < MU_FLOOR;
if any(below)
    vd_warn('mu_of_load:floored', ...
        'Extrapolated tire grip fell below %.2f and was held there: the tire load is far beyond the data.', ...
        MU_FLOOR);
    mu_raw(below) = MU_FLOOR;
end

mu = reshape(mu_raw * p.mu_derate, size(Fz_lbf));

% --- camber ---------------------------------------------------------------
% The load curve is fitted to near-zero-camber data and the camber factor is
% a ratio against that same condition, so the two multiply.
if nargin >= 3 && ~isempty(gamma_deg) && any(gamma_deg(:) ~= 0)
    fD = tire_camber(p, reshape(Fz, size(Fz_lbf)), gamma_deg);
    mu = mu .* fD;
end
end

function explain_missing(p, required)
% Off the hot path: name the missing field.
for k = 1:numel(required)
    if ~isfield(p, required{k})
        error('mu_of_load:missingField', ...
            ['p is missing required field "%s" - was it loaded from ' ...
             'tire_coeffs_<CAR>.mat?'], required{k});
    end
end
end
