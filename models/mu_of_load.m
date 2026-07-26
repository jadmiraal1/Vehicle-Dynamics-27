function mu = mu_of_load(p, Fz_lbf)
% MU_OF_LOAD  Load-sensitive peak lateral mu of the design tire, derated. Vectorized.
% Measured fit to Fz_fit_max, donor-informed slope above; p.tire_hiload='low' brackets.
% Theory: references/VD_physics_reference.md sec 13.

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
