function vc = corner_speed(p, kappa, ayfun)
% CORNER_SPEED  Highest steady speed [m/s] through curvature kappa [1/m].
%   vc = corner_speed(p, kappa)          uses ay_limit
%   vc = corner_speed(p, kappa, ayfun)   ayfun(v) [g]: a precomputed lateral
%                                        limit (lap_sim passes one)
% Solves v^2*|kappa| = ay_max(v)*g by bisection, capped at the rev-limited
% top speed p.v_max. Assumes a single crossing: the cornering demand v^2*kappa
% grows faster with speed than the downforce-assisted grip.

if nargin < 3 || isempty(ayfun)
    ayfun = @(v) ay_limit(p, v);
end
kappa = abs(kappa);
if kappa < 1e-6, vc = p.v_max; return; end

lo = 0;    hi = p.v_max;             % v = 0 is always feasible
if hi^2*kappa <= ayfun(hi) * p.g     % the corner does not limit below top speed
    vc = hi;
    return;
end
NITER = 30;                          % resolves v to well under 1e-6 m/s
for k = 1:NITER
    mid = 0.5*(lo + hi);
    if mid^2*kappa <= ayfun(mid) * p.g
        lo = mid;
    else
        hi = mid;
    end
end
vc = lo;
end
