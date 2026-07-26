function vc = corner_speed(p, kappa, ayfun)
% CORNER_SPEED  Max steady speed [m/s] through curvature kappa [1/m].
% Bisection on v^2*kappa <= ay_max(v)*g. ayfun (optional): precomputed ay(v)
% handle (lap_sim builds one per lap); omitted -> ay_limit directly.
if nargin < 3 || isempty(ayfun)
    ayfun = @(v) ay_limit(p, v);
end
if kappa < 1e-6, vc = p.v_max; return; end
% lo = 0 is provably feasible: at v=0 the cornering demand v^2*kappa = 0 <=
lo = 0;    hi = p.v_max;
if hi^2*kappa <= ayfun(hi) * p.g     % even top speed is below the limit
    vc = hi;
    return;
end
% NITER=30 resolves v to sub-um/s over a ~25 m/s bracket - the old 60 was
% needless precision that also doubled the cost of every trial, since each
% ay_limit(p, mid) call is itself a full nested bisection in axle_grip.
NITER = 30;
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
