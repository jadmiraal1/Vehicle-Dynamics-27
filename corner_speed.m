function vc = corner_speed(p, kappa)
% CORNER_SPEED  Max steady speed [m/s] through curvature kappa [1/m].
% Bisection on v^2*kappa <= ay_max(v)*g; straights return the rev limit.
% The lateral limit ay_max(v) comes from ay_limit.m (p.grip_model: load-sensitive
% 'axle' by default, or point-mass 'pointmass'), NOT gg_envelope directly - so the
% corner-speed ceiling and lap_sim's ellipse share one lateral limit.
% VD_physics_reference.md, section 7.
if kappa < 1e-6, vc = p.v_max; return; end
% lo = 0 is PROVABLY feasible: at v=0 the cornering demand v^2*kappa = 0 <=
% ay_max(0)*g for any positive grip, so the bracket [lo,hi] is guaranteed to
% straddle the crossing. (The old lo = 0.5 m/s ASSUMED feasibility; if a corner's
% true limit fell below 0.5 - very tight curvature or badly dropped grip - the
% bisection would have returned 0.5, an OPTIMISTIC overspeed. For real corners the
% crossing is >> 0.5, so this changes nothing numerically.)
lo = 0;    hi = p.v_max;
if hi^2*kappa <= ay_limit(p, hi) * p.g     % even top speed is below the limit
    vc = hi;
    return;
end
% NITER=30 resolves v to sub-um/s over a ~25 m/s bracket - the old 60 was
% needless precision that also doubled the cost of every trial, since each
% ay_limit(p, mid) call is itself a full nested bisection in axle_grip.
NITER = 30;
for k = 1:NITER
    mid = 0.5*(lo + hi);
    if mid^2*kappa <= ay_limit(p, mid) * p.g
        lo = mid;
    else
        hi = mid;
    end
end
vc = lo;
end
