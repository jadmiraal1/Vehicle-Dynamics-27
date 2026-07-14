function vc = corner_speed(p, kappa)
% CORNER_SPEED  Max steady speed [m/s] through curvature kappa [1/m].
% Bisection on v^2*kappa <= ay_max(v)*g; straights return the rev limit.
% VD_physics_reference.md, section 7.
if kappa < 1e-6, vc = p.v_max; return; end
lo = 0.5;  hi = p.v_max;
if hi^2*kappa <= gg_envelope(p, hi).ay * p.g     % even top speed is below the limit
    vc = hi;  
    return;
end
for k = 1:60
    mid = 0.5*(lo + hi);
    if mid^2*kappa <= gg_envelope(p, mid).ay * p.g
        lo = mid; 
    else
        hi = mid; 
    end
end
vc = lo;
end
