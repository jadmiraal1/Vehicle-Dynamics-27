function out = axle_grip(p, v)
% AXLE_GRIP  Mid-tier lateral limit: per-axle saturation with lateral load
% transfer and tire load sensitivity. The physics the point mass cannot see:
% transferring load to the outside tire LOSES total grip because mu falls
% with load (mu_coef), so ay_lim < mu*g and depends on LLTD, track, CoP.
% Theory: VD_physics_reference.md, section 11.
%
%   out = axle_grip(p, v)   v = speed [m/s] (sets downforce)
%
% Knobs read from p: LLTD (front fraction of lateral transfer, set by roll
% stiffness split), aero_df_front (CoP), ClA, plus the usual mass/geometry.
%
% CAVEAT: single-knob LLTD (no roll-center geometry, no unsprung split),
% no camber, symmetric left/right, steady state.

DF   = 0.5 * p.rho * p.ClA * v^2;                      % downforce [N]
W_f  = p.m * p.g * p.mass_dist_f       + p.aero_df_front     * DF;
W_r  = p.m * p.g * (1 - p.mass_dist_f) + (1 - p.aero_df_front) * DF;

% Demands from steady-state moment balance: Fy_f = m*ay*b/L, Fy_r = m*ay*a/L
dem_frac_f = p.b / p.L;
dem_frac_r = p.a / p.L;

% Bisection on ay: capacity - demand crosses zero once. Bracket is physically
% bounded, not an arbitrary wide guess: the axle model can only LOSE grip
% relative to the constant-mu point mass (that is the whole point of this
% model - vd_selftest enforces ay_lim < mu_y), so 1.5*mu_y*g is comfortably
% above the true root across the LLTD/CoP sweeps in run_balance_targets,
% without re-opening a ~4.6g bracket whose early trial points push tire
% loads far past any data support. min(45, ...) is a hard fallback only.
% NITER=30 still resolves ay to ~1e-6 m/s^2 (30 halvings of a ~20 m/s^2
% bracket) - the old 60 bought precision nobody needed, at the cost of
% doubling every downstream nested bisection (corner_speed, lap_sim).
NITER = 30;
lo = 0;  hi = min(45, 1.5 * p.mu_y * p.g);             % [m/s^2]

% The interior search deliberately walks through unphysical trial ay values
% (that's inherent to bisection needing a bracket wider than the answer) -
% those trials can push tire loads past mu_of_load's data-support range and
% fire its 'beyondDonorCoverage' / 'belowFloor' warnings thousands of times
% per lap_sim call for no diagnostic value. Silence them for the search only;
% the final recompute below runs with warnings restored, so a converged
% answer that is genuinely out of range still warns exactly once.
w1 = warning('off', 'mu_of_load:beyondDonorCoverage');
w2 = warning('off', 'mu_of_load:belowFloor');
restore1 = onCleanup(@() warning(w1.state, w1.identifier));
restore2 = onCleanup(@() warning(w2.state, w2.identifier));

for it = 1:NITER
    ay = (lo + hi) / 2;
    [cap_f, cap_r] = capacities(p, W_f, W_r, ay);
    if min(cap_f - p.m*ay*dem_frac_f, cap_r - p.m*ay*dem_frac_r) > 0
        lo = ay;
    else
        hi = ay;
    end
end
ay = lo;

clear restore1 restore2   % re-enable both warnings before the final, real evaluation

[cap_f, cap_r, Fz, lift] = capacities(p, W_f, W_r, ay);
dem_f = p.m * ay * dem_frac_f;
dem_r = p.m * ay * dem_frac_r;

out.ay_lim_g   = ay / p.g;
out.v          = v;
if (cap_f - dem_f) < (cap_r - dem_r), out.limiting = 'front';
else,                                 out.limiting = 'rear';
end
out.wheel_lift = lift;
out.Fz         = Fz;          % [N] fields: fo fi ro ri (outer/inner)
out.cap_f = cap_f;  out.cap_r = cap_r;
out.dem_f = dem_f;  out.dem_r = dem_r;
out.W_f   = W_f;    out.W_r   = W_r;
end


function [cap_f, cap_r, Fz, lift] = capacities(p, W_f, W_r, ay)
dF_f = p.LLTD       * p.m * ay * p.h_cg / p.t_f;
dF_r = (1 - p.LLTD) * p.m * ay * p.h_cg / p.t_r;

[cap_f, Fz.fo, Fz.fi, lf] = axle_cap(p, W_f, dF_f);
[cap_r, Fz.ro, Fz.ri, lr] = axle_cap(p, W_r, dF_r);
lift = lf || lr;
end


function [cap, F_out, F_in, lifted] = axle_cap(p, W, dF)
F_out = W/2 + dF;
F_in  = W/2 - dF;
lifted = F_in <= 0;
if lifted                       % inner wheel off the ground
    F_out = W;  F_in = 0;
    cap = mu_of(p, F_out) * F_out;
else
    cap = mu_of(p, F_out) * F_out + mu_of(p, F_in) * F_in;
end
end


function mu = mu_of(p, Fz_N)
% Load-sensitive friction, DERATED, via the shared evaluator. Below the design
% tire's data edge this is the measured linear fit; above it, the donor-informed
% flattening (mu_of_load.m) instead of a blind linear run-off. Set
% p.tire_hiload = 'low' to bracket with the pessimistic linear extension.
mu = mu_of_load(p, Fz_N / 4.44822);
end
