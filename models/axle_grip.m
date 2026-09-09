function out = axle_grip(p, v)
% AXLE_GRIP  Per-axle lateral limit [g] with load transfer, load-sensitive mu
% and per-corner camber.
%   out = axle_grip(p, v)   v sets downforce. Theory: ref doc sec 11, 8b.
%
% CAMBER enters through p.camber_deg.{fo,fi,ro,ri} - the camber each corner
% actually runs, in degrees, positive = the helpful lean (models/tire_camber.m).
% All four default to ZERO, and at zero this function returns exactly the number
% it returned before camber existed. Supplying camber is a deliberate act:
% nothing in the toolchain yet computes it from suspension geometry, so it is an
% input until VD_model_architecture seam 3 (suspension_state) is built.

DF   = 0.5 * p.rho * p.ClA * v^2;                      % downforce [N]
W_f  = p.m * p.g * p.mass_dist_f       + p.aero_df_front     * DF;
W_r  = p.m * p.g * (1 - p.mass_dist_f) + (1 - p.aero_df_front) * DF;

% Demands from steady-state moment balance: Fy_f = m*ay*b/L, Fy_r = m*ay*a/L
dem_frac_f = p.b / p.L;
dem_frac_r = p.a / p.L;

% Bisection on ay: capacity - demand crosses zero once. Bracket is physically
NITER = 30;
lo = 0;  hi = min(45, 1.5 * p.mu_y * p.g);             % [m/s^2]

% The interior search deliberately walks through unphysical trial ay values
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

g = corner_camber(p);
[cap_f, Fz.fo, Fz.fi, lf] = axle_cap(p, W_f, dF_f, g.fo, g.fi);
[cap_r, Fz.ro, Fz.ri, lr] = axle_cap(p, W_r, dF_r, g.ro, g.ri);
lift = lf || lr;
end

function g = corner_camber(p)
% Per-corner camber [deg], defaulting to zero. Kept in one place so the
% "no camber field = old behaviour" rule has exactly one implementation.
g = struct('fo', 0, 'fi', 0, 'ro', 0, 'ri', 0);
if ~isfield(p, 'camber_deg') || isempty(p.camber_deg), return; end
for f = {'fo','fi','ro','ri'}
    if isfield(p.camber_deg, f{1}), g.(f{1}) = p.camber_deg.(f{1}); end
end
end

function [cap, F_out, F_in, lifted] = axle_cap(p, W, dF, gam_out, gam_in)
F_out = W/2 + dF;
F_in  = W/2 - dF;
lifted = F_in <= 0;
if lifted                       % inner wheel off the ground
    F_out = W;  F_in = 0;
    cap = mu_of(p, F_out, gam_out) * F_out;
else
    cap = mu_of(p, F_out, gam_out) * F_out + mu_of(p, F_in, gam_in) * F_in;
end
end

function mu = mu_of(p, Fz_N, gamma_deg)
% Load-sensitive friction, DERATED, with camber, via the shared evaluator.
% Note WHICH tire gets which camber matters here and it is easy to get
% backwards: the outer tire carries most of the load, so its camber dominates
% the axle capacity, and on this tire camber COSTS peak grip at high load.
mu = mu_of_load(p, Fz_N / 4.44822, gamma_deg);
end
