function out = axle_grip(p, v)
% AXLE_GRIP  Steady-state lateral limit [g] with per-axle load transfer,
% load-sensitive tire grip and per-corner camber.
%   out = axle_grip(p, v)      v [m/s] sets the downforce
%
% Method (tier T1, "axle QSS"):
%   1. Static axle loads at speed: weight by mass_dist_f, downforce by
%      aero_df_front (road_loads).
%   2. For a trial ay, each axle's lateral load transfer is
%      LLTD_axle * m*ay*h_cg / track_axle, giving outer and inner tire loads.
%   3. Each tire's peak grip comes from mu_of_load at its own load and camber;
%      axle capacity = mu_out*F_out + mu_in*F_in.
%   4. Demand per axle from the steady-state yaw moment balance:
%      front m*ay*b/L, rear m*ay*a/L.
%   5. Bisect on ay for the largest value both axles can carry. The axle
%      that runs out first is out.limiting.
%
% Camber enters through p.camber_deg.{fo,fi,ro,ri} (degrees, positive = the
% helpful lean, see tire_camber). All four default to zero, which reproduces
% the zero-camber model exactly. Nothing computes camber from suspension
% geometry yet, so it is an input.
%
% LLTD here splits the roll MOMENT m*ay*h_cg between axles; with unequal
% tracks the force fractions differ slightly from LLTD.

R   = road_loads(p, v);
W_f = R.Nf;
W_r = R.Nr;

dem_frac_f = p.b / p.L;
dem_frac_r = p.a / p.L;

% Trial ay values walk through unphysical loads; silence mu_of_load's
% warnings during the search and re-enable them for the final evaluation.
w1 = warning('off', 'mu_of_load:beyondDonorCoverage');
w2 = warning('off', 'mu_of_load:belowFloor');
restore1 = onCleanup(@() warning(w1.state, w1.identifier));
restore2 = onCleanup(@() warning(w2.state, w2.identifier));

feasible = @(ay) feasible_at(p, W_f, W_r, ay, dem_frac_f, dem_frac_r);
NITER = 30;
lo = 0;  hi = min(45, 1.5 * p.mu_y * p.g);       % [m/s^2]
while feasible(hi) && hi < 400                   % widen if the car can beat the bracket
    lo = hi;  hi = 2*hi;
end
for it = 1:NITER
    ay = (lo + hi) / 2;
    if feasible(ay), lo = ay; else, hi = ay; end
end
ay = lo;

clear restore1 restore2

[cap_f, cap_r, Fz, lift] = capacities(p, W_f, W_r, ay);
dem_f = p.m * ay * dem_frac_f;
dem_r = p.m * ay * dem_frac_r;

out.ay_lim_g   = ay / p.g;
out.v          = v;
if (cap_f - dem_f) < (cap_r - dem_r), out.limiting = 'front';
else,                                 out.limiting = 'rear';
end
out.wheel_lift = lift;
out.Fz         = Fz;          % [N] per tire: fo fi ro ri (outer/inner)
out.cap_f = cap_f;  out.cap_r = cap_r;
out.dem_f = dem_f;  out.dem_r = dem_r;
out.W_f   = W_f;    out.W_r   = W_r;
end

function ok = feasible_at(p, W_f, W_r, ay, dem_frac_f, dem_frac_r)
[cap_f, cap_r] = capacities(p, W_f, W_r, ay);
ok = min(cap_f - p.m*ay*dem_frac_f, cap_r - p.m*ay*dem_frac_r) > 0;
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
% Per-corner camber [deg]; a missing field means zero.
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
    mu2 = mu_of(p, [F_out F_in], [gam_out gam_in]);   % both tires in one call
    cap = mu2(1) * F_out + mu2(2) * F_in;
end
end

function mu = mu_of(p, Fz_N, gamma_deg)
% Derated, load-sensitive grip with camber. The outer tire carries most of
% the load, so its camber dominates the axle capacity.
mu = mu_of_load(p, Fz_N / p.N_PER_LBF, gamma_deg);
end
