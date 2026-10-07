function [t, v_end] = accel_time(p, dist_m)
% ACCEL_TIME  Standing-start time over a straight [s]. The one 75 m integrator.
%   [t, v_end] = accel_time(p)            75 m (fsae_rules().accel_m)
%   [t, v_end] = accel_time(p, dist_m)
%
% Integrates in fixed VELOCITY steps: within one dv step the acceleration
% (looked up from v) is nearly constant, so x = v*dt + a*dt^2/2 is close to
% exact and the result converges quickly (it moves 0.0002 s between
% dv = 0.02 and 0.001 m/s). Stepping distance and advancing time by
% dx/v_end instead reads fast unless the step is very small.
%
% Assumptions:
% - Straight line, no lateral demand. Traction comes from ax_limit, so it is
%   load-sensitive (p.long_model applies) and capped by the motor.
% - The car starts from rest AT the timing line. The rules stage the car
%   0.3 m behind the line (D.9.2.3); that roll-in is not modelled.
% - At the rev limit (or if drag ever matches thrust) the car holds its
%   speed to the line.
% - The loop stops on the first step past dist_m: t overshoots by at most
%   one step (~0.001 s).

if nargin < 2 || isempty(dist_m), dist_m = fsae_rules().accel_m; end
assert(dist_m > 0, 'accel_time:badDistance', 'dist_m must be positive.');

DV_MPS  = 0.005;    % velocity step [m/s]
N_LUT   = 60;       % speeds at which ax_limit is solved, then interpolated

vg  = linspace(0, p.v_max, N_LUT);
axf = arrayfun(@(vv) ax_limit(p, vv, 'accel'), vg);   % [g]

v = 0;  x = 0;  t = 0;
while x < dist_m
    if v >= p.v_max                       % rev limiter: hold v_max to the line
        t = t + (dist_m - x) / p.v_max;
        v = p.v_max;
        break
    end
    a = interp1(vg, axf, min(max(v, 0), p.v_max)) * p.g;   % [m/s^2]
    if a <= 0                             % drag ceiling: cruise to the line
        t = t + (dist_m - x) / max(v, eps);
        break
    end
    dt = DV_MPS / a;
    x  = x + v*dt + 0.5*a*dt^2;
    t  = t + dt;
    v  = v + DV_MPS;
end
v_end = v;
end
