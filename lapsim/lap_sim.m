function [v, t, E] = lap_sim(p, s, kappa, v0, closed)
% LAP_SIM  Quasi-steady-state (QSS) lap solver.
%   [v, t, E] = lap_sim(p, s, kappa, v0, closed)
%
%   s       distance along the centreline [m] (node positions)
%   kappa   path curvature [1/m]; the sign (left/right) is ignored
%   v0      start speed [m/s], or [] (default: free)
%   closed  true = a lap that ends where it starts (endurance);
%           false = an open course from v0 (accel, autocross)
%   v       speed at each node [m/s];  t lap time [s];  E energy (lap_energy)
%
% The car is a point that follows the centreline. Its acceleration limits
% come from the vehicle models:
%   - lateral limit ay(v) from ay_limit (p.grip_model; default 'axle':
%     per-axle load transfer with the load-sensitive tire);
%   - longitudinal limit from ax_combined per axle with the measured friction
%     ellipse (p.long_model = 'combined', default), or from ax_limit's
%     straight-line edges scaled by a car-level ellipse ('axle'/'pointmass').
% Method: a speed ceiling from the corner limit at every node, a forward pass
% (accelerate), a backward pass (brake), each node taking the lower speed.
% A closed lap repeats the passes with the end speed fed back to the start.
% No yaw dynamics, no transients, no racing line.
% Theory: VD_physics_reference.md sec 7.

if nargin < 4, v0 = []; end
if nargin < 5, closed = true; end
s = s(:);
kappa = abs(kappa(:));     % the limits are symmetric left/right
n = numel(s);
ds = diff(s);

% Each edge model is a solver (bisection / fixed point). Tabulate them once
% per call and interpolate; per-point calls would be ~100x slower for
% ~1e-4 g of difference.
ayf  = vlut(p, @(vv) ay_limit(p, vv));
comb = isfield(p, 'long_model') && strcmpi(p.long_model, 'combined');
if comb
    % 2-D table in speed x lateral-usage fraction (ay/ay_max)
    vg = linspace(0, p.v_max, 24);
    rg = linspace(0, 1, 13);
    [VV, RR] = ndgrid(vg, rg);
    Aacc = arrayfun(@(vv, rr) ax_combined(p, vv, rr*ayf(vv), 'accel'), VV, RR);
    Abrk = arrayfun(@(vv, rr) ax_combined(p, vv, rr*ayf(vv), 'brake'), VV, RR);
    Fa = griddedInterpolant({vg, rg}, Aacc, 'linear');
    Fb = griddedInterpolant({vg, rg}, Abrk, 'linear');
else
    axaf = vlut(p, @(vv) ax_limit(p, vv, 'accel'));
    axbf = vlut(p, @(vv) ax_limit(p, vv, 'brake'));
end
vlim = arrayfun(@(k) corner_speed(p, k, ayf), kappa);   % speed ceiling per node
v = vlim;
if ~isempty(v0), v(1) = v0; end

n_d = ellipse_exp(p, 'drive');   % used by the car-level ellipse only
n_b = ellipse_exp(p, 'brake');

niter = 3;  if ~closed, niter = 1; end
for it = 1:niter
    for i = 1:n-1                                   % forward: accelerate
        used = min(v(i)^2*kappa(i) / max(ayf(v(i))*p.g, 1e-6), 1.0);
        if comb
            ax = Fa(min(max(v(i),0),p.v_max), used) * p.g;
        else
            ax = axaf(v(i)) * p.g * (max(0, 1 - used^n_d))^(1/n_d);
        end
        v(i+1) = min(vlim(i+1), sqrt(max(v(i)^2 + 2*ax*ds(i), 0)));
    end
    for i = n:-1:2                                  % backward: brake
        used = min(v(i)^2*kappa(i) / max(ayf(v(i))*p.g, 1e-6), 1.0);
        if comb
            ax = Fb(min(max(v(i),0),p.v_max), used) * p.g;
        else
            ax = axbf(v(i)) * p.g * (max(0, 1 - used^n_b))^(1/n_b);
        end
        v(i-1) = min(v(i-1), sqrt(v(i)^2 + 2*ax*ds(i-1)));
    end
    if closed, v(1) = v(end); elseif ~isempty(v0), v(1) = v0; end
end

vm = 0.5*(v(1:end-1) + v(2:end));
t  = sum(ds ./ max(vm, 0.1));    % 0.1 m/s floor: a standing start has v = 0

if nargout > 2
    E = lap_energy(p, v, s);
end
end

function n = ellipse_exp(p, mode)
% Friction-ellipse exponent: (ax/ax_max)^n + (ay/ay_max)^n = 1.
switch lower(mode)
    case 'brake', f = 'n_env_brake';
    otherwise,    f = 'n_env_drive';
end
if isfield(p, f) && isfinite(p.(f)) && p.(f) > 0
    n = p.(f);
else
    n = 2;   % no measured exponent: a circle
end
end

function h = vlut(p, fh, npts)
% Tabulate a function of speed on [0, v_max] and return a fast interpolant.
% pchip is shape-preserving: monotone data stays monotone, no overshoot.
if nargin < 3 || isempty(npts), npts = 120; end
vmax = p.v_max;
vg = linspace(0, vmax, npts);
vals = arrayfun(fh, vg);
F = griddedInterpolant(vg, vals, 'pchip');
h = @(v) F(min(max(v, 0), vmax));
end

function E = lap_energy(p, v, s)
% Energy over the speed trace, summed from lap_duty_cycle.
%   drive_wheel_Wh   positive tractive work at the wheels
%   drive_acc_Wh     energy drawn from the accumulator (through chain,
%                    inverter and the motor map)
%   brake_wheel_Wh   braking work at the wheels: the upper bound on regen
D = lap_duty_cycle(p, v, s);

J_PER_WH = 3600;
E.drive_wheel_Wh = sum(max(D.F_thrust, 0) .* D.ds) / J_PER_WH;
if isfield(p, 'eta_chain') && isfield(p, 'eta_inv')
    % The 0.5 floor matches motor_eff's clamp and keeps a NaN out of the sum.
    E.drive_acc_Wh = sum(max(D.F_thrust, 0) .* D.ds ./ max(D.eta_e, 0.5)) / J_PER_WH;
else
    E.drive_acc_Wh = E.drive_wheel_Wh / p.eta_dt;      % flat efficiency fallback
end
E.brake_wheel_Wh = sum(max(-D.F_thrust, 0) .* D.ds) / J_PER_WH;
end
