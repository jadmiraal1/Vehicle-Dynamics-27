function [v, t, E] = lap_sim(p, s, kappa, v0, closed)
% LAP_SIM  QSS point-mass lap solver: ceiling pass + forward/backward passes,
% friction ellipse with measured exponents, lateral limit via ay_limit (p.grip_model).
%   [v, t, E] = lap_sim(p, s, kappa, v0, closed)   Theory: ref doc sec 7.

if nargin < 4, 
    v0 = []; 
end
if nargin < 5, 
    closed = true; 
end
s = s(:);  
kappa = kappa(:);  
n = numel(s);  
ds = diff(s);

% Build the ay_limit lookup once (local ay_lut below); ceiling + ellipse share it.
ayf  = ay_lut(p);
vlim = arrayfun(@(k) corner_speed(p, k, ayf), kappa);   % per-point speed ceiling
v = vlim;
if ~isempty(v0), v(1) = v0; end

n_d = ellipse_exp(p, 'drive');   % friction-ellipse exponent, accel (measured)
n_b = ellipse_exp(p, 'brake');   % ... braking

niter = 3;  if ~closed, niter = 1; end
for it = 1:niter
    for i = 1:n-1                                   % forward / accelerate
        G     = gg_envelope(p, v(i));               % longitudinal edges (point mass)
        aymax = ayf(v(i));                          % lateral edge (p.grip_model), via LUT
        used = min(v(i)^2*kappa(i) / max(aymax*p.g, 1e-6), 1.0);
        frac = (max(0, 1 - used^n_d))^(1/n_d);      % friction ellipse, exponent n_d
        ax  = G.ax_accel * p.g * frac;
        v(i+1) = min(vlim(i+1), sqrt(max(v(i)^2 + 2*ax*ds(i), 0))); % min between vlim and accel velocity based on curvature only
    end
    for i = n:-1:2                                  % backward / brake
        G     = gg_envelope(p, v(i));               % longitudinal edges (point mass)
        aymax = ayf(v(i));                          % lateral edge (p.grip_model), via LUT
        used = min(v(i)^2*kappa(i) / max(aymax*p.g, 1e-6), 1.0);
        frac = (max(0, 1 - used^n_b))^(1/n_b);      % friction ellipse, exponent n_b
        ax  = G.ax_brake * p.g * frac;
        v(i-1) = min(v(i-1), sqrt(v(i)^2 + 2*ax*ds(i-1))); % minimum between accel pass velocity and braking velocity based on curvature only
    end
    if closed, v(1) = v(end); elseif ~isempty(v0), v(1) = v0; end
end

vm = 0.5*(v(1:end-1) + v(2:end));
t  = sum(ds ./ max(vm, 0.1));

if nargout > 2
    E = lap_energy(p, v, s);
end
end

function n = ellipse_exp(p, mode)
% Combined-grip friction-ellipse exponent: (ax/axmax)^n + (ay/aymax)^n = 1.
switch lower(mode)
    case 'brake', f = 'n_env_brake';
    otherwise,    f = 'n_env_drive';
end
if isfield(p, f) && isfinite(p.(f)) && p.(f) > 0
    n = p.(f);
else
    n = 2;   % no measured exponent -> circle
end
end

function ayfun = ay_lut(p, npts)
% Precomputed ay_limit(p,v) lookup handle; build once per lap, interpolate.
% ay(v) is smooth/monotone (downforce only), so 120 pts is exact to ~1e-4 g.
if nargin < 2 || isempty(npts), npts = 120; end
vmax = p.v_max;
vg = linspace(0, vmax, npts);
ag = arrayfun(@(v) ay_limit(p, v), vg);     % the only axle_grip calls per lap
% pchip = shape-preserving (monotone data stays monotone, no overshoot).
F = griddedInterpolant(vg, ag, 'pchip');
ayfun = @(v) F(min(max(v, 0), vmax));
end

function E = lap_energy(p, v, s)
% Longitudinal energy along the speed trace. Wheel thrust from Newton's
% law per segment; positive = drive energy, negative = braking energy
% (upper bound on regen; ignores hydraulic/regen split).
ds = diff(s(:));
vm = 0.5*(v(1:end-1) + v(2:end));
a  = (v(2:end).^2 - v(1:end-1).^2) ./ (2*ds);

N      = p.m*p.g + 0.5*p.rho*p.ClA .* vm.^2;
F_drag = 0.5*p.rho*p.CdA .* vm.^2;
F_rr   = p.Crr .* N;
F_dl   = (p.b_driveline .* vm ./ p.Re + p.Tc_driveline) ./ p.Re;

F_thrust = p.k_rot*p.m .* a + F_drag + F_rr + F_dl;   % [N], +drive / -brake

J_PER_WH = 3600;
E.drive_wheel_Wh = sum(max(F_thrust, 0) .* ds) / J_PER_WH;
E.drive_acc_Wh   = E.drive_wheel_Wh / p.eta_dt;        % drawn from accumulator
E.brake_wheel_Wh = sum(max(-F_thrust, 0) .* ds) / J_PER_WH;  % regen upper bound
end
