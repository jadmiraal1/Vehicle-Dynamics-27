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

% Build the edge lookups once (local vlut below). Lateral feeds the ceiling and
% the lateral-usage fraction; longitudinal comes either from the per-axle
% combined model (2D lookup in speed x lateral usage - the ellipse lives INSIDE
% ax_combined, per axle) or from the scalar edges + car-level ellipse.
ayf  = vlut(p, @(vv) ay_limit(p, vv));
comb = isfield(p, 'long_model') && strcmpi(p.long_model, 'combined');
if comb
    vg = linspace(0, p.v_max, 24);
    rg = linspace(0, 1, 13);                        % lateral usage fraction ay/ay_max
    [VV, RR] = ndgrid(vg, rg);
    Aacc = arrayfun(@(vv, rr) ax_combined(p, vv, rr*ayf(vv), 'accel'), VV, RR);
    Abrk = arrayfun(@(vv, rr) ax_combined(p, vv, rr*ayf(vv), 'brake'), VV, RR);
    Fa = griddedInterpolant({vg, rg}, Aacc, 'linear');
    Fb = griddedInterpolant({vg, rg}, Abrk, 'linear');
else
    axaf = vlut(p, @(vv) ax_limit(p, vv, 'accel'));
    axbf = vlut(p, @(vv) ax_limit(p, vv, 'brake'));
end
vlim = arrayfun(@(k) corner_speed(p, k, ayf), kappa);   % per-point speed ceiling
v = vlim;
if ~isempty(v0), v(1) = v0; end

n_d = ellipse_exp(p, 'drive');   % friction-ellipse exponent, accel (measured)
n_b = ellipse_exp(p, 'brake');   % ... braking (scalar path only; combined has its own)

niter = 3;  if ~closed, niter = 1; end
for it = 1:niter
    for i = 1:n-1                                   % forward / accelerate
        aymax = ayf(v(i));                          % lateral edge (p.grip_model)
        used = min(v(i)^2*kappa(i) / max(aymax*p.g, 1e-6), 1.0);
        if comb
            ax = Fa(min(max(v(i),0),p.v_max), used) * p.g;   % per-axle combined
        else
            frac = (max(0, 1 - used^n_d))^(1/n_d);  % car-level ellipse
            ax  = axaf(v(i)) * p.g * frac;
        end
        v(i+1) = min(vlim(i+1), sqrt(max(v(i)^2 + 2*ax*ds(i), 0))); % min between vlim and accel velocity based on curvature only
    end
    for i = n:-1:2                                  % backward / brake
        aymax = ayf(v(i));                          % lateral edge (p.grip_model)
        used = min(v(i)^2*kappa(i) / max(aymax*p.g, 1e-6), 1.0);
        if comb
            ax = Fb(min(max(v(i),0),p.v_max), used) * p.g;   % per-axle combined
        else
            frac = (max(0, 1 - used^n_b))^(1/n_b);  % car-level ellipse
            ax  = axbf(v(i)) * p.g * frac;
        end
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

function h = vlut(p, fh, npts)
% Tabulate a smooth function of speed once, return a fast interpolant handle.
% The edge models are bisections/fixed-points; calling them per point per pass
% would be ~100x slower for ~1e-4 g of difference.
if nargin < 3 || isempty(npts), npts = 120; end
vmax = p.v_max;
vg = linspace(0, vmax, npts);
vals = arrayfun(fh, vg);
% pchip = shape-preserving (monotone data stays monotone, no overshoot).
F = griddedInterpolant(vg, vals, 'pchip');
h = @(v) F(min(max(v, 0), vmax));
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
if isfield(p, 'eta_chain') && isfield(p, 'eta_inv')
    % Electrical draw through the digitized EMRAX map: per-segment motor state,
    % eta = chain x inverter x motor_eff(rpm, T). Replaces the flat eta_dt lump
    % for ENERGY only; the thrust force path is unchanged.
    rpm   = vm ./ p.Re .* p.gear_ratio .* (60/(2*pi));
    T_mot = max(F_thrust, 0) .* p.Re ./ (p.gear_ratio * p.eta_chain);
    eta_e = p.eta_chain .* p.eta_inv .* motor_eff(rpm, T_mot);
    E.drive_acc_Wh = sum(max(F_thrust, 0) .* ds ./ max(eta_e, 0.5)) / J_PER_WH;
else
    E.drive_acc_Wh = E.drive_wheel_Wh / p.eta_dt;      % legacy flat lump
end
E.brake_wheel_Wh = sum(max(-F_thrust, 0) .* ds) / J_PER_WH;  % regen upper bound
end
