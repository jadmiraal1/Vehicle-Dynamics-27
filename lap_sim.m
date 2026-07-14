function [v, t, E] = lap_sim(p, s, kappa, v0, closed)
% LAP_SIM  Quasi-steady-state point-mass lap solver.
%   [v, t, E] = lap_sim(p, s, kappa, v0, closed)
%   s [m], kappa [1/m] (same length) -> speed trace v [m/s], lap time t [s],
%   optional E: energy accounting struct (Wh; drive at wheel/accumulator,
%   braking energy as regen upper bound).
%   v0 : start speed (m/s), [] to free it.  closed : wrap the lap (default true).
% Method (corner-speed ceiling, forward/backward passes, friction ellipse):
% VD_physics_reference.md, section 7.

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

vlim = arrayfun(@(k) corner_speed(p, k), kappa);   % per-point speed ceiling
v = vlim;
if ~isempty(v0), v(1) = v0; end

niter = 3;  if ~closed, niter = 1; end
for it = 1:niter
    for i = 1:n-1                                   % forward / accelerate
        G   = gg_envelope(p, v(i));
        used = min(v(i)^2*kappa(i) / max(G.ay*p.g, 1e-6), 1.0);
        frac = sqrt(max(0, 1 - used^2));            % friction-ellipse margin
        ax  = G.ax_accel * p.g * frac;
        v(i+1) = min(vlim(i+1), sqrt(max(v(i)^2 + 2*ax*ds(i), 0))); % min between vlim and accel velocity based on curvature only
    end
    for i = n:-1:2                                  % backward / brake
        G   = gg_envelope(p, v(i));
        used = min(v(i)^2*kappa(i) / max(G.ay*p.g, 1e-6), 1.0);
        frac = sqrt(max(0, 1 - used^2));
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
