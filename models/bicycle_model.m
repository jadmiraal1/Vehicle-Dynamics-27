function B = bicycle_model(p, Ca_axle_f, Ca_axle_r, v_sweep)
% BICYCLE_MODEL  Linear single-track handling model.
%   B = bicycle_model(p, Ca_f, Ca_r, v_sweep)  -> K, v_crit/char, yaw gain, yaw mode.
% Theory: ref doc sec 10.

% --- Input validation ---
req_fields = {'m','Izz','a','b','L','g','Wf_static','Wr_static'};
missing = req_fields(~isfield(p, req_fields));
assert(isempty(missing), 'bicycle_model:missing_param', ...
    'p is missing required field(s): %s', strjoin(missing, ', '));
assert(Ca_axle_f > 0 && Ca_axle_r > 0, 'bicycle_model:bad_stiffness', ...
    'Ca_axle_f and Ca_axle_r must be positive (got %.4g, %.4g).', Ca_axle_f, Ca_axle_r);
assert(all(v_sweep > 0), 'bicycle_model:bad_speed', ...
    'v_sweep must be strictly positive (yaw-plane A(v) is singular at v=0).');

% Steady state: delta = L/R + K*ay
B.K_rad = p.Wf_static / Ca_axle_f - p.Wr_static / Ca_axle_r;
B.K_deg = B.K_rad * 180/pi;
B.v_crit = Inf;   % default: neutral or understeer -> no finite instability speed
B.v_char = Inf;   % default: neutral or oversteer -> no finite half-sensitivity speed
if B.K_rad < 0
    B.v_crit = sqrt(p.g * p.L / (-B.K_rad));
elseif B.K_rad > 0
    B.v_char = sqrt(p.g * p.L / B.K_rad);
end
% Note: if K_rad == 0 (neutral steer), both fields stay Inf -- the car is
% linearly stable at all speeds and steering sensitivity never decays.

% Response and transient sweeps
B.v        = v_sweep;
B.yaw_gain = v_sweep ./ (p.L + B.K_rad .* v_sweep.^2 ./ p.g);
B.A        = @(v) yaw_plane_A(p, Ca_axle_f, Ca_axle_r, v);

B.tau_slow = nan(size(v_sweep));
B.zeta_eq  = nan(size(v_sweep));
for i = 1:numel(v_sweep)
    ev = eig(B.A(v_sweep(i)));
    B.tau_slow(i) = 1 / min(abs(real(ev)));
    wn            = sqrt(prod(abs(ev)));
    B.zeta_eq(i)  = -sum(real(ev)) / (2*wn);
end
end

function A = yaw_plane_A(p, Caf, Car, v)
% States [beta; r]: m*v*(beta_dot + r) = Fyf + Fyr, Izz*r_dot = a*Fyf - b*Fyr
A = [-(Caf + Car)/(p.m*v),        -1 + (p.b*Car - p.a*Caf)/(p.m*v^2);
     (p.b*Car - p.a*Caf)/p.Izz,   -(p.a^2*Caf + p.b^2*Car)/(p.Izz*v)];
end