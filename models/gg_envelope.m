function GG = gg_envelope(p, v)
% GG_ENVELOPE  Point-mass g-g-V capability at speed v [m/s]; outputs in [g].
% Field names used by vd_selftest and downstream - keep. Theory: ref doc sec 5.

v      = max(v, 0);
N      = p.m*p.g + 0.5*p.rho*p.ClA .* v.^2;    % vertical load: weight + downforce [N]
F_drag = 0.5*p.rho*p.CdA .* v.^2;              % aero drag [N]
F_rr   = p.Crr .* N;                           % rolling resistance [N]
F_dl   = (p.b_driveline .* v ./ p.Re + p.Tc_driveline) ./ p.Re;  % driveline drag [N]

m_eff       = p.k_rot .* p.m;                  % effective mass incl. rotating inertia
mu_x_usable = p.k_trac .* p.mu_x;              % launch-utilization derated grip

% Traction-limited accel by drive layout (transfer coupling in denominator)
switch upper(p.drive)
  case 'AWD'
    a_traction = (mu_x_usable.*N - F_drag - F_rr - F_dl) ./ m_eff;
  case 'RWD'
    N_drv_static = (1 - p.mass_dist_f) .* N;
    a_traction   = (mu_x_usable.*N_drv_static - F_drag - F_rr - F_dl) ...
                   ./ (m_eff - mu_x_usable.*p.m.*p.h_cg./p.L);
  case 'FWD'
    N_drv_static = p.mass_dist_f .* N;
    a_traction   = (mu_x_usable.*N_drv_static - F_drag - F_rr - F_dl) ...
                   ./ (m_eff + mu_x_usable.*p.m.*p.h_cg./p.L);
  otherwise
    error('gg_envelope:drive', 'p.drive must be AWD, RWD or FWD');
end

% Motor-limited accel: torque cap below base speed, P/v above
F_torque = p.T_motor_max .* p.gear_ratio .* p.eta_dt ./ p.Re;
F_power  = p.eta_dt .* p.P_max ./ max(v, 1e-3);
F_motor  = min(F_torque, F_power);
a_motor  = (F_motor - F_drag - F_rr - F_dl) ./ m_eff;

% All three edges in SI [m/s^2]
a_lat   = p.mu_y .* N ./ p.m;                    % no m_eff: constant-speed cornering
a_accel = min(a_traction, a_motor);
a_brake = (p.mu_x .* N + F_drag + F_rr + F_dl) ./ m_eff;   % losses assist braking

% Convert to [g] at the interface
GG.v             = v;
GG.N             = N;
GG.ay            = a_lat   ./ p.g;
GG.ax_accel      = a_accel ./ p.g;
GG.ax_brake      = a_brake ./ p.g;
GG.power_limited = F_power < F_torque;
end
