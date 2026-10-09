function GG = gg_envelope(p, v)
% GG_ENVELOPE  Point-mass g-g-V capability at speed v [m/s]. Outputs in [g].
%   GG = gg_envelope(p, v)      vectorized over v
%
% Tier T0: one constant friction coefficient (p.mu_x, p.mu_y at the design
% load), no load sensitivity. The traction edge still accounts for the drive
% axle's share of the load and for longitudinal load transfer, because
% without that a RWD launch is grossly optimistic.
%
%   GG.ay           lateral limit, constant-speed cornering
%   GG.ax_accel     min(traction, motor) acceleration limit
%   GG.ax_brake     braking limit (all four wheels, ideal bias)
%   GG.ax_motor     motor/power-limited acceleration alone (ax_limit reuses it)
%   GG.ax_traction  traction-limited acceleration alone
%   GG.power_limited  true above the motor's base speed

R = road_loads(p, v);
N = R.N;  m_eff = R.m_eff;  F_loss = R.F_loss;
mu_x_usable = p.k_trac .* p.mu_x;              % launch traction utilisation

% Traction limit. Drive-axle load = static share + m*a*h/L transfer, so a
% appears on both sides; solving for it gives the denominators below.
switch upper(p.drive)
  case 'AWD'
    a_traction = (mu_x_usable.*N - F_loss) ./ m_eff;
  case 'RWD'
    a_traction = (mu_x_usable.*R.Nr - F_loss) ./ (m_eff - mu_x_usable.*p.m.*p.h_cg./p.L);
  case 'FWD'
    a_traction = (mu_x_usable.*R.Nf - F_loss) ./ (m_eff + mu_x_usable.*p.m.*p.h_cg./p.L);
  otherwise
    error('gg_envelope:drive', 'p.drive must be AWD, RWD or FWD');
end

% Motor limit: torque cap below base speed, constant power above.
% P_max is the pack-side limit; eta_dt carries it to the wheel.
F_torque = p.T_motor_max .* p.gear_ratio .* p.eta_dt ./ p.Re;
F_power  = p.eta_dt .* p.P_max ./ max(v, 1e-3);
F_motor  = min(F_torque, F_power);
a_motor  = (F_motor - F_loss) ./ m_eff;

a_lat   = p.mu_y .* N ./ p.m;                  % no rotating inertia: constant speed
a_accel = min(a_traction, a_motor);
a_brake = (p.mu_x .* N + F_loss) ./ m_eff;     % drag and losses help braking

GG.v             = max(v, 0);
GG.N             = N;
GG.ay            = a_lat   ./ p.g;
GG.ax_accel      = a_accel ./ p.g;
GG.ax_brake      = a_brake ./ p.g;
GG.ax_motor      = a_motor ./ p.g;
GG.ax_traction   = a_traction ./ p.g;
GG.power_limited = F_power < F_torque;
end
