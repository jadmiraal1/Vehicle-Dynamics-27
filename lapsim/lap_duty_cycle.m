function D = lap_duty_cycle(p, v, s)
% LAP_DUTY_CYCLE  What the powertrain does over one lap, segment by segment.
%   D = lap_duty_cycle(p, v, s)      v, s as returned by lap_sim (node values)
%
% The single home for thrust, motor state and loss accounting along a lap:
% lap_sim's energy totals sum these fields and run_cooling_targets
% integrates them over time. Everything is per SEGMENT (n-1 values between
% the n nodes) and SI.
%
%   ds, vm, a, dt, t   segment length [m], mean speed [m/s], accel [m/s^2],
%                      duration [s], time at segment midpoint [s]
%   F_thrust           wheel force [N] = k_rot*m*a + drag + rolling + driveline;
%                      > 0 the motor drives, < 0 the brakes hold back
%   P_wheel_W          wheel drive power (thrust > 0 only)
%   P_brake_W          power into the friction brakes (no regen modelled)
%   rpm, T_mot_Nm      motor speed and torque (through the chain)
%   P_motor_mech_W     motor shaft power
%   P_motor_loss_W     motor heat from motor_eff, including the spin loss at
%                      zero torque
%   P_inv_loss_W       inverter heat, flat (1/eta_inv - 1) x inverter output
%   P_chain_loss_W     chain + sprocket heat (not on the coolant loop)
%   P_elec_W           DC power from the accumulator
%   I_pack_A           pack current at V_pack_nom (voltage sag ignored)
%   P_pack_loss_W      I^2*R_pack heat in the accumulator
%   Q_coolant_W        motor + inverter heat: what the radiator rejects
%   eta_e              chain x inverter x clamped motor efficiency
%   t_lap              lap time [s], identical to lap_sim's
%   E_elec_Wh          DC energy over the lap from P_elec
%   E_elec_lapenergy_Wh  the same energy computed as lap_sim does (work/eta_e)
%
% Off throttle the motor still spins, so its iron and friction loss is fed
% by the wheels (no current flows) yet still heats the coolant. The pack pays
% for motor losses only while driving. That is why E_elec_Wh and the
% coolant heat are computed from losses, and why they differ slightly from
% the work/efficiency estimate.
%
% Note: the thrust limit in the lap simulation uses the lumped eta_dt to
% carry P_max to the wheel, while this accounting uses chain x inverter x
% motor map. The two efficiencies differ by a few percent, so P_elec_W can
% read slightly above P_max at full power.
% Motor data: EMRAX 228 technical data, p.2-3.

v  = v(:);  s = s(:);
ds = diff(s);
vm = 0.5*(v(1:end-1) + v(2:end));
a  = (v(2:end).^2 - v(1:end-1).^2) ./ (2*ds);

dt = ds ./ max(vm, 0.1);          % same 0.1 m/s floor as lap_sim
t  = cumsum(dt) - 0.5*dt;

% --- wheel thrust: Newton's law with rotating inertia, plus resistances ---
RL = road_loads(p, vm);
F_thrust = RL.m_eff .* a + RL.F_drag + RL.F_rr + RL.F_dl;   % [N]

P_wheel = max(F_thrust, 0) .* vm;
P_brake = max(-F_thrust, 0) .* vm;

D = struct('ds', ds, 'vm', vm, 'a', a, 'dt', dt, 't', t, ...
           'F_thrust', F_thrust, 'P_wheel_W', P_wheel, 'P_brake_W', P_brake, ...
           't_lap', sum(dt));

% --- motor, inverter, pack ------------------------------------------------
% Every config supplies eta_chain and eta_inv; a hand-built p without them
% gets NaN here and lap_sim falls back to the flat eta_dt.
if ~(isfield(p, 'eta_chain') && isfield(p, 'eta_inv'))
    nanv = nan(size(ds));
    D.rpm = nanv; D.T_mot_Nm = nanv; D.P_motor_mech_W = nanv;
    D.P_motor_loss_W = nanv; D.P_inv_loss_W = nanv; D.P_chain_loss_W = nanv;
    D.P_elec_W = nanv; D.I_pack_A = nanv; D.P_pack_loss_W = nanv;
    D.Q_coolant_W = nanv; D.eta_e = nanv;
    D.E_elec_Wh = NaN; D.E_elec_lapenergy_Wh = NaN;
    return
end

rpm   = vm ./ p.Re .* p.gear_ratio .* (60/(2*pi));
T_mot = max(F_thrust, 0) .* p.Re ./ (p.gear_ratio * p.eta_chain);
[eta_m, P_motor_loss, ~] = motor_eff(rpm, T_mot);
eta_e = p.eta_chain .* p.eta_inv .* eta_m;             % what lap_energy uses

P_motor_mech = T_mot .* rpm * (2*pi/60);              % shaft power [W]
P_chain_loss = P_motor_mech - P_wheel;                % the chain's cut [W]

% Electrical input to the motor: shaft power plus all its losses while
% driving; zero off throttle (the spin loss is then fed by the wheels).
driving = F_thrust > 0;
P_motor_elec = (P_motor_mech + P_motor_loss) .* driving;

% Inverter: flat efficiency; idle switching loss (tens of W) not modelled.
P_inv_loss = P_motor_elec .* (1/p.eta_inv - 1);
P_elec     = P_motor_elec + P_inv_loss;

I_pack      = P_elec ./ p.V_pack_nom;
P_pack_loss = I_pack.^2 .* p.R_pack;

J_PER_WH = 3600;
D.rpm = rpm;  D.T_mot_Nm = T_mot;
D.P_motor_mech_W = P_motor_mech;
D.P_motor_loss_W = P_motor_loss;
D.P_inv_loss_W   = P_inv_loss;
D.P_chain_loss_W = P_chain_loss;
D.P_elec_W       = P_elec;
D.I_pack_A       = I_pack;
D.P_pack_loss_W  = P_pack_loss;
D.Q_coolant_W    = P_motor_loss + P_inv_loss;
D.eta_e          = eta_e;
D.E_elec_Wh      = sum(P_elec .* dt) / J_PER_WH;
% The work / clamped-eta estimate that lap_sim reports, for comparison.
D.E_elec_lapenergy_Wh = sum(max(F_thrust, 0) .* ds ./ max(eta_e, 0.5)) / J_PER_WH;
end
