function c = config_TR27()
% CONFIG_TR27  The developing car - EVERY physical input that describes it.
%
% This file is the complete specification of one car. Mass, geometry, tire,
% aero, accumulator, motor, inverter, driveline and the strategy assumptions
% that depend on this car's hardware all live here. To change the motor, the
% pack, or the tire in a future year, write a new config - never edit
% vehicle_params.m.
%
% vehicle_params.m keeps only what is NOT a property of a car: universal
% constants (g, rho), the competition scenario (endurance distance, benchmark
% lap basis), the model-fidelity switches, and every derived quantity.
%
% Values are design targets or TR26 carryover until measured; the tracker
% (TR27 Vehicle Dynamics Target Tracker.xlsx) holds each one's status.

% --- Mass ---
c.m_car        = 233.8;   % dry, no driver [kg] = 515 lb - TR27 mass target (#1)
c.m_driver     = 80.00;   % design driver [kg] = 180 lb body + 10 lb gear (heaviest at comp)
c.m_driver_min = 61.23;   % lightest driver [kg] = 125 lb + gear (binding case for rollover)
c.m_driver_max = 95.25;   % heaviest plausible driver [kg] = 200 lb + gear (peak tire load)
c.mass_dist_f  = 0.45;    % static front fraction [-] - sim baseline (band 44-47%, #2)
c.h_cg         = 0.2794;  % CG height [m] - carryover until measured (ceiling 0.288, #3)

% --- Sprung / unsprung split ---
c.f_unsprung   = 0.180;   % unsprung mass / DRY car mass [-] PROVISIONAL (#7)
                          %   TR26 as-built 90/500 lbm. Dry-car basis, not total:
                          %   the driver is 100% sprung. Held as a ratio so mass
                          %   sweeps re-derive it instead of freezing kg.
c.unsprung_f   = 0.4444;  % front share of unsprung mass [-] PROV (TR26 40/90 lbm)

% --- Geometry ---
c.L            = 1.5621;  % wheelbase [m] - carryover, suspension retained (#4)
c.t_f          = 1.20;    % front track [m] (floor 1.153, #5)
c.t_r          = 1.18;    % rear track [m]

% --- Tire identity + belt->track scalings ---
c.tire_id          = 'Hoosier 43075 LC0 16x7.5-10, 8in rim';
c.tire_data_prefix = 'LC0_16x75';   % TTC_Data filename prefix of the design tire
c.mu_derate    = 0.67;    % lambda_muY, belt->track grip scaling [-] PROVISIONAL, fit at skidpad
c.lambda_Ca    = 0.90;    % lambda_KyA, belt->track slip-stiffness scaling [-] PROVISIONAL
                          %   Literature-consistent: cornering stiffness measures ~4-8%
c.Re           = 0.20;    % loaded tire radius [m]

% --- Aero ---
c.ClA          = 4.0;  % current measured aero until the TR27 package lands
c.CdA          = c.ClA/2.5;  %   (the ClA 4.0 TARGET lives in c.scenario, #36/#37)

% --- Chassis balance ---
c.LLTD         = 0.60;    % lateral load transfer distribution, front frac [-] PROVISIONAL
c.aero_df_front = 0.40;   % front share of downforce (CoP) [-] PROVISIONAL

% --- Accumulator -------------------------------------------------------
% Sony/Murata VTC6 18650. Change PACK_S to change the pack: voltage, energy
% and current all follow. Current config reproduces the previously hard-coded
% 298.8 V / 348.6 V / 6274.8 Wh / 210 A exactly.
CELL_V_NOM = 3.6;   CELL_V_MAX = 4.2;   % [V] per cell
CELL_AH    = 3.0;   CELL_I_MAX = 30;    % [Ah], [A] per cell
c.cell_id  = 'Sony/Murata US18650VTC6';
c.pack_S   = 83;          % cells in SERIES  <- the pack-sizing decision knob
c.pack_P   = 7;           % cells in PARALLEL (sets pack current, not voltage)

c.V_pack_nom  = c.pack_S * CELL_V_NOM;                  % nominal voltage [V]
c.V_pack_max  = c.pack_S * CELL_V_MAX;                  % max voltage [V]
c.I_pack_max  = c.pack_P * CELL_I_MAX;                  % max discharge current [A]
c.E_pack_Wh   = c.pack_S * c.pack_P * CELL_V_NOM * CELL_AH;  % nominal energy [Wh]
c.I_fuse_main = 100;      % main TSB fuse [A], de-facto CONTINUOUS current rating
c.R_pack      = 0.154;    % pack resistance [ohm] PROVISIONAL

% --- Motor / inverter ---
c.motor_id      = 'Emrax 228 (axial flux, MV)';
c.inverter_id   = 'Cascadia PM100DX (GEN2, s/n 275)';
c.T_motor_max   = 220;    % operating torque cap [Nm] (inverter-current limited)
c.T_motor_cont  = 125;    % continuous motor torque [Nm] (reference, unused)
c.rpm_motor_max = 4500;   % controller over-speed limit [rpm] PROVISIONAL
                          %   Voltage-governed, NOT a free choice. Emrax 228 MV
                          %   datasheet: 14 rpm/Vdc no-load, 11-14 under load
                          %   (470 Vdc -> 6500/5170 rpm). 4500 at this pack
                          %   implies 15.06 rpm/V - above the no-load figure.
                          %   Re-issue after the PM100DX EEPROM read (#47).
c.drive         = 'RWD';  % single motor, rear-wheel drive

% --- Driveline ---
c.gear_ratio    = 3.82;   % final-drive ratio [-]; freeze band 3.45-3.7 preferred (#41)
c.eta_dt        = 0.88;   % driveline eff. battery->ground [-] PROVISIONAL - force-path lump
c.eta_chain     = 0.97;   % chain + sprocket mechanical [-] PROVISIONAL
c.eta_inv       = 0.95;   % inverter electrical [-] PROVISIONAL
                          %   energy accounting uses eta_chain*eta_inv*motor_eff(map);
                          %   eta_dt remains the thrust-delivery lump in gg_envelope.
c.Crr           = 0.014;  % rolling resistance [-] (TTC LC0 free-rolling FX)
c.k_trac        = 0.90;   % launch traction utilization [-] PROVISIONAL
c.b_driveline   = 0.066;  % driveline viscous coefficient [N*m*s/rad] PROVISIONAL
c.Tc_driveline  = 0.53;   % driveline coulomb torque [N*m] PROVISIONAL

% --- Inertias ---
c.DI       = 0.75;        % dynamic index k^2/(a*b) [-] PROVISIONAL
c.I_rotor  = 0.0421;      % motor rotor inertia [kg*m^2]
c.I_wheel  = 0.217;       % per-corner wheel+tire inertia [kg*m^2]

% --- Strategy assumptions that depend on THIS car's hardware ---
% Not measured car properties, but not shared either: a different pack, BMS or
% regen implementation changes them. Competition constants (endurance distance,
% benchmark lap basis) stay in vehicle_params.
c.scenario.regen_capture = 0.00;   % NO REGEN FOR TR27 - team decision, Aug 2026.
                                   %   Set to 0 deliberately: #45 (regen torque target) and
                                   %   #51 (regen/friction blend) will not be built this year.
                                   %   Every energy number therefore assumes friction braking
                                   %   only. Raise this ONLY alongside a real implementation.
c.scenario.regen_rt      = 0.65;   % round-trip efficiency [-] - unused while capture is 0
c.scenario.pack_usable_f = 0.90;   % usable fraction of nominal pack energy (BMS SoC window) [-] PROVISIONAL
c.scenario.margin        = 0.90;   % design margin on usable energy (heat/driver/cones) [-] CHOICE
c.scenario.ClA_target    = 4.0;    % TR27 downforce target [m^2] (#36) - not yet the built car
c.scenario.CdA_target    = 1.6275; % drag budget at that target [m^2] (#37)
end
