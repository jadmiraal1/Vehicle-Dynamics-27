function c = config_TR27()
% CONFIG_TR27  The car in development: every physical input that describes it.
%
% This file is the complete specification of one car - mass, geometry,
% tire, aero, accumulator, motor, driveline, cooling, and the strategy
% assumptions that depend on its hardware. A new car or a new year gets a
% new config; vehicle_params.m holds only what is not a car property.
%
% Values are design targets or TR26 carry-over until measured. Tags:
%   PROVISIONAL  an estimate that a measurement should replace
%   CHOICE       a decision, not a measurement

% --- Mass ---
c.m_car        = 233.8;   % dry, no driver [kg] = 515 lb - TR27 mass target
c.m_driver     = 80.00;   % design driver [kg] = 180 lb body + 10 lb gear (heaviest at comp)
c.m_driver_min = 61.23;   % lightest driver [kg] = 125 lb + gear (binding case for rollover)
c.m_driver_max = 95.25;   % heaviest plausible driver [kg] = 200 lb + gear (peak tire load)
c.mass_dist_f  = 0.45;    % static front fraction [-] - sim baseline (target 44-47%)
c.h_cg         = 0.2794;  % CG height [m] - carry-over until measured (ceiling 0.288)

% --- Sprung / unsprung split ---
c.f_unsprung   = 0.180;   % unsprung mass / DRY car mass [-] PROVISIONAL
                          %   TR26 as built: 90/500 lbm. The driver is all sprung.
c.unsprung_f   = 0.4444;  % front share of unsprung mass [-] PROVISIONAL (TR26 40/90 lbm)

% --- Geometry ---
c.L            = 1.5621;  % wheelbase [m] - carry-over, suspension retained
c.t_f          = 1.20;    % front track [m] (minimum 1.153)
c.t_r          = 1.18;    % rear track [m]

% --- Tire identity and belt-to-track scaling ---
c.tire_id          = 'Hoosier 43075 LC0 16x7.5-10, 8in rim';
c.tire_data_prefix = 'LC0_16x75';   % TTC_Data file prefix of the design tire
c.mu_derate    = 0.67;    % lambda_muY, belt -> track grip scaling [-] PROVISIONAL, calibrate at skidpad
c.lambda_Ca    = 0.90;    % lambda_KyA, belt -> track cornering-stiffness scaling [-] PROVISIONAL
c.Re           = 0.20;    % loaded tire radius [m]

% --- Aero ---
c.ClA          = 4.0;          % [m^2] set to the TR27 target; TR26 measured 1.301
c.CdA          = c.ClA/2.5;    % [m^2] assumes package L/D = 2.5 (drag budget in c.scenario)

% --- Chassis balance ---
c.LLTD         = 0.60;    % lateral load transfer distribution, front fraction [-] PROVISIONAL
c.aero_df_front = 0.40;   % front share of downforce (centre of pressure) [-] PROVISIONAL

% --- Accumulator ---
% Change pack_S to change the pack: voltage, energy and current follow.
c.cell_id  = 'Sony/Murata US18650VTC6';
c.cell     = struct('V_nom', 3.6, 'V_max', 4.2, 'Ah', 3.0, ...   % per cell [V], [V], [Ah]
                    'I_max', 30, 'mass_kg', 0.0466);              % [A], [kg]
c.pack_S   = 83;          % cells in series (sets voltage) - the pack-sizing knob
c.pack_P   = 7;           % cells in parallel (sets current)

c.V_pack_nom  = c.pack_S * c.cell.V_nom;                 % nominal voltage [V]
c.V_pack_max  = c.pack_S * c.cell.V_max;                 % max voltage [V]
c.I_pack_max  = c.pack_P * c.cell.I_max;                 % max discharge current [A]
c.E_pack_Wh   = c.pack_S * c.pack_P * c.cell.V_nom * c.cell.Ah;   % nominal energy [Wh]
c.I_fuse_main = 100;      % main TSB fuse [A], effectively the continuous current rating
c.R_pack      = 0.154;    % pack resistance [ohm] PROVISIONAL

% --- Motor / inverter ---
c.motor_id      = 'Emrax 228 (axial flux, MV)';
c.inverter_id   = 'Cascadia PM100DX (GEN2, s/n 275)';
c.T_motor_max   = 220;    % operating torque cap [Nm] (inverter-current limited)
c.T_motor_cont  = 125;    % continuous motor torque [Nm] (reference only)
c.rpm_motor_max = 4500;   % controller over-speed limit [rpm] PROVISIONAL
                          %   Set by pack voltage: EMRAX 228 MV is 14 rpm/V no-load,
                          %   11-14 under load. 4500 rpm at 298.8 V is 15.1 rpm/V,
                          %   above the no-load figure. Confirm from the PM100DX EEPROM.
c.drive         = 'RWD';  % single motor, rear-wheel drive

% --- Driveline ---
c.gear_ratio    = 3.82;   % final-drive ratio [-]; chosen range 3.45-3.7 preferred (run_gear_targets)
c.eta_dt        = 0.88;   % battery -> wheel efficiency for the THRUST limit [-] PROVISIONAL
c.eta_chain     = 0.97;   % chain + sprockets [-] PROVISIONAL  } energy accounting uses
c.eta_inv       = 0.95;   % inverter [-] PROVISIONAL           } chain x inverter x motor map
c.Crr           = 0.014;  % rolling resistance [-] (TTC LC0 free-rolling Fx)
c.k_trac        = 0.90;   % launch traction utilisation [-] PROVISIONAL
c.b_driveline   = 0.066;  % driveline viscous coefficient [N*m*s/rad] PROVISIONAL
c.Tc_driveline  = 0.53;   % driveline Coulomb torque [N*m] PROVISIONAL

% --- Inertias ---
c.DI       = 0.75;        % dynamic index k^2/(a*b) [-] PROVISIONAL
c.I_rotor  = 0.0421;      % motor rotor inertia [kg*m^2]
c.I_wheel  = 0.217;       % per-corner wheel + tire inertia [kg*m^2]

% --- Strategy assumptions that depend on this car's hardware ---
c.scenario.regen_capture = 0.00;   % NO REGEN for TR27 (team decision, Aug 2026; regen
                                   %   not built). Every energy number assumes friction
                                   %   braking only. Raise only with a real implementation.
c.scenario.regen_rt      = 0.65;   % regen round-trip efficiency [-] (unused while capture is 0)
c.scenario.pack_usable_f = 0.90;   % usable fraction of nominal energy (BMS SoC window) [-] PROVISIONAL
c.scenario.margin        = 0.90;   % design margin on usable energy (heat, driver, cones) [-] CHOICE
c.scenario.ClA_target    = 4.0;    % TR27 downforce target [m^2]
c.scenario.CdA_target    = 1.6275; % drag budget at that target [m^2]

% --- Cooling loop ---
% TR26 hardware carried over (Sep 2026): one series water loop - radiator,
% pump, inverter, motor (TR26 powertrain deck slide 92: Honda CRF450R
% radiator, SPAL 4in fan, two Davies Craig EBP40 pumps). Every value is
% PROVISIONAL or a CHOICE; run_cooling_targets says what each one needs.
c.cooling.loop_order      = 'inverter_first';  % 'inverter_first' | 'motor_first' PROVISIONAL - check the hoses
c.cooling.coolant         = 'water';    % distilled water; a glycol mix changes cp and rho (not modelled)
c.cooling.flow_Lpm        = 10;         % coolant flow [L/min] PROVISIONAL - MEASURE
                                        %   EBP40 free flow 35 L/min (12 V); PM100DX wants 8-10 L/min
                                        %   (0.4 bar drop at 10); EMRAX ratings quoted at 8 L/min.
c.cooling.T_amb_design_C  = 35;         % hot-day ambient [degC] CHOICE
c.cooling.T_cold_target_C = 50;         % coolant leaving the radiator [degC] CHOICE
c.cooling.T_inv_full_C    = 60;         % PM100DX: coolant < 60 degC for full current  [datasheet]
c.cooling.T_inv_derate_C  = 80;         % PM100DX: derates to zero between 80 and 100 degC [datasheet]
c.cooling.T_motor_rated_C = 50;         % EMRAX 228: continuous ratings at 50 degC inlet, 8 L/min [datasheet]
c.cooling.FOS             = 2.0;        % factor on mean heat for radiator sizing [-] CHOICE
                                        %   (covers eta_inv, the fan curve and ram air)
% Fan: the TR26 deck says a 4in PULL fan; the TR26 sizing script named the
% PUSHER VA32-A101-62S, whose vendor curve is below (124 CFM free air). Read
% the part number on the car.
c.cooling.fan_id          = 'SPAL VA32-A101-62S (4in, pusher curve) PROVISIONAL';
c.cooling.fan_dp_mmH2O    = [0 2.5 5 7.5 10 12.5 15 17.5 20];   % static pressure [mm H2O]
c.cooling.fan_m3h         = [210 190 160 120 90 70 60 30 0];    % airflow at that pressure [m^3/h]
c.cooling.core_dp_mmH2O   = 5;          % core pressure drop at the fan's flow [mm H2O] PROVISIONAL
c.cooling.ram_capture     = 0.30;       % fraction of car speed reaching the core face [-] PROVISIONAL
c.cooling.rad_id          = 'Honda CRF450R (TR26 carry-over)';
c.cooling.rad_core_w_m    = NaN;        % core width [m]     MEASURE
c.cooling.rad_core_h_m    = NaN;        % core height [m]    MEASURE
c.cooling.rad_core_t_m    = NaN;        % core thickness [m] MEASURE
c.cooling.rad_UA_W_per_K  = NaN;        % radiator UA at fan flow [W/K] - from a bench test
c.cooling.thermal_mass_J_per_K = 16000; % coolant + motor + inverter heat capacity [J/K] PROVISIONAL
                                        %   (~1.5 L water + 12 kg motor + ~6 kg inverter)
end
