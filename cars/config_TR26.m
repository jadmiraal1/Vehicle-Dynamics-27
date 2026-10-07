function c = config_TR26()
% CONFIG_TR26  Last year's car as built: the validation / calibration reference.
% Simulate it to compare with TR26 test data (e.g. measured braking) when
% calibrating the belt-to-track scalings.
%
% Fields marked [verify] were carried over from the TR27 file and need the
% TR26 records before any calibration conclusion is drawn: a copied TR27
% value would make a calibration compare TR27 with itself.

% --- Mass ---
c.m_car        = 247.66;  % dry, no driver [kg] = 546 lb (measured). The suspension
                          %   sheet lists 500 lb dry - resolve before calibrating on sprung mass.
c.m_driver     = 77.11;   % TR26 driver [kg] = 170 lbm, per the suspension sheet
c.m_driver_min = 61.23;   % [verify] TR26 lightest driver [kg]
c.m_driver_max = 95.25;   % [verify] TR26 heaviest driver [kg]
c.mass_dist_f  = 0.45;    % [verify] actual TR26 corner-weight split
c.h_cg         = 0.3048;  % 12 in, suspension sheet (TR26 as built)

% --- Sprung / unsprung split ---
c.f_unsprung   = 0.180;   % [verify] 90/500 lbm from the suspension sheet; on the
                          %   measured 546 lb it would be 0.165.
c.unsprung_f   = 0.4444;  % front share of unsprung mass [-] (TR26 sheet: 40/90 lbm)

% --- Geometry ---
c.L            = 1.5621;  % wheelbase [m] (61.5 in, matches the suspension sheet)
c.t_f          = 1.1557;  % front track [m] = 45.5 in, suspension sheet
c.t_r          = 1.1049;  % rear track [m]  = 43.5 in, suspension sheet

% --- Tire identity and belt-to-track scaling ---
c.tire_id          = 'Hoosier 43075 LC0 16x7.5-10, 8in rim';   % [verify] TR26 tire
c.tire_data_prefix = 'LC0_16x75';                               % [verify]
c.mu_derate    = 0.67;    % [verify] the TR27 assumption; TR26 measured braking
                          %   (1.6 g peak) suggests it may be pessimistic
c.lambda_Ca    = 0.90;    % [verify]
c.Re           = 0.20;    % [verify] TR26 loaded tire radius [m]

% --- Aero ---
c.ClA          = 1.3010;  % measured TR26 aero
c.CdA          = 0.9527;  % measured TR26 drag

% --- Chassis balance ---
c.LLTD         = 0.60;    % [verify] TR26 roll-stiffness split
c.aero_df_front = 0.40;   % [verify] TR26 CoP

% --- Accumulator ---
% [verify] TR26 as-run pack, carried from the TR27 file until the build
% records are found.
c.cell_id  = 'Sony/Murata US18650VTC6';   % [verify]
c.cell     = struct('V_nom', 3.6, 'V_max', 4.2, 'Ah', 3.0, ...   % per cell [V], [V], [Ah]
                    'I_max', 30, 'mass_kg', 0.0466);              % [A], [kg]
c.pack_S   = 83;          % [verify] TR26 series count
c.pack_P   = 7;           % [verify] TR26 parallel count

c.V_pack_nom  = c.pack_S * c.cell.V_nom;
c.V_pack_max  = c.pack_S * c.cell.V_max;
c.I_pack_max  = c.pack_P * c.cell.I_max;
c.E_pack_Wh   = c.pack_S * c.pack_P * c.cell.V_nom * c.cell.Ah;
c.I_fuse_main = 100;      % [verify] TR26 main fuse [A]
c.R_pack      = 0.154;    % [verify] PROVISIONAL

% --- Motor / inverter ---
c.motor_id      = 'Emrax 228 (axial flux, MV)';
c.inverter_id   = 'Cascadia PM100DX (GEN2, s/n 275)';
c.T_motor_max   = 220;    % [verify] TR26 as-run torque cap [Nm]
c.T_motor_cont  = 125;    % [verify]
c.rpm_motor_max = 4500;   % [verify] TR26 as-run rev cap [rpm] - set by pack voltage
c.drive         = 'RWD';

% --- Driveline ---
c.gear_ratio    = 3.82;   % as run
c.eta_dt        = 0.88;   % [verify] PROVISIONAL
c.eta_chain     = 0.97;   % [verify] PROVISIONAL
c.eta_inv       = 0.95;   % [verify] PROVISIONAL
c.Crr           = 0.014;  % [verify]
c.k_trac        = 0.90;   % [verify] PROVISIONAL
c.b_driveline   = 0.066;  % [verify] PROVISIONAL
c.Tc_driveline  = 0.53;   % [verify] PROVISIONAL

% --- Inertias ---
c.DI       = 0.75;        % [verify] PROVISIONAL
c.I_rotor  = 0.0421;      % motor rotor inertia [kg*m^2]
c.I_wheel  = 0.217;       % [verify] TR26 per-corner wheel+tire inertia [kg*m^2]

% --- Strategy assumptions ---
% TR26 ran without regen as far as the records show.
c.scenario.regen_capture = 0.00;   % [verify] TR26 had no regen implementation
c.scenario.regen_rt      = 0.65;   % unused while capture is 0
c.scenario.pack_usable_f = 0.90;   % [verify] TR26 BMS SoC window
c.scenario.margin        = 0.90;   % CHOICE
c.scenario.ClA_target    = 1.3010; % no future target: the built car
c.scenario.CdA_target    = 0.9527;

% --- Cooling loop ---
% [verify] Same block as config_TR27 (the TR26 hardware, carried over).
c.cooling.loop_order      = 'inverter_first';  % [verify] check the hoses
c.cooling.coolant         = 'water';
c.cooling.flow_Lpm        = 10;         % [L/min] PROVISIONAL - MEASURE
c.cooling.T_amb_design_C  = 35;         % [degC] CHOICE
c.cooling.T_cold_target_C = 50;         % [degC] CHOICE
c.cooling.T_inv_full_C    = 60;         % PM100DX [datasheet]
c.cooling.T_inv_derate_C  = 80;         % PM100DX [datasheet]
c.cooling.T_motor_rated_C = 50;         % EMRAX 228 [datasheet]
c.cooling.FOS             = 2.0;        % CHOICE
c.cooling.fan_id          = 'SPAL VA32-A101-62S (4in, pusher curve) PROVISIONAL';
c.cooling.fan_dp_mmH2O    = [0 2.5 5 7.5 10 12.5 15 17.5 20];   % [mm H2O]
c.cooling.fan_m3h         = [210 190 160 120 90 70 60 30 0];    % [m^3/h]
c.cooling.core_dp_mmH2O   = 5;          % [mm H2O] PROVISIONAL
c.cooling.ram_capture     = 0.30;       % [-] PROVISIONAL
c.cooling.rad_id          = 'Honda CRF450R (TR26 carry-over)';
c.cooling.rad_core_w_m    = NaN;        % MEASURE
c.cooling.rad_core_h_m    = NaN;        % MEASURE
c.cooling.rad_core_t_m    = NaN;        % MEASURE
c.cooling.rad_UA_W_per_K  = NaN;        % bench test
c.cooling.thermal_mass_J_per_K = 16000; % [J/K] PROVISIONAL
end
