function c = config_TR26()
% CONFIG_TR26  Last year's car AS BUILT - the validation/calibration reference.
% Simulate this config to compare against TR26 test data (e.g. measured braking)
% when pinning the belt-to-track scalings.
%
% Same contract as config_TR27: this file is the COMPLETE specification of one
% car. Fields marked [verify] are carried from the TR27 file and need the actual
% TR26 records before any calibration conclusion is drawn - a copied TR27 value
% left here would make a calibration run compare TR27 against itself and report
% a perfect match.

% --- Mass ---
c.m_car        = 247.66;  % dry, no driver [kg] = 546 lb (measured)
                          %   NOTE: the suspension sheet that supplied the
                          %   unsprung ratio below lists 500 lb dry. Resolve the
                          %   46 lb disagreement before calibrating on sprung mass.
c.m_driver     = 77.11;   % TR26 driver [kg] = 170 lbm, per the suspension sheet
c.m_driver_min = 61.23;   % [verify] TR26 lightest driver [kg]
c.m_driver_max = 95.25;   % [verify] TR26 heaviest driver [kg]
c.mass_dist_f  = 0.45;    % [verify] actual TR26 corner-weight split
c.h_cg         = 0.3048;  % 12 in, suspension sheet (TR26 as built)

% --- Sprung / unsprung split ---
c.f_unsprung   = 0.180;   % [verify] 90/500 lbm from the suspension sheet - but that
                          %   sheet lists 500 lb dry where c.m_car above is 546 lb.
                          %   On 546 lb the ratio is 0.165. Resolve before any
                          %   calibration conclusion rests on TR26 sprung mass.
c.unsprung_f   = 0.4444;  % front share of unsprung mass [-] (TR26 sheet: 40/90 lbm)

% --- Geometry ---
c.L            = 1.5621;  % wheelbase [m] (61.5 in, matches the suspension sheet)
c.t_f          = 1.1557;  % front track [m] = 45.5 in, suspension sheet
c.t_r          = 1.1049;  % rear track [m]  = 43.5 in, suspension sheet

% --- Tire identity + belt->track scalings ---
c.tire_id          = 'Hoosier 43075 LC0 16x7.5-10, 8in rim';   % [verify] TR26 tire
c.tire_data_prefix = 'LC0_16x75';                               % [verify]
c.mu_derate    = 0.67;    % [verify] this is the TR27 assumption; TR26 measured
                          %   braking (1.6 g peak) suggests it may be pessimistic
c.lambda_Ca    = 0.90;    % [verify]
c.Re           = 0.20;    % [verify] TR26 loaded tire radius [m]

% --- Aero ---
c.ClA          = 1.3010;  % measured TR26 aero
c.CdA          = 0.9527;

% --- Chassis balance ---
c.LLTD         = 0.60;    % [verify] TR26 roll-stiffness split
c.aero_df_front = 0.40;   % [verify] TR26 CoP

% --- Accumulator ---
% [verify] TR26 as-run pack. Carried from the TR27 file until the build records
% are found; the ratio-based form makes a correction a one-number change.
CELL_V_NOM = 3.6;   CELL_V_MAX = 4.2;
CELL_AH    = 3.0;   CELL_I_MAX = 30;
c.cell_id  = 'Sony/Murata US18650VTC6';   % [verify]
c.pack_S   = 83;          % [verify] TR26 series count
c.pack_P   = 7;           % [verify] TR26 parallel count

c.V_pack_nom  = c.pack_S * CELL_V_NOM;
c.V_pack_max  = c.pack_S * CELL_V_MAX;
c.I_pack_max  = c.pack_P * CELL_I_MAX;
c.E_pack_Wh   = c.pack_S * c.pack_P * CELL_V_NOM * CELL_AH;
c.I_fuse_main = 100;      % [verify] TR26 main fuse [A]
c.R_pack      = 0.154;    % [verify] PROVISIONAL

% --- Motor / inverter ---
c.motor_id      = 'Emrax 228 (axial flux, MV)';
c.inverter_id   = 'Cascadia PM100DX (GEN2, s/n 275)';
c.T_motor_max   = 220;    % [verify] TR26 as-run torque cap [Nm]
c.T_motor_cont  = 125;    % [verify]
c.rpm_motor_max = 4500;   % [verify] TR26 as-run rev cap [rpm] - voltage-governed
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
% TR26 ran WITHOUT regen as far as the records show - set capture to 0 to
% simulate the car as it actually ran. Change only with evidence.
c.scenario.regen_capture = 0.00;   % [verify] TR26 had no regen implementation
c.scenario.regen_rt      = 0.65;   % unused while capture is 0
c.scenario.pack_usable_f = 0.90;   % [verify] TR26 BMS SoC window
c.scenario.margin        = 0.90;   % CHOICE
c.scenario.ClA_target    = 1.3010; % TR26 has no future target - it is the built car
c.scenario.CdA_target    = 0.9527;
end
