function p = vehicle_params(mode)
% VEHICLE_PARAMS  Single source of truth for the car.
% Units: SI throughout (kg, m, N, s, rad) unless noted.
%
%   p = vehicle_params()             % normal: grip loaded from tire_coeffs.mat
%   p = vehicle_params('bootstrap')  % car only, grip = NaN. For the tire fits.
%
% THREE TIERS, and nothing else:
%   INPUTS  - measured or decided. Each carries a source comment.
%   LOADED  - produced by a model, read from a generated artifact. NEVER
%             hand-typed here. Currently: tire grip, from tire_coeffs.mat.
%   DERIVED - computed below from the two above. Never hand-typed.
%
% The LOADED tier is new (Jul 2026). Grip used to be hand-copied out of
% ttc_fit's printout; mu_y_raw = 2.602 and mu_anisotropy = 1.008 sat here as
% literals, the second with no code behind it anywhere. Now build_tire_coeffs
% is the only producer and this file only consumes. vd_selftest fails if the
% artifact is stale, so the fit and the car cannot silently disagree.
%
% 'bootstrap' exists solely to break the circular dependency: ttc_fit and
% pacejka_fit need the car's mass (to know the design corner load) but must
% not require the artifact they are about to produce.

if nargin < 1, mode = 'full'; end
bootstrap = strcmpi(mode, 'bootstrap');

% ======================= INPUTS =========================================

% --- Mass ---
% SPLIT ON PURPOSE. Mass used to be one opaque number (331.1224 kg) that mixed
% a MEASUREMENT and an ASSUMPTION, and that is exactly how the units slip in
% Jul 2026 happened: 546 (the car's weight in POUNDS, dry) was written into a
% kilograms field that was supposed to hold car + driver. It doubled the car and
% pushed the design corner load off the end of the tire data, silently.
%
% Now the two are separable, so a units error in one cannot hide inside the other,
% and driver mass is SWEEPABLE - it is an assumption, not a constant of nature.
p.m_car    = 247.66;  % car, DRY, no driver [kg] = 546 lb (TR25 measured)

% DRIVER ENVELOPE. Gear (helmet + HANS + suit + gloves + shoes) is ~10 lb / 4.5 kg
% and is INCLUDED in every figure below - it is routinely forgotten.
p.m_driver     = 86.18;  % DESIGN driver [kg] = 180 lb body + 10 lb gear.
                         % The HEAVIEST DRIVER AT COMPETITION. This is the car as
                         % raced, so it is the design point for every performance
                         % and load target handed to another subteam.
p.m_driver_min = 61.23;  % LIGHTEST driver [kg] = 125 lb + gear (our primary).
                         % !! NOT just an optimistic case. It is the BINDING case
                         % for ROLLOVER (#3/#5) - see below.
p.m_driver_max = 95.25;  % heaviest PLAUSIBLE driver in testing [kg] = 200 lb + gear.
                         % Binding case for peak tire load and structural margin.
%
% THERE IS NO SINGLE CONSERVATIVE DRIVER MASS. The direction flips by target:
%
%   lap time / accel / energy   -> heavy driver is conservative (slower)
%   design grip mu(Fz)          -> heavy driver is conservative (mu falls with load)
%   brake bias, axle loads      -> heavy driver is conservative
%   ROLLOVER MARGIN             -> *LIGHT* DRIVER IS CONSERVATIVE  <-- the trap
%
% Rollover margin = (t/2)/(h*mu). Tire mu RISES as load falls, so a lighter driver
% makes MORE grip per tire and the car tips SOONER. Heavy driver flatters the
% rollover number. Margin runs 1.34x (light) -> 1.37x (200 lb) against a 1.3x
% target: the light driver is the one that gets closest to lifting a wheel.
% run_load_transfer_targets checks both ends. Do not quote the heavy-driver
% rollover figure as if it were the safe one.
p.m        = p.m_car + p.m_driver;   % DERIVED - never hand-type this
p.mass_dist_f = 0.40;      % static front mass fraction [-]
p.h_cg        = 0.2794;    % CG height above ground [m]

% --- Geometry ---
p.L   = 1.5621;    % wheelbase [m]
p.t_f = 1.20;      % front track [m]
p.t_r = 1.18;      % rear track [m]

% --- Tire identity + belt->track scalings (design DECISIONS, not fit outputs)
% Two scalings, never merged: peak grip is surface-dominated, cornering
% stiffness is carcass-dominated. Different physics, different validation test.
p.tire_id          = 'Hoosier 43075 LC0 16x7.5-10, 8in rim';
p.tire_data_prefix = 'LC0_16x75';   % TTC_Data filename prefix of the design tire
% TTC datasets are named <compound>_<size>. The folder is a compound x size
% factorial and the old names hid it -- 'R20' and 'BigR20' were the SAME R20
% compound in two different casings, which was impossible to tell from the
% name. Likewise 'LC0' and '18inLC0'. Now:
%
%              16x7.5-10 (8in rim)   18.0x6.0-10 (7in rim)
%   LC0        LC0_16x75  (43075)    LC0_18x60  (41100)   <- the ONLY drive/brake set
%   R20        R20_16x75  (43075)    R20_18x60  (43100)
%   Goodyear                         GY_18x65   (D0571, 18.0x6.5-10)
%
% Same part number 43075 => LC0_16x75 vs R20_16x75 is a clean COMPOUND A/B
% (identical casing). R20_16x75 vs R20_18x60 is a clean SIZE A/B (identical
% compound). That is what these names are for.
p.mu_derate = 0.67;   % lambda_muY, belt->track grip scaling [-]
                      % PROVISIONAL; fit at skidpad (steady-state lateral g).
                      % ALSO GATES THE ROLLOVER MARGIN (targets #3/#5): the
                      % margin is (t/2)/(h*mu) and re-binds above ~0.70.
p.lambda_Ca = 0.90;   % lambda_KyA, belt->track slip-stiffness scaling [-]
                      % PROVISIONAL; fit from steering-response tests.
                      % Applies to cornering stiffness, NOT to mu.
p.Re        = 0.20;   % loaded tire radius [m]

% --- Powertrain ---
p.motor_id      = 'Emrax 228 (axial flux, MV)';
p.inverter_id   = 'Cascadia PM100DX (GEN2, s/n 275)';
                          % 100-420 VDC, 300/350 Arms cont/peak -> the pack's
                          % 210 A, not the inverter, is the binding current
                          % limit. Efficiency curve + software IQ limit pending.
p.V_pack_nom    = 298.8;  % nominal voltage [V]  (83s7p VTC6, powertrain Jul 2026)
p.V_pack_max    = 348.6;  % max voltage [V]
p.I_pack_max    = 210;    % max discharge current [A] (= 30 A/cell, 10C burst)
p.I_fuse_main   = 100;    % main TSB fuse [A] (ESF schematic: Bourns PF-FB1-100).
                          % The de-facto CONTINUOUS current rating: ~30 kW
                          % sustained electrical power at nominal V. 210 A is
                          % transient-only (fuse i2t). Anchors endurance power
                          % strategy; future P(t) model should enforce both.
p.E_pack_Wh     = 6274.8; % nominal energy [Wh] (usable is less; TBD)
p.R_pack        = 0.154;  % pack resistance [ohm]: 83s/7p x 13 mOhm/cell (VTC6
                          % datasheet, AC 1 kHz). DC-IR runs 1.5-2x AC ->
                          % sag 32-50 V at 210 A, deliverable ~52-56 kW at
                          % nominal V. PROVISIONAL - confirm with a pack load
                          % test; feeds the future P(v,SoC) model.
p.T_motor_max   = 220;    % OPERATING torque cap [Nm]. Motor datasheet max is
                          % 240 Nm (few s); 220 is the inverter-current limit:
                          % 220 Nm / 0.75 Nm/Arms (MV) = 293 Arms ~ PM100DX
                          % 300 Arms continuous. Three sources agree (Emrax
                          % data, PM100DX rating, powertrain statement).
p.T_motor_cont  = 125;    % continuous motor torque [Nm] (reference, unused)
p.gear_ratio    = 3.82;   % final-drive ratio [-]
p.rpm_motor_max = 4500;   % controller over-speed limit [rpm]. RESOLVED Jul 2026:
                          % Cascadia Emrax setup app note, Motor_Type 128 (=
                          % Emrax 228 MV, matches team EEPROM config): over-speed
                          % fault 4500, break speed 4000 @360 V, FW parameterized
                          % (ID_Limit, 221 Apk). Supersedes the motor's mechanical
                          % 5500 rpm. Verify team's actual Max_Speed_EEPROM value.
                          % Course impact ~nil (2026 tracks peak ~25 m/s); accel
                          % v_end and T-VMAX (#56) re-issue required.
p.drive         = 'RWD';  % single motor, rear-wheel drive
p.eta_dt        = 0.88;   % driveline eff. battery->ground [-] PROVISIONAL
p.Crr           = 0.014;  % rolling resistance [-] (TTC LC0 free-rolling FX)
p.k_trac        = 0.90;   % launch traction utilization [-] PROVISIONAL

% Yaw inertia via dynamic index DI = k^2/(a*b) (Milliken); centralized cars
% run DI ~0.7-0.8. PROVISIONAL - replace with CAD/pendulum Izz (target #6)
p.DI = 0.75;

% Driveline drag at the rear axle, T = b*w + Tc (TR26 roll test, Jun 2026,
% chain-on, no-load spin-down, extrapolated - PROVISIONAL)
p.b_driveline  = 0.066;  % viscous coefficient [N*m*s/rad]
p.Tc_driveline = 0.53;   % coulomb torque [N*m]

% Rotating inertia (feeds k_rot below)
p.I_rotor = 0.0421;  % motor rotor inertia [kg*m^2] (Emrax 228 datasheet)
p.I_wheel = 0.217;   % per-corner wheel+tire inertia [kg*m^2] (CAD, Jul 2026)

% --- Environment / Aero ---
p.g   = 9.81;      % gravity [m/s^2]
p.rho = 1.225;     % air density [kg/m^3]
p.ClA = 1.3010;    % downforce coeff x area [m^2]
p.CdA = 0.9527;    % drag coeff x area [m^2]

% --- Chassis balance (mid-tier axle-grip model, run_balance_targets) ---
p.LLTD          = 0.60;  % lateral load transfer distribution, front frac [-]
                         % PROVISIONAL: neutral point 0.57; set just front of
                         % neutral -> front-limited (stable) limit balance for
                         % <=1.5% grip cost. Physically = roll stiffness split;
                         % re-tune when kinematics are real.
p.aero_df_front = 0.40;  % front share of downforce (CoP) [-] PROVISIONAL
                         % target #38 band 0.35-0.42 at LLTD 0.60

% ======================= LOADED (generated artifact) =====================
% Produced by build_tire_coeffs.m from the TTC data. Do NOT hand-type these.
if bootstrap
    p.mu_y_raw      = NaN;
    p.mu_anisotropy = NaN;
    p.Ca_coef       = [NaN NaN NaN];
    p.mu_coef       = [NaN NaN];
    p.Fz_fit_max    = NaN;
    p.mu_hiload_slope = NaN;
    p.tire_basis    = 'bootstrap (no grip)';
    p.tire_src_hash = '';
else
    here = fileparts(mfilename('fullpath'));
    art  = fullfile(here, 'tire_coeffs.mat');
    if ~exist(art, 'file')
        error('vehicle_params:noTireCoeffs', ...
              'tire_coeffs.mat not found. Grip is generated, not typed. Run: build_tire_coeffs');
    end
    T = load(art);
    p.mu_y_raw      = T.mu_y_raw;       % curve-based peak lateral mu @ design load
    p.mu_anisotropy = T.mu_anisotropy;  % mu_x/mu_y, computed cross-tire transfer
    p.Ca_coef       = T.Ca_coef;        % Ca(Fz) quadratic [lbf/deg]
    p.mu_coef       = T.mu_coef;        % mu(Fz) linear, VALID ONLY to Fz_fit_max
    p.Fz_fit_max    = T.Fz_fit_max;     % design tire's data edge [lbf]
    p.mu_hiload_slope = T.mu_hiload_slope; % donor-informed slope above the edge
    p.tire_basis    = T.basis;          % 'pacejka-curve'
    p.tire_src_hash = T.src_hash;       % what vd_selftest checks for staleness
end

% High-load mu model above the design tire's data edge (see mu_of_load.m):
%   'central' = donor-constrained (the 18in tires' measured flattening)
%   'low'     = pessimistic blind-linear extension. Run both to get the band.
p.tire_hiload = 'central';

% ======================= DERIVED ========================================
p.mu_x_raw = p.mu_y_raw * p.mu_anisotropy;  % follows the design tire automatically
p.mu_y     = p.mu_y_raw * p.mu_derate;      % design peak lateral mu (derated) [-]
p.mu_x     = p.mu_x_raw * p.mu_derate;      % design peak longitudinal mu (derated) [-]

p.a = p.L * (1 - p.mass_dist_f);            % CG to front axle [m]
p.b = p.L * p.mass_dist_f;                  % CG to rear axle [m]
p.Wf_static = p.m * p.g * p.mass_dist_f;        % static front axle load [N]
p.Wr_static = p.m * p.g * (1 - p.mass_dist_f);  % static rear axle load [N]

% Effective rotating-mass factor (m_eff = k_rot*m); reference doc section 5
sumI    = 4*p.I_wheel + p.I_rotor * p.gear_ratio^2;   % rot. inertia at wheel
p.k_rot = 1 + sumI / (p.m * p.Re^2);                  % [-]

% Yaw inertia about the vertical axis through the CG [kg*m^2]
p.Izz = p.DI * p.m * p.a * p.b;

% Usable peak power [W]: pack-limited (62.7 kW at nominal V, 73.2 kW at full
% charge) below the 80 kW rules cap -- the PACK limits power, not the rules.
p.P_max = min(80e3, p.V_pack_nom * p.I_pack_max);

% Top speed from the motor rev limit through the final drive [m/s]
p.v_max = (p.rpm_motor_max/p.gear_ratio) * (2*pi/60) * p.Re;

% Design corner load [lbf] - the load the tire fit is evaluated AT. Exposed so
% vd_selftest can check it against the artifact (a units slip here silently
% invalidates every grip number).
p.Fz_design_lbf = p.m * p.g / 4 / 4.44822;
end
