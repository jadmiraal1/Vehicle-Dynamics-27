function p = vehicle_params(mode)
% VEHICLE_PARAMS  Single source of truth for the car.
% Units: SI throughout (kg, m, N, s, rad) unless noted.
%
%   p = vehicle_params()             % normal: grip loaded from tire_coeffs.mat
%   p = vehicle_params('bootstrap')  % car only, grip = NaN. For the tire fits.
%
% Three tiers: INPUTS (measured/decided), LOADED (from tire_coeffs.mat, never
% hand-typed), DERIVED (computed below, never hand-typed).

if nargin < 1, mode = 'full'; end
bootstrap = strcmpi(mode, 'bootstrap');

% ======================= INPUTS =========================================

% --- Mass ---
p.m_car    = 247.66;  % car, DRY, no driver [kg] = 546 lb (TR25 measured)
p.m_driver     = 86.18;  % DESIGN driver [kg] = 180 lb body + 10 lb gear (heaviest at competition)
p.m_driver_min = 61.23;  % LIGHTEST driver [kg] = 125 lb + gear (binding case for rollover)
p.m_driver_max = 95.25;  % heaviest PLAUSIBLE driver [kg] = 200 lb + gear (binding case for peak tire load)
p.m        = p.m_car + p.m_driver;   % DERIVED - never hand-type this
p.mass_dist_f = 0.40;      % static front mass fraction [-]
p.h_cg        = 0.2794;    % CG height above ground [m]

% --- Geometry ---
p.L   = 1.5621;    % wheelbase [m]
p.t_f = 1.20;      % front track [m]
p.t_r = 1.18;      % rear track [m]

% --- Tire identity + belt->track scalings ---
p.tire_id          = 'Hoosier 43075 LC0 16x7.5-10, 8in rim';
p.tire_data_prefix = 'LC0_16x75';   % TTC_Data filename prefix of the design tire
p.mu_derate = 0.67;   % lambda_muY, belt->track grip scaling [-] PROVISIONAL, fit at skidpad
p.lambda_Ca = 0.90;   % lambda_KyA, belt->track slip-stiffness scaling [-] PROVISIONAL
                      % Literature-consistent: cornering stiffness measures ~4-8%
                      % higher on abrasive belt than asphalt (road-roughness study,
                      % Veh. Sys. Dyn. 2025) -> ~0.92-0.96. 0.90 is a slightly
                      % conservative choice; validate with a constant-radius /
                      % steer-torque test. (Contrast the grip derate 0.67: stiffness
                      % is structural, so far less surface-sensitive than peak mu.)
                      % Pneumatic trail = aligning/cornering stiffness, both
                      % structural -> its ratio is ~surface-invariant, so trail's
                      % belt->track factor is ~1.0 (used in aligning_moment.m).
p.Re        = 0.20;   % loaded tire radius [m]

% --- Powertrain ---
p.motor_id      = 'Emrax 228 (axial flux, MV)';
p.inverter_id   = 'Cascadia PM100DX (GEN2, s/n 275)';
p.V_pack_nom    = 298.8;  % nominal voltage [V]
p.V_pack_max    = 348.6;  % max voltage [V]
p.I_pack_max    = 210;    % max discharge current [A]
p.I_fuse_main   = 100;    % main TSB fuse [A], de-facto continuous current rating
p.E_pack_Wh     = 6274.8; % nominal energy [Wh]
p.R_pack        = 0.154;  % pack resistance [ohm] PROVISIONAL
p.T_motor_max   = 220;    % operating torque cap [Nm] (inverter-current limited)
p.T_motor_cont  = 125;    % continuous motor torque [Nm] (reference, unused)
p.gear_ratio    = 3.82;   % final-drive ratio [-]
p.rpm_motor_max = 4500;   % controller over-speed limit [rpm]
p.drive         = 'RWD';  % single motor, rear-wheel drive
p.eta_dt        = 0.88;   % driveline eff. battery->ground [-] PROVISIONAL
p.Crr           = 0.014;  % rolling resistance [-] (TTC LC0 free-rolling FX)
p.k_trac        = 0.90;   % launch traction utilization [-] PROVISIONAL
p.DI = 0.75;               % dynamic index k^2/(a*b) [-] PROVISIONAL

p.b_driveline  = 0.066;  % driveline viscous coefficient [N*m*s/rad] PROVISIONAL
p.Tc_driveline = 0.53;   % driveline coulomb torque [N*m] PROVISIONAL

p.I_rotor = 0.0421;  % motor rotor inertia [kg*m^2]
p.I_wheel = 0.217;   % per-corner wheel+tire inertia [kg*m^2]

% --- Environment / Aero ---
p.g   = 9.81;      % gravity [m/s^2]
p.rho = 1.225;     % air density [kg/m^3]
p.ClA = 1.3010;    % downforce coeff x area [m^2]
p.CdA = 0.9527;    % drag coeff x area [m^2]

% --- Chassis balance ---
p.LLTD          = 0.60;  % lateral load transfer distribution, front frac [-] PROVISIONAL
p.aero_df_front = 0.40;  % front share of downforce (CoP) [-] PROVISIONAL

% ======================= LOADED (generated artifact) =====================
if bootstrap
    p.mu_y_raw      = NaN;
    p.mu_anisotropy = NaN;
    p.Ca_coef       = [NaN NaN NaN];
    p.mu_coef       = [NaN NaN];
    p.Fz_fit_max    = NaN;
    p.mu_hiload_slope = NaN;
    p.hiload_cov_lbf = NaN;
    p.Fz_outer_limit_lbf = NaN;
    p.mu_outer_central   = NaN;
    p.mu_outer_low       = NaN;
    p.shape6        = NaN;
    p.mu_y_18_peak  = NaN;
    p.mu_y_at6      = NaN;
    p.Fz_lat6       = NaN;
    p.mu_x_18       = NaN;
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
    p.hiload_cov_lbf = T.hiload_cov_lbf;   % donor coverage limit [lbf] - past this, mu_of_load's 'central' slope is unvalidated even by donors
    p.Fz_outer_limit_lbf = T.Fz_outer_limit_lbf; % outer-tire cornering load [lbf] - computed ONCE in build_tire_coeffs.m, do not recompute elsewhere
    p.mu_outer_central   = T.mu_outer_central;   % mu at Fz_outer_limit_lbf, donor-informed
    p.mu_outer_low       = T.mu_outer_low;       % mu at Fz_outer_limit_lbf, pessimistic blind-linear
    p.shape6        = T.shape6;         % MF shape fraction at 6deg, load-matched to Fz_lat6
    p.mu_y_18_peak  = T.mu_y_18_peak;   % 18in LC0 lateral peak, back-derived from mu_y_at6/shape6
    p.mu_y_at6      = T.mu_y_at6;       % 18in LC0 measured mu_y at 6deg (raw, not peak)
    p.Fz_lat6       = T.Fz_lat6;        % load the 18in lateral data was actually taken at [lbf]
    p.mu_x_18       = T.mu_x_18;        % 18in LC0 mean drive/brake mu_x
    p.tire_basis    = T.basis;          % 'pacejka-curve'
    p.tire_src_hash = T.src_hash;       % what vd_selftest checks for staleness
end

% 'central' = donor-constrained, 'low' = pessimistic blind-linear. See mu_of_load.m.
% Meaningless in bootstrap mode (no grip curve loaded), so left unset there.
if ~bootstrap
    p.tire_hiload = 'central';
end

% ======================= DERIVED ========================================
p.mu_x_raw = p.mu_y_raw * p.mu_anisotropy;  % follows the design tire automatically
p.mu_y     = p.mu_y_raw * p.mu_derate;      % design peak lateral mu (derated) [-] STATIC, @ design load only - for load-dependent grip use mu_of_load(p, Fz)
p.mu_x     = p.mu_x_raw * p.mu_derate;      % design peak longitudinal mu (derated) [-] STATIC, @ design load only - for load-dependent grip use mu_of_load(p, Fz)

p.a = p.L * (1 - p.mass_dist_f);            % CG to front axle [m]
p.b = p.L * p.mass_dist_f;                  % CG to rear axle [m]
p.Wf_static = p.m * p.g * p.mass_dist_f;        % static front axle load [N]
p.Wr_static = p.m * p.g * (1 - p.mass_dist_f);  % static rear axle load [N]

sumI    = 4*p.I_wheel + p.I_rotor * p.gear_ratio^2;   % rot. inertia at wheel
p.k_rot = 1 + sumI / (p.m * p.Re^2);                  % effective rotating-mass factor [-]

p.Izz = p.DI * p.m * p.a * p.b;             % yaw inertia about CG [kg*m^2]

p.P_max = min(80e3, p.V_pack_nom * p.I_pack_max);   % usable peak power [W], pack-limited below rules cap

p.v_max = (p.rpm_motor_max/p.gear_ratio) * (2*pi/60) * p.Re;   % top speed from motor rev limit [m/s]

N_PER_LBF = 4.44822;  % must match build_tire_coeffs.m's N_PER_LBF - both derive Fz_design_lbf the same way
p.Fz_design_lbf = p.m * p.g / 4 / N_PER_LBF;  % design corner load [lbf], checked against tire_coeffs.mat by vd_selftest
end