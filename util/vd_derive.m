function p = vd_derive(p)
% VD_DERIVE  Recompute every quantity that FOLLOWS from the car's inputs.
%   p = vd_derive(p)
%
% WHAT "DERIVED" MEANS
% -------------------
% The params struct holds two kinds of number:
%
%   INPUTS   - things you measured, decided, or fitted. Mass, wheelbase, LLTD,
%              gear ratio, the tire coefficients. Nothing in the code computes
%              these; they come from cars/config_<CAR>.m or the tire artifact.
%
%   DERIVED  - things that are pure arithmetic on the inputs. Total mass, CG
%              distances, static axle loads, rotating-mass factor, top speed.
%              These have no independent existence: if you change an input,
%              every derived value that depends on it is now WRONG until it is
%              recomputed.
%
% This function is that recomputation, and it is the ONLY copy of it.
%
% WHY IT IS A SEPARATE FUNCTION
% -----------------------------
% Studies constantly ask "what if?" - what if the pack were 3 cells longer,
% what if the car were 10 kg heavier, what if the mass moved rearward. Each of
% those changes an INPUT, which silently invalidates a handful of DERIVED
% values. Before this function existed, six different places in targets/ each
% hand-copied a subset of the arithmetic below, and every one of them copied a
% DIFFERENT subset:
%
%   run_pack_targets      re-derived 9 quantities
%   run_aero_targets      re-derived 3   (forgot Izz, v_max, Fz_design_lbf)
%   run_stability_targets re-derived 4   (forgot k_rot, Izz, ...)
%   run_wdist_targets     re-derived 4   (inline, same omissions)
%   run_lap_targets       re-derived 1   (k_rot only)
%   run_gear_targets      re-derived 2   (k_rot, v_max)
%
% None of those were wrong for what each study needed at the time. But it means
% adding a new derived quantity to the car silently misses six places, and a
% newcomer has no way to know which subset is safe. One function, called
% everywhere, removes the whole class of mistake.
%
% ORDER MATTERS: p.m must exist before the axle loads, p.a/p.b before Izz.
% Read it top to bottom; each line only uses inputs or lines above it.
%
% See also VD_SET (change an input and re-derive in one call), VEHICLE_PARAMS.

k = vd_const();

% --- total mass ---------------------------------------------------------
% Never hand-type p.m. It is m_car + the design driver, always.
p.m = p.m_car + p.m_driver;                     % [kg]

% --- grip at the design load -------------------------------------------
% The artifact stores RAW (lab) grip; the belt-to-track derate is applied here
% so that changing mu_derate in the config takes effect without a tire refit.
% These are the static, design-load values - use mu_of_load(p,Fz) for the
% load-dependent grip that every solver actually wants.
p.mu_x_raw = p.mu_y_raw * p.mu_anisotropy;      % [-] follows the design tire
p.mu_y     = p.mu_y_raw * p.mu_derate;          % [-] derated peak lateral
p.mu_x     = p.mu_x_raw * p.mu_derate;          % [-] derated peak longitudinal

% --- CG position and static loads --------------------------------------
% mass_dist_f is the FRONT mass fraction. More mass at the front means the CG
% sits closer to the front axle, so a (CG->front) shrinks as mass_dist_f grows.
p.a = p.L * (1 - p.mass_dist_f);                % CG to front axle [m]
p.b = p.L * p.mass_dist_f;                      % CG to rear axle [m]
p.Wf_static = p.m * p.g * p.mass_dist_f;        % static front axle load [N]
p.Wr_static = p.m * p.g * (1 - p.mass_dist_f);  % static rear axle load [N]

% --- sprung / unsprung split -------------------------------------------
% Unsprung mass is a fraction of the CAR (the driver is entirely sprung).
p.m_unsprung   = p.f_unsprung * p.m_car;        % [kg]
p.m_unsprung_f = p.unsprung_f * p.m_unsprung;   % [kg]
p.m_unsprung_r = p.m_unsprung - p.m_unsprung_f; % [kg]
p.m_sprung     = p.m - p.m_unsprung;            % [kg] driver included

% --- rotating inertia ---------------------------------------------------
% Accelerating the car also spins up four wheels and the rotor. Referred to the
% wheel, that behaves like extra mass: k_rot > 1 is the penalty factor. The
% rotor term carries gear_ratio^2 because inertia referred through a gearbox
% scales with the square of the ratio.
sumI    = 4*p.I_wheel + p.I_rotor * p.gear_ratio^2;   % [kg*m^2] at the wheel
p.k_rot = 1 + sumI / (p.m * p.Re^2);                  % [-]

% --- yaw inertia --------------------------------------------------------
% DI is the dimensionless "dynamic index": Izz = DI * m * a * b. DI = 1 means
% the yaw inertia equals that of two point masses at the axles.
p.Izz = p.DI * p.m * p.a * p.b;                 % [kg*m^2]

% --- electrical ceilings ------------------------------------------------
RULES_P_MAX_W = 80e3;                           % FSAE tractive-system power cap
p.P_max = min(RULES_P_MAX_W, p.V_pack_nom * p.I_pack_max);   % usable peak [W]

% --- top speed ----------------------------------------------------------
% Motor rev limit through the final drive. NOTE this is a KINEMATIC ceiling,
% not a force balance - the car may not have the thrust to reach it.
p.v_max = (p.rpm_motor_max/p.gear_ratio) * (2*pi/60) * p.Re;  % [m/s]

% --- design corner load -------------------------------------------------
% Static load on one tire, in lbf because that is the tire fit's currency.
% vd_selftest checks this against the load the artifact was built at: if the
% car's mass moved, the tire artifact is describing a different car.
p.Fz_design_lbf = p.m * p.g / 4 / k.N_PER_LBF;  % [lbf]
end
