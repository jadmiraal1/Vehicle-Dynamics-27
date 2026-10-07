function p = vd_derive(p)
% VD_DERIVE  Recompute every quantity that follows from the car's inputs.
%   p = vd_derive(p)
%
% p holds two kinds of number:
%   INPUTS   measured, decided or fitted: mass, wheelbase, LLTD, gear ratio,
%            tire coefficients. They come from cars/config_<CAR>.m or the
%            tire artifact.
%   DERIVED  pure arithmetic on the inputs: total mass, CG position, static
%            axle loads, rotating-mass factor, top speed. Change an input and
%            these are wrong until recomputed.
%
% This function is the only place derived values are computed. vehicle_params
% calls it once; vd_set calls it after every "what if?" change, so a study
% never has to remember which derived values its change affects.
% Each line uses only inputs or lines above it.
%
% See also VD_SET, VEHICLE_PARAMS.

k = vd_const();

% Cached copy for hot loops (the tire evaluator runs ~1e5 times per g-g-V
% surface and a struct read is much cheaper than a function call). Code
% outside hot loops calls vd_const() directly.
p.N_PER_LBF = k.N_PER_LBF;                      % [N/lbf]

% --- total mass ---------------------------------------------------------
p.m = p.m_car + p.m_driver;                     % [kg]

% --- grip at the design load -------------------------------------------
% The artifact stores raw (TTC belt) grip; the belt-to-track scaling is
% applied here, so changing mu_derate needs no refit. These are values at
% the design load only - solvers use mu_of_load(p, Fz).
p.mu_x_raw = p.mu_y_raw * p.mu_anisotropy;      % [-] follows the design tire
p.mu_y     = p.mu_y_raw * p.mu_derate;          % [-] derated peak lateral
p.mu_x     = p.mu_x_raw * p.mu_derate;          % [-] derated peak longitudinal

% --- CG position and static loads --------------------------------------
% mass_dist_f is the front mass fraction, so the CG-to-front distance a
% shrinks as it grows.
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
% Spinning up four wheels and the rotor acts like extra mass: k_rot > 1.
% Rotor inertia referred to the wheel scales with gear_ratio^2.
sumI    = 4*p.I_wheel + p.I_rotor * p.gear_ratio^2;   % [kg*m^2] at the wheel
p.k_rot = 1 + sumI / (p.m * p.Re^2);                  % [-]

% --- yaw inertia --------------------------------------------------------
% Dynamic index DI = k^2/(a*b): DI = 1 is two point masses at the axles.
p.Izz = p.DI * p.m * p.a * p.b;                 % [kg*m^2]

% --- electrical ceilings ------------------------------------------------
% Pack-side power: the rules cap (measured at the energy meter) or what the
% pack can deliver at nominal voltage and maximum current, whichever is lower.
p.P_max = min(fsae_rules().P_max_W, p.V_pack_nom * p.I_pack_max);   % [W]

% --- top speed ----------------------------------------------------------
% Rev limit through the final drive: a kinematic ceiling, not a force
% balance - the car may not have the thrust to reach it.
p.v_max = (p.rpm_motor_max/p.gear_ratio) * (2*pi/60) * p.Re;  % [m/s]

% --- design corner load -------------------------------------------------
% Mean static load per tire [lbf]. vd_selftest compares it with the load
% the tire artifact was built at, which catches a mass change since the fit.
p.Fz_design_lbf = p.m * p.g / 4 / k.N_PER_LBF;  % [lbf]
end
