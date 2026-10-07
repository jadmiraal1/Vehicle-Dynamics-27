function r = fsae_rules()
% FSAE_RULES  Competition constants: event geometry, power cap, scoring inputs.
%   r = fsae_rules();
%
% Values follow the FSAE Rules 2026 V1.0 (unchanged in 2027 V1.0). Rule numbers
% are given so each value can be checked against the rulebook. These are
% properties of the competition, not of a car - change them only when the
% rulebook changes.

r.rules_version      = 'FSAE Rules 2026 V1.0';

% --- tractive system ------------------------------------------------------
r.P_max_W            = 80e3;     % EV.3.3.1  max power at the Energy Meter [W]

% --- event geometry -------------------------------------------------------
r.accel_m            = 75;       % D.9.1.1   start line to finish line [m]
r.accel_staging_m    = 0.30;     % D.9.2.3   foremost part of car behind the start line [m]
r.skidpad_R_m        = 9.125;    % D.10.1.1  path centre radius = 15.25/2 + 3.0/2 [m]
r.endurance_m        = 22000;    % D.12.1.3  approximate total distance [m]

% --- efficiency scoring (D.13.4) -----------------------------------------
r.co2_kg_per_kWh     = 0.65;     % D.13.4.1  EV energy -> CO2
r.co2_ref_kg_per_km  = 0.2002;   % D.13.4.5  EV reference 20.02 kg / 100 km (sets EF_min)
r.ef_min_time_factor = 1.45;     % D.13.4.5  T_your = 1.45 x T_min when computing EF_min
end
