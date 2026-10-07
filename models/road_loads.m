function R = road_loads(p, v)
% ROAD_LOADS  Normal loads and resistive forces at speed v. Vectorized over v.
%   R = road_loads(p, v)      v [m/s]
%
%   R.W        car weight m*g [N]
%   R.DF       aero downforce [N]
%   R.N        total normal load W + DF [N]
%   R.Nf, R.Nr static axle loads at speed: weight split by mass_dist_f,
%              downforce split by aero_df_front (the centre of pressure) [N]
%   R.F_drag   aero drag [N]
%   R.F_rr     rolling resistance Crr*N [N]
%   R.F_dl     driveline drag (viscous + Coulomb, referred to the wheel) [N]
%   R.F_loss   F_drag + F_rr + F_dl [N]
%   R.m_eff    translating mass plus rotating inertia, k_rot*m [kg]
%
% Every longitudinal model (gg_envelope, ax_limit, ax_combined) and the lap
% energy accounting (lap_duty_cycle) read their loads from here, so the axle
% split and the resistance terms cannot differ between them. axle_grip uses
% the same split for the lateral limit.
%
% Not modelled: the pitch moment from drag acting above the ground, and
% load transfer from wheel/rotor inertia reaction torques.

v = max(v, 0);
R.W      = p.m * p.g;
R.DF     = 0.5*p.rho*p.ClA .* v.^2;
R.N      = R.W + R.DF;
R.Nf     = p.mass_dist_f .* R.W       + p.aero_df_front .* R.DF;
R.Nr     = (1 - p.mass_dist_f) .* R.W + (1 - p.aero_df_front) .* R.DF;
R.F_drag = 0.5*p.rho*p.CdA .* v.^2;
R.F_rr   = p.Crr .* R.N;
R.F_dl   = (p.b_driveline .* v ./ p.Re + p.Tc_driveline) ./ p.Re;
R.F_loss = R.F_drag + R.F_rr + R.F_dl;
R.m_eff  = p.k_rot .* p.m;
end
