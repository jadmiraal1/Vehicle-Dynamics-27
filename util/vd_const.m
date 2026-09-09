function k = vd_const()
% VD_CONST  Unit conversions and universal constants. The ONE home for them.
%   k = vd_const();   k.N_PER_LBF, k.DEG_PER_RAD, ...
%
% WHY THIS FILE EXISTS
% --------------------
% The repo's rule is "every number has exactly one home" (codebase_guide sec 3).
% Before this file, 4.44822 was written out in 15 different places across 11
% files - three of them as bare literals with no name at all. That is not a
% style problem: it means there is no single place to look when you are chasing
% a factor-of-4.448 error, and no way to be sure all 15 agree.
%
% Physical constants that belong to the WORLD (gravity, air density) live on
% the params struct as p.g and p.rho, because a study may legitimately want to
% change them (a hot day, altitude). Unit conversions belong HERE, because
% there is no such thing as a different number of newtons per pound-force.
%
% USAGE
%   k = vd_const();
%   Fz_lbf = Fz_N / k.N_PER_LBF;
%
% Anything derived from these lives in vd_derive.m, not here.

k.N_PER_LBF      = 4.44822;      % force: 1 lbf = 4.44822 N (exact by definition)
k.DEG_PER_RAD    = 180/pi;       % angle
k.MM_PER_IN      = 25.4;         % length (exact)
k.M_PER_IN       = 0.0254;       % length (exact)
k.NM_PER_FTLB    = 1.35582;      % torque
k.MPS_PER_KPH    = 1/3.6;        % speed
k.MPH_PER_MPS    = 2.23694;      % speed

% Tire data is imperial because the TTC rig is imperial; cornering stiffness
% comes out of the fit in lbf/deg and has to reach the vehicle model in N/rad.
k.LBF_DEG_TO_N_RAD = k.N_PER_LBF * k.DEG_PER_RAD;
end
