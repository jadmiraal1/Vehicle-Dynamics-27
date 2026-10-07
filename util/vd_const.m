function k = vd_const()
% VD_CONST  Unit conversions. The single home for every conversion factor.
%   k = vd_const();   Fz_lbf = Fz_N / k.N_PER_LBF;
%
% Physical constants that a study may legitimately change (gravity, air
% density) live on the params struct as p.g and p.rho. Conversion factors
% never change, so they live here. Hot loops read the cached copy
% p.N_PER_LBF instead of calling this function (see vd_derive).

k.N_PER_LBF        = 4.4482216152605;   % force (exact by definition)
k.KG_PER_LB        = 0.45359237;        % mass (exact)
k.DEG_PER_RAD      = 180/pi;            % angle
k.MM_PER_IN        = 25.4;              % length (exact)
k.M_PER_IN         = 0.0254;            % length (exact)
k.IN_PER_FT        = 12;                % length (exact)
k.NM_PER_FTLB      = 1.3558179483314;   % torque (exact to 14 s.f.)
k.MPS_PER_KPH      = 1/3.6;             % speed (exact)
k.MPS_PER_MPH      = 0.44704;           % speed (exact)
k.MPH_PER_MPS      = 1/0.44704;         % speed (exact)

% The TTC rig reports cornering stiffness in lbf/deg; the vehicle models
% want N/rad.
k.LBF_DEG_TO_N_RAD = k.N_PER_LBF * k.DEG_PER_RAD;
end
