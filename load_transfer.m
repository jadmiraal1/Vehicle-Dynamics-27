function LT = load_transfer(p, ay, ax)
% LOAD_TRANSFER  Quasi-static load transfer; ay, ax in [g].
% ay>0 cornering, ax>0 accel, ax<0 braking. VD_physics_reference.md, section 2.
ay = ay * p.g;
ax = ax * p.g;

LT.dW_long      = p.m * ax * p.h_cg / p.L;                  % [N], +ve to rear
LT.dW_lat_total = p.m * ay * p.h_cg / mean([p.t_f p.t_r]);  % [N], both axles

LT.Wf = p.Wf_static - LT.dW_long;   % front axle load [N]
LT.Wr = p.Wr_static + LT.dW_long;   % rear axle load [N]

% Per-axle lateral split by static axle load (roll-stiffness split is mid-tier)
front_frac  = p.Wf_static / (p.m*p.g);
LT.dW_lat_f = LT.dW_lat_total * front_frac;
LT.dW_lat_r = LT.dW_lat_total * (1 - front_frac);
end
