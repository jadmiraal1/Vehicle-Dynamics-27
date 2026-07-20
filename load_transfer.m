function LT = load_transfer(p, ay, ax)
% LOAD_TRANSFER  Quasi-static load transfer; ay, ax in [g].
% ay>0 cornering, ax>0 accel, ax<0 braking. VD_physics_reference.md, section 2.
%
% Lateral split: by roll-stiffness fraction p.LLTD, per axle track (t_f/t_r) -
% matches axle_grip.m's capacities() exactly, so this and the axle-grip model
% agree on how a given ay distributes load. (Previously split by STATIC axle
% weight fraction over the mean track - a different physical model that only
% happened not to disagree with axle_grip because every caller here used
% ay=0. Fixed so calling this with ay!=0 doesn't silently diverge from what
% the lap sim / axle_grip assume.)
ay = ay * p.g;
ax = ax * p.g;

LT.dW_long  = p.m * ax * p.h_cg / p.L;              % [N], +ve to rear
LT.dW_lat_f = p.LLTD       * p.m * ay * p.h_cg / p.t_f;   % [N]
LT.dW_lat_r = (1 - p.LLTD) * p.m * ay * p.h_cg / p.t_r;   % [N]
LT.dW_lat_total = LT.dW_lat_f + LT.dW_lat_r;        % [N], kept for reporting

LT.Wf = p.Wf_static - LT.dW_long;   % front axle load [N]
LT.Wr = p.Wr_static + LT.dW_long;   % rear axle load [N]
end
