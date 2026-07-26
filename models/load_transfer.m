function LT = load_transfer(p, ay, ax)
% LOAD_TRANSFER  Quasi-static load transfer; ay, ax in [g] (+ax accel).
% Lateral split by p.LLTD per axle track (matches axle_grip); longitudinal by h/L.
ay = ay * p.g;
ax = ax * p.g;

LT.dW_long  = p.m * ax * p.h_cg / p.L;              % [N], +ve to rear
LT.dW_lat_f = p.LLTD       * p.m * ay * p.h_cg / p.t_f;   % [N]
LT.dW_lat_r = (1 - p.LLTD) * p.m * ay * p.h_cg / p.t_r;   % [N]
LT.dW_lat_total = LT.dW_lat_f + LT.dW_lat_r;        % [N], kept for reporting

LT.Wf = p.Wf_static - LT.dW_long;   % front axle load [N]
LT.Wr = p.Wr_static + LT.dW_long;   % rear axle load [N]
end
