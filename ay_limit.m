function ay_g = ay_limit(p, v)
% AY_LIMIT  Lateral grip limit [g] at speed v [m/s] - the SINGLE source of the
% cornering ceiling used by corner_speed and lap_sim, so they cannot disagree.
% Which model produces it is set by p.grip_model:
%
%   'axle'      (DEFAULT) load-sensitive per-axle limit (axle_grip). Transferring
%               load onto the outer tire LOSES total grip because mu falls with
%               load (mu_of_load), so ay_lim < mu_y*g and depends on LLTD, track
%               and CoP. This is the realistic mid-tier limit (~0.85-0.95*mu_y).
%   'pointmass' constant-mu g-g envelope (gg_envelope.ay). Ignores load transfer,
%               so it OVER-predicts cornering (ay ~ mu_y*g). Kept ONLY so the old
%               optimistic lap can be reproduced for before/after comparison.
%
% Both include downforce through v. SCALAR v (axle_grip bisects; not vectorized).
% Theory: VD_physics_reference.md, sections 7 (corner speed) & 11 (axle grip).
%
% SCOPE NOTE: this upgrades the LATERAL edge only. lap_sim still takes its
% longitudinal edges (ax_accel, ax_brake) from the point-mass gg_envelope; the
% friction ellipse then couples them against THIS ay limit.

model = 'axle';
if isfield(p, 'grip_model') && ~isempty(p.grip_model)
    model = lower(p.grip_model);
end

switch model
    case 'axle'
        G = axle_grip(p, v);      % load-sensitive lateral saturation
        ay_g = G.ay_lim_g;
    case 'pointmass'
        G = gg_envelope(p, v);    % constant-mu point mass (optimistic)
        ay_g = G.ay;
    otherwise
        error('ay_limit:badModel', ...
              'p.grip_model = ''%s'' not recognized - use ''axle'' or ''pointmass''.', model);
end
end
