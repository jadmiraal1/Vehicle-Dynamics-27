function ay_g = ay_limit(p, v)
% AY_LIMIT  Lateral grip limit [g] at speed v; single source for corner_speed/lap_sim.
% p.grip_model: 'axle' (default, load-sensitive) | 'pointmass' (comparison only).

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
