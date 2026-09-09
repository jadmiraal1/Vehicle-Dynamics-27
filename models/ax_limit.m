function ax_g = ax_limit(p, v, mode)
% AX_LIMIT  Longitudinal limit [g] at speed v; single source for lap_sim's edges.
%   ax_g = ax_limit(p, v, 'accel' | 'brake')
% p.long_model: 'axle' (default) prices longitudinal load transfer with the
% load-sensitive tire (mu_of_load); 'pointmass' reproduces gg_envelope exactly.
% Mirrors ay_limit / axle_grip. Theory: ref doc sec 5 and 13.

model = 'axle';
if isfield(p, 'long_model') && ~isempty(p.long_model)
    model = lower(p.long_model);
end
if strcmp(model, 'combined'), model = 'axle'; end   % pure edges are identical; the
                                                    % ellipse coupling lives in ax_combined

G = gg_envelope(p, v);                      % motor model + point-mass edges (single source)

if strcmp(model, 'pointmass')
    if strcmp(mode, 'accel'), ax_g = G.ax_accel; else, ax_g = G.ax_brake; end
    return;
elseif ~strcmp(model, 'axle')
    error('ax_limit:badModel', ...
          'p.long_model = ''%s'' not recognized - use ''axle'' or ''pointmass''.', model);
end

% 'axle' probes tire loads past the fitted range by design; mu_of_load handles
% them (donor slope / clamp) - silence its expected warnings for this call only.
w1 = warning('off', 'mu_of_load:beyondDonorCoverage');
w2 = warning('off', 'mu_of_load:belowFloor');
restore1 = onCleanup(@() warning(w1.state, w1.identifier)); %#ok<NASGU>
restore2 = onCleanup(@() warning(w2.state, w2.identifier)); %#ok<NASGU>

N_PER_LBF = 4.44822;
v      = max(v, 0);
N      = p.m*p.g + 0.5*p.rho*p.ClA * v^2;
F_drag = 0.5*p.rho*p.CdA * v^2;
F_rr   = p.Crr * N;
F_dl   = (p.b_driveline * v / p.Re + p.Tc_driveline) / p.Re;
F_loss = F_drag + F_rr + F_dl;
m_eff  = p.k_rot * p.m;

switch mode
case 'accel'
    % RWD traction with mu priced AT the transferred rear load. The transfer
    % feedback (accel -> rear load -> traction) is a contraction (gain ~0.26),
    % so a short fixed-point iteration converges tightly.
    a = 8;                                          % [m/s^2] starting guess
    for it = 1:12
        Nr   = (1 - p.mass_dist_f)*N + p.m*a*p.h_cg/p.L;   % rear axle load
        mu_t = p.k_trac * mu_of_load(p, Nr/2/N_PER_LBF) * p.mu_anisotropy;
        a    = max((mu_t*Nr - F_loss) / m_eff, 0);
    end
    ax_g = min(a / p.g, G.ax_motor);                % motor cap from gg_envelope

case 'brake'
    % All four tires brake at ideal bias; load transfer moves load forward and
    % the concave mu(Fz) taxes the total. Bisect on decel D [m/s^2]:
    % capacity(D) - demand crosses zero once.
    lo = 0;  hi = 30;
    for it = 1:30
        D    = (lo + hi)/2;
        Wf   = p.mass_dist_f*N       + p.m*D*p.h_cg/p.L;
        Wr   = (1 - p.mass_dist_f)*N - p.m*D*p.h_cg/p.L;
        if Wr <= 0                                   % rear lifted: front does it all
            cap = mu_of_load(p, (Wf+max(Wr,0))/2/N_PER_LBF) * (Wf + max(Wr,0));
        else
            mu2 = mu_of_load(p, [Wf Wr]/2/N_PER_LBF);   % both axles, one call
            cap = mu2(1)*Wf + mu2(2)*Wr;
        end
        cap = cap * p.mu_anisotropy;                 % lateral fit -> longitudinal
        if (cap + F_loss) / m_eff >= D, lo = D; else, hi = D; end
    end
    ax_g = lo / p.g;

otherwise
    error('ax_limit:badMode', 'mode must be ''accel'' or ''brake''.');
end
end
