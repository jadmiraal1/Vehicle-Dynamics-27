function ax_g = ax_limit(p, v, mode)
% AX_LIMIT  Straight-line longitudinal limit [g] at speed v.
%   ax_g = ax_limit(p, v, 'accel' | 'brake')
%
% p.long_model selects the tier:
%   'pointmass'  returns gg_envelope's edges (constant mu)
%   'axle'       prices longitudinal load transfer with the load-sensitive
%                tire (mu_of_load) on each axle
%   'combined'   same as 'axle' here: with no lateral demand the two are
%                identical. The friction-ellipse coupling lives in ax_combined.
%
% 'accel' assumes rear-wheel drive and is capped by the motor (gg_envelope).
% 'brake' assumes ideal brake bias (every tire at its own limit), which is
% the upper bound a fixed-bias car can only match at one deceleration.
%
% Theory: VD_physics_reference.md sec 5 and 13.

model = 'axle';
if isfield(p, 'long_model') && ~isempty(p.long_model)
    model = lower(p.long_model);
end
if strcmp(model, 'combined'), model = 'axle'; end

G = gg_envelope(p, v);

if strcmp(model, 'pointmass')
    if strcmp(mode, 'accel'), ax_g = G.ax_accel; else, ax_g = G.ax_brake; end
    return;
elseif ~strcmp(model, 'axle')
    error('ax_limit:badModel', ...
          'p.long_model = ''%s'' not recognized - use ''axle'', ''combined'' or ''pointmass''.', model);
end
if ~strcmpi(p.drive, 'RWD')
    error('ax_limit:drive', ['The axle tier models rear-wheel drive only; ' ...
          'p.drive = ''%s''. Use p.long_model = ''pointmass''.'], p.drive);
end

% Trial loads past the fitted tire range are expected inside the search;
% mu_of_load handles them. Silence its warnings for this call only.
w1 = warning('off', 'mu_of_load:beyondDonorCoverage');
w2 = warning('off', 'mu_of_load:belowFloor');
restore1 = onCleanup(@() warning(w1.state, w1.identifier)); %#ok<NASGU>
restore2 = onCleanup(@() warning(w2.state, w2.identifier)); %#ok<NASGU>

N_PER_LBF = p.N_PER_LBF;
R      = road_loads(p, v);
F_loss = R.F_loss;
m_eff  = R.m_eff;
dW_per_a = p.m * p.h_cg / p.L;                  % axle load transfer per m/s^2

switch mode
case 'accel'
    % Traction at the transferred rear load. The loop a -> rear load -> a is
    % a contraction (gain mu*m*h/(L*m_eff) ~ 0.3), so 12 passes converge to
    % well below 1e-6.
    a = 8;                                      % [m/s^2] starting guess
    for it = 1:12
        Nr   = R.Nr + dW_per_a*a;
        mu_t = p.k_trac * mu_of_load(p, Nr/2/N_PER_LBF) * p.mu_anisotropy;
        a    = max((mu_t*Nr - F_loss) / m_eff, 0);
    end
    ax_g = min(a / p.g, G.ax_motor);

case 'brake'
    % Bisect on deceleration D [m/s^2]: braking capacity minus demand
    % changes sign once.
    cap_ok = @(D) brake_capacity(p, R, dW_per_a*D, N_PER_LBF) + F_loss >= m_eff*D;
    lo = 0;  hi = 30;
    while cap_ok(hi) && hi < 400                % widen if the car can beat the bracket
        lo = hi;  hi = 2*hi;
    end
    for it = 1:30
        D = (lo + hi)/2;
        if cap_ok(D), lo = D; else, hi = D; end
    end
    ax_g = lo / p.g;

otherwise
    error('ax_limit:badMode', 'mode must be ''accel'' or ''brake''.');
end
end

function cap = brake_capacity(p, R, dW, N_PER_LBF)
% Longitudinal force all four tires can make at axle loads shifted by dW.
Wf = R.Nf + dW;
Wr = R.Nr - dW;
if Wr <= 0                                      % rear lifted: front does it all
    W = Wf + max(Wr, 0);
    cap = mu_of_load(p, W/2/N_PER_LBF) * W;
else
    mu2 = mu_of_load(p, [Wf Wr]/2/N_PER_LBF);
    cap = mu2(1)*Wf + mu2(2)*Wr;
end
cap = cap * p.mu_anisotropy;                    % lateral fit -> longitudinal
end
