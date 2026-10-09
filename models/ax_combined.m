function ax_g = ax_combined(p, v, ay_g, mode)
% AX_COMBINED  Longitudinal limit [g] at speed v while cornering at ay_g [g].
%   ax_g = ax_combined(p, v, ay_g, 'accel' | 'brake')
%
% The measured friction ellipse is applied PER AXLE, not to the whole car:
%   - each axle carries its moment-balance share of the lateral force
%     (front m*ay*b/L, rear m*ay*a/L) and its own longitudinal duty
%     (accel: rear-wheel drive; brake: ideal split);
%   - axle loads include lateral (LLTD) and longitudinal (h/L) transfer, and
%     each tire's grip comes from mu_of_load;
%   - the force left for the longitudinal direction is
%     Fx = mu_x/mu_y * cap * (1 - (Fy/cap)^n)^(1/n), n = measured exponent.
% If an axle cannot hold its lateral share, the car is at its balance limit
% (front under power = push; rear under braking = trail-brake oversteer) and
% the available longitudinal acceleration is zero.
%
% At ay_g = 0 this reduces to ax_limit's 'axle' edges (checked in vd_selftest).

if ~strcmpi(p.drive, 'RWD')
    error('ax_combined:drive', ['The per-axle combined model assumes rear-wheel ' ...
          'drive; p.drive = ''%s''.'], p.drive);
end

w1 = warning('off', 'mu_of_load:beyondDonorCoverage');
w2 = warning('off', 'mu_of_load:belowFloor');
restore1 = onCleanup(@() warning(w1.state, w1.identifier)); %#ok<NASGU>
restore2 = onCleanup(@() warning(w2.state, w2.identifier)); %#ok<NASGU>

N_PER_LBF = p.N_PER_LBF;
ay     = max(ay_g, 0) * p.g;                    % [m/s^2]
R      = road_loads(p, v);
F_loss = R.F_loss;
m_eff  = R.m_eff;
dW_per_a = p.m * p.h_cg / p.L;

% Lateral demand per axle (moment balance) and lateral transfer per axle
% (LLTD split), the same construction as axle_grip.
Fy_f = p.m * ay * p.mass_dist_f;
Fy_r = p.m * ay * (1 - p.mass_dist_f);
dFf  = p.LLTD       * p.m * ay * p.h_cg / p.t_f;
dFr  = (1 - p.LLTD) * p.m * ay * p.h_cg / p.t_r;

switch mode
case 'accel'
    n_e = expo(p, 'n_env_drive');
    G   = gg_envelope(p, v);
    F_motor = G.ax_motor * p.g * m_eff + F_loss;    % motor thrust at the wheel [N]
    ok_at = @(ax) accel_ok(p, R, ax, dW_per_a, dFf, dFr, Fy_f, Fy_r, n_e, ...
                           m_eff, F_loss, F_motor, N_PER_LBF);
    lo = 0;  hi = 15;
    while ok_at(hi) && hi < 200
        lo = hi;  hi = 2*hi;
    end
    for it = 1:30
        ax = (lo + hi)/2;
        if ok_at(ax), lo = ax; else, hi = ax; end
    end
    ax_g = lo / p.g;

case 'brake'
    n_e = expo(p, 'n_env_brake');
    ok_at = @(D) brake_ok(p, R, D, dW_per_a, dFf, dFr, Fy_f, Fy_r, n_e, ...
                          m_eff, F_loss, N_PER_LBF);
    lo = 0;  hi = 30;
    while ok_at(hi) && hi < 400
        lo = hi;  hi = 2*hi;
    end
    for it = 1:30
        D = (lo + hi)/2;
        if ok_at(D), lo = D; else, hi = D; end
    end
    ax_g = lo / p.g;

otherwise
    error('ax_combined:badMode', 'mode must be ''accel'' or ''brake''.');
end
end


function ok = accel_ok(p, R, ax, dW_per_a, dFf, dFr, Fy_f, Fy_r, n_e, m_eff, F_loss, F_motor, N_PER_LBF)
% Can the car accelerate at ax [m/s^2] while holding its lateral load?
dWl = dW_per_a * ax;
Wf  = R.Nf - dWl;
Wr  = R.Nr + dWl;
ok = Wf > 0;                                    % front still on the ground
if ok
    capF = axle_cap_lat(p, Wf, dFf, N_PER_LBF);
    capR = axle_cap_lat(p, Wr, dFr, N_PER_LBF);
    ok = Fy_f <= capF;                          % front can still hold the line
end
if ok
    rem = 1 - min(Fy_r/max(capR,1e-9), 1)^n_e;
    Fx_avail = p.k_trac * p.mu_anisotropy * capR * max(rem,0)^(1/n_e);
    ok = m_eff*ax + F_loss <= min(Fx_avail, F_motor);
end
end


function ok = brake_ok(p, R, D, dW_per_a, dFf, dFr, Fy_f, Fy_r, n_e, m_eff, F_loss, N_PER_LBF)
% Can the car brake at D [m/s^2] while holding its lateral load?
dWl = dW_per_a * D;
Wf  = R.Nf + dWl;
Wr  = max(R.Nr - dWl, 0);
capF = axle_cap_lat(p, Wf, dFf, N_PER_LBF);
capR = axle_cap_lat(p, Wr, dFr, N_PER_LBF);
ok = Fy_f <= capF && Fy_r <= capR;              % both axles hold the line
if ok
    remF = 1 - min(Fy_f/max(capF,1e-9), 1)^n_e;
    remR = 1 - min(Fy_r/max(capR,1e-9), 1)^n_e;
    Fx_budget = p.mu_anisotropy * (capF*max(remF,0)^(1/n_e) + capR*max(remR,0)^(1/n_e));
    ok = m_eff*D <= Fx_budget + F_loss;         % losses help braking
end
end


function cap = axle_cap_lat(p, W, dF, N_PER_LBF)
% Axle lateral capacity: outer/inner tire loads from the lateral transfer,
% grip per tire from mu_of_load. Same split as axle_grip.
F_out = W/2 + dF;
F_in  = W/2 - dF;
if F_in <= 0
    cap = mu_of_load(p, W/N_PER_LBF) * W;       % inner wheel lifted
else
    mu2 = mu_of_load(p, [F_out F_in]/N_PER_LBF);
    cap = mu2(1)*F_out + mu2(2)*F_in;
end
end


function n = expo(p, f)
% Measured friction-ellipse exponent; n = 2 (a circle) if none is loaded.
if isfield(p, f) && isfinite(p.(f)) && p.(f) > 0, n = p.(f); else, n = 2; end
end
