function ax_g = ax_combined(p, v, ay_g, mode)
% AX_COMBINED  Available longitudinal accel [g] at speed v WHILE cornering at
% ay_g [g], with the measured friction ellipse applied PER AXLE (not per car).
%   ax_g = ax_combined(p, v, ay_g, 'accel' | 'brake')
%
% Each axle carries its moment-balance share of the lateral force plus its own
% longitudinal duty (RWD: rear does all thrust; braking: ideal split). Loads
% include both lateral (LLTD) and longitudinal (h/L) transfer, priced by
% mu_of_load. Infeasibility of an axle's lateral share IS the balance limit
% (front under power = push; rear under braking = trail-brake oversteer).
% At ay_g = 0 this reduces to ax_limit's 'axle' edges. Theory: ref sec 5/11/13.

% Expected out-of-range tire loads are handled by mu_of_load - run quiet.
w1 = warning('off', 'mu_of_load:beyondDonorCoverage');
w2 = warning('off', 'mu_of_load:belowFloor');
restore1 = onCleanup(@() warning(w1.state, w1.identifier)); %#ok<NASGU>
restore2 = onCleanup(@() warning(w2.state, w2.identifier)); %#ok<NASGU>

N_PER_LBF = 4.44822;
v      = max(v, 0);
ay     = max(ay_g, 0) * p.g;                       % [m/s^2]
N      = p.m*p.g + 0.5*p.rho*p.ClA * v^2;
F_drag = 0.5*p.rho*p.CdA * v^2;
F_rr   = p.Crr * N;
F_dl   = (p.b_driveline * v / p.Re + p.Tc_driveline) / p.Re;
F_loss = F_drag + F_rr + F_dl;
m_eff  = p.k_rot * p.m;

% Lateral demand split by moment balance (front share = chi), lateral transfer
% split by LLTD - identical to axle_grip.
Fy_f = p.m * ay * p.mass_dist_f;
Fy_r = p.m * ay * (1 - p.mass_dist_f);
dFf  = p.LLTD       * p.m * ay * p.h_cg / p.t_f;
dFr  = (1 - p.LLTD) * p.m * ay * p.h_cg / p.t_r;

switch mode
case 'accel'
    n_e = expo(p, 'n_env_drive');
    G   = gg_envelope(p, v);
    F_motor = G.ax_motor * p.g * m_eff + F_loss;    % motor thrust available [N]
    lo = 0;  hi = 15;                               % [m/s^2]
    for it = 1:30
        ax = (lo + hi)/2;
        dWl = p.m * ax * p.h_cg / p.L;
        Wf  = p.mass_dist_f*N - dWl;
        Wr  = (1 - p.mass_dist_f)*N + dWl;
        ok = Wf > 0;                                % front still on the ground
        if ok
            capF = axle_cap_lat(p, Wf, dFf, N_PER_LBF);
            capR = axle_cap_lat(p, Wr, dFr, N_PER_LBF);
            ok = Fy_f <= capF;                      % front can still hold the line
        end
        if ok
            rem = 1 - min(Fy_r/max(capR,1e-9), 1)^n_e;
            Fx_avail = p.k_trac * p.mu_anisotropy * capR * max(rem,0)^(1/n_e);
            Fx_need  = m_eff*ax + F_loss;
            ok = Fx_need <= min(Fx_avail, F_motor);
        end
        if ok, lo = ax; else, hi = ax; end
    end
    ax_g = lo / p.g;

case 'brake'
    n_e = expo(p, 'n_env_brake');
    lo = 0;  hi = 30;                               % [m/s^2]
    for it = 1:30
        D   = (lo + hi)/2;
        dWl = p.m * D * p.h_cg / p.L;
        Wf  = p.mass_dist_f*N + dWl;
        Wr  = max((1 - p.mass_dist_f)*N - dWl, 0);
        capF = axle_cap_lat(p, Wf, dFf, N_PER_LBF);
        capR = axle_cap_lat(p, Wr, dFr, N_PER_LBF);
        ok = Fy_f <= capF && Fy_r <= capR;          % both axles hold the line
        if ok
            remF = 1 - min(Fy_f/max(capF,1e-9), 1)^n_e;
            remR = 1 - min(Fy_r/max(capR,1e-9), 1)^n_e;
            Fx_budget = p.mu_anisotropy * (capF*max(remF,0)^(1/n_e) + capR*max(remR,0)^(1/n_e));
            ok = m_eff*D <= Fx_budget + F_loss;     % losses assist braking
        end
        if ok, lo = D; else, hi = D; end
    end
    ax_g = lo / p.g;

otherwise
    error('ax_combined:badMode', 'mode must be ''accel'' or ''brake''.');
end
end


function cap = axle_cap_lat(p, W, dF, N_PER_LBF)
% Axle lateral capacity: outer/inner split by the lateral transfer, per-tire
% mu_of_load. Keep in sync with axle_grip's axle_cap.
F_out = W/2 + dF;
F_in  = W/2 - dF;
if F_in <= 0
    cap = mu_of_load(p, W/N_PER_LBF) * W;           % inner lifted
else
    cap = mu_of_load(p, F_out/N_PER_LBF)*F_out + mu_of_load(p, F_in/N_PER_LBF)*F_in;
end
end


function n = expo(p, f)
% Measured combined-slip ellipse exponent; circle fallback.
if isfield(p, f) && isfinite(p.(f)) && p.(f) > 0, n = p.(f); else, n = 2; end
end
