function [K_deg, info] = understeer_at(p, ax_g)
% UNDERSTEER_AT  Understeer gradient K [deg/g] at longitudinal accel ax_g [g].
% Linear bicycle at load-transferred axle loads. Trust sign/trend, not decimals.
% Cannot see trail-brake rotation (friction-circle effect). Theory: ref sec 13.

LT = load_transfer(p, 0, ax_g);        % ay = 0: pure longitudinal transfer

% A non-positive axle load means the quasi-static model has run past wheel lift;
% polyval would then be evaluated at a negative load and come back as nonsense.
if LT.Wf <= 0 || LT.Wr <= 0
    error('understeer_at:axleLifted', ...
        ['Axle load went non-positive at ax = %+.2f g (Wf = %.0f N, Wr = %.0f N). ' ...
         'Quasi-static longitudinal transfer is past wheel lift here - the linear ' ...
         'model does not apply.'], ax_g, LT.Wf, LT.Wr);
end

Ca_f = axle_Ca(p, LT.Wf);      % local function below
Ca_r = axle_Ca(p, LT.Wr);
K_deg = (LT.Wf/Ca_f - LT.Wr/Ca_r) * 180/pi;

if nargout > 1
    N_PER_LBF = 4.44822;
    info.Wf = LT.Wf;    info.Wr = LT.Wr;
    info.Ca_f = Ca_f;   info.Ca_r = Ca_r;
    info.Fz_tire_f_lbf = LT.Wf / 2 / N_PER_LBF;
    info.Fz_tire_r_lbf = LT.Wr / 2 / N_PER_LBF;
    info.compliance_f  = LT.Wf / Ca_f;   % [rad/g-ish] slip the front needs
    info.compliance_r  = LT.Wr / Ca_r;   % ... and the rear
end
end

function Ca_axle = axle_Ca(p, Fz_axle_N)
% Axle cornering stiffness [N/rad] at axle load [N]: artifact Ca(Fz) quadratic,
% x2 tires, lambda_Ca. Symmetric axle (exact for straight-line braking).
N_PER_LBF        = 4.44822;
LBF_DEG_TO_N_RAD = N_PER_LBF * 180/pi;
Fz_tire_lbf = Fz_axle_N / 2 / N_PER_LBF;                 % per-tire load [lbf]
Ca_axle = 2 * polyval(p.Ca_coef, Fz_tire_lbf) * LBF_DEG_TO_N_RAD * p.lambda_Ca;
end
