function [K_deg, info] = understeer_at(p, ax_g)
% UNDERSTEER_AT  Understeer gradient K [deg/g] at longitudinal accel ax_g [g].
%   [K_deg, info] = understeer_at(p, ax_g)     ax_g > 0 accelerating, < 0 braking
%
% Linear bicycle model evaluated at the axle loads after longitudinal load
% transfer: K = Wf/Ca_f - Wr/Ca_r, with each axle's cornering stiffness read
% from the tire at its new load. Valid sub-limit (roughly below 0.4 g lateral)
% and for sign and trend; it cannot see trail-brake rotation, which is a
% friction-circle effect.

LT = load_transfer(p, 0, ax_g);        % ay = 0: pure longitudinal transfer

% A non-positive axle load means the quasi-static model is past wheel lift.
if LT.Wf <= 0 || LT.Wr <= 0
    error('understeer_at:axleLifted', ...
        ['Axle load went non-positive at ax = %+.2f g (Wf = %.0f N, Wr = %.0f N). ' ...
         'Quasi-static longitudinal transfer is past wheel lift here - the linear ' ...
         'model does not apply.'], ax_g, LT.Wf, LT.Wr);
end

k    = vd_const();
Ca_f = axle_Ca(p, LT.Wf, k);
Ca_r = axle_Ca(p, LT.Wr, k);
K_deg = (LT.Wf/Ca_f - LT.Wr/Ca_r) * k.DEG_PER_RAD;

if nargout > 1
    info.Wf = LT.Wf;    info.Wr = LT.Wr;
    info.Ca_f = Ca_f;   info.Ca_r = Ca_r;
    info.Fz_tire_f_lbf = LT.Wf / 2 / k.N_PER_LBF;
    info.Fz_tire_r_lbf = LT.Wr / 2 / k.N_PER_LBF;
    info.compliance_f  = LT.Wf / Ca_f;   % slip angle the front needs per g [rad]
    info.compliance_r  = LT.Wr / Ca_r;
end
end

function Ca_axle = axle_Ca(p, Fz_axle_N, k)
% Axle cornering stiffness [N/rad] at axle load [N]: two tires at half the
% load each, zero camber, through the single tire evaluator.
TF = tire_forces(p, Fz_axle_N / 2 / k.N_PER_LBF, [], 0);
Ca_axle = 2 * TF.Ca_lbf_deg * k.LBF_DEG_TO_N_RAD;
end
