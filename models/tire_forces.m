function F = tire_forces(p, Fz_lbf, alpha_deg, gamma_deg)
% TIRE_FORCES  One tire, pure lateral, with load and camber. Vectorized.
%   F = tire_forces(p, Fz_lbf, alpha_deg)               zero camber
%   F = tire_forces(p, Fz_lbf, alpha_deg, gamma_deg)    with camber
%   F = tire_forces(p, Fz_lbf, [],        gamma_deg)    capability only, no curve
%
% This is the single tire evaluator. Everything that wants to know what one
% tire can do should come through here rather than reaching for mu_of_load and
% p.Ca_coef separately, because those two now have to agree about camber.
%
% RETURNS  (tire boundary units: lbf and deg, per the repo convention)
%   F.mu_y        [-]        peak lateral friction available, DERATED (mu_derate)
%   F.Fy_max_lbf  [lbf]      = mu_y * Fz, the capability the grip solvers want
%   F.Ca_lbf_deg  [lbf/deg]  cornering stiffness, scaled by lambda_Ca
%   F.Fy_lbf      [lbf]      force at the requested slip angle (NaN if none given)
%   F.dSH_deg     [deg]      camber thrust as an equivalent slip angle
%   F.Fy_at_zero_alpha_lbf   [lbf] the camber thrust itself
%   F.fD, F.fC    [-]        the camber factors that were applied
%   F.clamped_*   logical    the request left the fitted box (see below)
%
% INPUTS
%   Fz_lbf     vertical load, positive
%   alpha_deg  slip angle. Positive alpha -> positive Fy.
%   gamma_deg  camber. POSITIVE = leaning so camber thrust ADDS to the force
%              generated at positive alpha. That is the loaded outside wheel of
%              a car in a corner, and it is NEGATIVE camber in the usual SAE
%              chassis convention. models/tire_camber.m has the full story.
%
% Theory: references/VD_physics_reference.md sec 8 (curve), 8b (camber), 13 (load).
%
% NOT MODELLED HERE (deliberately, and each is a real gap):
%   - longitudinal force and combined slip: still the friction ellipse in
%     ax_combined, which has no camber term
%   - aligning moment: targets/aligning_moment.m, separate and outside the
%     hash gate
%   - inflation pressure: the fit reference pressure is baked in. The TTC data
%     DOES contain a pressure sweep, but pressure is confounded with test order
%     in these files - see VD_physics_reference.md sec 8b
%   - temperature, wear, relaxation length

if nargin < 4 || isempty(gamma_deg), gamma_deg = 0; end
if nargin < 3, alpha_deg = []; end

req = {'Ca_coef', 'lambda_Ca'};
for k = 1:numel(req)
    assert(isfield(p, req{k}), 'tire_forces:missingField', ...
        'p is missing "%s" - was it loaded from the tire artifact?', req{k});
end

% --- capability: peak grip and stiffness ---------------------------------
[fD, fC, dSH, cinfo] = tire_camber(p, Fz_lbf, gamma_deg);

F.mu_y       = mu_of_load(p, Fz_lbf, gamma_deg);      % camber applied inside
F.Fy_max_lbf = F.mu_y .* Fz_lbf;

% Ca(Fz) is a quadratic fitted over the tested loads. It has a maximum, and
% past that maximum it falls - which is a property of the polynomial, not of
% the tire. Clamp at the vertex so a heavy outer tire never reads a stiffness
% that is coming back down the wrong side of a parabola.
Fz_Ca = Fz_lbf;
if numel(p.Ca_coef) == 3 && p.Ca_coef(1) < 0
    Fz_vertex = -p.Ca_coef(2) / (2*p.Ca_coef(1));
    Fz_Ca = min(Fz_Ca, Fz_vertex);
end
F.Ca_lbf_deg = polyval(p.Ca_coef, Fz_Ca) .* p.lambda_Ca .* fC;

F.dSH_deg = dSH;
F.fD = fD;  F.fC = fC;
F.clamped_gamma = cinfo.clamped_gamma;
F.clamped_load  = cinfo.clamped_load;
F.camber_active = cinfo.active;

% --- the curve -----------------------------------------------------------
% B, C and E come from the per-load-bin Magic Formula fits stored in the
% artifact; D is the capability computed above. This is the same construction
% pacejka_fit uses internally (mf_at_load), with the derates applied and the
% camber shift added.
have_curve = isfield(p, 'mf_bin_Fz_lbf') && ~isempty(p.mf_bin_Fz_lbf);
if isempty(alpha_deg)
    F.Fy_lbf = NaN(size(Fz_lbf));
else
    F.Fy_lbf = NaN(size(Fz_lbf + alpha_deg));   % broadcast to the caller's shape
end
F.Fy_at_zero_alpha_lbf = NaN(size(Fz_lbf + gamma_deg));

if have_curve
    Fzc = min(max(Fz_lbf, min(p.mf_bin_Fz_lbf)), max(p.mf_bin_Fz_lbf));
    Cs  = interp1(p.mf_bin_Fz_lbf, p.mf_bin_C, Fzc, 'linear');
    Es  = interp1(p.mf_bin_Fz_lbf, p.mf_bin_E, Fzc, 'linear');
    Ds  = F.Fy_max_lbf;
    Bs  = F.Ca_lbf_deg ./ max(Cs .* Ds, eps);   % B*C*D = slope at alpha = 0
    F.mf = struct('B', Bs, 'C', Cs, 'D', Ds, 'E', Es);
    F.Fy_at_zero_alpha_lbf = mf_eval(Bs, Cs, Ds, Es, dSH);
    if ~isempty(alpha_deg)
        F.Fy_lbf = mf_eval(Bs, Cs, Ds, Es, alpha_deg + dSH);
    end
elseif ~isempty(alpha_deg)
    warning('tire_forces:noCurve', ...
        ['the artifact has no per-bin Magic Formula table (mf_bin_*), so only ' ...
         'mu and Ca are available. Run: build_tire_coeffs']);
end
end

% =========================================================================
function y = mf_eval(B, C, D, E, alpha_deg)
% Magic Formula, pure slip. Same expression as pacejka_fit's local mf().
Bx = B .* alpha_deg;
y  = D .* sin(C .* atan(Bx - E .* (Bx - atan(Bx))));
end
