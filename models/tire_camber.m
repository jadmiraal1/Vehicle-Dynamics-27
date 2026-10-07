function [fD, fC, dSH_deg, info] = tire_camber(p, Fz_lbf, gamma_deg)
% TIRE_CAMBER  The three camber factors. Vectorized over Fz and gamma.
%   [fD, fC, dSH_deg, info] = tire_camber(p, Fz_lbf, gamma_deg)
%
%   fD   [-]    multiplies PEAK lateral force (or mu)
%   fC   [-]    multiplies CORNERING STIFFNESS
%   dSH  [deg]  added to slip angle: camber thrust expressed as a slip angle
%
%   fD  = 1 + (kD1 + kD2*dfz + kD3*dfz^2)*gamma + kD4*gamma^2
%   fC  = 1 + kC1*gamma^2
%   dSH = (kS1 + kS2*dfz)*gamma,         dfz = (Fz - Fz_ref)/Fz_ref
%
% SIGN: gamma > 0 means the tire leans so camber thrust ADDS to the force at
% positive slip angle - the loaded outside wheel in a corner, which is
% NEGATIVE camber in the SAE chassis convention. tire/camber_fit.m shows how
% the sign was established from the data.
%
% This is the only implementation of these polynomials; everything that uses
% camber calls it. Coefficients come from the tire artifact (p.camber_*).
% Theory: VD_physics_reference.md sec 8b.

% --- no camber terms in the artifact: every factor is neutral -----------
if ~isfield(p, 'camber_kD') || all(p.camber_kD == 0)
    fD = ones(size(Fz_lbf + gamma_deg));   % broadcast to the caller's shape
    fC = ones(size(fD));
    dSH_deg = zeros(size(fD));
    info = struct('clamped_gamma', false, 'clamped_load', false, 'active', false);
    return
end

g  = gamma_deg;
Fz = Fz_lbf;

% --- guard rails ---------------------------------------------------------
% The polynomials are fitted over a measured box (|gamma| <= gamma_max, load
% Fz_min..Fz_max). Outside it they are clamped, not extrapolated: a tire past
% the box is treated as if it were at the edge. The outer tire at the
% cornering limit is usually past the load edge (info.clamped_load).
gmax = p.camber_gamma_max_deg;
clamped_gamma = any(abs(g(:)) > gmax + 1e-12);
g = max(min(g, gmax), -gmax);

clamped_load = any(Fz(:) < p.camber_Fz_min_lbf) || any(Fz(:) > p.camber_Fz_max_lbf);
Fz_c = max(min(Fz, p.camber_Fz_max_lbf), p.camber_Fz_min_lbf);
dfz  = (Fz_c - p.camber_Fz_ref_lbf) / p.camber_Fz_ref_lbf;

% --- the three factors ---------------------------------------------------
kD = p.camber_kD;  kC = p.camber_kC;  kS = p.camber_kS;

fD = 1 + (kD(1) + kD(2).*dfz + kD(3).*dfz.^2).*g + kD(4).*g.^2;
fC = 1 +  kC(1).*g.^2;
dSH_deg = (kS(1) + kS(2).*dfz).*g;

% --- floors --------------------------------------------------------------
% A factor near zero is the polynomial failing, not the tire.
FLOOR = 0.30;
if any(fD(:) < FLOOR) || any(fC(:) < FLOOR)
    vd_warn('tire_camber:floored', ...
        ['A camber factor fell below %.2f and was held there (camber up to %.1f deg): ' ...
         'the camber fit is being used far outside its data.'], FLOOR, max(abs(g(:))));
end
fD = max(fD, FLOOR);
fC = max(fC, FLOOR);

if nargout > 3
    info = struct('clamped_gamma', clamped_gamma, 'clamped_load', clamped_load, ...
                  'active', true, 'dfz', dfz, 'gamma_used_deg', g);
end
end
