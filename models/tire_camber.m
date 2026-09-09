function [fD, fC, dSH_deg, info] = tire_camber(p, Fz_lbf, gamma_deg)
% TIRE_CAMBER  The three camber factors, in one place. Vectorized over Fz/gamma.
%   [fD, fC, dSH] = tire_camber(p, Fz_lbf, gamma_deg)
%
%   fD  [-]    multiply PEAK lateral force (or mu) by this
%   fC  [-]    multiply CORNERING STIFFNESS by this
%   dSH [deg]  ADD this to slip angle: camber thrust as an equivalent slip angle
%
% Every consumer of camber goes through this function. Do not re-implement the
% polynomials anywhere else - if the fit form changes, it must change once.
%
% SIGN: gamma > 0 means the tire leans so camber thrust ADDS to the cornering
% force at positive slip angle. For the loaded outside wheel of a car in a
% corner that is what you want, and it corresponds to NEGATIVE camber in the
% usual SAE chassis convention. See tire/camber_fit.m for how this was
% established from the data (it was measured, not assumed).
%
% Coefficients come from the tire artifact (p.camber_*), fitted by
% tire/camber_fit.m. Theory: references/VD_physics_reference.md sec 8b.

% --- an artifact without camber terms must behave exactly as before ------
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
% The camber terms are polynomials fitted over a measured box: |gamma| up to
% 4 deg, load 50-250 lbf. Outside that box a quadratic does whatever it likes.
% CLAMP rather than extrapolate, and say so. The outer tire at the cornering
% limit sits past 300 lbf, well outside the data, and the peak factor's load
% term is a QUADRATIC in dfz - run it out that far and it does something
% arbitrary. Clamped, the worst case is that a heavy tire is treated as if it
% were at 250 lbf, which is a stated approximation rather than a made-up number.
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
% A factor at or below zero is the polynomial failing, not the tire vanishing.
FLOOR = 0.30;
if any(fD(:) < FLOOR) || any(fC(:) < FLOOR)
    warning('tire_camber:floored', ...
        ['camber factor fell below %.2f and was clamped (min fD %.3f, fC %.3f ' ...
         'at |gamma| up to %.1f deg). The camber polynomial has been pushed ' ...
         'past where it means anything.'], FLOOR, min(fD(:)), min(fC(:)), max(abs(g(:))));
end
fD = max(fD, FLOOR);
fC = max(fC, FLOOR);

if nargout > 3
    info = struct('clamped_gamma', clamped_gamma, 'clamped_load', clamped_load, ...
                  'active', true, 'dfz', dfz, 'gamma_used_deg', g);
end
end
