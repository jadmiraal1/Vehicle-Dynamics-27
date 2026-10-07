function was = vd_plots(state)
% VD_PLOTS  Global on/off switch for figure generation.
%   vd_plots()          true if targets should draw (the default)
%   old = vd_plots(false)   turn drawing off, return the previous state
%   vd_plots(old)       put it back
%
% Targets draw and save figures when a person runs them; vd_selftest and
% vd_golden switch drawing off for speed and restore it afterwards:
%
%     old = vd_plots(false);
%     restore = onCleanup(@() vd_plots(old));
%
% A session setting, not a car property, so it does not live on p.

persistent enabled
if isempty(enabled), enabled = true; end

if nargin >= 1
    was = enabled;
    enabled = logical(state);
else
    was = enabled;
end
end
