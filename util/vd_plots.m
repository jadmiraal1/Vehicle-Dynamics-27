function was = vd_plots(state)
% VD_PLOTS  Global on/off switch for figure generation.
%   vd_plots()          true if targets should draw (the default)
%   old = vd_plots(false)   turn drawing off, return the previous state
%   vd_plots(old)       put it back
%
% WHY THIS EXISTS
% ---------------
% Every run_* target ends by drawing and saving a figure. That is correct when a
% person runs it and wrong when a test suite does: rendering and writing PNGs is
% most of the wall-clock cost of a test run, it leaves files and windows behind,
% and none of it is what the test is checking. vd_selftest and vd_golden turn
% drawing off for the duration and put it back afterwards.
%
% Use onCleanup so an error mid-suite cannot leave plotting switched off for the
% rest of the session:
%
%     old = vd_plots(false);
%     restore = onCleanup(@() vd_plots(old));
%
% This is a session-level switch, not a car property, so it deliberately does
% NOT live on p - it must not travel through vd_set or end up in an artifact.

persistent enabled
if isempty(enabled), enabled = true; end

if nargin >= 1
    was = enabled;
    enabled = logical(state);
else
    was = enabled;
end
end
