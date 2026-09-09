function vd_setup()
% VD_SETUP  Add the toolchain folders to the MATLAB path (run once per session).
r = fileparts(mfilename('fullpath'));
addpath(r, fullfile(r,'util'), fullfile(r,'cars'), fullfile(r,'tire'), fullfile(r,'models'), ...
        fullfile(r,'lapsim'), fullfile(r,'targets'), fullfile(r,'tests'));
fprintf('VD toolchain on path. Entry points: run_* targets, build_tire_coeffs, vd_selftest.\n');
end
