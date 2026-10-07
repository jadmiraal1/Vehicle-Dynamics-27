function vd_setup()
% VD_SETUP  Add the toolchain folders to the MATLAB path (run once per session).
r = fileparts(mfilename('fullpath'));
addpath(r, fullfile(r,'util'), fullfile(r,'cars'), fullfile(r,'tire'), fullfile(r,'models'), ...
        fullfile(r,'lapsim'), fullfile(r,'targets'), fullfile(r,'tests'));
vd_warn('reset');
fprintf('Vehicle dynamics toolchain added to the path. New here? Read README.md, then run vd_selftest.\n');
end
