function files = tire_src_files()
% TIRE_SRC_FILES  The files the tire artifact is built from. One definition.
%   files = tire_src_files()
%
% vd_hash stamps this list into tire_coeffs_<CAR>.mat at build time and
% vd_selftest re-hashes it to detect a stale artifact. List only files the
% artifact is generated from: build_tire_coeffs -> pacejka_fit -> camber_fit,
% plus the TTC data (hashed by content). ttc_fit is a screening tool the
% build never calls, so it is not listed.
%
% Without TTC_Data/ the list is shorter and the hash cannot be compared;
% vd_selftest then skips the check and says so. The car config is not
% hashed: vd_selftest checks tire identity and design load directly.
%
% tests/ci_staleness.py reads the fullfile(root, 'tire', ...) entries below;
% keep that form.

root = vd_root();
files = {fullfile(root, 'tire', 'pacejka_fit.m'), ...
         fullfile(root, 'tire', 'camber_fit.m'), ...
         fullfile(root, 'tire', 'build_tire_coeffs.m')};

d = dir(fullfile(root, 'TTC_Data', '*.mat'));
for i = 1:numel(d)
    if contains(d(i).name, 'raw'), continue; end     % raw files are not read
    files{end+1} = fullfile(root, 'TTC_Data', d(i).name); %#ok<AGROW>
end
end
