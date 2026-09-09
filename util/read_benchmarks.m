function bm = read_benchmarks(fname)
% READ_BENCHMARKS  Parse organization/comp_benchmarks_2026.csv into a struct.
%   bm = read_benchmarks(fullfile(vd_root(), 'organization', 'comp_benchmarks_2026.csv'))
%
% WHY THIS FILE EXISTS
% --------------------
% Moved out of run_aero_targets.m when the points model became shared
% (util/fsae_points.m). Two scripts scoring against two separately-parsed
% copies of the same CSV is the same drift risk as two copies of the formulas.
%
% The CSV is the REAL 2026 Michigan results - other teams' times, not ours.
% They are what the FSAE score is measured against, which is why they live in
% version control (the only file kept out of the organization/ gitignore) and
% not in anyone's params. Rows are `metric,value,note`; comment lines start
% with # and the header row starts with `metric`. Non-numeric values (ranges
% like "3.4-5.3") are skipped on purpose - if you need one, add a numeric row.

fid = fopen(fname, 'r');
if fid < 0
    error('read_benchmarks:missing', 'missing %s', fname);
end
cleanup = onCleanup(@() fclose(fid));

bm = struct();
while true
    ln = fgetl(fid);
    if ~ischar(ln), break; end
    ln = strtrim(ln);
    if isempty(ln) || ln(1) == '#' || startsWith(ln, 'metric'), continue; end
    c = strsplit(ln, ',');
    if numel(c) < 2, continue; end
    v = str2double(c{2});
    if ~isnan(v)
        bm.(matlab.lang.makeValidName(c{1})) = v;
    end
end
end
