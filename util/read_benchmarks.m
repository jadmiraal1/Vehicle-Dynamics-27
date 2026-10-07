function bm = read_benchmarks(fname)
% READ_BENCHMARKS  Parse organization/comp_benchmarks_2026.csv into a struct.
%   bm = read_benchmarks(fullfile(vd_root(), 'organization', 'comp_benchmarks_2026.csv'))
%
% The CSV holds real competition results (other teams' times) that the
% points model scores against. Rows are `metric,value,notes`; lines starting
% with # and the header row are skipped, and so are non-numeric values
% (e.g. ranges like "3.4-5.3") - add a numeric row if you need one.

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
