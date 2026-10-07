function out = params_report(p)
% PARAMS_REPORT  Every parameter of the active car, with its value, where it
% comes from and its note -> organization/vehicle_params_report_<CAR>.xlsx
%
%   out = params_report()      the active car (vd_car / cars/config_<CAR>.m)
%   out = params_report(p)     an explicit params struct; build "what if?" cars with vd_set
%
% Source column:
%   config     cars/config_<CAR>.m      (measured or decided about the car)
%   universal  vehicle_params.m         (constants, scenario, model switches)
%   artifact   tire_coeffs_<CAR>.mat    (fitted from tire data)
%   derived    util/vd_derive.m         (arithmetic on the above)
% Notes are the end-of-line comments in those files; a note containing
% PROVISIONAL, [verify] or MEASURE is flagged in the 'status' column.
% The spreadsheet is generated - edit the source files, then re-run.

if nargin < 1 || isempty(p), p = vehicle_params(); end
here = vd_root();

src = { ...
    'config',    fullfile(here, 'cars', ['config_' p.car '.m']), 'c';
    'universal', fullfile(here, 'vehicle_params.m'),            'p';
    'derived',   fullfile(here, 'util', 'vd_derive.m'),         'p'};
notes = struct('name', {}, 'source', {}, 'note', {});
for i = 1:size(src, 1)
    notes = [notes, read_notes(src{i,2}, src{i,3}, src{i,1})]; %#ok<AGROW>
end

rows = {'parameter', 'value', 'source', 'status', 'note'};
names = fieldnames(p);
for i = 1:numel(names)
    v = p.(names{i});
    if isstruct(v)                          % one row per sub-field
        sub = fieldnames(v);
        for j = 1:numel(sub)
            nm = [names{i} '.' sub{j}];
            rows(end+1, :) = make_row(nm, v.(sub{j}), notes); %#ok<AGROW>
        end
    else
        rows(end+1, :) = make_row(names{i}, v, notes); %#ok<AGROW>
    end
end

outdir = fullfile(here, 'organization');
if ~exist(outdir, 'dir'), mkdir(outdir); end
fout = fullfile(outdir, ['vehicle_params_report_' p.car '.xlsx']);
if exist(fout, 'file'), delete(fout); end
writecell(rows, fout, 'Sheet', 'params');

n_flag = sum(~cellfun(@isempty, rows(2:end, 4)));
fprintf('Wrote %d parameters (%d marked provisional, to verify or to measure) to\n  %s\n', ...
        size(rows, 1) - 1, n_flag, fout);
out = struct('n_params', size(rows, 1) - 1, 'n_flagged', n_flag, 'file', fout);
end

function r = make_row(name, v, notes)
k = find(strcmp({notes.name}, name), 1, 'last');
if isempty(k)
    source = 'artifact';  note = '';
else
    source = notes(k).source;  note = notes(k).note;
end
r = {name, fmt(v), source, flag(note), note};
end

function N = read_notes(file, prefix, source)
% Parse 'prefix.name = value;  % note' lines plus indented comment lines
% that continue the note. Later definitions of the same name win.
N = struct('name', {}, 'source', {}, 'note', {});
if ~isfile(file), return; end
lines = regexp(fileread(file), '\r?\n', 'split');
for i = 1:numel(lines)
    tok = regexp(lines{i}, ['^\s*' prefix '\.([\w.]+)\s*=([^%]*)(?:%\s?(.*))?$'], 'tokens', 'once');
    if isempty(tok), continue; end
    note = '';
    if numel(tok) > 2, note = strtrim(tok{3}); end
    src_i = source;
    if ~isempty(regexp(tok{2}, '^\s*T\.', 'once')), src_i = 'artifact'; end   % copied from the tire artifact
    j = i + 1;
    while j <= numel(lines) && ~isempty(regexp(lines{j}, '^\s{6,}%', 'once'))
        note = strtrim([note ' ' strtrim(regexprep(lines{j}, '^\s*%\s?', ''))]);
        j = j + 1;
    end
    N(end+1) = struct('name', tok{1}, 'source', src_i, 'note', note); %#ok<AGROW>
end
end

function s = fmt(v)
if ischar(v) || isstring(v)
    s = char(v);
elseif islogical(v) && isscalar(v)
    if v, s = 'true'; else, s = 'false'; end
elseif isnumeric(v) && isscalar(v)
    s = sprintf('%.6g', v);
elseif isnumeric(v) || islogical(v)
    s = mat2str(v, 5);
else
    s = class(v);
end
end

function s = flag(note)
u = upper(note);
if contains(u, 'PROVISIONAL'),  s = 'PROVISIONAL';
elseif contains(u, '[VERIFY]'), s = 'VERIFY';
elseif contains(u, 'MEASURE'),  s = 'MEASURE';
else,                           s = '';
end
end
