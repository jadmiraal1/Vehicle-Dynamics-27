function out = params_report(p)
% PARAMS_REPORT  One-page summary of vehicle_params -> console/plots.
%
%   out = params_report()      the active car, from vd_car / cars/config_<CAR>.m
%   out = params_report(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = params_report(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)
here = vd_root();

% Parse the SOURCE for tier banners and provenance comments; values come
% from the evaluated struct so derived numbers are real numbers.
src   = fileread(fullfile(here, 'vehicle_params.m'));
lines = regexp(src, '\r?\n', 'split');

tier = 'input';
meta = containers.Map('KeyType', 'char', 'ValueType', 'any');
order = {};
i = 1;
last_used = 0;   % last line already consumed by a previous param's comments
while i <= numel(lines)
    ln = lines{i};
    if contains(ln, 'Loaded (generated artifact)'), tier = 'loaded';  end
    if contains(ln, '= DERIVED =')
        tier = 'derived';
    end
    tok = regexp(ln, '^\s*p\.([A-Za-z]\w*)\s*=[^;]*;\s*(?:%\s?(.*))?$', ...
                 'tokens', 'once');
    if ~isempty(tok)
        name = tok{1};
        note = '';
        if numel(tok) > 1 && ~isempty(tok{2}), note = strtrim(tok{2}); end
        j = i + 1;                          % continuation comment lines below
        while j <= numel(lines)
            c = regexp(lines{j}, '^\s*%\s?(.*)$', 'tokens', 'once');
            if isempty(c) || ~isempty(regexp(lines{j}, '^\s*%\s*[-=]{3}', 'once'))
                break;
            end
            note = strtrim([note ' ' strtrim(c{1})]);
            j = j + 1;
        end
        if isempty(note)                    % else: comment block directly above
            k = i - 1;  pre = {};
            while k > last_used
                c = regexp(lines{k}, '^\s*%\s?(.*)$', 'tokens', 'once');
                if isempty(c) || ~isempty(regexp(lines{k}, '^\s*%\s*[-=]{3}', 'once'))
                    break;
                end
                pre = [{strtrim(c{1})}, pre]; %#ok<AGROW>
                k = k - 1;
            end
            note = strtrim(strjoin(pre, ' '));
        end
        last_used = j - 1;
        % duplicates (bootstrap/else branches): keep the richer note
        if ~isKey(meta, name)
            order{end+1} = name; %#ok<AGROW>
            meta(name) = struct('tier', tier, 'note', note);
        elseif numel(note) > numel(meta(name).note)
            m = meta(name);  m.note = note;  m.tier = tier;  meta(name) = m;
        end
        i = j;
    else
        i = i + 1;
    end
end

rows = {'parameter', 'value', 'tier', 'provisional', 'source / note'};
for k = 1:numel(order)
    name = order{k};
    if ~isfield(p, name), continue; end
    m = meta(name);
    rows(end+1, :) = {name, fmt(p.(name)), m.tier, ...
                      prov_flag(m.note), m.note}; %#ok<AGROW>
end

fout = fullfile(here, 'organization', 'vehicle_params_report.xlsx');
if exist(fout, 'file'), delete(fout); end   % no stale rows from old runs
writecell(rows, fout, 'Sheet', 'params');

n_prov = sum(strcmp(rows(2:end, 4), 'PROVISIONAL'));
fprintf('params_report: %d parameters written (%d provisional) -> %s\n', ...
        size(rows, 1) - 1, n_prov, fout);
fprintf('This file is GENERATED - edit vehicle_params.m, then re-run.\n');

out = struct('n_params', size(rows, 1) - 1, 'n_provisional', n_prov, ...
             'file', fout);
end

function s = fmt(v)
if ischar(v) || isstring(v)
    s = char(v);
elseif islogical(v)
    if v, s = 'true'; else, s = 'false'; end
elseif isnumeric(v) && isscalar(v)
    s = sprintf('%.6g', v);
elseif isnumeric(v)
    s = mat2str(v, 5);
elseif isa(v, 'function_handle')
    s = func2str(v);
else
    s = class(v);
end
end

function s = prov_flag(note)
if contains(upper(note), 'PROVISIONAL'), s = 'PROVISIONAL'; else, s = ''; end
end
