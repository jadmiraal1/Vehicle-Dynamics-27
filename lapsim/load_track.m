function [s, kappa, x, y, prov] = load_track(fname)
% LOAD_TRACK  Read a digitized track CSV.
%   [s, kappa, x, y, prov] = load_track(fname)
%
% Columns: s_m, x_m, y_m, kappa_1perm. Leading lines starting with '#' are a
% provenance header written by tracks/digitize_track.py; prov returns what
% they say:
%   prov.closed             true for a lap that ends where it starts. Read
%                           from 'closed=True/False'; a file without the
%                           tag is treated as a closed lap.
%   prov.m_per_px           image scale used by the digitizer
%   prov.spacings_verified  slalom cone spacings checked by hand. Defaults
%                           to FALSE when the header does not say.
%   prov.source_png         the scanned course map

fid = fopen(fname, 'r');
if fid < 0
    error('load_track:notFound', 'cannot open %s', fname);
end

nhdr    = 0;
hdrtext = {};
while true
    ln = fgetl(fid);
    if ~ischar(ln)
        fclose(fid);
        error('load_track:noData', 'no data rows in %s', fname);
    end
    t = strtrim(ln);
    is_comment = ~isempty(t) && t(1) == '#';
    is_colhdr  = ~isempty(regexp(t, '^\s*s_m\s*,', 'once'));
    if isempty(t) || is_comment || is_colhdr
        nhdr = nhdr + 1;
        if is_comment
            hdrtext{end+1} = strtrim(t(2:end)); %#ok<AGROW>
        end
    else
        break;
    end
end
fclose(fid);

M = readmatrix(fname, 'NumHeaderLines', nhdr);
s = M(:,1);  x = M(:,2);  y = M(:,3);  kappa = M(:,4);

prov = struct('file', fname, 'header', {hdrtext}, 'closed', true, ...
              'm_per_px', NaN, 'spacings_verified', false, 'source_png', '');
for i = 1:numel(hdrtext)
    h = hdrtext{i};
    tok = regexp(h, '(?<![\w])closed=(\w+)', 'tokens', 'once');
    if ~isempty(tok), prov.closed = strcmpi(tok{1}, 'True'); end
    tok = regexp(h, 'm_per_px=([\d.]+)', 'tokens', 'once');
    if ~isempty(tok), prov.m_per_px = str2double(tok{1}); end
    tok = regexp(h, 'slalom_spacings_verified=(\w+)', 'tokens', 'once');
    if ~isempty(tok), prov.spacings_verified = strcmpi(tok{1}, 'True'); end
    tok = regexp(h, 'digitized from (\S+)', 'tokens', 'once');
    if ~isempty(tok), prov.source_png = tok{1}; end
end
end
