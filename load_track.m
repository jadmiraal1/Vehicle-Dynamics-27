function [s, kappa, x, y, prov] = load_track(fname)
% LOAD_TRACK  Read a digitized track CSV into (s, kappa[, x, y, prov]).
%
% Columns: s_m, x_m, y_m, kappa_1perm.
% Produce with `python digitize_track.py --all` (in tracks/).
%
% The CSV carries a '#' provenance header written by the digitizer: source
% PNG + md5, the AUTO-CALIBRATED grid scale, course length, and whether the
% slalom cone spacings were verified against a readable map export. This
% function skips those lines and (optionally) returns them, so a run script
% can print exactly which map and which scale produced its lap time.
%
% Header length is DETECTED, not assumed - the previous version hard-coded
% 'NumHeaderLines', 1 and would silently misread any CSV with provenance.

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

% Default spacings_verified to FALSE, not true: a CSV with no provenance
% header (an old file, or a hand-made one) should be treated as UNVERIFIED.
% Fail closed on a data-quality flag, never open.
prov = struct('file', fname, 'header', {hdrtext}, ...
              'm_per_px', NaN, 'spacings_verified', false, 'source_png', '');
for i = 1:numel(hdrtext)
    h = hdrtext{i};
    tok = regexp(h, 'm_per_px=([\d.]+)', 'tokens', 'once');
    if ~isempty(tok), prov.m_per_px = str2double(tok{1}); end
    tok = regexp(h, 'slalom_spacings_verified=(\w+)', 'tokens', 'once');
    if ~isempty(tok), prov.spacings_verified = strcmpi(tok{1}, 'True'); end
    tok = regexp(h, 'digitized from (\S+)', 'tokens', 'once');
    if ~isempty(tok), prov.source_png = tok{1}; end
end
end
