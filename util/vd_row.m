function vd_row(label, value, note, status)
% VD_ROW  Print one result line in the standard layout used by the run_* scripts.
%   vd_row(label, value)
%   vd_row(label, value, note)
%   vd_row(label, value, note, status)
%
%   label   what the number is, in plain words
%   value   the number with its unit, already formatted (use sprintf)
%   note    optional context, e.g. the current car's value
%   status  optional verdict in capitals, e.g. 'OK', 'TOO HIGH'
%
% Example:
%   vd_row('Max CG height', sprintf('%.3f m', 0.275), 'now 0.279 m', 'TOO HIGH')
%   prints
%     Max CG height                                0.275 m        now 0.279 m       TOO HIGH

if nargin < 3, note = ''; end
if nargin < 4, status = ''; end
line = sprintf('  %-44s %-14s %-17s %s', label, value, note, status);
fprintf('%s\n', deblank(line));
end
