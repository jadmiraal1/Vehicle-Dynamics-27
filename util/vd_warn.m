function vd_warn(id, varargin)
% VD_WARN  Issue a warning at most once per run.
%   vd_warn(id, fmt, arg1, ...)   warn, unless ID was already raised this run
%   vd_warn('reset')              start a new run (each run_* script calls this first)
%
% Functions such as mu_of_load are called thousands of times in one study.
% A plain warning() inside them prints thousands of identical lines and
% buries the results. vd_warn prints the first occurrence as one line (no
% call stack) and stays quiet for the rest of the run.
%
% A warning switched off with warning('off', id) is neither printed nor
% counted, so code that silences a warning for a search loop still gets the
% warning later, when the result is final.

persistent seen
if isempty(seen), seen = {}; end

if strcmp(id, 'reset')
    seen = {};
    return
end
if any(strcmp(seen, id)), return; end          % already said this run
s = warning('query', id);
if strcmp(s.state, 'off'), return; end

seen{end+1} = id;
bt = warning('off', 'backtrace');
restore = onCleanup(@() warning(bt));
warning(id, varargin{:});
end
