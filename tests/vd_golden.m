function varargout = vd_golden(mode, tol)
% VD_GOLDEN  Every number this toolchain issues, in one committed text file.
%
%   vd_golden()          compare against the baseline; error if anything moved
%   vd_golden('bless')   regenerate the baseline - a deliberate act, see below
%   vd_golden('show')    print the current values, compare nothing
%   vd_golden(mode, tol) relative tolerance (default 1e-6)
%
% ---------------------------------------------------------------------------
% WHAT THIS IS FOR
% ---------------------------------------------------------------------------
% vd_selftest answers "is the physics still wired up correctly?" - it checks
% relationships. This answers a different and blunter question: "did ANY number
% this repo produces change, and by how much?"
%
% It runs every run_* target and every core model probe, flattens the results
% into name/value pairs, and writes them to a TAB-SEPARATED TEXT file under
% tests/refs/. Text, not .mat, and sorted by name, for one reason: when a
% change moves a number, the pull request DIFF SHOWS YOU WHICH ONE AND BY HOW
% MUCH. A binary reference file would just say "reference changed".
%
% That property is the whole point. It is what lets you accept a change quickly
% ("moved nothing") or interrogate it ("moved skidpad by 0.4%, why?") without
% re-deriving anything, and it is what makes it safe to let someone - or
% something - propose changes faster than you can read them line by line.
%
% ---------------------------------------------------------------------------
% BLESSING
% ---------------------------------------------------------------------------
% A failure here is NOT a bug report. It says a number moved. Sometimes that is
% the point of the change. The workflow is:
%
%   1. make your change
%   2. run vd_golden        -> it lists exactly what moved
%   3. READ THE LIST. Every line should be a change you meant to make.
%   4. vd_golden('bless')   -> rewrite the baseline
%   5. commit the code AND tests/refs/golden_<CAR>.tsv in the same commit
%
% Step 3 is the only step that matters. Blessing without reading turns this
% file into a rubber stamp, which is worse than not having it - it will make
% you confident about a change nobody checked.
%
% ---------------------------------------------------------------------------
% WHAT IS NOT IN HERE
% ---------------------------------------------------------------------------
% Nothing that needs TTC_Data. The data is licensed and gitignored, so a CI
% clone does not have it, and a baseline that only some machines can reproduce
% is worse than no baseline. So: no ttc_fit, no pacejka_fit, no camber_fit, no
% aligning_moment. Those are covered by vd_selftest's hash gate when you run it
% locally with the data present. Everything downstream of the tire artifact IS
% covered, because the artifact itself is tracked.

if nargin < 1 || isempty(mode), mode = 'check'; end
if nargin < 2 || isempty(tol),  tol  = 1e-6;    end
mode = lower(mode);
assert(ismember(mode, {'check','bless','show'}), 'vd_golden:badMode', ...
    'mode must be ''check'', ''bless'' or ''show'' (got ''%s'').', mode);

here = fileparts(fileparts(mfilename('fullpath')));
car  = vd_car();
ref  = fullfile(here, 'tests', 'refs', ['golden_' car '.tsv']);

% Targets draw. This one runs EVERY target, so the rendering cost is the single
% largest thing it would otherwise do - and it checks numbers, not pictures.
fig0 = get(0, 'DefaultFigureVisible');
set(0, 'DefaultFigureVisible', 'off');
restore_fig = onCleanup(@() set(0, 'DefaultFigureVisible', fig0));
plots0 = vd_plots(false);
restore_plots = onCleanup(@() vd_plots(plots0));

fprintf('\n================= VD GOLDEN VALUES =================\n');
fprintf('car %s   mode %s   tol %.0e\n', car, mode, tol);

t0 = tic;
[G, broken] = collect(car);
close all;
fprintf('collected %d values in %.1f s\n', size(G,1), toc(t0));

% A target that will not run is a failure in its own right, and it must not be
% able to hide as an "added value" in the diff below - nor be blessed into the
% baseline, which would bake the breakage in.
if ~isempty(broken)
    error('vd_golden:targetFailed', ...
        ['%d target(s) could not run: %s\nFix them before blessing or ' ...
         'comparing - a baseline is meaningless if part of the toolchain is ' ...
         'dead.'], numel(broken), strjoin(broken, ', '));
end

switch mode
    case 'show'
        print_table(G);
        if nargout, varargout{1} = G; end
        return

    case 'bless'
        write_tsv(ref, G, car);
        fprintf('\nBLESSED %s (%d values).\n', shortpath(ref, here), size(G,1));
        fprintf('Commit it in the SAME commit as the code change that moved it.\n');
        if nargout, varargout{1} = G; end
        return
end

% ---- check ---------------------------------------------------------------
if ~exist(ref, 'file')
    error('vd_golden:noBaseline', ...
        ['No baseline at %s.\n' ...
         'This is the first run for car %s. Generate it with:\n\n' ...
         '    vd_golden(''bless'')\n\n' ...
         'then commit tests/refs/. Generate it on a machine with the ' ...
         'Optimization Toolbox -\nwithout it the tire fit takes a different ' ...
         'code path (see the README CI section).'], shortpath(ref, here), car);
end

B = read_tsv(ref);
[moved, gone, added] = compare(B, G, tol);

fprintf('\n%-34s %14s %14s %10s\n', 'value', 'baseline', 'now', 'change');
if isempty(moved) && isempty(gone)
    fprintf('  (nothing moved)\n');
else
    for i = 1:size(moved,1)
        b = moved{i,2};  g = moved{i,3};
        if b == 0, pct = Inf; else, pct = 100*(g/b - 1); end
        fprintf('  %-32s %14.6g %14.6g %9.3f%%\n', moved{i,1}, b, g, pct);
    end
    for i = 1:numel(gone)
        fprintf('  %-32s %14s %14s %10s\n', gone{i}, '(present)', 'GONE', '--');
    end
end
for i = 1:numel(added)
    fprintf('  %-32s %14s %14.6g %10s\n', added{i}, '(new)', ...
            G{strcmp(G(:,1), added{i}), 2}, 'ADDED');
end

nbad = size(moved,1) + numel(gone);
fprintf('---------------------------------------------------\n');
fprintf('%d moved, %d removed, %d added   (of %d baseline values)\n', ...
        size(moved,1), numel(gone), numel(added), size(B,1));

pass = (nbad == 0);
if nargout, varargout{1} = pass; end
if ~pass
    error('vd_golden:drift', ...
        ['%d value(s) moved or disappeared. If every line above is a change ' ...
         'you MEANT to make,\nrun vd_golden(''bless'') and commit the updated ' ...
         'tests/refs/golden_%s.tsv with your code.'], nbad, car);
end
if ~isempty(added)
    fprintf(['NOTE: %d new value(s). They are not failures, but the baseline does ' ...
             'not cover them\nuntil you run vd_golden(''bless'').\n'], numel(added));
end
fprintf('ALL GOLDEN VALUES MATCH\n');
end

% =========================================================================
function [G, broken] = collect(car)
% Every number, in one place. Add probes here as the toolchain grows; a new
% row is free coverage and shows up in the next diff as ADDED.
p = vehicle_params();
R = {};
broken = {};

% --- the car itself -------------------------------------------------------
R = add(R, 'params', p, {'m','a','b','Wf_static','Wr_static','k_rot','Izz', ...
                         'P_max','v_max','Fz_design_lbf','mu_y','mu_x', ...
                         'mu_y_raw','mu_anisotropy','m_sprung','m_unsprung'});
R = addv(R, 'params.mu_coef',  p.mu_coef);
R = addv(R, 'params.Ca_coef',  p.Ca_coef);
R = addv(R, 'params.camber_kD', p.camber_kD);
R = addv(R, 'params.camber_kC', p.camber_kC);
R = addv(R, 'params.camber_kS', p.camber_kS);

% --- tire, across the range the car actually uses -------------------------
for Fz = [40 100 176 250 310]
    R = addv(R, sprintf('tire.mu_of_load.%d', Fz), mu_of_load(p, Fz));
    for g = [-4 -2 0 2 4]
        [fD, fC, dSH] = tire_camber(p, Fz, g);
        tag = sprintf('tire.camber.Fz%d.g%+d', Fz, g);
        R = addv(R, [tag '.fD'], fD);
        R = addv(R, [tag '.fC'], fC);
        R = addv(R, [tag '.dSH'], dSH);
    end
    F = tire_forces(p, Fz, [0 3 6 9 12], 2);
    R = addv(R, sprintf('tire.forces.Fz%d.mu', Fz),   F.mu_y);
    R = addv(R, sprintf('tire.forces.Fz%d.Ca', Fz),   F.Ca_lbf_deg);
    R = addv(R, sprintf('tire.forces.Fz%d.Fy', Fz),   F.Fy_lbf);
    R = addv(R, sprintf('tire.forces.Fz%d.Fy0', Fz),  F.Fy_at_zero_alpha_lbf);
end

% --- limits vs speed ------------------------------------------------------
for v = [0 5 12 20 25]
    R = addv(R, sprintf('limit.ay.%d', v),        ay_limit(p, v));
    R = addv(R, sprintf('limit.ax_accel.%d', v),  ax_limit(p, max(v,1), 'accel'));
    R = addv(R, sprintf('limit.ax_brake.%d', v),  ax_limit(p, max(v,1), 'brake'));
    gg = gg_envelope(p, v);
    R = add(R, sprintf('limit.gg.%d', v), gg, {'ay','ax_accel','ax_brake'});
end
G12 = axle_grip(p, 12);
R = add(R, 'axle_grip.12', G12, {'ay_lim_g','cap_f','cap_r','dem_f','dem_r','W_f','W_r'});
R = add(R, 'axle_grip.12.Fz', G12.Fz, {'fo','fi','ro','ri'});
for ax = [-1.0 -0.5 0 0.5]
    [K, info] = understeer_at(p, ax);
    R = addv(R, sprintf('understeer.K.ax%+.1f', ax), K);
    R = addv(R, sprintf('understeer.Caf.ax%+.1f', ax), info.Ca_f);
    R = addv(R, sprintf('understeer.Car.ax%+.1f', ax), info.Ca_r);
end
R = addv(R, 'corner_speed.0',  corner_speed(p, 0));
R = addv(R, 'corner_speed.9',  corner_speed(p, 1/9.125));

% --- camber wired through the car ----------------------------------------
for g = [0 2 4]
    q = vd_set(p, 'camber_deg', struct('fo',g,'fi',-g,'ro',g,'ri',-g));
    R = addv(R, sprintf('camber.ay.g%d', g), axle_grip(q, 12).ay_lim_g);
end

% --- every target ---------------------------------------------------------
% aligning_moment and params_report are deliberately absent: the first needs
% TTC_Data, the second writes a spreadsheet and computes nothing new.
TARGETS = {'run_load_transfer_targets','run_gg_targets','run_lap_targets', ...
           'run_handling_targets','run_stability_targets','run_balance_targets', ...
           'run_wdist_targets','run_aero_targets','run_energy_strategy', ...
           'run_gear_targets','run_pack_targets','run_camber_targets', ...
           'run_aero_gear_sensitivity'};
for i = 1:numel(TARGETS)
    name = TARGETS{i};
    try
        out = [];
        evalc(sprintf('out = %s(p);', name));
        R = flatten(R, ['target.' name], out);
    catch e
        % A target that cannot run is a failure, but a LOUD one - record it as
        % a value so the diff shows the day it broke instead of a silent gap.
        fprintf(2, '  ! %s errored: %s\n', name, e.message);
        broken{end+1} = name; %#ok<AGROW>
    end
end

G = sortrows(R, 1);
end

% =========================================================================
function R = add(R, prefix, S, fields)
for i = 1:numel(fields)
    if isfield(S, fields{i})
        R = addv(R, [prefix '.' fields{i}], S.(fields{i}));
    end
end
end

function R = addv(R, name, v)
% One value in, one or more rows out. Long vectors are summarised rather than
% expanded: a 300-point lap trace as 300 rows would drown the diff, and its
% min/max/mean move whenever the trace does.
if islogical(v), v = double(v); end
if ~isnumeric(v) || isempty(v), return; end
v = v(:);
if any(~isfinite(v))
    R(end+1,:) = {[name '.n_nonfinite'], nnz(~isfinite(v))}; %#ok<AGROW>
    v = v(isfinite(v));
    if isempty(v), return; end
end
if isscalar(v)
    R(end+1,:) = {name, double(v)};                       %#ok<AGROW>
elseif numel(v) <= 12
    for i = 1:numel(v)
        R(end+1,:) = {sprintf('%s(%d)', name, i), double(v(i))}; %#ok<AGROW>
    end
else
    R(end+1,:) = {[name '.n'],    numel(v)};   %#ok<AGROW>
    R(end+1,:) = {[name '.min'],  min(v)};     %#ok<AGROW>
    R(end+1,:) = {[name '.max'],  max(v)};     %#ok<AGROW>
    R(end+1,:) = {[name '.mean'], mean(v)};    %#ok<AGROW>
end
end

function R = flatten(R, prefix, S)
% Walk a target's out struct. Text, cells and handles are skipped on purpose -
% this file is for NUMBERS. Doc and wiring text is checked by tests/ci_checks.py.
if isnumeric(S) || islogical(S)
    R = addv(R, prefix, S);  return
end
if ~isstruct(S) || numel(S) ~= 1, return; end
f = fieldnames(S);
for i = 1:numel(f)
    R = flatten(R, [prefix '.' f{i}], S.(f{i}));
end
end

% =========================================================================
function write_tsv(ref, G, car)
d = fileparts(ref);
if ~exist(d, 'dir'), mkdir(d); end
fid = fopen(ref, 'w');
assert(fid > 0, 'vd_golden:write', 'cannot write %s', ref);
fprintf(fid, '# golden values for car %s - generated by vd_golden(''bless'')\n', car);
fprintf(fid, '# DO NOT hand-edit. Regenerate and read the diff.\n');
fprintf(fid, '# name\tvalue\n');
for i = 1:size(G,1)
    fprintf(fid, '%s\t%.10g\n', G{i,1}, G{i,2});
end
fclose(fid);
end

function B = read_tsv(ref)
fid = fopen(ref, 'r');
assert(fid > 0, 'vd_golden:read', 'cannot read %s', ref);
B = cell(0,2);
while true
    l = fgetl(fid);
    if ~ischar(l), break; end
    if isempty(l) || l(1) == '#', continue; end
    t = find(l == sprintf('\t'), 1, 'last');
    if isempty(t), continue; end
    B(end+1,:) = {strtrim(l(1:t-1)), str2double(l(t+1:end))}; %#ok<AGROW>
end
fclose(fid);
end

function [moved, gone, added] = compare(B, G, tol)
moved = cell(0,3);  gone = {};  added = {};
gname = G(:,1);
for i = 1:size(B,1)
    k = find(strcmp(gname, B{i,1}), 1);
    if isempty(k), gone{end+1} = B{i,1}; continue; end %#ok<AGROW>
    b = B{i,2};  g = G{k,2};
    if abs(g - b) > tol * max(1, abs(b))
        moved(end+1,:) = {B{i,1}, b, g}; %#ok<AGROW>
    end
end
bname = B(:,1);
for i = 1:size(G,1)
    if ~any(strcmp(bname, G{i,1})), added{end+1} = G{i,1}; end %#ok<AGROW>
end
end

function print_table(G)
fprintf('\n%-46s %16s\n', 'value', 'now');
for i = 1:size(G,1)
    fprintf('%-46s %16.8g\n', G{i,1}, G{i,2});
end
end

function s = shortpath(f, root)
s = strrep(f, [root filesep], '');
end
