function varargout = vd_golden(mode, tol)
% VD_GOLDEN  Every number the toolchain produces, compared with a committed baseline.
%
%   vd_golden()          compare against the baseline; error if anything moved
%   vd_golden('bless')   rewrite the baseline (a deliberate act, see below)
%   vd_golden('show')    print the current values, compare nothing
%   vd_golden(mode, tol) relative tolerance (default 1e-6)
%
% vd_selftest checks relationships ("is the physics wired up correctly?").
% This asks a blunter question: did ANY number change, and by how much? It
% runs every run_* target and a set of model probes, flattens the results to
% name/value pairs and compares them with tests/refs/golden_<CAR>.tsv - a
% sorted text file, so a pull-request diff shows which value moved.
%
% A failure is not a bug report; it says a number moved. Workflow:
%   1. make the change
%   2. vd_golden                -> lists exactly what moved
%   3. read the list: every line should be a change you meant to make
%   4. vd_golden('bless')       -> rewrite the baseline
%   5. commit the code and tests/refs/golden_<CAR>.tsv together
% Blessing without reading the list defeats the purpose.
%
% Not covered: anything that needs TTC_Data (ttc_fit, pacejka_fit,
% camber_fit, aligning_moment). Those are checked by vd_selftest's hash gate
% on a machine with the data. Everything downstream of the committed tire
% artifact is covered.

if nargin < 1 || isempty(mode), mode = 'check'; end
if nargin < 2 || isempty(tol),  tol  = 1e-6;    end
mode = lower(mode);
assert(ismember(mode, {'check','bless','show'}), 'vd_golden:badMode', ...
    'mode must be ''check'', ''bless'' or ''show'' (got ''%s'').', mode);

here = fileparts(fileparts(mfilename('fullpath')));
car  = vd_car();
ref  = fullfile(here, 'tests', 'refs', ['golden_' car '.tsv']);

% Numbers only: switch figures off for the run and restore afterwards.
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

% A target that fails to run is a failure, and must never be blessed in.
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
% Every number, in one place. A new probe shows up in the next diff as ADDED.
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
R = addv(R, 'corner_speed.9',  corner_speed(p, 1/fsae_rules().skidpad_R_m));

% --- camber wired through the car ----------------------------------------
for g = [0 2 4]
    q = vd_set(p, 'camber_deg', struct('fo',g,'fi',-g,'ro',g,'ri',-g));
    R = addv(R, sprintf('camber.ay.g%d', g), axle_grip(q, 12).ay_lim_g);
end

% --- every target ---------------------------------------------------------
% Not listed: aligning_moment (needs TTC_Data) and params_report (writes a
% spreadsheet, computes nothing new).
TARGETS = {'run_load_transfer_targets','run_gg_targets','run_lap_targets', ...
           'run_handling_targets','run_stability_targets','run_balance_targets', ...
           'run_wdist_targets','run_aero_targets','run_energy_strategy', ...
           'run_gear_targets','run_pack_targets','run_camber_targets', ...
           'run_aero_gear_sensitivity','run_cooling_targets'};
for i = 1:numel(TARGETS)
    name = TARGETS{i};
    try
        out = [];
        evalc(sprintf('out = %s(p);', name));
        R = flatten(R, ['target.' name], out);
    catch e
        % Record the failure loudly; the caller refuses to compare or bless.
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
% One value in, one or more rows out. Vectors longer than 12 are summarised
% (n, min, max, mean) so a lap trace does not drown the diff.
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
% Walk a target's out struct; text, cells and handles are skipped.
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
