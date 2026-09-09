function out = run_pack_targets(p)
% RUN_PACK_TARGETS  Accumulator series-count study (#42/#43/#44/#65).
%
% Holds the parallel count - and therefore pack CURRENT - fixed, and sweeps the
% series count. In a 7P architecture that is the only knob: series count moves
% peak power, pack energy and the voltage-governed rev ceiling together, in the
% same proportion. There is no power-vs-energy trade to optimise; the question
% is only how many rows.
%
% METHOD - why this is a solve, not a sweep over packs
% ---------------------------------------------------
% The energy a lap costs barely depends on the pack (only ~2 Wh per series row,
% via cell mass), while the energy BUDGET is exactly linear in series count
% (0.9 x 0.9 x S x P x 10.8 Wh). So the whole answer follows from ONE cap sweep
% plus arithmetic - sampling three packs and reporting the best of them, as the
% first version of this script did, cannot locate a threshold and will quietly
% step over the right answer.
%
% Anchors are run at a few series counts only to capture the small mass effect;
% everything between them is interpolated and solved continuously.
%
% Endurance is the binding event. Two lap counts are deliberately different:
%   feasibility  22 km / lap length  - the physical requirement (rules distance)
%   scoring      p.scenario.benchmark_laps - the basis the 2026 Tmins were set on
%
% Writes plots/pack_targets.png.
%
%   out = run_pack_targets()      the active car, from vd_car / cars/config_<CAR>.m
%   out = run_pack_targets(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = run_pack_targets(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)
p0   = p;
here = vd_root();

S_ANCHOR = [83 87 91];                      % series counts actually simulated
S_GRID   = 83:0.25:91;                      % series counts solved for
CAPS_KW  = [45 40 35 30 28 25 22 20 18 16 14];   % endurance power caps to test
                   % The low end matters: a small pack's budget is met at a LOW
                   % cap, so if the sweep stops at 18 kW the study has to
                   % extrapolate to place 83-84S. Simulate the floor instead.
CELL     = struct('V_nom',3.6, 'V_max',4.2, 'Ah',3.0, 'I_max',30, 'mass_kg',0.0466);
INTERCON = 1.15;   % busbar/holder/segment overhead on cell mass [-] PROVISIONAL - a
                   % GUESS, not a measurement. Small (2 kg across the whole sweep),
                   % so not load-bearing, but replace it with a real segment mass.
RULES_W  = 80e3;   % FSAE tractive-system power cap [W]
R_SKID   = 9.125;  % skidpad path radius [m] - matches run_aero_targets

REGEN = p0.scenario.regen_capture * p0.scenario.regen_rt;   % 0 = friction braking only

[se, ke, ~, ~, prov] = load_track(fullfile(here, 'tracks', 'track_endurance.csv'));
[sx, kx]             = load_track(fullfile(here, 'tracks', 'track_autocross.csv'));
laps_feas  = p0.scenario.endurance_m / se(end);      % laps to cover the rules distance
laps_score = p0.scenario.benchmark_laps;             % laps the 2026 Tmins are quoted on
bm = read_benchmarks(fullfile(here, 'organization', 'comp_benchmarks_2026.csv'));

% ---------------------- simulate at the anchors -------------------------
nA = numel(S_ANCHOR);  nC = numel(CAPS_KW);
E_lap = nan(nA, nC);   t_lap = nan(nA, nC);          % per-lap energy [Wh] and lap time [s]
t_acc = nan(nA,1); t_skid = nan(nA,1); t_ax = nan(nA,1);
V_nom = nan(nA,1); E_pack = nan(nA,1); P_nom = nan(nA,1); P_full = nan(nA,1);
budget = nan(nA,1); dmass = nan(nA,1);

for i = 1:nA
    pa = set_pack(p0, S_ANCHOR(i), CELL, INTERCON);
    V_nom(i)  = pa.V_pack_nom;  E_pack(i) = pa.E_pack_Wh;
    P_nom(i)  = min(RULES_W, pa.V_pack_nom * pa.I_pack_max);
    P_full(i) = min(RULES_W, pa.V_pack_max * pa.I_pack_max);
    budget(i) = pa.scenario.pack_usable_f * pa.scenario.margin * pa.E_pack_Wh;
    dmass(i)  = pa.m - p0.m;

    % Sprint events run at FULL power - the endurance cap does not apply to them.
    t_acc(i)  = accel_time(pa, 75);
    t_skid(i) = 2*pi*R_SKID / corner_speed(pa, 1/R_SKID);
    [~, t_ax(i)] = lap_sim(pa, sx, kx, 0, false);

    for j = 1:nC
        pc = pa;  pc.P_max = min(pa.P_max, CAPS_KW(j)*1e3);
        [~, t_lap(i,j), E] = lap_sim(pc, se, ke, [], true);
        E_lap(i,j) = E.drive_acc_Wh - E.brake_wheel_Wh * REGEN;
    end
end

% -------------------- solve continuously over series count ---------------
nG = numel(S_GRID);
cap = nan(nG,1); lap_s = nan(nG,1); pts = nan(nG,1);
pts_end = nan(nG,1); pts_eff = nan(nG,1); pts_acc = nan(nG,1);
Ecap = nan(nG,1); extrap = false(nG,1);
caps_asc = fliplr(CAPS_KW);

for g = 1:nG
    S = S_GRID(g);
    Eg = interp_row(S_ANCHOR, E_lap, S) * laps_feas;   % event energy vs cap [Wh]
    Tg = interp_row(S_ANCHOR, t_lap, S);               % lap time vs cap [s]
    Eg = fliplr(Eg);  Tg = fliplr(Tg);                 % ascending in cap
    bud = interp1(S_ANCHOR, budget, S, 'linear', 'extrap');

    if bud < Eg(1)      % budget below the cheapest cap tested - extrapolate the local slope
        extrap(g) = true;
        slope = (Eg(2) - Eg(1)) / (caps_asc(2) - caps_asc(1));      % [Wh per kW]
        cap(g)   = caps_asc(1) - (Eg(1) - bud)/slope;
        tslope   = (Tg(1) - Tg(2)) / (caps_asc(2) - caps_asc(1));   % [s per kW removed]
        lap_s(g) = Tg(1) + (caps_asc(1) - cap(g))*tslope;
    else
        cap(g)   = interp1(Eg, caps_asc, bud);
        lap_s(g) = interp1(caps_asc, Tg, cap(g));
    end
    Ecap(g) = bud;

    ev = struct('accel',    interp1(S_ANCHOR, t_acc,  S, 'linear', 'extrap'), ...
                'skidpad',  interp1(S_ANCHOR, t_skid, S, 'linear', 'extrap'), ...
                'autocross',interp1(S_ANCHOR, t_ax,   S, 'linear', 'extrap'), ...
                'endurance_lap', lap_s(g), 'laps', laps_score, ...
                'energy_kWh', bud/laps_feas*laps_score/1000);
    [pts(g), brk] = fsae_points(ev, bm);
    pts_end(g) = brk.endurance;  pts_eff(g) = brk.efficiency;  pts_acc(g) = brk.accel;
end

% ------------------------------- report ---------------------------------
fprintf('\nACCUMULATOR SERIES-COUNT STUDY  (%dP fixed, %s)\n', p0.pack_P, p0.cell_id);
fprintf('%s, %.0f m/lap. Feasibility on %.1f laps (%.0f km); scoring on %d laps.\n', ...
        prov.source_png, se(end), laps_feas, p0.scenario.endurance_m/1000, laps_score);
if REGEN > 0
    fprintf('Regen ACTIVE: %.0f%% capture x %.0f%% round-trip = %.0f%% of braking energy.\n', ...
            100*p0.scenario.regen_capture, 100*p0.scenario.regen_rt, 100*REGEN);
else
    fprintf('NO REGEN (regen_capture = 0) - friction braking only, per the TR27 decision.\n');
end
fprintf('Pack current fixed at %.0f A (%dP x %.0f A/cell) - series count is the only knob.\n', ...
        p0.pack_P*CELL.I_max, p0.pack_P, CELL.I_max);
fprintf('Simulated at %s; everything between is solved, not sampled.\n\n', mat2str(S_ANCHOR));

SHOW = [83 84 86 87 88 89 90 91];
fprintf('%5s %7s %8s %8s %8s %9s %8s %9s %9s %8s\n', ...
        'S','V_nom','E[Wh]','P_nom','budget','cap[kW]','lap[s]','end.pts','eff.pts','TOTAL');
for S = SHOW
    if S < S_GRID(1) || S > S_GRID(end), continue; end
    g = idx(S_GRID, S);
    fprintf('%4dS %7.1f %8.0f %7.1fk %8.0f %9.2f%s %8.2f %9.1f %9.1f %8.1f\n', ...
        S, interp1(S_ANCHOR,V_nom,S,'linear','extrap'), ...
        interp1(S_ANCHOR,E_pack,S,'linear','extrap'), ...
        interp1(S_ANCHOR,P_nom,S,'linear','extrap')/1e3, Ecap(g), ...
        cap(g), tern(extrap(g),'*',' '), lap_s(g), pts_end(g), pts_eff(g), pts(g));
end
if any(extrap), fprintf('  * extrapolated below the %.0f kW sweep floor - add lower caps to confirm\n', min(CAPS_KW)); end

fprintf('\nDECISION EVIDENCE\n');
g84 = idx(S_GRID,84);
fprintf('  * One series row (%d cells, %.2f kg) = %+.2f kW of cap = %+.2f total points.\n', ...
        p0.pack_P, p0.pack_P*CELL.mass_kg*INTERCON, ...
        (cap(idx(S_GRID,91))-cap(g84))/7, (pts(idx(S_GRID,91))-pts(g84))/7);
fprintf('  * Endurance and efficiency move OPPOSITE ways: a bigger pack runs a higher\n');
fprintf('    cap, which burns more energy. Across 84S->91S endurance %+.1f, efficiency %+.1f.\n', ...
        pts_end(idx(S_GRID,91))-pts_end(g84), pts_eff(idx(S_GRID,91))-pts_eff(g84));
fprintf('    Most of the net gain is ACCEL (%+.1f pts), via the voltage-governed rev ceiling.\n', ...
        pts_acc(idx(S_GRID,91))-pts_acc(g84));
fprintf('  * Continuous power is fuse-limited to %.1f kW at 84S and %.1f kW at 91S (%.0f A\n', ...
        p0.I_fuse_main*84*CELL.V_nom/1e3, p0.I_fuse_main*91*CELL.V_nom/1e3, p0.I_fuse_main);
fprintf('    main fuse) - check that against the caps above before buying cells.\n');
fprintf('  * The curve is smooth. There is no cliff and no optimum: the model gives an\n');
fprintf('    exchange rate, and packaging (segment split) decides which count to take.\n');
fprintf(['Caveat: QSS lap sim; the two 0.9 derates (BMS window, design margin) are\n' ...
         '  unexamined CHOICES; interconnect mass is a guess; tire artifact not rebuilt\n' ...
         '  for the small mass change; rev ceiling scaled with pack voltage off the\n' ...
         '  active config''s value (which is itself PROVISIONAL, #47).\n']);

out = struct('S_anchor', S_ANCHOR, 'S', S_GRID, 'caps_kW', CAPS_KW, ...
             'E_lap_Wh', E_lap, 't_lap', t_lap, 't_acc', t_acc, 't_skid', t_skid, ...
             't_autocross', t_ax, 'budget_Wh', budget, 'cap_kW', cap, ...
             'lap_s', lap_s, 'points', pts, 'points_endurance', pts_end, ...
             'points_efficiency', pts_eff, 'points_accel', pts_acc, ...
             'extrapolated', extrap, 'laps_feas', laps_feas, 'laps_score', laps_score, ...
             'regen_fraction', REGEN);

outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
if vd_plots()
try
    pack_plot(S_GRID, cap, pts, outdir);
    fprintf('Plot written: plots/pack_targets.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end
end


function p = set_pack(p, S, CELL, intercon)
% Same car with a different series count. vd_set applies the inputs and calls
% vd_derive, so total mass, axle loads, k_rot, Izz, P_max, v_max and the design
% corner load are all rebuilt together - see util/vd_derive.m.
% Rev ceiling is voltage-governed (VD_physics_reference, "Rev limit is
% voltage-governed"), scaled off the active config's value so the baseline is
% preserved exactly and only the RELATIVE effect of series count is studied.
dS = S - p.pack_S;
p = vd_set(p, ...
    'pack_S',        S, ...
    'V_pack_nom',    S * CELL.V_nom, ...
    'V_pack_max',    S * CELL.V_max, ...
    'I_pack_max',    p.pack_P * CELL.I_max, ...
    'E_pack_Wh',     S * p.pack_P * CELL.V_nom * CELL.Ah, ...
    'rpm_motor_max', p.rpm_motor_max * S / p.pack_S, ...
    'm_car',         p.m_car + dS * p.pack_P * CELL.mass_kg * intercon);
end


function v = interp_row(Sa, M, S)
% Linear interpolation (and extrapolation) of a row of per-cap results in S.
v = nan(1, size(M,2));
for j = 1:size(M,2)
    v(j) = interp1(Sa, M(:,j), S, 'linear', 'extrap');
end
end


function t = accel_time(p, dist)
% Fixed-VELOCITY-step integration, matching run_gg_targets' accel_event - the
% scheme that reproduces the issued 75 m time. NOT the dx-stepped version in
% run_gear_targets, which advances time as dx/v_end and reads ~0.25 s fast.
vg  = linspace(0, p.v_max, 60);
axf = arrayfun(@(vv) ax_limit(p, vv, 'accel'), vg);
v = 0; x = 0; t = 0; dv = 0.005;
while x < dist
    if v >= p.v_max, t = t + (dist - x)/p.v_max; return; end
    a = interp1(vg, axf, min(max(v,0), p.v_max)) * p.g;
    if a <= 0, return; end
    dt = dv/a;  x = x + v*dt + 0.5*a*dt^2;  t = t + dt;  v = v + dv;
end
end


function i = idx(grid, val)
[~, i] = min(abs(grid - val));
end

function v = tern(c, a, b)
if c, v = a; else, v = b; end
end


function pack_plot(S, cap, pts, outdir)
% Endurance power cap and the dynamic points it is worth, both against series
% count. Twin axes because the two measures share an x and nothing else; each
% line carries its own colour AND line style so the pair survives greyscale.
% Points are referenced to the smallest pack on the grid - the decision is the
% difference between counts, not the absolute score.
BLUE = [0.122 0.310 0.847];  ORANGE = [0.761 0.255 0.047];

d  = pts - pts(1);
Si = ceil(min(S)):floor(max(S));
f  = figure('Visible','off','Position',[80 80 760 540],'Color','w');
ax = axes(f); hold(ax,'on');

yyaxis(ax,'left');
h1 = plot(ax, S, cap, '-', 'LineWidth',1.7, 'Color',BLUE);
plot(ax, Si, interp1(S,cap,Si), 'o', 'Color',BLUE, 'MarkerSize',5, ...
     'MarkerFaceColor','w', 'LineWidth',1.3);
ylabel(ax, 'Endurance power cap (kW)');

yyaxis(ax,'right');
h2 = plot(ax, S, d, '--', 'LineWidth',1.7, 'Color',ORANGE);
plot(ax, Si, interp1(S,d,Si), 's', 'Color',ORANGE, 'MarkerSize',5, ...
     'MarkerFaceColor','w', 'LineWidth',1.3);
ylabel(ax, 'Dynamic points gained');

xlim(ax, [min(Si)-0.4, max(Si)+0.4]);  xticks(ax, Si);
xlabel(ax, 'Cells in series');
ax.YAxis(1).Color = [0 0 0];  ax.YAxis(2).Color = [0 0 0];
set(ax, 'FontSize',10, 'Box','on', 'TickDir','in', 'LineWidth',0.8, ...
        'XGrid','on', 'YGrid','on', 'GridLineStyle','--', 'GridAlpha',0.30, ...
        'Layer','bottom');
legend(ax, [h1 h2], {'Endurance power cap','Dynamic points gained'}, ...
       'Location','northwest', 'FontSize',9, 'Box','on');
title(ax, 'Accumulator cell count study', 'FontWeight','normal', 'FontSize',12);

saveas(f, fullfile(outdir, 'pack_targets.png')); close(f);
end

