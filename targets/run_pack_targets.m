function out = run_pack_targets(p)
% RUN_PACK_TARGETS  Accumulator series-count study.
%
% Holds the parallel count (and so the pack current) fixed and varies the
% series count, which moves peak power, pack energy and the voltage-set rev
% ceiling together.
%
% Method: the energy a lap costs barely depends on the pack (only through cell
% mass), while the energy budget is linear in series count. So the endurance
% power cap that fits each pack is SOLVED continuously from one cap sweep,
% rather than read off a few sampled packs. Three anchor packs capture the
% small mass effect; everything between them is interpolated.
%
% Endurance is the binding event. Two lap counts are deliberately different:
%   feasibility  rules distance / lap length (the physical requirement)
%   scoring      p.scenario.benchmark_laps (the basis of the 2026 results)
%
% Writes plots/pack_targets.png.
%
%   out = run_pack_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_pack_targets(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');
p0   = p;
here = vd_root();

S_ANCHOR = [83 87 91];                      % series counts actually simulated
S_GRID   = 83:0.25:91;                      % series counts solved for
CAPS_KW  = [45 40 35 30 28 25 22 20 18 16 14];   % endurance power caps to test
                   % (low enough that the smallest pack is solved, not extrapolated)
CELL     = p.cell; % per-cell values from the car config
INTERCON = 1.15;   % busbar/holder/segment mass overhead on cell mass [-] PROVISIONAL
                   %   (a guess; ~2 kg across the sweep - replace with a segment mass)
R        = fsae_rules();

REGEN = p0.scenario.regen_capture * p0.scenario.regen_rt;   % 0 = friction braking only

[se, ke]             = load_track(fullfile(here, 'tracks', 'track_endurance.csv'));
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
    P_nom(i)  = min(R.P_max_W, pa.V_pack_nom * pa.I_pack_max);
    P_full(i) = min(R.P_max_W, pa.V_pack_max * pa.I_pack_max);
    budget(i) = pa.scenario.pack_usable_f * pa.scenario.margin * pa.E_pack_Wh;
    dmass(i)  = pa.m - p0.m;

    % Sprint events run at full power; the endurance cap does not apply.
    t_acc(i)  = accel_time(pa);
    t_skid(i) = 2*pi*R.skidpad_R_m / corner_speed(pa, 1/R.skidpad_R_m);
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

    if bud < Eg(1)      % budget below the lowest cap tested: extrapolate the local slope
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
                'energy_kWh', bud/laps_feas*laps_score/1000, 'lap_km', se(end)/1000);
    [pts(g), brk] = fsae_points(ev, bm);
    pts_end(g) = brk.endurance;  pts_eff(g) = brk.efficiency;  pts_acc(g) = brk.accel;
end

% ------------------------------- report ---------------------------------
fprintf('\nAccumulator series count - %s  (%dP, %s; pack current %.0f A; %s)\n', ...
        p0.car, p0.pack_P, p0.cell_id, p0.pack_P*CELL.I_max, ...
        tern(REGEN > 0, sprintf('regen %.0f %% of braking energy', 100*REGEN), 'no regen'));
fprintf(['  Endurance needs %.1f laps (%.0f km) of %.0f m; points are scored on %d laps,\n' ...
         '  the basis of the 2026 results. Packs of %s cells in series are\n' ...
         '  simulated; counts in between are interpolated.\n\n'], ...
        laps_feas, p0.scenario.endurance_m/1000, se(end), laps_score, ...
        strjoin(arrayfun(@num2str, S_ANCHOR, 'UniformOutput', false), ', '));

SHOW = [83 84 86 87 88 89 90 91];
fprintf('    %-7s %-8s %-7s %-7s %-8s %-8s %-9s %-10s %-10s %s\n', 'series', 'pack', 'pack', 'pack', ...
        'energy', 'power', 'endur.', 'endurance', 'efficiency', 'total');
fprintf('    %-7s %-8s %-7s %-7s %-8s %-8s %-9s %-10s %-10s %s\n', 'cells', 'voltage', 'energy', 'power', ...
        'budget', 'cap', 'lap', 'points', 'points', 'points');
fprintf('    %-7s %-8s %-7s %-7s %-8s %-8s %-9s\n', '', '[V]', '[Wh]', '[kW]', '[Wh]', '[kW]', '[s]');
for S = SHOW
    if S < S_GRID(1) || S > S_GRID(end), continue; end
    g = idx(S_GRID, S);
    fprintf('    %-7d %-8.1f %-7.0f %-7.1f %-8.0f %-8s %-9.2f %-10.1f %-10.1f %.1f\n', ...
        S, interp1(S_ANCHOR,V_nom,S,'linear','extrap'), ...
        interp1(S_ANCHOR,E_pack,S,'linear','extrap'), ...
        interp1(S_ANCHOR,P_nom,S,'linear','extrap')/1e3, Ecap(g), ...
        [sprintf('%.2f', cap(g)) tern(extrap(g),'*','')], lap_s(g), pts_end(g), pts_eff(g), pts(g));
end
if any(extrap)
    fprintf('  * below the lowest power cap tested (%.0f kW), so extrapolated.\n', min(CAPS_KW));
end

g84 = idx(S_GRID,84);  g91 = idx(S_GRID,91);
fprintf('\n');
vd_row(sprintf('Per extra series row (%d cells, %.2f kg)', p0.pack_P, p0.pack_P*CELL.mass_kg*INTERCON), ...
       sprintf('%+.2f points', (pts(g91)-pts(g84))/7), ...
       sprintf('%+.2f kW cap', (cap(g91)-cap(g84))/7));
vd_row('84 -> 91 cells: endurance points', sprintf('%+.1f', pts_end(g91)-pts_end(g84)));
vd_row('84 -> 91 cells: efficiency points', sprintf('%+.1f', pts_eff(g91)-pts_eff(g84)));
vd_row('84 -> 91 cells: acceleration points', sprintf('%+.1f', pts_acc(g91)-pts_acc(g84)), ...
       'higher voltage, higher rev limit');
vd_row(sprintf('Power at the %.0f A main fuse, 84 / 91 cells', p0.I_fuse_main), ...
       sprintf('%.1f / %.1f kW', p0.I_fuse_main*84*CELL.V_nom/1e3, p0.I_fuse_main*91*CELL.V_nom/1e3));
fprintf(['  A bigger pack runs a higher endurance power cap, which uses more energy, so\n' ...
         '  endurance and efficiency points move in opposite directions. The table gives\n' ...
         '  the exchange rate; packaging decides the count.\n']);
fprintf(['Assumes: the BMS usable window and energy margin are choices; interconnect mass\n' ...
         '         is a guess (%.2f x cell mass); the tire fit is not rebuilt for the small\n' ...
         '         mass change; the rev limit scales with pack voltage from the configured\n' ...
         '         value (provisional).\n'], INTERCON);

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
    fprintf('Saved plots/pack_targets.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end


function p = set_pack(p, S, CELL, intercon)
% Same car with S cells in series. The rev ceiling is set by pack voltage, so
% it scales with S from the config value (the baseline pack reproduces the
% config exactly). vd_set re-derives mass, P_max, v_max and the rest.
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


function i = idx(grid, val)
[~, i] = min(abs(grid - val));
end

function v = tern(c, a, b)
if c, v = a; else, v = b; end
end


function pack_plot(S, cap, pts, outdir)
% Endurance power cap and dynamic points against series count. Points are
% relative to the smallest pack: the decision is the difference between counts.
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

