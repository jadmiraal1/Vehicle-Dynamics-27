function out = run_lap_targets()
% RUN_LAP_TARGETS  Whole-lap targets from the QSS point-mass lap sim:
% T-ACC2 75 m | T-SKID2 skidpad | T-LAP lap times | T-VMAX | T-MS3 mass.
% Tracks: real CSVs from `python tr26_sim.py tracks`, else the synthetic
% representative loop. Details: VD_physics_reference.md, section 7.

p = vehicle_params();
here = fileparts(mfilename('fullpath'));

fprintf('\nCONCEPT-TIER LAP-SIM TARGETS  (%s, %s, v_max %.1f m/s = %.0f mph)\n', ...
        p.tire_id, p.drive, p.v_max, p.v_max*2.237);

% --- T-ACC2 : 75 m acceleration (open, from rest) ---
s = linspace(0, 75, 751)';
[va, t_acc] = lap_sim(p, s, zeros(size(s)), 0, false);
fprintf('\n[T-ACC2 ] 75 m ACCELERATION : %.2f s  (v_end %.1f m/s, rev-limited)\n', t_acc, va(end));

% --- T-SKID2 : skidpad (steady) ---
R = 9.125;  vsk = corner_speed(p, 1/R);
fprintf('[T-SKID2] SKIDPAD          : %.2f g, %.2f s  (R=%.2f m)\n', vsk^2/(R*p.g), 2*pi*R/vsk, R);

% --- T-VMAX : top speed ---
fprintf('[T-VMAX ] TOP SPEED        : %.1f m/s (%.0f mph) rev limit @ %d rpm\n', ...
        p.v_max, p.v_max*2.237, p.rpm_motor_max);

% --- T-LAP : lap times on available tracks ---
out.laps = struct();
tracks = {'autocross','endurance'};
ran_any = false;  lap_s = [];  lap_k = [];
for i = 1:numel(tracks)
    f = fullfile(here, 'tracks', ['track_' tracks{i} '.csv']);
    if isfile(f)
        [s, k, xt, yt] = load_track(f);
        [vl, tl, E] = lap_sim(p, s, k, [], true);
        save_lap_map(here, tracks{i}, xt, yt, vl, tl);
        fprintf('[T-LAP  ] %-10s lap : %.2f s  (%.0f m, v_avg %.1f, v_max %.1f m/s)\n', ...
                tracks{i}, tl, s(end), s(end)/tl, max(vl));
        out.laps.(tracks{i}) = tl;  ran_any = true;
        if isempty(lap_s), lap_s = s; lap_k = k; end
    end
end
if ~ran_any
    f = fullfile(here, 'tracks', 'track_representative.csv');
    if isfile(f)
        [s, k, xt, yt] = load_track(f);
        [vl, tl, E] = lap_sim(p, s, k, [], true);
        save_lap_map(here, 'representative', xt, yt, vl, tl);
        fprintf('[T-LAP  ] REPRESENTATIVE lap : %.2f s  (%.0f m loop) — digitize real maps to replace\n', tl, s(end));
        out.laps.representative = tl;  lap_s = s;  lap_k = k;
    else
        fprintf('[T-LAP  ] no track CSV found — run the Python digitizer first.\n');
    end
end

% --- T-NRG : energy per lap (drive at accumulator; regen upper bound) ---
if exist('E', 'var')
    laps_22km = 22000 / lap_s(end);
    fprintf('[T-NRG  ] ENERGY/LAP       : %.0f Wh drawn (%.0f Wh at wheel), %.0f Wh braking (regen bound)\n', ...
            E.drive_acc_Wh, E.drive_wheel_Wh, E.brake_wheel_Wh);
    fprintf('          endurance ~22 km : %.1f kWh no-regen (%.0f laps of this layout)\n', ...
            E.drive_acc_Wh*laps_22km/1000, laps_22km);
    out.E_lap = E;
end

% --- T-MS3 : full-lap mass sensitivity ---
if ~isempty(lap_s)
    [~, t0] = lap_sim(p, lap_s, lap_k, [], true);
    p2 = p;  p2.m = p.m + 10;
    p2.k_rot = 1 + (4*p2.I_wheel + p2.I_rotor*p2.gear_ratio^2)/(p2.m*p2.Re^2);
    [~, t1] = lap_sim(p2, lap_s, lap_k, [], true);
    dtdm = (t1 - t0)/10;
    fprintf('[T-MS3  ] MASS SENSITIVITY : %.1f ms/kg over the lap (%.4f s/kg)\n', dtdm*1000, dtdm);
    out.dtdm_lap = dtdm;
end

out.t_acc = t_acc;  out.v_skid_g = vsk^2/(R*p.g);  out.t_skid = 2*pi*R/vsk;  out.v_max = p.v_max;
fprintf('CAVEATS: point mass (no balance/per-wheel transfer), centreline racing\n');
fprintf('line, mu derated %.2f, k_trac/eta provisional. Calibrate vs skidpad in Fall.\n', p.mu_derate);
fprintf('Animate any lap with lap_replay(''track_<name>.csv'').\n\n');
end


function save_lap_map(here, name, x, y, v, t_lap)
% Speed-colored track map: visual check that the right course loaded.
try
    outdir = fullfile(here, 'plots');
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    f = figure('Visible', 'off', 'Position', [100 100 640 520]);
    scatter(x, y, 10, v, 'filled'); hold on;
    plot(x(1), y(1), 'ks', 'MarkerSize', 9, 'LineWidth', 1.5);
    axis equal; grid on;
    cb = colorbar; cb.Label.String = 'speed [m/s]';
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('%s lap: %.2f s (square = start/finish)', name, t_lap));
    saveas(f, fullfile(outdir, ['lap_map_' name '.png'])); close(f);
    fprintf('          map: plots/lap_map_%s.png\n', name);
catch e
    fprintf('          [map skipped: %s]\n', e.message);
end
end
