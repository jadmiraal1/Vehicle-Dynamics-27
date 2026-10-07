function out = run_lap_targets(p)
% RUN_LAP_TARGETS  Lap-simulation targets: 75 m, skidpad, top speed, lap times,
% endurance energy and mass sensitivity.
%
%   out = run_lap_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_lap_targets(p)     an explicit params struct, e.g. a "what if?":
%       out = run_lap_targets(vd_set(vehicle_params(), 'm_car', 240, 'ClA', 4.0));
%
% Tracks are read from tracks/track_<name>.csv. A closed track (endurance)
% is simulated as a flying lap; an open one (autocross) from rest at the
% start line. Theory: VD_physics_reference.md sec 7.

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');
here = vd_root();
R    = fsae_rules();
k    = vd_const();

% 75 m acceleration, from rest at the line
[t_acc, v_end] = accel_time(p, R.accel_m);

% Skidpad: steady state, axle model
vsk = corner_speed(p, 1/R.skidpad_R_m);

% Lap time and mass sensitivity per track
out.laps = struct();
out.dtdm = struct();
rows  = {};                                   % printed after the header
saved = {};                                   % plot files written
tracks = {'autocross', 'endurance'};
have = cellfun(@(n) isfile(fullfile(here, 'tracks', ['track_' n '.csv'])), tracks);
if ~any(have), tracks = {'representative'}; end   % synthetic fallback loop
for i = 1:numel(tracks)
    f = fullfile(here, 'tracks', ['track_' tracks{i} '.csv']);
    if ~isfile(f), continue; end
    [s, kap, xt, yt, prov] = load_track(f);
    [vl, tl] = run_track(p, s, kap, prov);
    p2 = vd_set(p, 'm_car', p.m_car + 10);          % +10 kg; derived values follow
    [~, tl2] = run_track(p2, s, kap, prov);
    dtdm = (tl2 - tl) / 10;
    saved{end+1} = save_lap_map(here, tracks{i}, xt, yt, vl, tl); %#ok<AGROW>
    name = [upper(tracks{i}(1)) tracks{i}(2:end)];
    rows(end+1, :) = {sprintf('%s %s time', name, ternary(prov.closed, 'lap', 'run')), ...
                      sprintf('%.2f s', tl), ...
                      sprintf('%.0f m, average %.1f m/s', s(end), s(end)/tl)}; %#ok<AGROW>
    rows(end+1, :) = {sprintf('%s time per added kg', name), ...
                      sprintf('%.1f ms/kg', dtdm*1000), ''}; %#ok<AGROW>
    out.laps.(tracks{i}) = tl;
    out.dtdm.(tracks{i}) = dtdm;
end

% Endurance energy drawn from the pack (braking energy = regen upper bound)
f = fullfile(here, 'tracks', 'track_endurance.csv');
if isfile(f)
    [s, kap] = load_track(f);
    [~, ~, E] = lap_sim(p, s, kap, [], true);
    laps = p.scenario.endurance_m / s(end);
    out.E_lap = E;
    out.E_endurance_kWh = E.drive_acc_Wh*laps/1000;
end

out.t_acc = t_acc;  out.v_skid_g = vsk^2/(R.skidpad_R_m*p.g);
out.t_skid = 2*pi*R.skidpad_R_m/vsk;  out.v_max = p.v_max;

fprintf('\nLap simulation - %s  (%s, %s, top speed %.1f m/s)\n', ...
        p.car, p.tire_id, p.drive, p.v_max);
vd_row(sprintf('%.0f m acceleration time', R.accel_m), sprintf('%.2f s', t_acc), ...
       sprintf('ends at %.1f m/s%s', v_end, ternary(v_end >= p.v_max - 1e-9, ', rev limit', '')));
vd_row('Skidpad lap time', sprintf('%.2f s', out.t_skid), sprintf('%.2f g', out.v_skid_g));
vd_row(sprintf('Top speed (rev limit %d rpm)', p.rpm_motor_max), sprintf('%.1f m/s', p.v_max), ...
       sprintf('%.0f mph', p.v_max*k.MPH_PER_MPS));
for i = 1:size(rows, 1)
    vd_row(rows{i, :});
end
if isempty(fieldnames(out.laps))
    fprintf('  No track files found in tracks/ - run tracks/digitize_track.py first.\n');
end
if isfield(out, 'E_lap')
    vd_row('Energy per endurance lap, from the pack', sprintf('%.0f Wh', E.drive_acc_Wh), ...
           sprintf('%.0f Wh at the wheels', E.drive_wheel_Wh));
    vd_row('Braking energy per lap (regen upper bound)', sprintf('%.0f Wh', E.brake_wheel_Wh));
    vd_row(sprintf('Endurance energy, %.0f km at full power', p.scenario.endurance_m/1000), ...
           sprintf('%.1f kWh', out.E_endurance_kWh), sprintf('%.1f laps, no regen', laps));
end
fprintf(['Assumes: point mass on the track centreline (no yaw dynamics, no racing line)\n' ...
         '         with limits from the axle models; starts from rest at the line.\n' ...
         '         Provisional: tire grip scale %.2f, launch traction and efficiencies.\n' ...
         '         Calibrate against skidpad and acceleration data.\n'], p.mu_derate);
for i = 1:numel(saved)
    if ~isempty(saved{i}), fprintf('%s\n', saved{i}); end
end
fprintf('Animate a lap: lap_replay(''track_endurance.csv'')\n');
end

function [v, t] = run_track(p, s, kap, prov)
% Closed track: flying lap. Open track: from rest at the start line.
if prov.closed
    [v, t] = lap_sim(p, s, kap, [], true);
else
    [v, t] = lap_sim(p, s, kap, 0, false);
end
end

function msg = save_lap_map(here, name, x, y, v, t_lap)
% Speed-coloured track map: a visual check that the right course loaded.
% Returns the line to print ('' when plotting is off).
msg = '';
if ~vd_plots(), return; end
try
    outdir = fullfile(here, 'plots');
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    f = figure('Visible', 'off', 'Position', [100 100 640 520]);
    scatter(x, y, 10, v, 'filled'); hold on;
    plot(x(1), y(1), 'ks', 'MarkerSize', 9, 'LineWidth', 1.5);
    axis equal; grid on;
    cb = colorbar; cb.Label.String = 'speed [m/s]';
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('%s: %.2f s (square = start)', name, t_lap));
    saveas(f, fullfile(outdir, ['lap_map_' name '.png'])); close(f);
    msg = sprintf('Saved plots/lap_map_%s.png', name);
catch e
    msg = sprintf('Plot not saved (%s map): %s', name, e.message);
end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
