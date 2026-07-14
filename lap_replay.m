function lap_replay(track_csv, speedup)
% LAP_REPLAY  Animate the QSS lap: moving point mass + live g-g usage.
%   lap_replay                            representative track, real time
%   lap_replay('track_autocross.csv')     named track
%   lap_replay('track_autocross.csv', 3)  3x fast-forward
% Left: speed-colored track with the car as a moving dot. Right: the g-g
% envelope at the car's current speed with the instantaneous (ax, ay) point
% - it should ride the boundary in corners and braking zones (limit driving)
% and sit inside it only on power-limited straights.

if nargin < 1 || isempty(track_csv), track_csv = 'track_representative.csv'; end
if nargin < 2, speedup = 1; end

p    = vehicle_params();
here = fileparts(mfilename('fullpath'));
[s, kappa, x, y] = load_track(fullfile(here, 'tracks', track_csv));
[v, t_lap] = lap_sim(p, s, kappa, [], true);

% Per-point time stamps and accelerations
ds   = diff(s);
v_mid = 0.5*(v(1:end-1) + v(2:end));
t    = [0; cumsum(ds ./ max(v_mid, 0.1))];
ax_g = [diff(v.^2) ./ (2*ds); 0] / p.g;        % longitudinal [g]
ay_g = v.^2 .* kappa / p.g;                    % lateral, signed [g]

% Layout
fig = figure('Name', sprintf('lap replay: %s  (%.2f s)', track_csv, t_lap), ...
             'Position', [80 80 1100 480]);

subplot(1,2,1);
scatter(x, y, 8, v, 'filled'); hold on; axis equal; grid on;
cb = colorbar; cb.Label.String = 'speed [m/s]';
h_dot   = plot(x(1), y(1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
h_title = title('');
xlabel('x [m]'); ylabel('y [m]');

subplot(1,2,2); hold on; grid on; axis equal;
theta  = linspace(0, 2*pi, 120);
h_env  = plot(nan, nan, '-', 'LineWidth', 1.4);
h_pt   = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
h_trail = plot(nan, nan, '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 4);
xlabel('a_x [g]  (+accel / -brake)'); ylabel('a_y [g]');
xlim([-2.4 1.6]); ylim([-2.4 2.4]);
title('g-g usage vs envelope at current speed');

% Animate against wall-clock time
i = 1; t0 = tic;
while i < numel(s) && ishandle(fig)
    tw = toc(t0) * speedup;
    while i < numel(s) && t(i) < tw, i = i + 1; end

    G = gg_envelope(p, v(i));
    ay_env = G.ay .* sin(theta);
    ax_env = zeros(size(theta));
    fwd = cos(theta) >= 0;
    ax_env(fwd)  = G.ax_accel .* cos(theta(fwd));
    ax_env(~fwd) = G.ax_brake .* cos(theta(~fwd));

    set(h_env,  'XData', ax_env, 'YData', ay_env);
    set(h_pt,   'XData', ax_g(i), 'YData', ay_g(i));
    set(h_trail,'XData', ax_g(1:i), 'YData', ay_g(1:i));
    set(h_dot,  'XData', x(i), 'YData', y(i));
    set(h_title, 'String', sprintf('t=%5.2f s   v=%4.1f m/s   ax=%+.2f g   ay=%+.2f g', ...
                                   t(i), v(i), ax_g(i), ay_g(i)));
    drawnow limitrate;
end
if ishandle(fig)
    set(h_title, 'String', sprintf('lap complete: %.2f s', t_lap));
end
end
