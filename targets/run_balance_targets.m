function out = run_balance_targets(p)
% RUN_BALANCE_TARGETS  Mid-tier balance: skidpad w/ load sensitivity, LLTD + CoP bands.
%
%   out = run_balance_targets()      the active car, from vd_car / cars/config_<CAR>.m
%   out = run_balance_targets(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = run_balance_targets(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)

V_SKID_R   = 9.125;          % skidpad radius [m]
LLTD_SWEEP = 0.30:0.01:0.80;
COP_SWEEP  = 0.25:0.01:0.55;
V_LOW      = 12;             % skidpad-ish speed [m/s]
V_HIGH     = 0.95 * p.v_max; % fast-corner speed [m/s]
CLA_TARGET = 4.0;            % aero target (run_aero_targets), for the CoP band
CDA_TARGET = 1.63;
GRIP_COST_MAX = 0.02;        % max ay sacrifice for a stable band [-]
SKID_REAL_BEST = 4.782;      % 2026 best (comp_benchmarks_2026.csv)

% T-SKD2: skidpad with load sensitivity (v and ay converge by fixed point)
v = 10;
for it = 1:40
    G = axle_grip(p, v);
    v = sqrt(G.ay_lim_g * p.g * V_SKID_R);
end
t_skid_mid = 2*pi*V_SKID_R / v;
t_skid_pm  = 2*pi*V_SKID_R / corner_speed(p, 1/V_SKID_R);

fprintf('\nBALANCE TARGETS  (mid-tier axle-grip model; LLTD=%.2f, CoP_f=%.2f)\n', ...
        p.LLTD, p.aero_df_front);
fprintf('T-SKD2 skidpad w/ load sensitivity : %.3f s (%.3f g, %s-limited)\n', ...
        t_skid_mid, G.ay_lim_g, G.limiting);
fprintf('       point-mass %.3f s | real 2026 best %.3f s -- the two models\n', ...
        t_skid_pm, SKID_REAL_BEST);
fprintf('       BRACKET reality: load sensitivity is most of the QSS haircut.\n');

% T-LLTD: grip and limit balance vs roll stiffness split
n  = numel(LLTD_SWEEP);
ay_l = nan(1,n);  front_lim = false(1,n);  lift = false(1,n);
for i = 1:n
    p2 = p;  p2.LLTD = LLTD_SWEEP(i);
    Gi = axle_grip(p2, V_LOW);
    ay_l(i)      = Gi.ay_lim_g;
    front_lim(i) = strcmp(Gi.limiting, 'front');
    lift(i)      = Gi.wheel_lift;
end
[ay_peak, ipk] = max(ay_l);
LLTD_neutral   = LLTD_SWEEP(ipk);
% recommended: smallest front-limited LLTD within the grip-cost budget
cand = find(front_lim & ay_l >= (1-GRIP_COST_MAX)*ay_peak);
LLTD_rec = LLTD_SWEEP(cand(1));
LLTD_hi  = LLTD_SWEEP(cand(end));

fprintf('T-LLTD roll stiffness split : neutral %.2f (ay peaks %.3f g);\n', ...
        LLTD_neutral, ay_peak);
fprintf('       recommend %.2f-%.2f front (front-limited = stable at the limit,\n', ...
        LLTD_rec, LLTD_hi);
fprintf('       <=%.0f%% grip cost). Below neutral the car is rear-limited: snap.\n', ...
        100*GRIP_COST_MAX);
if any(lift)
    fprintf('       inner wheel lift above LLTD %.2f.\n', LLTD_SWEEP(find(lift, 1)));
end

% T-BAL (#38): CoP band at TARGET aero, recommended LLTD, both speeds.
% Stable band = front-limited at V_HIGH; within it, prefer max high-speed ay.
p3 = p;  p3.LLTD = LLTD_rec;  p3.ClA = CLA_TARGET;  p3.CdA = CDA_TARGET;
m2 = numel(COP_SWEEP);
ay_hi = nan(1,m2);  stab = false(1,m2);
for i = 1:m2
    p3.aero_df_front = COP_SWEEP(i);
    Gh = axle_grip(p3, V_HIGH);
    ay_hi(i) = Gh.ay_lim_g;
    stab(i)  = strcmp(Gh.limiting, 'front');
end
band = COP_SWEEP(stab & ay_hi >= (1-GRIP_COST_MAX)*max(ay_hi(stab)));

fprintf('T-BAL  aero CoP band (#38)  : front DF fraction %.2f-%.2f\n', ...
        band(1), band(end));
fprintf('       (at ClA %.1f, LLTD %.2f: front-limited at %.0f m/s, high-speed\n', ...
        CLA_TARGET, LLTD_rec, V_HIGH);
fprintf('       ay %.2f-%.2f g across the band). Rear of the band flips the\n', ...
        min(ay_hi(stab)), max(ay_hi(stab)));
fprintf('       fast-corner limit to the rear axle: high-speed oversteer.\n');
fprintf('Caveat: single-knob LLTD (no roll centers/unsprung split), no camber,\n');
fprintf('        steady state, CoP fixed with speed (real undertray CoP moves\n');
fprintf('        with ride height/pitch -- transient model will see this).\n');

out = struct('t_skid_mid', t_skid_mid, 't_skid_pm', t_skid_pm, ...
             'LLTD_neutral', LLTD_neutral, 'LLTD_rec', LLTD_rec, ...
             'LLTD_hi', LLTD_hi, 'cop_band', [band(1) band(end)], ...
             'lltd_sweep', LLTD_SWEEP, 'ay_lltd', ay_l, 'front_lim', front_lim, ...
             'cop_sweep', COP_SWEEP, 'ay_cop_high', ay_hi, 'cop_stable', stab, ...
             'G_skid', G);

try
    make_plot(p, out, SKID_REAL_BEST, V_LOW, V_HIGH, CLA_TARGET);
    fprintf('Plot written: plots/balance_targets.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end

function make_plot(p, o, t_real, v_low, v_high, cla_t)
f = figure('Visible', 'off', 'Position', [60 60 1240 420], 'Color', 'w');

subplot(1,3,1); hold on;
r = ~o.front_lim;
plot(o.lltd_sweep(r),  o.ay_lltd(r),  'o', 'MarkerSize', 4, 'DisplayName', 'rear-limited (snap)');
plot(o.lltd_sweep(~r), o.ay_lltd(~r), 'o', 'MarkerSize', 4, 'DisplayName', 'front-limited (stable)');
xline(o.LLTD_neutral, ':', 'neutral', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(p.LLTD, '--', 'current', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xlabel('LLTD (front fraction of lateral transfer)'); ylabel('a_y limit [g]');
title(sprintf('T-LLTD @ %.0f m/s', v_low)); legend('Location', 'southwest'); grid on;

subplot(1,3,2); hold on;
vals = [o.t_skid_pm, o.t_skid_mid, t_real];
bar(1:3, vals, 0.5);
set(gca, 'XTick', 1:3, 'XTickLabel', {'point mass', 'mid-tier', 'real best'});
for i = 1:3, text(i, vals(i)+0.05, sprintf('%.2f s', vals(i)), 'HorizontalAlignment', 'center'); end
ylabel('skidpad time [s]'); ylim([4 5.6]);
title('T-SKD2: the models bracket reality'); grid on;

subplot(1,3,3); hold on;
s = o.cop_stable;
plot(o.cop_sweep(s),  o.ay_cop_high(s),  'o', 'MarkerSize', 4, 'DisplayName', 'front-limited (stable)');
plot(o.cop_sweep(~s), o.ay_cop_high(~s), 'o', 'MarkerSize', 4, 'DisplayName', 'rear-limited');
yl = [min(o.ay_cop_high)-0.05, max(o.ay_cop_high)+0.05];
fill([o.cop_band(1) o.cop_band(2) o.cop_band(2) o.cop_band(1)], ...
     [yl(1) yl(1) yl(2) yl(2)], [0.13 0.40 0.67], 'FaceAlpha', 0.08, ...
     'EdgeColor', 'none', 'DisplayName', 'target band #38');
xlabel('front downforce fraction (CoP)'); ylabel(sprintf('a_y limit @ %.0f m/s [g]', v_high));
title(sprintf('T-BAL @ ClA %.1f, LLTD %.2f', cla_t, o.LLTD_rec));
legend('Location', 'southwest'); grid on;

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'balance_targets.png'));
close(f);
end
