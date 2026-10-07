function out = run_balance_targets(p)
% RUN_BALANCE_TARGETS  Balance targets from the axle model: load-sensitive
% skidpad, the LLTD band and the aero centre-of-pressure band.
%
%   out = run_balance_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_balance_targets(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

V_SKID_R   = fsae_rules().skidpad_R_m;   % skidpad path radius [m]
LLTD_SWEEP = 0.30:0.01:0.80;
COP_SWEEP  = 0.25:0.01:0.55;
V_LOW      = 12;             % skidpad-like speed [m/s]
V_HIGH     = 0.95 * p.v_max; % fast-corner speed [m/s]
% The CoP band is evaluated at the target aero package.
CLA_TARGET = p.scenario.ClA_target;
CDA_TARGET = p.scenario.CdA_target;
GRIP_COST_MAX = 0.02;        % max ay sacrifice for a stable band [-]
bm = read_benchmarks(fullfile(vd_root(), 'organization', 'comp_benchmarks_2026.csv'));
SKID_REAL_BEST = bm.skidpad_tmin_s;   % best real 2026 skidpad time [s]

% Skidpad with load sensitivity. Speed sets downforce and downforce sets the
% limit, so iterate v and ay to a fixed point.
v = 10;
for it = 1:40
    G = axle_grip(p, v);
    v = sqrt(G.ay_lim_g * p.g * V_SKID_R);
end
t_skid_mid = 2*pi*V_SKID_R / v;
p_pm = p;  p_pm.grip_model = 'pointmass';     % same car, constant-mu lateral limit
t_skid_pm  = 2*pi*V_SKID_R / corner_speed(p_pm, 1/V_SKID_R);

% Grip and limit balance vs roll stiffness split (LLTD)
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
% Recommended: front-limited (stable at the limit) LLTDs that cost no more
% than GRIP_COST_MAX of the peak lateral grip.
cand = find(front_lim & ay_l >= (1-GRIP_COST_MAX)*ay_peak);
LLTD_rec = LLTD_SWEEP(cand(1));
LLTD_hi  = LLTD_SWEEP(cand(end));

% Aero centre-of-pressure range at the target aero and the recommended LLTD.
% Stable = front-limited at V_HIGH; within that, keep the highest-grip CoPs.
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

in_lltd = p.LLTD >= LLTD_rec - 1e-9 && p.LLTD <= LLTD_hi + 1e-9;
in_cop  = p.aero_df_front >= band(1) - 1e-9 && p.aero_df_front <= band(end) + 1e-9;

fprintf('\nBalance - %s  (axle model; LLTD %.2f, front downforce %.0f %%)\n', ...
        p.car, p.LLTD, 100*p.aero_df_front);
vd_row('Skidpad lap time, load-sensitive tire', sprintf('%.3f s', t_skid_mid), ...
       sprintf('%.3f g, %s axle saturates first', G.ay_lim_g, G.limiting));
vd_row('Skidpad lap time, constant-grip point mass', sprintf('%.3f s', t_skid_pm), ...
       sprintf('best real 2026: %.3f s', SKID_REAL_BEST));
vd_row(sprintf('LLTD for maximum lateral grip (%.0f m/s)', V_LOW), sprintf('%.2f', LLTD_neutral), ...
       sprintf('%.3f g', ay_peak));
vd_row('Recommended LLTD', sprintf('%.2f-%.2f', LLTD_rec, LLTD_hi), ...
       sprintf('now %.2f', p.LLTD), ternary(in_lltd, 'OK', 'OUTSIDE'));
if any(lift)
    vd_row('Inside wheel lifts above LLTD', sprintf('%.2f', LLTD_SWEEP(find(lift, 1))));
end
vd_row(sprintf('Recommended front downforce share (%.0f m/s)', V_HIGH), ...
       sprintf('%.0f-%.0f %%', 100*band(1), 100*band(end)), ...
       sprintf('now %.0f %%', 100*p.aero_df_front), ternary(in_cop, 'OK', 'OUTSIDE'));
fprintf(['  LLTD is the front share of lateral load transfer. The recommended ranges keep\n' ...
         '  the front axle saturating first (the car pushes rather than spins at the limit)\n' ...
         '  and cost at most %.0f %% of peak grip. The downforce range is for ClA %.1f and\n' ...
         '  LLTD %.2f; further rearward the fast-corner limit moves to the rear axle.\n'], ...
        100*GRIP_COST_MAX, CLA_TARGET, LLTD_rec);
fprintf(['Assumes: LLTD is one input (no roll centres or unsprung split), zero camber,\n' ...
         '         steady state, downforce centre fixed with speed (a real undertray''s\n' ...
         '         moves with ride height and pitch).\n']);

out = struct('t_skid_mid', t_skid_mid, 't_skid_pm', t_skid_pm, ...
             'LLTD_neutral', LLTD_neutral, 'LLTD_rec', LLTD_rec, ...
             'LLTD_hi', LLTD_hi, 'cop_band', [band(1) band(end)], ...
             'lltd_sweep', LLTD_SWEEP, 'ay_lltd', ay_l, 'front_lim', front_lim, ...
             'cop_sweep', COP_SWEEP, 'ay_cop_high', ay_hi, 'cop_stable', stab, ...
             'G_skid', G);

if vd_plots()
try
    make_plot(p, out, SKID_REAL_BEST, V_LOW, V_HIGH, CLA_TARGET);
    fprintf('Saved plots/balance_targets.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

function make_plot(p, o, t_real, v_low, v_high, cla_t)
f = figure('Visible', 'off', 'Position', [60 60 1240 420], 'Color', 'w');

subplot(1,3,1); hold on;
r = ~o.front_lim;
plot(o.lltd_sweep(r),  o.ay_lltd(r),  'o', 'MarkerSize', 4, 'DisplayName', 'rear saturates first (oversteer)');
plot(o.lltd_sweep(~r), o.ay_lltd(~r), 'o', 'MarkerSize', 4, 'DisplayName', 'front saturates first');
xline(o.LLTD_neutral, ':', 'max grip', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xline(p.LLTD, '--', 'this car', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xlabel('LLTD (front share of lateral load transfer)'); ylabel('lateral limit [g]');
title(sprintf('Lateral limit vs LLTD at %.0f m/s', v_low)); legend('Location', 'southwest'); grid on;

subplot(1,3,2); hold on;
vals = [o.t_skid_pm, o.t_skid_mid, t_real];
bar(1:3, vals, 0.5);
set(gca, 'XTick', 1:3, 'XTickLabel', {'point mass', 'axle model', 'best real 2026'});
for i = 1:3, text(i, vals(i)+0.05, sprintf('%.2f s', vals(i)), 'HorizontalAlignment', 'center'); end
ylabel('skidpad time [s]'); ylim([0.9*min(vals) 1.1*max(vals)]);
title('Skidpad time by model'); grid on;

subplot(1,3,3); hold on;
s = o.cop_stable;
plot(o.cop_sweep(s),  o.ay_cop_high(s),  'o', 'MarkerSize', 4, 'DisplayName', 'front saturates first');
plot(o.cop_sweep(~s), o.ay_cop_high(~s), 'o', 'MarkerSize', 4, 'DisplayName', 'rear saturates first');
yl = [min(o.ay_cop_high)-0.05, max(o.ay_cop_high)+0.05];
fill([o.cop_band(1) o.cop_band(2) o.cop_band(2) o.cop_band(1)], ...
     [yl(1) yl(1) yl(2) yl(2)], [0.13 0.40 0.67], 'FaceAlpha', 0.08, ...
     'EdgeColor', 'none', 'DisplayName', 'recommended range');
xlabel('front share of downforce'); ylabel(sprintf('lateral limit at %.0f m/s [g]', v_high));
title(sprintf('Downforce balance at ClA %.1f, LLTD %.2f', cla_t, o.LLTD_rec));
legend('Location', 'southwest'); grid on;

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'balance_targets.png'));
close(f);
end

function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end
