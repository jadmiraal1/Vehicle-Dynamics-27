function out = run_wdist_targets(p)
% RUN_WDIST_TARGETS  Weight-distribution sweep.
% For each static front mass fraction: skidpad-speed lateral limit (with LLTD
% re-trimmed to balance the car), launch acceleration and braking. Steady
% state favours a rearward split; the rearward LIMIT is transient yaw
% stability, which this model cannot see, so a provisional floor stands in.
%
%   out = run_wdist_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_wdist_targets(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

CHI_SWEEP   = 0.38:0.01:0.52;   % static front mass fraction
V_LOW       = 12;               % skidpad-like speed [m/s]
V_ACC       = 3;                % launch speed for the traction limit [m/s]
LLTD_RANGE  = [0.35 0.72];      % practical roll-stiffness authority [front frac]
GRIP_COST   = 0.02;             % ignore <2% grip differences (flat anyway)
CHI_FLOOR_CONV = 0.44;          % transient-stability floor [-] PROVISIONAL convention -
                                % replace with a transient-sim or track result

n = numel(CHI_SWEEP);
ay = nan(1,n); accel = nan(1,n); brake = nan(1,n);
lltd_neu = nan(1,n); trimmable = false(1,n);
for i = 1:n
    chi = CHI_SWEEP(i);
    p2 = vd_set(p, 'mass_dist_f', chi);      % derived values follow

    lltd_neu(i)  = balance_lltd(p2, V_LOW, LLTD_RANGE);
    trimmable(i) = lltd_neu(i) > LLTD_RANGE(1) && lltd_neu(i) < LLTD_RANGE(2);

    p2.LLTD = min(max(lltd_neu(i), LLTD_RANGE(1)), LLTD_RANGE(2));
    G = axle_grip(p2, V_LOW);   ay(i)    = G.ay_lim_g;
    Gg = gg_envelope(p2, V_ACC); accel(i) = Gg.ax_accel;
    brake(i) = ax_limit(p2, V_LOW, 'brake');
end

% Recommendation: lateral grip is nearly flat in the split, so take the
% rearmost split that (a) LLTD can still balance and (b) sits at or above
% the provisional transient floor.
cand   = CHI_SWEEP(trimmable & CHI_SWEEP >= CHI_FLOOR_CONV - 1e-9);
if isempty(cand)
    rec_lo = NaN;
else
    rec_lo = cand(1);
end

[~, icur] = min(abs(CHI_SWEEP - p.mass_dist_f));

fprintf('\nWeight distribution - %s  (axle model, steady state; LLTD re-tuned to balance each split)\n', p.car);
fprintf('    %-8s %-9s %-9s %-9s %-9s\n', 'front', 'skidpad', 'launch', 'braking', 'LLTD to');
fprintf('    %-8s %-9s %-9s %-9s %-9s\n', 'weight', 'lateral', 'accel', 'decel', 'balance');
fprintf('    %-8s %-9s %-9s %-9s %-9s\n', '[%]', '[g]', '[g]', '[g]', '[-]');
for i = 1:n
    tag = '';
    if ~trimmable(i),                     tag = 'LLTD cannot balance';
    elseif CHI_SWEEP(i) < CHI_FLOOR_CONV, tag = 'below stability floor';
    end
    if i == icur, tag = strtrim([tag '   this car']); end
    fprintf('    %-8.0f %-9.3f %-9.3f %-9.3f %-9.2f %s\n', 100*CHI_SWEEP(i), ay(i), ...
            accel(i), brake(i), lltd_neu(i), tag);
end
fprintf('\n');
if isnan(rec_lo)
    vd_row('Recommended front weight', 'none', ...
           sprintf('no split above %.0f %% balances', 100*CHI_FLOOR_CONV));
else
    in_rec = p.mass_dist_f >= rec_lo - 1e-9 && p.mass_dist_f <= rec_lo + 0.03 + 1e-9;
    if p.mass_dist_f < CHI_FLOOR_CONV - 1e-9, st = 'BELOW FLOOR';
    elseif in_rec,                           st = 'OK';
    else,                                    st = 'OUTSIDE';
    end
    vd_row('Recommended front weight', sprintf('%.0f-%.0f %%', 100*rec_lo, 100*(rec_lo+0.03)), ...
           sprintf('now %.0f %%', 100*p.mass_dist_f), st);
end
vd_row('LLTD to balance this car', sprintf('%.2f', lltd_neu(icur)), ...
       sprintf('range %.2f-%.2f', LLTD_RANGE(1), LLTD_RANGE(2)), ...
       ternary(trimmable(icur), 'OK', 'OUTSIDE'));
fprintf(['  Lateral grip changes only %.1f %% across the sweep, so the split trades launch\n' ...
         '  traction (%+.0f %% at %.0f %% front vs %.0f %%) against stability in transients.\n'], ...
        100*(max(ay)/min(ay)-1), 100*(accel(1)/accel(end)-1), 100*CHI_SWEEP(1), 100*CHI_SWEEP(end));
fprintf(['Assumes: steady-state axle model. Transient yaw stability is not modelled, so\n' ...
         '         the %.0f %% stability floor is a placeholder until a transient model or a\n' ...
         '         track test sets it.\n'], 100*CHI_FLOOR_CONV);

out = struct('chi', CHI_SWEEP, 'ay', ay, 'accel', accel, 'brake', brake, ...
             'lltd_neutral', lltd_neu, 'trimmable', trimmable, ...
             'target_front', [rec_lo rec_lo+0.03], 'chi_floor_conv', CHI_FLOOR_CONV, ...
             'current', p.mass_dist_f);

if vd_plots()
try
    make_plot(p, CHI_SWEEP, ay, accel, brake, lltd_neu, CHI_FLOOR_CONV, LLTD_RANGE);
    fprintf('Saved plots/wdist_targets.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

function Ln = balance_lltd(p, v, rng)
% LLTD that balances the car at its limit (front and rear run out together).
% More front LLTD moves the limit toward the front axle, so bisect on it.
lo = rng(1) - 0.15;  hi = rng(2) + 0.15;
for it = 1:40
    mid = (lo + hi)/2;  p2 = p;  p2.LLTD = mid;
    G = axle_grip(p2, v);
    if strcmp(G.limiting, 'front'), hi = mid; else, lo = mid; end
end
Ln = (lo + hi)/2;
end


function make_plot(p, chi, ay, accel, brake, lltd, floor_conv, rng)
f = figure('Visible','off','Position',[60 60 1180 400],'Color','w');

subplot(1,3,1); hold on;
plot(100*chi, ay, 'o-', 'LineWidth',1.4);
xline(100*p.mass_dist_f, '--', 'this car', 'HandleVisibility','off');
xline(100*floor_conv, ':', 'stability floor', 'HandleVisibility','off');
xlabel('front weight [%]'); ylabel('skidpad lateral limit [g]'); grid on;
title('Lateral grip vs weight split');

subplot(1,3,2); hold on;
plot(100*chi, accel, 'o-', 'DisplayName','launch');
plot(100*chi, brake, 's-', 'DisplayName','braking');
xline(100*p.mass_dist_f, '--', 'HandleVisibility','off');
xlabel('front weight [%]'); ylabel('longitudinal limit [g]'); grid on;
legend('Location','best'); title('Launch and braking vs weight split');

subplot(1,3,3); hold on;
plot(100*chi, lltd, 'o-', 'LineWidth',1.4);
yline(rng(1), ':', 'HandleVisibility','off'); yline(rng(2), ':', 'HandleVisibility','off');
xline(100*floor_conv, ':', 'stability floor', 'HandleVisibility','off');
xlabel('front weight [%]'); ylabel('LLTD that balances the car');
ylim([0.3 0.8]); grid on; title('LLTD needed (dotted lines: practical range)');

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'wdist_targets.png'));
close(f);
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
