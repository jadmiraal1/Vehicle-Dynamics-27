function out = run_wdist_targets(p)
% RUN_WDIST_TARGETS  Weight-distribution sweep (T-WDIST). Grip ~flat vs split;
% steady-state favours rear; rearward LIMIT needs the transient model (not this).
%
%   out = run_wdist_targets()      the active car, from vd_car / cars/config_<CAR>.m
%   out = run_wdist_targets(p)     an explicit params struct - use vd_set to build a
%                            "what if?" car - no file on disk is touched:
%       p = vehicle_params();
%       out = run_wdist_targets(vd_set(p, 'm_car', 240, 'ClA', 4.0));

if nargin < 1 || isempty(p), p = vehicle_params(); end   % no argument = the active car (vd_car)

CHI_SWEEP   = 0.38:0.01:0.52;   % static front mass fraction
V_LOW       = 12;               % skidpad-ish speed [m/s]
V_ACC       = 3;                % launch speed for the traction limit [m/s]
LLTD_RANGE  = [0.35 0.72];      % practical roll-stiffness authority [front frac]
GRIP_COST   = 0.02;             % ignore <2% grip differences (flat anyway)
CHI_FLOOR_CONV = 0.44;          % PROVISIONAL transient-stability floor (convention;
                                % replace with the transient-sim / track result)

n = numel(CHI_SWEEP);
ay = nan(1,n); accel = nan(1,n); brake = nan(1,n);
lltd_neu = nan(1,n); trimmable = false(1,n);
for i = 1:n
    chi = CHI_SWEEP(i);
    p2 = vd_set(p, 'mass_dist_f', chi);      % a, b, Wf/Wr_static, Izz all follow

    lltd_neu(i)  = balance_lltd(p2, V_LOW, LLTD_RANGE);
    trimmable(i) = lltd_neu(i) > LLTD_RANGE(1) && lltd_neu(i) < LLTD_RANGE(2);

    p2.LLTD = min(max(lltd_neu(i), LLTD_RANGE(1)), LLTD_RANGE(2));
    G = axle_grip(p2, V_LOW);   ay(i)    = G.ay_lim_g;
    Gg = gg_envelope(p2, V_ACC); accel(i) = Gg.ax_accel;
    brake(i) = ax_limit(p2, V_LOW, 'brake');   % models/ax_limit is the ONE brake solver
end

% Recommendation: grip is flat, so the target is packaging-rearmost SUBJECT to
% (a) LLTD able to balance it and (b) the provisional transient floor.
rec_lo = CHI_FLOOR_CONV;
cand   = CHI_SWEEP(trimmable & CHI_SWEEP >= rec_lo);
rec    = cand(1);                          % rearmost trimmable at/above the floor

[~, icur] = min(abs(CHI_SWEEP - p.mass_dist_f));

fprintf('\nWEIGHT-DISTRIBUTION TARGET  (#2, T-WDIST; mid-tier, steady-state)\n');
fprintf('%6s %8s %8s %8s %9s  %s\n','front%','skid_ay','accel_g','brake_g','LLTD_neu','stability');
for i = 1:n
    tag = '';
    if ~trimmable(i),                tag = 'LLTD out of range';
    elseif CHI_SWEEP(i) < CHI_FLOOR_CONV, tag = 'below transient floor (unproven)';
    end
    mark = ''; if i==icur, mark = '  <- current'; end
    fprintf('%6.0f %8.3f %8.3f %8.3f %9.2f  %s%s\n', 100*CHI_SWEEP(i), ay(i), ...
            accel(i), brake(i), lltd_neu(i), tag, mark);
end

fprintf('\nT-WDIST target : %.0f-%.0f%% front (PROVISIONAL)\n', 100*rec_lo, 100*(rec_lo+0.03));
fprintf('  Basis: grip is flat vs split (%.1f%% over the sweep) so weight\n', ...
        100*(max(ay)/min(ay)-1));
fprintf('  distribution is a traction vs transient-stability call, not a grip\n');
fprintf('  one. Steady-state models favour rearward monotonically (accel +%.0f%%\n', ...
        100*(accel(1)/accel(end)-1));
fprintf('  at %.0f%% vs %.0f%% front); the rearward limit is transient yaw\n', ...
        100*CHI_SWEEP(1), 100*CHI_SWEEP(end));
fprintf('  stability, which is not modelled here (roadmap #5 / track data).\n');
fprintf('  Current %.0f%% front is accel-strong but below the provisional\n', ...
        100*p.mass_dist_f);
fprintf('  transient floor (%.0f%%): validate turn-in / trail-brake stability\n', ...
        100*CHI_FLOOR_CONV);
fprintf('  before committing this rearward. LLTD to balance it: %.2f (in range).\n', ...
        lltd_neu(icur));

out = struct('chi', CHI_SWEEP, 'ay', ay, 'accel', accel, 'brake', brake, ...
             'lltd_neutral', lltd_neu, 'trimmable', trimmable, ...
             'target_front', [rec_lo rec_lo+0.03], 'chi_floor_conv', CHI_FLOOR_CONV, ...
             'current', p.mass_dist_f);

try
    make_plot(p, CHI_SWEEP, ay, accel, brake, lltd_neu, CHI_FLOOR_CONV, LLTD_RANGE);
    fprintf('Plot written: plots/wdist_targets.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end

function Ln = balance_lltd(p, v, rng)
% LLTD (front roll-stiffness fraction) that equalizes front/rear limit margin.
% More front LLTD -> pushes the limit toward the front axle. Bisect on it.
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
xline(100*p.mass_dist_f, '--', 'current', 'HandleVisibility','off');
xline(100*floor_conv, ':', 'transient floor', 'HandleVisibility','off');
xlabel('front mass %'); ylabel('skidpad a_y [g]'); grid on;
title('Lateral grip: ~flat vs split');

subplot(1,3,2); hold on;
plot(100*chi, accel, 'o-', 'DisplayName','launch accel [g]');
plot(100*chi, brake, 's-', 'DisplayName','braking [g]');
xline(100*p.mass_dist_f, '--', 'HandleVisibility','off');
xlabel('front mass %'); ylabel('long. limit [g]'); grid on;
legend('Location','best'); title('Accel favours REAR; brake ~flat');

subplot(1,3,3); hold on;
plot(100*chi, lltd, 'o-', 'LineWidth',1.4);
yline(rng(1), ':', 'HandleVisibility','off'); yline(rng(2), ':', 'HandleVisibility','off');
xline(100*floor_conv, ':', 'transient floor', 'HandleVisibility','off');
xlabel('front mass %'); ylabel('LLTD to balance the limit');
ylim([0.3 0.8]); grid on; title('LLTD authority (in band = trimmable)');

outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'wdist_targets.png'));
close(f);
end
