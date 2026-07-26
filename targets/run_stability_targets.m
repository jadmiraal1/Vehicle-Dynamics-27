function out = run_stability_targets()
% RUN_STABILITY_TARGETS  K under longitudinal load transfer + K(chi,ax) map.
% Sign/trend tool. Does not set the rearward mass limit (needs combined-slip model).

p = vehicle_params();

AX  = [-1.00 -0.50 -0.25 0 0.25 0.50];   % longitudinal accel grid [g] (- brake / + power)
CHI = 0.38:0.02:0.54;                    % front mass fraction sweep [-]

K_now  = understeer_at(p, 0);                          % steady-state K at current split
K_ax   = arrayfun(@(a) understeer_at(p, a), AX);       % K vs accel at current split
chi_ss = neutral_chi(p, 0);                            % steady-state neutral front frac

K_map = nan(numel(CHI), numel(AX));
for i = 1:numel(CHI)
    pc = set_chi(p, CHI(i));
    for j = 1:numel(AX)
        K_map(i,j) = understeer_at(pc, AX(j));
    end
end

fprintf('\nSTABILITY / LOAD-TRANSFER TARGETS  (understeer gradient under long. load transfer)\n');
fprintf('Current split %.0f%% front:  K(steady) = %+.3f deg/g  (%s)\n', ...
        100*p.mass_dist_f, K_now, bword(K_now));

fprintf('\nK [deg/g] vs longitudinal accel at %.0f%% front:\n', 100*p.mass_dist_f);
fprintf('   %-9s', 'ax [g] :'); fprintf(' %+7.2f', AX); fprintf('\n');
fprintf('   %-9s', 'K      :'); fprintf(' %+7.3f', K_ax); fprintf('\n');
fprintf('   (braking loads the front -> K up / understeer ; power loads the rear -> K down / oversteer)\n');

fprintf('\nSteady-state neutral front mass fraction : %.1f%%  (K=0 at ax=0)\n', 100*chi_ss);
fprintf('   -> %.0f%% front is %+.3f deg/g; reaching neutral needs %+.1f%% more front mass.\n', ...
        100*p.mass_dist_f, K_now, 100*(chi_ss - p.mass_dist_f));

fprintf('\nK(chi, ax) map [deg/g]  (+ understeer / - oversteer):\n');
fprintf('   %-7s', 'front%'); fprintf(' %+7.2f', AX); fprintf('   <- ax [g]\n');
for i = 1:numel(CHI)
    mark = ''; if abs(CHI(i)-p.mass_dist_f) < 1e-9, mark = ' <- current'; end
    fprintf('   %5.0f%% ', 100*CHI(i)); fprintf(' %+7.3f', K_map(i,:)); fprintf('%s\n', mark);
end

fprintf('\nLIMITS: linear / sub-limit (~0.4 g lateral); K ill-conditioned -> sign and trend only.\n');
fprintf('   This does not set the rearward mass limit: K is negative under power for all\n');
fprintf('   realistic splits (normal RWD, managed by diff/throttle/LLTD, not static mass).\n');
fprintf('   The rearward limit is a friction-circle limit effect -> needs combined-slip\n');
fprintf('   per-axle grip (axle_grip extension), not this model. Weight distribution is a\n');
fprintf('   traction/packaging call (rearward = faster, see run_wdist_targets); use this to\n');
fprintf('   read off the balance cost of a chosen split, not to pick the split.\n');

out = struct('ax', AX, 'chi', CHI, 'K_ax', K_ax, 'K_map', K_map, ...
             'chi_neutral_ss', chi_ss, 'K_static', K_now, 'chi_current', p.mass_dist_f);

try
    make_plot(p, CHI, AX, K_map, chi_ss);
    fprintf('Plot written: plots/stability_targets.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end

function pc = set_chi(p, chi)
% Same car at a different static front mass fraction (rebuild the derived loads
% understeer_at reads, so the sweep is self-consistent).
pc = p;
pc.mass_dist_f = chi;
pc.Wf_static = pc.m * pc.g * chi;
pc.Wr_static = pc.m * pc.g * (1 - chi);
pc.a = pc.L * (1 - chi);
pc.b = pc.L * chi;
end

function chi = neutral_chi(p, ax)
% Front mass fraction giving K=0 at longitudinal accel ax. K rises monotonically
% with front mass, so bisect. Returns the bracket end if no crossing in range.
lo = 0.30; hi = 0.70;
if understeer_at(set_chi(p, lo), ax) > 0, chi = lo; return; end
if understeer_at(set_chi(p, hi), ax) < 0, chi = hi; return; end
for it = 1:60
    mid = 0.5*(lo + hi);
    if understeer_at(set_chi(p, mid), ax) < 0, lo = mid; else, hi = mid; end
end
chi = 0.5*(lo + hi);
end

function s = bword(K)
if     K >  0.1, s = 'understeer';
elseif K < -0.1, s = 'oversteer';
else,            s = 'near neutral';
end
end

function make_plot(p, CHI, AX, K_map, chi_ss)
f = figure('Visible','off','Position',[80 80 760 540],'Color','w');
axh = axes(f); hold(axh,'on'); grid(axh,'on');
cols = parula(numel(AX));
for j = 1:numel(AX)
    plot(axh, 100*CHI, K_map(:,j), '-o', 'MarkerSize',3, 'LineWidth',1.4, ...
         'Color', cols(j,:), 'DisplayName', sprintf('a_x = %+.2f g', AX(j)));
end
yline(axh, 0, 'k-', 'neutral', 'HandleVisibility','off', 'LabelHorizontalAlignment','left');
xline(axh, 100*p.mass_dist_f, '--', 'current', 'HandleVisibility','off');
xline(axh, 100*chi_ss, ':', 'steady neutral', 'HandleVisibility','off');
xlabel(axh, 'front mass fraction [%]');
ylabel(axh, 'understeer gradient K [deg/g]');
title(axh, 'Balance vs weight split & longitudinal accel  (+K understeer / -K oversteer)');
legend(axh, 'Location','northwest', 'FontSize',8);
outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir,'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'stability_targets.png'));
close(f);
end
