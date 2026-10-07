function out = run_stability_targets(p)
% RUN_STABILITY_TARGETS  Understeer gradient under longitudinal load transfer,
% and the map K(front mass fraction, ax). A sign-and-trend tool: it does not
% set the rearward mass limit, which is a limit-handling (combined-slip,
% transient) question.
%
%   out = run_stability_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_stability_targets(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

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

fprintf('\nBalance under braking and power - %s  (understeer gradient K: + understeer, - oversteer)\n', p.car);
vd_row('Understeer gradient, steady state', sprintf('%+.3f deg/g', K_now), bword(K_now));
vd_row('Front weight fraction for neutral steer', sprintf('%.1f %%', 100*chi_ss), ...
       sprintf('now %.0f %%', 100*p.mass_dist_f));

fprintf('\n  K [deg/g] at %.0f %% front weight vs longitudinal acceleration\n', 100*p.mass_dist_f);
fprintf('    %-10s', 'accel [g]'); fprintf(' %+7.2f', AX); fprintf('\n');
fprintf('    %-10s', 'K');         fprintf(' %+7.3f', K_ax); fprintf('\n');
fprintf('  Braking moves load forward and adds understeer; power does the opposite.\n');

fprintf('\n  K [deg/g] by front weight fraction (rows) and acceleration [g] (columns)\n');
fprintf('    %-10s', 'front'); fprintf(' %+7.2f', AX); fprintf('\n');
for i = 1:numel(CHI)
    mark = ''; if abs(CHI(i)-p.mass_dist_f) < 1e-9, mark = '   this car'; end
    fprintf('    %5.0f %%   ', 100*CHI(i)); fprintf(' %+7.3f', K_map(i,:)); fprintf('%s\n', mark);
end

fprintf(['Assumes: linear tires (below about 0.4 g lateral), static loads plus longitudinal\n' ...
         '         transfer; read the sign and trend, not the exact value. It does not set\n' ...
         '         the rearmost weight split, which depends on limit and transient handling\n' ...
         '         (see run_wdist_targets).\n']);

out = struct('ax', AX, 'chi', CHI, 'K_ax', K_ax, 'K_map', K_map, ...
             'chi_neutral_ss', chi_ss, 'K_static', K_now, 'chi_current', p.mass_dist_f);

if vd_plots()
try
    make_plot(p, CHI, AX, K_map, chi_ss);
    fprintf('Saved plots/stability_targets.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

function pc = set_chi(p, chi)
% Same car at a different static front mass fraction; vd_set rebuilds a, b,
% the static axle loads and Izz.
pc = vd_set(p, 'mass_dist_f', chi);
end

function chi = neutral_chi(p, ax)
% Front mass fraction giving K = 0 at longitudinal accel ax. K rises with
% front mass, so bisect; returns a bracket end if there is no crossing.
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
xline(axh, 100*p.mass_dist_f, '--', 'this car', 'HandleVisibility','off');
xline(axh, 100*chi_ss, ':', 'steady neutral', 'HandleVisibility','off');
xlabel(axh, 'front mass fraction [%]');
ylabel(axh, 'understeer gradient K [deg/g]');
title(axh, 'Balance vs weight split and longitudinal acceleration  (+K understeer / -K oversteer)');
legend(axh, 'Location','northwest', 'FontSize',8);
outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir,'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'stability_targets.png'));
close(f);
end
