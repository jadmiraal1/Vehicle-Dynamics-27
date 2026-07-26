function tire_report()
% TIRE_REPORT  Presentation figures for the tire fits -> plots/.

p = vehicle_params();
evalc('R = pacejka_fit();');
TIRES  = {'LC0_16x75','R20_16x75','R20_18x60','GY_18x65'};
tire_c = [0.00 0.45 0.70; 0.90 0.62 0.00; 0.00 0.62 0.45; 0.84 0.37 0.00];
loads  = [50 100 150 200 250];
load_c = parula(numel(loads)+1); load_c = load_c(1:end-1, :);
outdir = fullfile(vd_root(), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end

% Fig 1: per-tire fits vs binned data
f = new_fig([100 60 1000 760]);
aa = linspace(0, 12.6, 240);
for t = 1:4
    T = R.(TIRES{t});
    ax = subplot(2,2,t); hold(ax, 'on'); style(ax);
    for k = find(~isnan(T.Fz_lbf))
        plot(T.curves{k}(:,1), T.curves{k}(:,2), 'o', 'MarkerSize', 3.5, ...
             'Color', load_c(k,:));
        plot(aa, mf([T.B(k) T.C(k) T.D(k) T.E(k)], aa), '-', ...
             'LineWidth', 1.7, 'Color', load_c(k,:), ...
             'DisplayName', sprintf('%.0f lbf', T.Fz_lbf(k)));
    end
    title(TIRES{t}, 'Color', tire_c(t,:), 'Interpreter', 'none');
    xlabel('slip angle |\alpha|  [deg]'); ylabel('|F_Y|  [lbf]');
    xlim([0 12.8]); ylim([0 720]);
end
legend(findobj(subplot(2,2,1), 'Type', 'line', '-not', 'Marker', 'o'), ...
       'Location', 'southeast', 'FontSize', 8);
sgtitle('Magic Formula pure-lateral fits — TTC R8/R9, IA<1.5\circ, 10–12 psi', 'FontWeight', 'bold');
save_fig(f, fullfile(outdir, 'tire_curves.png'));

% Fig 2: 3D blanket, design tire (fit interpolated across load)
T  = R.(p.tire_data_prefix);
ok = ~isnan(T.Fz_lbf);
fzq = linspace(45, 255, 60);
aq  = linspace(-13, 13, 121);
Bq = interp1(T.Fz_lbf(ok), T.B(ok), fzq, 'linear', 'extrap');
Cq = interp1(T.Fz_lbf(ok), T.C(ok), fzq, 'linear', 'extrap');
Eq = interp1(T.Fz_lbf(ok), T.E(ok), fzq, 'linear', 'extrap');
Dq = polyval(T.mu_coef, fzq) .* fzq;
FY = zeros(numel(aq), numel(fzq));
for j = 1:numel(fzq)
    FY(:,j) = mf([Bq(j) Cq(j) Dq(j) Eq(j)], aq');
end
f = new_fig([100 60 1050 780]);
% surfc adds the floor contour projection; view puts the rising wing
% toward the viewer; colorbar goes south so it can't hit the z-label
surfc(repmat(aq',1,numel(fzq)), repmat(fzq,numel(aq),1), FY, ...
      'EdgeColor', 'none', 'FaceAlpha', 0.95);
colormap(viridis_safe()); hold on;
for k = find(ok)
    c = T.curves{k};
    plot3([-flipud(c(:,1)); c(:,1)], T.Fz_lbf(k)*ones(2*size(c,1),1), ...
          [-flipud(c(:,2)); c(:,2)], 'k-', 'LineWidth', 1.8);
end
xlabel('slip angle \alpha  [deg]'); ylabel('F_Z  [lbf]'); zlabel('F_Y  [lbf]');
view(-134, 24); grid on; zlim([-780 700]);
title(sprintf('%s lateral force surface F_Y(\\alpha, F_Z) — black ribs: measured bins', ...
      strrep(p.tire_data_prefix, '_', '\_')), 'FontWeight', 'bold');
cb = colorbar('southoutside'); cb.Label.String = 'F_Y [lbf]';
save_fig(f, fullfile(outdir, 'tire_surface.png'));

% Fig 3: aligning moment + pneumatic trail (design tire)
D = load_with_mz(fullfile(vd_root(), 'TTC_Data'), ...
                 [p.tire_data_prefix '_*.mat']);
Fz_mag = -D.FZ;
base = (abs(D.FX./D.FZ) < 0.10) & (Fz_mag > 30) & (abs(D.IA) < 1.5) ...
       & (D.P > 9) & (D.P < 13) & ~((D.SA > 5) & (D.SA < 7));
f = new_fig([100 60 1000 420]);
ax1 = subplot(1,2,1); hold on; style(ax1);
ax2 = subplot(1,2,2); hold on; style(ax2);
for k = 1:numel(loads)
    sel = base & (abs(Fz_mag - loads(k)) < loads(k)*0.15);
    if nnz(sel) < 3000, continue; end
    [xs, mz] = odd_bins(D.SA(sel),  D.MZ(sel));
    [~,  fy] = odd_bins(D.SA(sel), -D.FY(sel));
    n = min(numel(mz), numel(fy));
    plot(ax1, xs(1:n), mz(1:n), 'o-', 'MarkerSize', 3, 'LineWidth', 1.5, ...
         'Color', load_c(k,:), 'DisplayName', sprintf('%d lbf', loads(k)));
    plot(ax2, xs(3:n), mz(3:n)./fy(3:n)*12, 'o-', 'MarkerSize', 3, ...
         'LineWidth', 1.5, 'Color', load_c(k,:));
end
xlabel(ax1, 'slip angle |\alpha|  [deg]'); ylabel(ax1, 'M_Z  [lbf\cdotft]');
title(ax1, 'Aligning moment vs. slip angle');
legend(ax1, 'Location', 'northeast', 'FontSize', 8);
xlabel(ax2, 'slip angle |\alpha|  [deg]');
ylabel(ax2, 'pneumatic trail t_p = M_Z/F_Y  [in]');
title(ax2, 'Pneumatic trail vs. slip angle'); ylim(ax2, [0 inf]);
sgtitle(sprintf('%s aligning moment & pneumatic trail — TTC R8', ...
        p.tire_data_prefix), 'FontWeight', 'bold', 'Interpreter', 'none');
save_fig(f, fullfile(outdir, 'tire_mz_trail.png'));

% Fig 4: cross-tire load dependence
f = new_fig([100 60 1000 420]);
ax1 = subplot(1,2,1); hold on; style(ax1);
ax2 = subplot(1,2,2); hold on; style(ax2);
fzq = linspace(45, 255, 80);
for t = 1:4
    T = R.(TIRES{t}); ok = ~isnan(T.Fz_lbf);
    plot(ax1, T.Fz_lbf(ok), T.Ca_lbf_deg(ok), 'o', 'MarkerSize', 5, 'Color', tire_c(t,:));
    plot(ax1, fzq, polyval(T.Ca_coef, fzq), '-', 'LineWidth', 1.7, ...
         'Color', tire_c(t,:), 'DisplayName', TIRES{t});
    idp = ok & T.peak_in_sweep;
    plot(ax2, T.Fz_lbf(idp), T.mu_peak(idp), 'o', 'MarkerSize', 5, ...
         'MarkerFaceColor', tire_c(t,:), 'Color', tire_c(t,:));
    plot(ax2, T.Fz_lbf(ok & ~T.peak_in_sweep), T.mu_peak(ok & ~T.peak_in_sweep), ...
         'o', 'MarkerSize', 6, 'Color', tire_c(t,:));   % hollow: peak beyond sweep
    plot(ax2, fzq, polyval(T.mu_coef, fzq), '-', 'LineWidth', 1.7, 'Color', tire_c(t,:));
end
xlabel(ax1, 'F_Z  [lbf]'); ylabel(ax1, 'C_\alpha  [lbf/deg]');
title(ax1, 'Cornering stiffness vs load');
legend(ax1, findobj(ax1,'Type','line','-not','Marker','o'), 'Location', 'northwest', 'Interpreter', 'none');
xlabel(ax2, 'F_Z  [lbf]'); ylabel(ax2, 'peak \mu_Y  [-]');
title(ax2, 'Load sensitivity'); xline(ax2, p.Fz_design_lbf, ':', 'design load');
sgtitle('Candidate tire comparison — MF fits, TTC R8/R9', 'FontWeight', 'bold');
save_fig(f, fullfile(outdir, 'tire_load_sensitivity.png'));

% Fig 5: combined friction cloud, 18in LC0 (tire + vehicle axes)
Dc = load_cloud(fullfile(vd_root(), 'TTC_Data'));
Fzc = -Dc.FZ;
sel = (Fzc > 60) & (abs(Dc.IA) < 3);
nfx = Dc.FX(sel)./Fzc(sel);   nfy = -Dc.FY(sel)./Fzc(sel);
sar = deg2rad(Dc.SA(sel));
nfx_v = (Dc.FX(sel).*cos(sar) - (-Dc.FY(sel)).*sin(sar))./Fzc(sel);
nfy_v = (Dc.FX(sel).*sin(sar) + (-Dc.FY(sel)).*cos(sar))./Fzc(sel);

% envelope axes: measured long18 peaks + heaviest lateral bin peak
mux_d = R.long18.drive.mu_x;  mux_b = R.long18.brake.mu_x;
T = R.(p.tire_data_prefix);  okb = find(~isnan(T.Fz_lbf));
muy = T.mu_peak(okb(end));

n_env = mean([R.long18.drive.n_envelope, R.long18.brake.n_envelope]); % measured combined-slip exponent (drive/brake avg) - do not hardcode
f = new_fig([60 60 1250 640]);
frames = {{nfx, nfy, 'tire axes'}, {nfx_v, nfy_v, 'vehicle axes (slip-angle rotated)'}};
for q = 1:2
    ax = subplot(1,2,q); hold(ax,'on'); style(ax);
    scatter(frames{q}{1}, frames{q}{2}, 2, Fzc(sel), 'filled', ...
            'MarkerFaceAlpha', 0.3);
    th = linspace(0, 2*pi, 200);
    for r = [1.0 1.5 2.0 2.5]
        plot(r*cos(th), r*sin(th), '-', 'Color', [0.78 0.78 0.78], 'LineWidth', 0.7);
    end
    tt = linspace(0, 2*pi, 400);
    for nc = {{2.0, [0.84 0.37 0], '--'}, {n_env, [0 0.45 0.70], '-'}}
        n = nc{1}{1};
        cx = sign(cos(tt)).*abs(cos(tt)).^(2/n);
        cy = sign(sin(tt)).*abs(sin(tt)).^(2/n);
        ex = cx;  ex(cx>=0) = mux_d*cx(cx>=0);  ex(cx<0) = mux_b*cx(cx<0);
        plot(ex, muy*cy, nc{1}{3}, 'Color', nc{1}{2}, 'LineWidth', 2);
    end
    axis equal; xlim([-3.05 3.05]); ylim([-3 3]);
    xlabel('F_X/F_Z  (+drive / -brake)'); ylabel('F_Y/F_Z');
    title(frames{q}{3});
end
legend(subplot(1,2,1), {'', '', '', '', '', 'assumed ellipse n=2', ...
       sprintf('measured envelope n\\approx%.2f', n_env)}, 'Location', 'southeast', 'FontSize', 8);
cb = colorbar; cb.Label.String = 'F_Z [lbf]';
sgtitle('Combined corner/drive friction cloud - Hoosier 18.0x6.0-10 LC0, TTC R6', 'FontWeight', 'bold');
save_fig(f, fullfile(outdir, 'tire_friction_cloud.png'));

% Fig 6: longitudinal MF fit, 18in LC0 (drive + brake, ~250 lbf)
f = new_fig([80 80 1000 580]);
ax = axes(f); hold(ax, 'on'); style(ax);
srq = linspace(0.001, 0.22, 300);
sides = {{'drive', +1, [0 0.62 0.45]}, {'brake', -1, [0.84 0.37 0]}};
for q = 1:2
    name = sides{q}{1};  sgn = sides{q}{2};  c = sides{q}{3};
    L = R.long18.(name);
    plot(sgn*L.curve(:,1), sgn*L.curve(:,2), 'o', 'MarkerSize', 5, ...
         'MarkerFaceColor', c, 'Color', 'k');
    plot(sgn*srq, sgn*mf([L.B L.C L.D L.E], srq), '-', 'LineWidth', 2.2, ...
         'Color', c, 'DisplayName', sprintf('%s: mu_x=%.2f, K_x/F_Z=%.0f, R^2=%.3f', ...
         name, L.mu_x, L.Kx_per_Fz, L.R2));
end
xline(0, 'Color', [0.6 0.6 0.6]); yline(0, 'Color', [0.6 0.6 0.6]);
xlabel('slip ratio \kappa  [-]   (+drive / -brake)');
ylabel('longitudinal force F_X  [lbf]');
legend(findobj(ax, 'Type', 'line', '-not', 'Marker', 'o'), ...
       'Location', 'southeast', 'FontSize', 9);
title('Longitudinal MF fit - Hoosier 18.0x6.0-10 LC0, TTC R6, ~250 lbf, SA\approx0', ...
      'FontWeight', 'bold');
save_fig(f, fullfile(outdir, 'tire_longitudinal.png'));

% Fig 7: high-load extrapolation band + side-by-side load-sensitivity fits.
edge = p.Fz_fit_max;
cov  = p.hiload_cov_lbf;         % SAME value donor_hiload_slope computed and mu_of_load respects - do not recompute here
Fz_outer = p.Fz_outer_limit_lbf; % SAME value build_tire_coeffs.m computed - do not recompute here
xmax     = max(Fz_outer, cov) + 15;

f = new_fig([80 80 1300 560]);

% -- panel A: the four fits side by side --
axA = subplot(1,2,1); hold(axA,'on'); style(axA);
for t = 1:4
    T  = R.(TIRES{t});
    ip = ~isnan(T.Fz_lbf) & T.peak_in_sweep;
    ho = ~isnan(T.Fz_lbf) & ~T.peak_in_sweep;
    plot(axA, T.Fz_lbf(ip), T.mu_peak(ip)*p.mu_derate, 'o', 'MarkerSize', 6, ...
         'MarkerFaceColor', tire_c(t,:), 'Color', tire_c(t,:), 'HandleVisibility','off');
    if any(ho)
        plot(axA, T.Fz_lbf(ho), T.mu_peak(ho)*p.mu_derate, 'o', 'MarkerSize', 7, ...
             'Color', tire_c(t,:), 'HandleVisibility','off');
    end
    fe = linspace(45, max(T.Fz_lbf(ip)), 60);
    plot(axA, fe, polyval(T.mu_coef, fe)*p.mu_derate, '-', 'LineWidth', 1.9, ...
         'Color', tire_c(t,:), 'DisplayName', sprintf('%s (edge %.0f)', TIRES{t}, max(T.Fz_lbf(ip))));
end
xline(axA, p.Fz_design_lbf, ':', 'design load', 'HandleVisibility','off');
xlim(axA, [40 280]); xlabel(axA, 'F_Z  [lbf]'); ylabel(axA, 'derated peak \mu_Y');
title(axA, 'Load-sensitivity fits, side by side');
legend(axA, 'Location','northeast', 'FontSize',8, 'Interpreter', 'none');

% -- panel B: LC0 high-load extrapolation band --
axB  = subplot(1,2,2); hold(axB,'on'); style(axB);
Tdes = R.(p.tire_data_prefix);
okd  = ~isnan(Tdes.Fz_lbf) & Tdes.peak_in_sweep;
fe   = linspace(45, edge, 60);
xhi  = linspace(edge, xmax, 60);
pl = p; pl.tire_hiload = 'low';
ws = warning('off', 'mu_of_load:beyondDonorCoverage');   % expected here - xhi intentionally runs past cov
mu_cen = mu_of_load(p,  xhi);                  % exactly the model's central branch
mu_low = mu_of_load(pl, xhi);                  % ... and the pessimistic bracket
warning(ws);
fill(axB, [xhi fliplr(xhi)], [mu_low fliplr(mu_cen)], [0 0.55 0.30], ...
     'FaceAlpha', 0.15, 'EdgeColor','none', 'DisplayName','extrapolation band');
plot(axB, fe, polyval(p.mu_coef, fe)*p.mu_derate, '-', 'Color', tire_c(1,:), ...
     'LineWidth', 2.4, 'DisplayName', sprintf('measured fit (<= %.0f lbf)', edge));
plot(axB, xhi, mu_low, '--', 'Color', [0.75 0.15 0.15], 'LineWidth', 2, 'DisplayName','LOW (blind-linear)');
plot(axB, xhi, mu_cen, '-',  'Color', [0 0.55 0.30], 'LineWidth', 2, 'DisplayName','CENTRAL (mu\_of\_load)');
plot(axB, Tdes.Fz_lbf(okd), Tdes.mu_peak(okd)*p.mu_derate, 'o', 'MarkerSize', 7, ...
     'MarkerFaceColor', tire_c(1,:), 'Color', tire_c(1,:), 'DisplayName','LC0 measured');
for t = 2:4
    T = R.(TIRES{t}); hi = ~isnan(T.Fz_lbf) & T.peak_in_sweep & (T.Fz_lbf > edge-6);
    if any(hi)
        plot(axB, T.Fz_lbf(hi), T.mu_peak(hi)*p.mu_derate, 's', 'MarkerSize', 6, ...
             'MarkerFaceColor', tire_c(t,:), 'Color', tire_c(t,:), 'HandleVisibility','off');
    end
end
xline(axB, edge, ':', 'HandleVisibility','off');
xline(axB, Fz_outer, ':', 'outer tire', 'HandleVisibility','off');
xlim(axB, [40 xmax]); ylim(axB, [1.30 1.90]);
xlabel(axB, 'F_Z  [lbf]'); ylabel(axB, 'derated peak \mu_Y');
title(axB, 'LC0 high-load band: central vs low');
legend(axB, 'Location','northeast', 'FontSize',8);
sgtitle('High-load extrapolation: fits side by side & the central/low band', 'FontWeight','bold');
save_fig(f, fullfile(outdir, 'tire_extrapolation_band.png'));

% Fig 8: aligning moment + pneumatic trail, ACROSS TIRES (Fig 3 only ever
% looked at the design tire). One representative load (150 lbf, the middle
% TTC bin) so all four tires compare at a common condition.
REP_LOAD = 150;
f = new_fig([100 60 1000 420]);
ax1 = subplot(1,2,1); hold(ax1,'on'); style(ax1);
ax2 = subplot(1,2,2); hold(ax2,'on'); style(ax2);
for t = 1:4
    Dt = load_with_mz(fullfile(vd_root(), 'TTC_Data'), ...
                       [TIRES{t} '_*.mat']);
    Fz_mag_t = -Dt.FZ;
    base_t = (abs(Dt.FX./Dt.FZ) < 0.10) & (Fz_mag_t > 30) & (abs(Dt.IA) < 1.5) ...
             & (Dt.P > 9) & (Dt.P < 13) & ~((Dt.SA > 5) & (Dt.SA < 7));
    sel_t = base_t & (abs(Fz_mag_t - REP_LOAD) < REP_LOAD*0.15);
    if nnz(sel_t) < 3000, continue; end   % same density floor as Fig 3
    [xs, mz] = odd_bins(Dt.SA(sel_t),  Dt.MZ(sel_t));
    [~,  fy] = odd_bins(Dt.SA(sel_t), -Dt.FY(sel_t));
    n = min(numel(mz), numel(fy));
    plot(ax1, xs(1:n), mz(1:n), 'o-', 'MarkerSize', 3, 'LineWidth', 1.5, ...
         'Color', tire_c(t,:), 'DisplayName', TIRES{t});
    plot(ax2, xs(3:n), mz(3:n)./fy(3:n)*12, 'o-', 'MarkerSize', 3, ...
         'LineWidth', 1.5, 'Color', tire_c(t,:));
end
xlabel(ax1, 'slip angle |\alpha|  [deg]'); ylabel(ax1, 'M_Z  [lbf\cdotft]');
title(ax1, 'Aligning moment vs. slip angle');
legend(ax1, 'Location', 'northeast', 'FontSize', 8, 'Interpreter', 'none');
xlabel(ax2, 'slip angle |\alpha|  [deg]');
ylabel(ax2, 'pneumatic trail t_p = M_Z/F_Y  [in]');
title(ax2, 'Pneumatic trail vs. slip angle'); ylim(ax2, [0 inf]);
sgtitle(sprintf('Cross-tire aligning moment & pneumatic trail — TTC R8, %.0f lbf', REP_LOAD), ...
        'FontWeight', 'bold');
save_fig(f, fullfile(outdir, 'tire_mz_trail_by_tire.png'));

% Fig 9: the anisotropy shape correction, visualized. build_tire_coeffs.m
f = new_fig([80 80 900 620]);
ax = axes(f); hold(ax, 'on'); style(ax);
aa9 = linspace(0, 20, 400);
curve9 = R.eval(aa9, p.Fz_lat6);          % design tire's MF curve AT THE LOAD-MATCHED Fz
[peak9, ipk] = max(curve9);
plot(ax, aa9, curve9, '-', 'Color', tire_c(1,:), 'LineWidth', 2.2, ...
     'DisplayName', sprintf('%s curve @ Fz=%.0f lbf (load-matched)', strrep(p.tire_data_prefix,'_','\_'), p.Fz_lat6));
plot(ax, aa9(ipk), peak9, '^', 'MarkerSize', 9, 'MarkerFaceColor', tire_c(1,:), ...
     'Color', tire_c(1,:), 'DisplayName', sprintf('curve peak = %.3f', peak9));
plot(ax, 6.0, p.mu_y_at6, 's', 'MarkerSize', 9, 'MarkerFaceColor', [0.84 0.37 0], ...
     'Color', [0.84 0.37 0], 'DisplayName', sprintf('18in LC0 measured @ 6deg = %.3f', p.mu_y_at6));
plot(ax, [6.0 6.0], [0 p.mu_y_at6], ':', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off');
yline(ax, p.mu_y_18_peak, '--', sprintf('implied peak = mu\\_y\\_at6 / shape6 = %.3f', p.mu_y_18_peak), ...
      'Color', [0.84 0.37 0], 'LabelHorizontalAlignment','left');
xlabel(ax, 'slip angle |\alpha|  [deg]'); ylabel(ax, '\mu_Y  [-]');
xlim(ax, [0 20]);
legend(ax, 'Location', 'southeast', 'FontSize', 8);
title(ax, sprintf('shape6 = %.4f  (curve value @6deg / curve peak, both at Fz=%.0f lbf)', ...
      p.shape6, p.Fz_lat6));
sgtitle('Anisotropy shape correction — 18in LC0 peak back-derived from its 6deg point', ...
        'FontWeight', 'bold');
save_fig(f, fullfile(outdir, 'tire_anisotropy_shape.png'));

fprintf('tire_report: 9 figures written to plots/\n');
end

function D = load_cloud(data_dir)
channels = {'SA','FY','FX','FZ','IA'};
files = dir(fullfile(data_dir, 'LC0_18x60_*.mat'));
chunks = cell(numel(files), numel(channels));
for i = 1:numel(files)
    S = load(fullfile(data_dir, files(i).name));
    for c = 1:numel(channels)
        chunks{i,c} = S.(channels{c})(:);
    end
end
for c = 1:numel(channels)
    D.(channels{c}) = vertcat(chunks{:,c});
end
end

function y = mf(prm, a)
Bx = prm(1).*a;
y  = prm(3).*sin(prm(2).*atan(Bx - prm(4).*(Bx - atan(Bx))));
end

function [x, y] = odd_bins(sa, val)
v = val;  v(sa < 0) = -v(sa < 0);
a = abs(sa); edges = 0.25:0.5:12.25;
x = []; y = [];
for i = 1:numel(edges)-1
    m = (a >= edges(i)) & (a < edges(i+1));
    if nnz(m) > 40
        x(end+1,1) = mean(a(m));   %#ok<AGROW>
        y(end+1,1) = median(v(m)); %#ok<AGROW>
    end
end
end

function D = load_with_mz(data_dir, pattern)
channels = {'SA','FY','FX','FZ','IA','P','MZ'};
files = dir(fullfile(data_dir, pattern));
chunks = cell(numel(files), numel(channels));
for i = 1:numel(files)
    if contains(files(i).name, 'raw'), continue; end
    S = load(fullfile(data_dir, files(i).name));
    for c = 1:numel(channels)
        chunks{i,c} = S.(channels{c})(:);
    end
end
for c = 1:numel(channels)
    D.(channels{c}) = vertcat(chunks{:,c});
end
end

function f = new_fig(pos)
f = figure('Visible', 'off', 'Position', pos, 'Color', 'w');
end

function style(ax)
set(ax, 'Box', 'off', 'FontSize', 10, 'GridAlpha', 0.15, 'LineWidth', 0.8);
grid(ax, 'on');
end

function cmap = viridis_safe()
if exist('viridis', 'file'), cmap = viridis(); else, cmap = parula(); end
end

function save_fig(f, path)
if exist('exportgraphics', 'file')
    exportgraphics(f, path, 'Resolution', 180);
else
    saveas(f, path);
end
close(f);
end