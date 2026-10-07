function out = aligning_moment(p)
% ALIGNING_MOMENT  Steering targets from the tire's self-aligning moment: peak
% aligning torque per tire and a caster / mechanical-trail chart.
% Reads the design tire's TTC data directly (needs TTC_Data/; not in the
% tire artifact, so not covered by the staleness check or vd_golden).
% Mechanical trail is taken as Re*tan(caster), i.e. zero caster offset at
% the hub. Theory: VD_physics_reference.md sec 8 and 13.
%
%   out = aligning_moment()      the active car (vd_car / cars/config_<CAR>.m)
%   out = aligning_moment(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');

LAMBDA_T   = 1.0;          % pneumatic-trail belt -> track scaling PROVISIONAL
V_LOW      = 12;           % low-speed corner = steering worst case (no aero help)
u          = vd_const();
N_PER_LBF  = u.N_PER_LBF;
NM_PER_FTLB= u.NM_PER_FTLB;
MM_PER_IN  = u.MM_PER_IN;
IN_PER_FT  = u.IN_PER_FT;

% --- front-outer tire load at the low-speed lateral limit (steering worst case)
G         = axle_grip(p, V_LOW);            % returns per-tire loads in G.Fz.{fo,fi,ro,ri}
Fz_fo_lbf = G.Fz.fo / N_PER_LBF;

% --- design tire: peak |Mz| and initial pneumatic trail per load bin (own data)
here  = vd_root();
D     = load_mz(fullfile(here, 'TTC_Data'), [p.tire_data_prefix '_*.mat']);
Fz    = -D.FZ;
% Same mask as pacejka_fit: pure lateral, near-zero camber, 9-13 psi,
% rolling, and without the 5-7 deg band where the TTC slip sweep reverses
% (the tire is not in steady state there).
base  = (abs(D.FX./D.FZ) < 0.10) & (Fz > 30) & (abs(D.IA) < 1.5) & (D.P > 9) & (D.P < 13) ...
        & ~(abs(D.SA) > 5.0 & abs(D.SA) < 7.0) & (D.V > 20);
loads = [50 100 150 200 250];
Fzb = []; mzpk = []; tp0 = [];
for k = 1:numel(loads)
    sel = base & (abs(Fz - loads(k)) < loads(k)*0.15);
    if nnz(sel) < 3000, continue; end
    Fzb(end+1)  = mean(Fz(sel));                                    %#ok<AGROW>
    mzpk(end+1) = pctl(abs(D.MZ(sel)), 95);                        %#ok<AGROW> peak |Mz| [lbf-ft]
    % operating-range trail (1-3 deg slip); near zero slip Mz/Fy is 0/0
    lo = sel & (abs(D.SA) > 1) & (abs(D.SA) < 3);
    tp0(end+1)  = median(abs(D.MZ(lo)) ./ max(abs(D.FY(lo)),1)) * IN_PER_FT;  %#ok<AGROW> [in]
end

% --- fits + short self-extrapolation to the front-outer load
mz_coef = polyfit(Fzb, mzpk, 2);      % peak Mz ~ quadratic in load
tp_coef = polyfit(Fzb, tp0,  1);      % initial trail ~ linear in load
edge     = max(Fz(base));          % furthest measured load (not the bin mean)
edge_fit = max(Fzb);               % last fit anchor (bin means thin out up high)
gap_pct  = 100*(Fz_fo_lbf/edge - 1);

Mz_belt   = polyval(mz_coef, Fz_fo_lbf);                 % [lbf-ft], belt
Mz_lo_Nm  = Mz_belt * p.mu_derate  * NM_PER_FTLB;        % track, grip-limited end
Mz_hi_Nm  = Mz_belt * p.lambda_Ca  * NM_PER_FTLB;        % track, stiffness end
trail_mm  = polyval(tp_coef, Fz_fo_lbf) * LAMBDA_T * MM_PER_IN;

out = struct('Fz_fo_lbf', Fz_fo_lbf, 'edge_data_lbf', edge, 'edge_fit_lbf', edge_fit, 'gap_pct', gap_pct, ...
             'Mz_belt_Nm', Mz_belt*NM_PER_FTLB, 'Mz_track_Nm', [Mz_lo_Nm Mz_hi_Nm], ...
             'trail_mm', trail_mm, 'mz_coef', mz_coef, 'tp_coef', tp_coef, ...
             'Fzb', Fzb, 'mzpk', mzpk, 'tp0', tp0);

% --- caster / mechanical-trail design chart ------------------------------
[alc, Fyc, tpc] = curve_at_load(D, base, 250);      % Fy [lbf], trail [mm] vs |slip|
Fyc = Fyc * p.mu_derate;                            % track magnitude
[~, ilim] = max(Fyc);                               % grip-limit slip index
tp_peak_mm = max(tpc);                              % peak sub-limit pneumatic trail

cas  = [0 2 3 4 5 6 8];                             % candidate caster [deg]
tmm  = p.Re * tand(cas) * 1000;                     % -> mechanical trail [mm]
Mpk  = zeros(size(cas));  Mlim = zeros(size(cas));
for i = 1:numel(cas)
    Mtot    = (Fyc*N_PER_LBF) .* ((tmm(i)+tpc)/1000);   % [N*m] vs slip
    Mpk(i)  = max(Mtot);                                % peak effort (mid-corner)
    Mlim(i) = Mtot(ilim);                               % torque still there AT the limit
end
limfrac = 100*Mlim./Mpk;      % "limit feel": %% of peak torque still present at limit

REC = [3 6];                                        % deg, practice-consistent window
rmm = p.Re * tand(REC) * 1000;                      % -> mechanical trail [mm]
out.caster_deg_rec    = REC;
out.mech_trail_mm_rec = rmm;
out.Mtire_Nm_rec      = interp1(cas, Mpk, REC);
out.tp_peak_mm        = tp_peak_mm;
out.cas_table         = [cas(:) tmm(:) Mpk(:) Mlim(:) limfrac(:)];

fprintf('\nSteering: aligning moment - %s  (TTC data; outer front tire at the %d m/s cornering limit)\n', ...
        p.car, V_LOW);
if Fz_fo_lbf > edge
    vd_row('Outer front tire load', sprintf('%.0f lbf', Fz_fo_lbf), ...
           sprintf('%+.0f %% beyond data (%.0f)', gap_pct, edge));
else
    vd_row('Outer front tire load', sprintf('%.0f lbf', Fz_fo_lbf));
end
vd_row('Peak aligning torque per tire, on track', sprintf('%.1f-%.1f N*m', Mz_lo_Nm, Mz_hi_Nm));
vd_row('Peak aligning torque per tire, test rig', sprintf('%.1f N*m', Mz_belt*NM_PER_FTLB), ...
       'upper bound: size the rack to it');
vd_row('Pneumatic trail at 1-3 deg slip', sprintf('%.1f mm', trail_mm), 'for reference only');
if Fz_fo_lbf > edge
    fprintf(['  The aligning moment is extrapolated past the data (fit to %.0f lbf). For a\n' ...
             '  conservative rack, use the peak measured value plus a margin instead.\n'], edge_fit);
end

fprintf('\n  Caster and mechanical trail (outer front tire at 250 lbf)\n');
fprintf('    %-8s %-12s %-13s %-14s %s\n', 'caster', 'mechanical', 'peak torque', 'torque at', 'at limit /');
fprintf('    %-8s %-12s %-13s %-14s %s\n', '[deg]', 'trail [mm]', '[N*m]', 'limit [N*m]', 'peak [%]');
for i = 1:numel(cas)
    fprintf('    %-8.0f %-12.1f %-13.1f %-14.1f %.0f\n', cas(i), tmm(i), Mpk(i), Mlim(i), limfrac(i));
end
fprintf('\n');
vd_row('Suggested starting caster', sprintf('%.0f-%.0f deg', REC(1), REC(2)), ...
       sprintf('%.0f-%.0f mm trail', rmm(1), rmm(2)));
vd_row('Peak torque per front tire at that caster', ...
       sprintf('%.0f-%.0f N*m', out.Mtire_Nm_rec(1), out.Mtire_Nm_rec(2)));
fprintf(['  At the grip limit the tire''s own (pneumatic) trail falls to about zero, so the\n' ...
         '  steering feel at the limit comes from caster alone. The suggested caster gives\n' ...
         '  less trail than the tire''s peak pneumatic trail (%.0f mm), so it adds to the\n' ...
         '  tire''s feel without overpowering it. Choose the final value against the\n' ...
         '  steering effort budget.\n'], tp_peak_mm);
fprintf(['Assumes: mechanical trail = wheel radius x tan(caster) (no caster offset at the\n' ...
         '         hub); on-track torque scaled from the rig by the tire grip (%.2f) and\n' ...
         '         stiffness (%.2f) scales; rig-to-track trail scale %.1f (provisional).\n'], ...
        p.mu_derate, p.lambda_Ca, LAMBDA_T);

if vd_plots()
try
    caster_plot(alc, Fyc, tpc, [0 rmm(1) rmm(2)], N_PER_LBF, here);
    fprintf('Saved plots/caster_target.png\n');
catch ce
    fprintf('Plot not saved (caster): %s\n', ce.message);
end
end

if vd_plots()
try
    make_plot(p, D, base, loads, out, Fz_fo_lbf, edge);
    fprintf('Saved plots/aligning_moment.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

function [al, Fy, tp] = curve_at_load(D, base, load)
% Fy [lbf] and pneumatic trail [mm] vs |slip| at one load bin (curve shape).
u = vd_const();
Fz  = -D.FZ;
sel = base & (abs(Fz - load) < load*0.15);
a = D.SA(sel); fy = -D.FY(sel).*sign(a); mz = D.MZ(sel).*sign(a); aa = abs(a);
edges = 0.25:0.5:12.25; al = []; Fy = []; tp = [];
for i = 1:numel(edges)-1
    m = (aa >= edges(i)) & (aa < edges(i+1));
    if nnz(m) > 40
        al(end+1,1) = mean(aa(m));                              %#ok<AGROW>
        Fy(end+1,1) = median(fy(m));                            %#ok<AGROW>
        tp(end+1,1) = median(mz(m)./max(fy(m),1)) * u.IN_PER_FT * u.MM_PER_IN;  %#ok<AGROW> [mm]
    end
end
end

function caster_plot(al, Fy, tp, tmechs, NPL, here)
f = figure('Visible','off','Position',[80 80 660 480],'Color','w');
ax = axes(f); hold(ax,'on'); grid(ax,'on');
cols = lines(numel(tmechs));
for k = 1:numel(tmechs)
    Mtot = (Fy*NPL).*((tmechs(k)+tp)/1000);
    plot(ax, al, Mtot, '-o', 'MarkerSize',3, 'LineWidth',1.7, 'Color',cols(k,:), ...
         'DisplayName', sprintf('%.0f mm mech trail', tmechs(k)));
end
[~,il] = max(Fy); xline(ax, al(il), ':', 'grip limit', 'HandleVisibility','off');
xlabel(ax,'slip angle |\alpha| [deg]'); ylabel(ax,'aligning torque / front tire [N\cdotm]');
title(ax,'Caster fills in limit feel (pneumatic trail collapses)');
legend(ax,'Location','northwest','FontSize',9);
outdir = fullfile(here,'plots'); if ~exist(outdir,'dir'), mkdir(outdir); end
if exist('exportgraphics','file'), exportgraphics(f, fullfile(outdir,'caster_target.png'),'Resolution',170);
else, saveas(f, fullfile(outdir,'caster_target.png')); end
close(f);
end

function make_plot(p, D, base, loads, o, Fz_fo, edge)
u = vd_const();
Fz = -D.FZ;
load_c = parula(numel(loads)+1); load_c = load_c(1:end-1, :);
f = figure('Visible','off','Position',[80 80 1200 470],'Color','w');

% left: pneumatic trail vs slip (the collapse) per load
ax1 = subplot(1,2,1); hold(ax1,'on'); grid(ax1,'on');
for k = 1:numel(loads)
    sel = base & (abs(Fz-loads(k)) < loads(k)*0.15);
    if nnz(sel) < 3000, continue; end
    [sa, tp] = trail_curve(D.SA(sel), D.MZ(sel), D.FY(sel));
    plot(ax1, sa, tp*u.MM_PER_IN, 'o-', 'MarkerSize', 3, 'Color', load_c(k,:), ...
         'DisplayName', sprintf('%d lbf', loads(k)));
end
xlabel(ax1,'slip angle |\alpha| [deg]'); ylabel(ax1,'pneumatic trail t_p [mm]');
title(ax1,'Trail collapses toward the limit'); legend(ax1,'Location','northeast','FontSize',8);
ylim(ax1,[0 inf]);

% right: peak Mz(Fz) + trail(Fz) with fit and the extrapolated design point
ax2 = subplot(1,2,2); hold(ax2,'on'); grid(ax2,'on');
fq = linspace(45, max(Fz_fo,edge)+10, 60);
yyaxis(ax2,'left');
plot(ax2, o.Fzb, o.mzpk*u.NM_PER_FTLB, 'o', 'MarkerSize',6, 'MarkerFaceColor',[0 .45 .70], 'Color',[0 .45 .70]);
plot(ax2, fq, polyval(o.mz_coef,fq)*u.NM_PER_FTLB, '-', 'LineWidth',1.8, 'Color',[0 .45 .70]);
ylabel(ax2,'peak M_z [N\cdotm]');
yyaxis(ax2,'right');
plot(ax2, o.Fzb, o.tp0*u.MM_PER_IN, 's', 'MarkerSize',6, 'MarkerFaceColor',[.85 .33 .10], 'Color',[.85 .33 .10]);
plot(ax2, fq, polyval(o.tp_coef,fq)*u.MM_PER_IN, '-', 'LineWidth',1.8, 'Color',[.85 .33 .10]);
ylabel(ax2,'initial trail t_p [mm]');
xline(ax2, edge, ':', 'data edge', 'HandleVisibility','off');
xline(ax2, Fz_fo, '--', 'front-outer', 'HandleVisibility','off');
xlabel(ax2,'F_Z [lbf]'); title(ax2,'Peak M_z & trail vs load (design point extrapolated)');

outdir = fullfile(vd_root(),'plots');
if ~exist(outdir,'dir'), mkdir(outdir); end
if exist('exportgraphics','file'), exportgraphics(f, fullfile(outdir,'aligning_moment.png'),'Resolution',170);
else, saveas(f, fullfile(outdir,'aligning_moment.png')); end
close(f);
end

function [x, y] = trail_curve(sa, mz, fy)
% Median pneumatic trail [in] in each |slip| bin.
a = abs(sa); mza = abs(mz); fya = abs(fy);
edges = 0.25:0.5:12.25; x = []; y = [];
for i = 1:numel(edges)-1
    m = (a >= edges(i)) & (a < edges(i+1));
    if nnz(m) > 40
        x(end+1,1) = mean(a(m));                              %#ok<AGROW>
        y(end+1,1) = median(mza(m) ./ max(fya(m),1)) * 12;    %#ok<AGROW> [in] (Mz in lbf-ft)
    end
end
end

function D = load_mz(data_dir, pattern)
channels = {'SA','FY','FX','FZ','IA','P','MZ','V'};
files = dir(fullfile(data_dir, pattern));
chunks = cell(numel(files), numel(channels));
for i = 1:numel(files)
    if contains(files(i).name,'raw'), continue; end
    S = load(fullfile(data_dir, files(i).name));
    if ~isfield(S,'MZ'), continue; end
    for c = 1:numel(channels), chunks{i,c} = S.(channels{c})(:); end
end
for c = 1:numel(channels), D.(channels{c}) = vertcat(chunks{:,c}); end
end

function v = pctl(x, q)
x = sort(x(~isnan(x))); n = numel(x);
if n == 0, v = NaN; return; end
r = q/100*(n-1); lo = floor(r);
v = x(lo+1) + (r-lo)*(x(min(lo+2,n)) - x(lo+1));
end
