function out = aligning_moment()
% ALIGNING_MOMENT  Steering targets from tire self-aligning moment (#52-54):
% T-MZ peak torque/tire, T-CAS caster transfer chart, trail diagnostics.
% Theory: ref doc sec 8 and 13.

p = vehicle_params();

LAMBDA_T   = 1.0;          % pneumatic-trail belt->track (baseline; measure to refine)
V_LOW      = 12;           % low-speed corner = steering worst case (no aero help)
N_PER_LBF  = 4.44822;
NM_PER_FTLB= 1.35582;
MM_PER_IN  = 25.4;

% --- front-outer tire load at the low-speed lateral limit (steering worst case)
G         = axle_grip(p, V_LOW);            % returns per-tire loads in G.Fz.{fo,fi,ro,ri}
Fz_fo_lbf = G.Fz.fo / N_PER_LBF;

% --- design tire: peak |Mz| and initial pneumatic trail per load bin (own data)
here  = vd_root();
D     = load_mz(fullfile(here, 'TTC_Data'), [p.tire_data_prefix '_*.mat']);
Fz    = -D.FZ;
% ~(5..7 deg) drops the TTC sweep-reversal band: at the slip-sweep turnaround the
% tire is transient (not steady state), so Fy/Mz there dip artificially -> a fake
% valley at ~6 deg. Same exclusion pacejka_fit uses. Curves cross the gap smoothly.
base  = (abs(D.FX./D.FZ) < 0.10) & (Fz > 30) & (abs(D.IA) < 1.5) & (D.P > 9) & (D.P < 13) ...
        & ~(abs(D.SA) > 5.0 & abs(D.SA) < 7.0);
loads = [50 100 150 200 250];
Fzb = []; mzpk = []; tp0 = [];
for k = 1:numel(loads)
    sel = base & (abs(Fz - loads(k)) < loads(k)*0.15);
    if nnz(sel) < 3000, continue; end
    Fzb(end+1)  = mean(Fz(sel));                                    %#ok<AGROW>
    mzpk(end+1) = pctl(abs(D.MZ(sel)), 95);                        %#ok<AGROW> peak |Mz| [lbf-ft]
    % operating-range trail (1-3 deg), not the alpha->0 value: at 0 slip trail
    lo = sel & (abs(D.SA) > 1) & (abs(D.SA) < 3);
    tp0(end+1)  = median(abs(D.MZ(lo)) ./ max(abs(D.FY(lo)),1)) * 12;  %#ok<AGROW> [in]
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

fprintf('\nALIGNING-MOMENT / PNEUMATIC-TRAIL TARGETS  (steering #52-54)\n');
fprintf('  front-outer design load : %.0f lbf  (axle_grip @ %d m/s, low-speed limit)\n', ...
        Fz_fo_lbf, V_LOW);
if Fz_fo_lbf > edge
    fprintf('     -> %+.0f%% past the furthest measured load (%.0f lbf); fit anchored to %.0f lbf.\n', ...
            gap_pct, edge, edge_fit);
    fprintf('        Mz is smooth/monotonic - short reach. For a conservative rack you can\n');
    fprintf('        instead take the peak measured Mz + margin and skip extrapolation.\n');
end
fprintf('  (diag) operating pneumatic trail @1-3deg: %.1f mm  (lambda_t=%.1f) - diagnostic only.\n', ...
        trail_mm, LAMBDA_T);
fprintf('        Not a delivered target: the caster chart below uses the full measured\n');
fprintf('        trail(alpha) curve, so this single-angle value feeds nothing downstream.\n');
fprintf('  T-MZ  peak self-align torque  : %.1f-%.1f N*m/tire  (track band: mu_derate..lambda_Ca)\n', ...
        Mz_lo_Nm, Mz_hi_Nm);
fprintf('        belt (upper bound, size rack to it): %.1f N*m/tire\n', Mz_belt*NM_PER_FTLB);
fprintf('  Note: pneumatic trail collapses to ~0 at the limit -> at-limit aligning\n');
fprintf('        torque is mechanical trail (caster) only. Pick caster so mechanical\n');
fprintf('        trail alone gives limit feel; total trail sub-limit is not too heavy.\n');

% --- T-CAS: caster / mechanical-trail design chart ----------------------
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

fprintf('  T-CAS caster / mechanical-trail DESIGN CHART (VD gives the transfer;\n');
fprintf('        steering picks the value vs its effort budget once the ratio is set):\n');
fprintf('        caster  mech.trail   peak-torq  limit-torq  limit/peak\n');
fprintf('        [deg]     [mm]         [N*m]       [N*m]        [%%]\n');
for i = 1:numel(cas)
    fprintf('        %4.0f   %6.1f      %7.1f     %7.1f      %6.0f\n', ...
            cas(i), tmm(i), Mpk(i), Mlim(i), limfrac(i));
end
fprintf('     -> practice-consistent start: %.0f-%.0f deg caster (= %.0f-%.0f mm mech trail),\n', ...
        REC(1), REC(2), rmm(1), rmm(2));
fprintf('        %.0f-%.0f N*m/front tire peak. Both are below peak pneumatic trail (%.0f mm),\n', ...
        out.Mtire_Nm_rec(1), out.Mtire_Nm_rec(2), tp_peak_mm);
fprintf('        so mechanical trail complements - does not overpower - the tire''s own (FSAE\n');
fprintf('        guidance, no power steering). Finalize against the steering effort budget.\n');

try
    caster_plot(alc, Fyc, tpc, [0 rmm(1) rmm(2)], N_PER_LBF, here);
    fprintf('  Plot written: plots/caster_target.png\n');
catch ce
    fprintf('  [caster plot skipped: %s]\n', ce.message);
end

try
    make_plot(p, D, base, loads, out, Fz_fo_lbf, edge);
    fprintf('  Plot written: plots/aligning_moment.png\n');
catch e
    fprintf('  [plot skipped: %s]\n', e.message);
end
end

function [al, Fy, tp] = curve_at_load(D, base, load)
% Fy [lbf] and pneumatic trail [mm] vs |slip| at one load bin (curve shape).
Fz  = -D.FZ;
sel = base & (abs(Fz - load) < load*0.15);
a = D.SA(sel); fy = -D.FY(sel).*sign(a); mz = D.MZ(sel).*sign(a); aa = abs(a);
edges = 0.25:0.5:12.25; al = []; Fy = []; tp = [];
for i = 1:numel(edges)-1
    m = (aa >= edges(i)) & (aa < edges(i+1));
    if nnz(m) > 40
        al(end+1,1) = mean(aa(m));                              %#ok<AGROW>
        Fy(end+1,1) = median(fy(m));                            %#ok<AGROW>
        tp(end+1,1) = median(mz(m)./max(fy(m),1)) * 12 * 25.4;  %#ok<AGROW>
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
Fz = -D.FZ;
load_c = parula(numel(loads)+1); load_c = load_c(1:end-1, :);
f = figure('Visible','off','Position',[80 80 1200 470],'Color','w');

% left: pneumatic trail vs slip (the collapse) per load
ax1 = subplot(1,2,1); hold(ax1,'on'); grid(ax1,'on');
for k = 1:numel(loads)
    sel = base & (abs(Fz-loads(k)) < loads(k)*0.15);
    if nnz(sel) < 3000, continue; end
    [sa, tp] = trail_curve(D.SA(sel), D.MZ(sel), D.FY(sel));
    plot(ax1, sa, tp*25.4, 'o-', 'MarkerSize', 3, 'Color', load_c(k,:), ...
         'DisplayName', sprintf('%d lbf', loads(k)));
end
xlabel(ax1,'slip angle |\alpha| [deg]'); ylabel(ax1,'pneumatic trail t_p [mm]');
title(ax1,'Trail collapses toward the limit'); legend(ax1,'Location','northeast','FontSize',8);
ylim(ax1,[0 inf]);

% right: peak Mz(Fz) + trail(Fz) with fit and the extrapolated design point
ax2 = subplot(1,2,2); hold(ax2,'on'); grid(ax2,'on');
fq = linspace(45, max(Fz_fo,edge)+10, 60);
yyaxis(ax2,'left');
plot(ax2, o.Fzb, o.mzpk*1.35582, 'o', 'MarkerSize',6, 'MarkerFaceColor',[0 .45 .70], 'Color',[0 .45 .70]);
plot(ax2, fq, polyval(o.mz_coef,fq)*1.35582, '-', 'LineWidth',1.8, 'Color',[0 .45 .70]);
ylabel(ax2,'peak M_z [N\cdotm]');
yyaxis(ax2,'right');
plot(ax2, o.Fzb, o.tp0*25.4, 's', 'MarkerSize',6, 'MarkerFaceColor',[.85 .33 .10], 'Color',[.85 .33 .10]);
plot(ax2, fq, polyval(o.tp_coef,fq)*25.4, '-', 'LineWidth',1.8, 'Color',[.85 .33 .10]);
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
a = abs(sa); mza = abs(mz); fya = abs(fy);
edges = 0.25:0.5:12.25; x = []; y = [];
for i = 1:numel(edges)-1
    m = (a >= edges(i)) & (a < edges(i+1));
    if nnz(m) > 40
        x(end+1,1) = mean(a(m));                              %#ok<AGROW>
        y(end+1,1) = median(mza(m) ./ max(fya(m),1)) * 12;    %#ok<AGROW> [in]
    end
end
end

function D = load_mz(data_dir, pattern)
channels = {'SA','FY','FX','FZ','IA','P','MZ'};
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
