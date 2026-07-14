function R = ttc_fit()
% TTC_FIT  Peak tire grip per candidate tire from FSAE TTC data (R8/R9;
% the 18in LC0 drive/brake donor is R6 - see tireid/source fields).
% Method, units, and sign conventions: VD_physics_reference.md, section 4.
%
% ROLE CHANGED (Jul 2026): this is now a TIRE-SCREENING tool, not a source of
% design values. It reads the 99th-percentile UPPER ENVELOPE of a noisy point
% cloud, which ran ~10% high in mu_y and ~16% high in mu_x against the fitted
% median curve. Design grip now comes from pacejka_fit via build_tire_coeffs.
% Keep using this for what percentiles are good at: comparing tires on equal
% terms, and locating the camber / pressure windows.
%
% vd_selftest still anchors on this function's LC0 output (2.602) as a check
% that the TTC data path itself has not moved.

p = vehicle_params('bootstrap');   % car mass only; must not require the
                                   % artifact that this fit helps produce

% Design assumptions
R.derate      = p.mu_derate;        % grip scaling factor (single source: vehicle_params)
N_PER_LBF     = 4.44822;
Fz_design_N   = p.m * p.g / 4;      % static per-corner load [N]
Fz_design_lbf = Fz_design_N / N_PER_LBF;
LOAD_BAND     = 0.12;               % +/-12% band around design load

camber_sweep_deg   = [0 2 4];
pressure_sweep_psi = [8 10 12 14];

here     = fileparts(mfilename('fullpath'));
data_dir = fullfile(here, 'TTC_Data');

fprintf('\nTTC FIT  (scaling factor %.2f, design corner load %.0f N = %.0f lbf)\n', ...
        R.derate, Fz_design_N, Fz_design_lbf);
fprintf('%-8s %8s %8s %8s %7s\n', 'Tire', 'rawMuY', 'derated', 'bestIA', 'bestP');

tire_names = {'LC0_16x75', 'R20_16x75', 'R20_18x60', 'GY_18x65'};

for k = 1:numel(tire_names)
    tire = tire_names{k};
    D = load_tire_data(data_dir, [tire '_*.mat']);
    Fz_mag = -D.FZ;                                    % SAE: FZ < 0 under load

    % Pure-slip lateral samples in the design load band
    pure_slip      = (abs(D.FX ./ D.FZ) < 0.10) & (Fz_mag > 5);
    at_design_load = pure_slip & (abs(Fz_mag - Fz_design_lbf) < Fz_design_lbf * LOAD_BAND);

    mu_y_raw = peak_friction(D.FY, D.FZ, at_design_load);

    % Camber / pressure windows at design load
    mu_vs_camber = arrayfun(@(ia) ...
        peak_friction(D.FY, D.FZ, at_design_load & abs(D.IA - ia) < 1.0), camber_sweep_deg);
    mu_vs_pressure = arrayfun(@(press) ...
        peak_friction(D.FY, D.FZ, at_design_load & abs(D.P - press) < 1.0), pressure_sweep_psi);

    [~, i_cam] = max(mu_vs_camber);
    [~, i_prs] = max(mu_vs_pressure);

    R.(tire) = struct( ...
        'mu_y_raw',     mu_y_raw, ...
        'mu_y_derated', mu_y_raw * R.derate, ...
        'camber',       mu_vs_camber, ...
        'pressure',     mu_vs_pressure, ...
        'bestIA',       camber_sweep_deg(i_cam), ...
        'bestP',        pressure_sweep_psi(i_prs));

    fprintf('%-8s %8.3f %8.3f %7d%s %6dp\n', tire, mu_y_raw, ...
            mu_y_raw * R.derate, R.(tire).bestIA, char(176), R.(tire).bestP);
end

% Longitudinal mu from the only drive/brake file (near-zero slip angle, wider band)
D = load_tire_data(data_dir, 'LC0_18x60_*.mat');
Fz_mag         = -D.FZ;
pure_slip      = (abs(D.SA) < 1.0) & (Fz_mag > 5);
at_design_load = pure_slip & (abs(Fz_mag - Fz_design_lbf) < Fz_design_lbf * 0.30);

mu_x_raw = peak_friction(D.FX, D.FZ, at_design_load);
R.mu_x   = struct('raw', mu_x_raw, 'derated', mu_x_raw * R.derate);

fprintf('\nmu_x (LC0_18x60 drive/brake): raw %.3f -> derated %.3f  (cross-tire proxy)\n', ...
        mu_x_raw, mu_x_raw * R.derate);
fprintf('SCREENING ONLY. Design grip comes from build_tire_coeffs, not from here.\n');

try
    make_plot(R, tire_names, camber_sweep_deg, pressure_sweep_psi);
    fprintf('Plot written: ttc_fit.png\n');
catch e
    fprintf('[plot skipped: %s]\n', e.message);
end
end


function D = load_tire_data(data_dir, pattern)
% Concatenate channels from all non-raw files matching pattern; missing -> NaN.
channels = {'FY', 'FZ', 'FX', 'IA', 'P', 'SA'};
files    = dir(fullfile(data_dir, pattern));
chunks   = cell(numel(files), numel(channels));

for i = 1:numel(files)
    if contains(files(i).name, 'raw'), continue; end
    S = load(fullfile(data_dir, files(i).name));
    if ~isfield(S, 'FZ'), continue; end
    n = numel(S.FZ);
    for c = 1:numel(channels)
        chunks{i, c} = get_channel(S, channels{c}, n);
    end
end

for c = 1:numel(channels)
    D.(channels{c}) = vertcat(chunks{:, c});
end
end


function v = get_channel(S, name, n)
if isfield(S, name) && numel(S.(name)) == n
    v = S.(name)(:);
else
    v = nan(n, 1);
end
end


function mu = peak_friction(F, FZ, mask)
% Noise-robust peak |F/FZ|: 99th percentile; NaN if < 60 valid samples.
MIN_SAMPLES = 60;
fz = FZ(mask);
f  = F(mask);
loaded = (-fz) > 5;
fz = fz(loaded);
f  = f(loaded);
if numel(fz) < MIN_SAMPLES
    mu = NaN;
    return
end
mu = linear_percentile(abs(f ./ fz), 99);
end


function v = linear_percentile(x, q)
% Linear-interpolation percentile (numpy default); no toolbox needed.
x = sort(x(~isnan(x)));
n = numel(x);
if n == 0, v = NaN;  return; end
if n == 1, v = x(1); return; end
rank = q/100 * (n - 1);
lo   = floor(rank);
frac = rank - lo;
v    = x(lo+1) + frac * (x(min(lo+2, n)) - x(lo+1));
end


function make_plot(R, tire_names, camber_sweep_deg, pressure_sweep_psi)
colors = {[0.09 0.71 0.79], [0.88 0.57 0.10], [0.12 0.62 0.45], [0.75 0.23 0.17]};
f = figure('Visible', 'off', 'Position', [100 100 980 760]);

subplot(2, 2, 1); hold on;                      % camber window
for k = 1:numel(tire_names)
    plot(camber_sweep_deg, R.(tire_names{k}).camber * R.derate, 'o-', 'Color', colors{k});
end
xlabel('inclination angle [deg]'); ylabel('scaled peak \mu_y');
title('Camber window @ car load'); grid on;
legend(tire_names, 'Location', 'best');

subplot(2, 2, 2); hold on;                      % pressure window
for k = 1:numel(tire_names)
    plot(pressure_sweep_psi, R.(tire_names{k}).pressure * R.derate, 'o-', 'Color', colors{k});
end
xlabel('pressure [psi]'); ylabel('scaled peak \mu_y');
title('Pressure window @ car load'); grid on;

subplot(2, 2, [3 4]);                           % design grip comparison
mu_design = cellfun(@(t) R.(t).mu_y_derated, tire_names);
b = bar(mu_design);
b.FaceColor = 'flat';
for k = 1:numel(tire_names)
    b.CData(k, :) = colors{k};
    text(k, mu_design(k) + 0.01, sprintf('%.2f', mu_design(k)), ...
         'HorizontalAlignment', 'center');
end
set(gca, 'XTickLabel', tire_names); ylabel('design \mu_y (scaled)');
title('Design grip comparison (SCREENING ONLY - not design values)');
outdir = fullfile(fileparts(mfilename('fullpath')), 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'ttc_fit.png'));
close(f);
end
