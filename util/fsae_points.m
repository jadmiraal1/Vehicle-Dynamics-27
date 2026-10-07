function [total, brk] = fsae_points(ev, bm, opts)
% FSAE_POINTS  Dynamic-event points from simulated times. The one scoring model.
%   [total, brk] = fsae_points(ev, bm)
%   [total, brk] = fsae_points(ev, bm, opts)
%
% INPUTS
%   ev   simulated times in SECONDS, before the haircut:
%          ev.accel          75 m from rest
%          ev.skidpad        one timed skidpad lap
%          ev.autocross      one autocross run
%          ev.endurance_lap  one endurance lap
%          ev.laps           endurance lap count (default bm.endurance_laps, else 22)
%          ev.energy_kWh     energy used over ev.laps laps
%          ev.lap_km         endurance lap length [km] (needed for efficiency)
%        A field that is missing or NaN scores NaN, is listed in brk.missing
%        and is left out of total.
%   bm   real competition results from read_benchmarks (the Tmin values)
%   opts .haircut    sprint-time inflation applied to the QSS times (default 0.08)
%        .ef_max     best efficiency factor in the field (default 0.60) PROVISIONAL
%        .e_min_kWh  lowest finisher energy over bm.endurance_laps (default 2.696)
%
% OUTPUTS
%   total  sum of the events that scored
%   brk    per-event points (.accel .skidpad .autocross .endurance .efficiency),
%          .t_used (the post-haircut times actually scored), .ef, .ef_min,
%          .missing, .complete (true only when all five scored)
%
% Scoring follows FSAE Rules 2026 D.9-D.13 (see fsae_rules):
%   accel      95.5 * (Tmax/T - 1)/(Tmax/Tmin - 1) + 4.5,          Tmax = 1.50 Tmin
%   skidpad    71.5 * ((Tmax/T)^2 - 1)/((Tmax/Tmin)^2 - 1) + 3.5,  Tmax = 1.25 Tmin
%   autocross  118.5 * (Tmax/T - 1)/(Tmax/Tmin - 1) + 6.5,         Tmax = 1.45 Tmin
%   endurance  250 * (Tmax/T - 1)/(Tmax/Tmin - 1),                 Tmax = 1.45 Tmin,
%              plus up to 25 for laps completed (taken as 25 here: the
%              rules give no formula, and every lap is assumed finished)
%   efficiency 100 * (EF - EF_min)/(EF_max - EF_min), where
%              EF = (Tmin/T)_per lap * (CO2min/CO2)_per lap, and EF_min is EF at
%              T = 1.45 Tmin with the reference 20.02 kg CO2/100 km (D.13.4.5).
%              Zero if energy per lap exceeds that reference or the mean lap
%              is slower than 1.45x the fastest (D.13.3).
% Tmin is the real benchmark, floored at our own time: if we are quicker, we
% set Tmin. The haircut corrects the QSS optimism (perfect driver, no
% traffic); endurance gets half of it because its pace is power-capped.

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'haircut'),   opts.haircut   = 0.08;  end
if ~isfield(opts, 'ef_max'),    opts.ef_max    = 0.60;  end
if ~isfield(opts, 'e_min_kWh'), opts.e_min_kWh = 2.696; end
bm_laps = 22;
if isfield(bm, 'endurance_laps'), bm_laps = bm.endurance_laps; end
if ~isfield(ev, 'laps'), ev.laps = bm_laps; end

g = @(f) local_get(ev, f);

brk = struct('accel', NaN, 'skidpad', NaN, 'autocross', NaN, ...
             'endurance', NaN, 'efficiency', NaN);
brk.t_used = struct();
missing = {};

% --- sprint events: full haircut ------------------------------------------
t = g('accel');
if ~isnan(t)
    brk.t_used.accel = t * (1 + opts.haircut);
    brk.accel = tscore(brk.t_used.accel, bm.accel_tmin_s, 1.50, 95.5, 4.5, false);
else, missing{end+1} = 'accel'; end

t = g('skidpad');
if ~isnan(t)
    brk.t_used.skidpad = t * (1 + opts.haircut);
    brk.skidpad = tscore(brk.t_used.skidpad, bm.skidpad_tmin_s, 1.25, 71.5, 3.5, true);
else, missing{end+1} = 'skidpad'; end

t = g('autocross');
if ~isnan(t)
    brk.t_used.autocross = t * (1 + opts.haircut);
    brk.autocross = tscore(brk.t_used.autocross, bm.autocross_tmin_s, 1.45, 118.5, 6.5, false);
else, missing{end+1} = 'autocross'; end

% --- endurance: half haircut ----------------------------------------------
t = g('endurance_lap');
t_tot = NaN;
if ~isnan(t)
    t_tot = t * ev.laps * (1 + opts.haircut*0.5);
    brk.t_used.endurance_total = t_tot;
    brk.endurance = tscore(t_tot / ev.laps * bm_laps, bm.endurance_tmin_s, ...
                           1.45, 250.0, 25.0, false);
else, missing{end+1} = 'endurance'; end

% --- efficiency (needs time, energy and lap length) -------------------------
% It moves opposite to endurance: a bigger pack runs a higher power cap,
% which uses more energy and costs efficiency points.
E = g('energy_kWh');  L_km = g('lap_km');
if ~isnan(t_tot) && ~isnan(E) && ~isnan(L_km)
    R = fsae_rules();
    t_lap_min = bm.endurance_tmin_s / bm_laps;          % per-lap terms
    e_lap_min = opts.e_min_kWh / bm_laps;
    t_lap = t_tot / ev.laps;
    e_lap = E / ev.laps;
    e_lap_ref = R.co2_ref_kg_per_km * L_km / R.co2_kg_per_kWh;   % kWh per lap
    ef     = min(t_lap_min / t_lap, 1) * (e_lap_min / max(e_lap, e_lap_min));
    ef_min = (1 / R.ef_min_time_factor) * (e_lap_min / e_lap_ref);
    if e_lap > e_lap_ref || t_lap > R.ef_min_time_factor * t_lap_min
        brk.efficiency = 0;
    else
        brk.efficiency = min(100, max(0, 100 * (ef - ef_min) / (opts.ef_max - ef_min)));
    end
    brk.ef = ef;  brk.ef_min = ef_min;
else, missing{end+1} = 'efficiency'; end

vals = [brk.accel brk.skidpad brk.autocross brk.endurance brk.efficiency];
total = sum(vals(~isnan(vals)));
brk.missing  = missing;
brk.complete = isempty(missing);
brk.opts     = opts;
end


function s = tscore(t, tmin, fmax, pvar, pmin, squared)
% Points for time t against the benchmark tmin. Beyond tmax = fmax*tmin the
% event scores pmin only.
tmin = min(t, tmin);
tmax = fmax * tmin;
t    = min(t, tmax);
if squared, ratio = (tmax/t)^2 - 1;  rmax = (tmax/tmin)^2 - 1;
else,       ratio =  tmax/t    - 1;  rmax =  tmax/tmin    - 1;
end
s = pvar * ratio / rmax + pmin;
end


function v = local_get(ev, f)
if isfield(ev, f) && ~isempty(ev.(f)) && isfinite(ev.(f)), v = ev.(f); else, v = NaN; end
end
