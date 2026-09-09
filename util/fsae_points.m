function [total, brk] = fsae_points(ev, bm, opts)
% FSAE_POINTS  Dynamic-event scoring. The ONE home for the points model.
%   [total, brk] = fsae_points(ev, bm)
%   [total, brk] = fsae_points(ev, bm, opts)
%
% WHY THIS FILE EXISTS
% --------------------
% The scoring formulas lived as local functions inside run_aero_targets.m, so
% they were reachable from exactly one script. Any other study that wanted to
% state a result in POINTS - the pack sizing study, the gear freeze - had to
% either copy the formulas or stop at seconds and let the reader convert.
%
% Copies drift. The moment two scripts hold their own tscore, the aero answer
% and the pack answer stop being comparable and nobody notices, because both
% still print a plausible number. Same reasoning as vd_const and vd_derive:
% one home, or no home at all.
%
% The QSS haircut lives here too. It is not a scoring rule - it is the standing
% correction for the lap sim's optimism (perfect driver, no tyre wear, no
% traffic) - but it must be applied identically everywhere or the studies stop
% agreeing. Pass RAW simulated times; this function applies it.
%
% INPUTS
%   ev   event times in SECONDS, before the haircut:
%          ev.accel          75 m standing start
%          ev.skidpad        one timed skidpad lap
%          ev.autocross      one autocross run
%          ev.endurance_lap  one endurance lap        (with ev.laps)
%          ev.laps           lap count for scoring    (default bm-consistent 22)
%          ev.energy_kWh     energy consumed over ev.laps laps
%        Any field omitted or NaN scores NaN, is named in brk.missing, and is
%        left OUT of total. A partial score is honest; a silently-short total
%        pretending to be complete is not.
%
%   bm   benchmark struct from read_benchmarks (the real 2026 Tmins)
%
%   opts .haircut    sprint-time inflation vs QSS      (default 0.08)
%        .ef_max     best real efficiency factor       (default 0.60) PROVISIONAL
%        .e_min_kWh  lowest real 22-lap finisher energy(default 2.696)
%
% OUTPUTS
%   total  sum of the events that actually scored
%   brk    .accel .skidpad .autocross .endurance .efficiency  per-event points
%          .t_used   the post-haircut times actually scored (report these, not
%                    the raw inputs, or your printed times won't match the score)
%          .missing  cellstr of events that could not be scored
%          .complete true only when all five scored
%
% Event weights are the FSAE dynamic allocation: accel 100, skidpad 75,
% autocross 125, endurance 275, efficiency 100. The min/variable split within
% each is the rulebook's; the Tmins are real 2026 results, not our own times.

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'haircut'),   opts.haircut   = 0.08;  end
if ~isfield(opts, 'ef_max'),    opts.ef_max    = 0.60;  end
if ~isfield(opts, 'e_min_kWh'), opts.e_min_kWh = 2.696; end
if ~isfield(ev,   'laps'),      ev.laps        = 22;    end

g = @(f) local_get(ev, f);

brk = struct('accel', NaN, 'skidpad', NaN, 'autocross', NaN, ...
             'endurance', NaN, 'efficiency', NaN);
brk.t_used = struct();
missing = {};

% --- Sprint events: full haircut ---------------------------------------
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

% --- Endurance: HALF haircut. The pace is capped by the power limit, so the
% driver-and-conditions optimism the haircut corrects for is largely absent.
t = g('endurance_lap');
t_tot = NaN;
if ~isnan(t)
    t_tot = t * ev.laps * (1 + opts.haircut*0.5);
    brk.t_used.endurance_total = t_tot;
    brk.endurance = tscore(t_tot, bm.endurance_tmin_s, 1.45, 250.0, 25.0, false);
else, missing{end+1} = 'endurance'; end

% --- Efficiency: needs BOTH a time and an energy, and it moves the opposite
% way to endurance - a bigger pack lets you run a higher cap, which burns more
% energy, which costs efficiency points. Scoring one without the other is how
% a study talks itself into a pack it does not need.
E = g('energy_kWh');
if ~isnan(t_tot) && ~isnan(E)
    ef = (bm.endurance_tmin_s / t_tot) * (opts.e_min_kWh / max(E, opts.e_min_kWh));
    brk.efficiency = min(100, max(0, 100 * (ef - 0.1) / (opts.ef_max - 0.1)));
    brk.ef = ef;
else, missing{end+1} = 'efficiency'; end

vals = [brk.accel brk.skidpad brk.autocross brk.endurance brk.efficiency];
total = sum(vals(~isnan(vals)));
brk.missing  = missing;
brk.complete = isempty(missing);
brk.opts     = opts;
end


function s = tscore(t, tmin, fmax, pvar, pmin, squared)
% FSAE event score. Tmin floors at OUR time - if we are quicker than the real
% best, we would BE the benchmark, so the formula must not hand out more than
% the maximum. Beyond tmax = fmax*tmin the event scores its minimum only.
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
