"""
============================================================================
 TR26 FSAE EV Lap Simulator  —  single-file tool
============================================================================
Point-mass quasi-steady-state lap sim with a validated Hoosier 16x7.5-10 LC0
tire model, real FSAE Michigan 2026 track digitization, slalom injection,
endurance energy/strategy, and a competition points model.

USAGE (command line):
  python tr26_sim.py tracks      # digitize endurance+autocross maps -> CSVs + figure
  python tr26_sim.py events      # accel / skidpad / autocross / endurance times
  python tr26_sim.py strategy    # power-derate x regen energy sweep vs pack
  python tr26_sim.py points      # full points model + design sensitivities
  python tr26_sim.py animate     # animated lap replay GIF
  python tr26_sim.py all         # tracks -> events -> strategy -> points

  Optional flags (override placeholders without editing the file):
    --mass-lb 650         total mass with driver [lb]
    --pack-kwh 5.5        usable accumulator energy [kWh]
    --power-kw 80         power cap for sprints [kW]
    --tire-F 0.66         track grip factor (calibrate from skidpad)
    --no-regen            disable regen in energy/points

FIRST-TIME SETUP:
  pip install numpy scipy matplotlib scikit-image pillow
  Put the two track map PNGs in ./maps/ (endurance.png, autocross.png)
  Run:  python tr26_sim.py all --mass-lb 650 --pack-kwh 5.5

CALIBRATION (do this once you have real test data):
  1. Run skidpad on TR26, then set --tire-F so sim matches measured time.
  2. Confirm mass, pack capacity, rev limit, regen with subteams.
  Everything marked PLACEHOLDER below should become a measured number.
============================================================================
"""
import argparse
import os
import numpy as np
from dataclasses import dataclass, field, replace

LB2N = 4.4482216
G = 9.80665
RHO = 1.225          # kg/m^3
FT = 0.3048
MPH = 2.2369363

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, 'maps')
OUT = os.path.join(HERE, 'output')
os.makedirs(OUT, exist_ok=True)

# ===========================================================================
# TIRE MODEL — Hoosier 16x7.5-10 LC0 (teammate's TTC fit; FY validated R2=0.985
# against Calspan belt data. FX provenance unverified — no 16" drive/brake data.)
# Raw model units: SA [deg], FZ [lb, NEGATIVE per SAE], SL [-], IA [deg, camber]
# ===========================================================================
def _lc0_fy_lb(SA, FZ_lb, IA, F):
    assert np.all(FZ_lb < 0), "FZ must be negative (SAE convention)"
    b1 = -0.0054 * IA + 0.178533
    c1 = 0.02765 * IA + 1.3912
    d1 = 0.0362 * IA**2 - 0.05685 * IA - 8.4978
    e1 = 0.001975 * IA + 0.34625
    f1 = 1.18188 * IA - 6.27592
    return F * d1 * np.sin(c1 * np.arctan(b1 * SA - e1 * (b1 * SA - np.arctan(b1 * SA)))) * ((-FZ_lb)**0.775) + f1

def _lc0_fx_lb(SL, FZ_lb, IA, F):
    assert np.all(FZ_lb < 0), "FZ must be negative (SAE convention)"
    b3 = 0.348325 * IA + 9.59392
    c3 = -0.074825 * IA + 1.70918
    d3 = -0.064875 * IA**2 + 0.20785 * IA + 7.7488
    e3 = 0.021025 * IA + 0.341883
    f3 = 0.224225 * IA + 1.94952
    return F * d3 * np.sin(c3 * np.arctan(b3 * SL - e3 * (b3 * SL - np.arctan(b3 * SL)))) * ((-FZ_lb)**0.775) + f3

def build_mu_tables(F_track=0.66, IA=1.0, fz_lb=np.linspace(30, 400, 60)):
    """Peak lateral/longitudinal friction coefficient vs vertical load, per tire."""
    sa = np.linspace(-12, 12, 241)
    sl = np.linspace(-0.25, 0.25, 251)
    mu_y, mu_x = [], []
    for fz in fz_lb:
        mu_y.append(np.max(np.abs(_lc0_fy_lb(sa, -fz, IA, F_track))) / fz)
        mu_x.append(np.max(np.abs(_lc0_fx_lb(sl, -fz, IA, F_track))) / fz)
    return fz_lb * LB2N, np.array(mu_y), np.array(mu_x)

# ===========================================================================
# VEHICLE
# ===========================================================================
@dataclass
class Vehicle:
    mass: float = 295.0          # kg total w/ driver (~650 lb; PLACEHOLDER - verify!)
    cl_a: float = 2.91           # m^2 downforce coeff*area (CFD)
    cd_a: float = 0.995          # m^2 drag coeff*area (CFD)
    aero_corr: float = 0.85      # CFD-to-track correlation factor
    crr: float = 0.015           # rolling resistance coeff
    t_peak: float = 240.0        # Nm motor peak torque (Emrax 228)
    p_cap: float = 80e3          # W power cap at accumulator (FSAE rules max)
    eta_dt: float = 0.92         # battery -> wheel efficiency
    rev_limit_rpm: float = 5500  # PLACEHOLDER - confirm motor/inverter limit
    gear: float = 3.82           # final drive ratio
    r_wheel: float = 0.20        # m loaded radius
    wheelbase: float = 1.55      # m (PLACEHOLDER)
    cg_h: float = 0.28           # m CG height (PLACEHOLDER)
    rear_wt_frac: float = 0.53   # static rear weight fraction (PLACEHOLDER)
    tire_F: float = 0.66         # track grip derate vs TTC belt (CALIBRATE)
    # regen (PLACEHOLDERS - confirm with accumulator/powertrain teams)
    regen_frac: float = 0.30     # fraction of braking force on regen (rear) axle
    regen_eff: float = 0.85      # wheel -> pack efficiency
    regen_pcap: float = 30e3     # W max regen power into pack
    _mu: tuple = field(default=None, repr=False)

    def mu_tables(self):
        if self._mu is None:
            object.__setattr__(self, '_mu', build_mu_tables(self.tire_F))
        return self._mu

    def mu_y(self, fz_n):
        fz, my, _ = self.mu_tables(); return np.interp(fz_n, fz, my)

    def mu_x(self, fz_n):
        fz, _, mx = self.mu_tables(); return np.interp(fz_n, fz, mx)

    @property
    def v_max(self):
        return self.rev_limit_rpm * 2 * np.pi / 60 / self.gear * self.r_wheel

    def downforce(self, v): return 0.5 * RHO * self.cl_a * self.aero_corr * v**2
    def drag(self, v):      return 0.5 * RHO * self.cd_a * self.aero_corr * v**2

    def wheel_force_powertrain(self, v):
        if v >= self.v_max: return 0.0
        omega_m = max(v, 0.1) / self.r_wheel * self.gear
        t_m = min(self.t_peak, self.p_cap * self.eta_dt / omega_m)
        return t_m * self.gear / self.r_wheel

    def ay_max(self, v):
        n_total = self.mass * G + self.downforce(v)
        return self.mu_y(n_total / 4) * n_total / self.mass

    def ax_brake_max(self, v):
        n_total = self.mass * G + self.downforce(v)
        return self.mu_x(n_total / 4) * n_total / self.mass + self.drag(v) / self.mass

    def ax_drive_max(self, v):
        ax = 2.0
        for _ in range(8):
            n_rear = (self.mass * G * self.rear_wt_frac + self.downforce(v) * 0.5
                      + self.mass * ax * self.cg_h / self.wheelbase)
            f = min(self.mu_x(n_rear / 2) * n_rear, self.wheel_force_powertrain(v))
            ax = (f - self.drag(v) - self.crr * self.mass * G) / self.mass
        return max(ax, 0.0)

# ===========================================================================
# QSS SOLVER
# ===========================================================================
def corner_speed(veh, kappa):
    if kappa < 1e-6: return veh.v_max
    lo, hi = 0.5, veh.v_max
    if hi**2 * kappa <= veh.ay_max(hi): return hi
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if mid**2 * kappa <= veh.ay_max(mid): lo = mid
        else: hi = mid
    return lo

def solve_lap(veh, s, kappa, v0=None, closed=True):
    """s [m], kappa [1/m] -> (v[m/s], laptime[s]). closed=True wraps the lap."""
    n = len(s); ds = np.diff(s)
    vlim = np.array([corner_speed(veh, k) for k in kappa])
    v = vlim.copy()
    if v0 is not None: v[0] = v0
    for _ in range(3 if closed else 1):
        for i in range(n - 1):  # forward / accelerate
            frac = np.sqrt(max(0.0, 1 - min(v[i]**2 * kappa[i] / max(veh.ay_max(v[i]), 1e-6), 1.0)**2))
            ax = veh.ax_drive_max(v[i]) * frac
            v[i + 1] = min(vlim[i + 1], np.sqrt(max(v[i]**2 + 2 * ax * ds[i], 0)))
        for i in range(n - 1, 0, -1):  # backward / brake
            frac = np.sqrt(max(0.0, 1 - min(v[i]**2 * kappa[i] / max(veh.ay_max(v[i]), 1e-6), 1.0)**2))
            ax = veh.ax_brake_max(v[i]) * frac
            v[i - 1] = min(v[i - 1], np.sqrt(v[i]**2 + 2 * ax * ds[i - 1]))
        if closed: v[0] = v[-1]
        elif v0 is not None: v[0] = v0
    vm = 0.5 * (v[:-1] + v[1:])
    return v, np.sum(ds / np.maximum(vm, 0.1))

def lap_energy(veh, s, k, v, regen):
    """Net accumulator energy per lap [kWh]."""
    ds = np.diff(s); e_out = e_in = 0.0
    for i in range(len(ds)):
        vm = max(0.5 * (v[i] + v[i + 1]), 0.1)
        ax = (v[i + 1]**2 - v[i]**2) / (2 * ds[i])
        f_wheel = veh.mass * ax + veh.drag(vm) + veh.crr * veh.mass * G
        dt = ds[i] / vm
        if f_wheel > 0:
            e_out += f_wheel * vm / veh.eta_dt * dt
        elif regen:
            e_in += min(-f_wheel * vm * veh.regen_frac * veh.regen_eff, veh.regen_pcap) * dt
    return (e_out - e_in) / 3.6e6

# ===========================================================================
# TRACK DIGITIZER (image -> centerline -> curvature) + slalom injection
# ===========================================================================
M_PER_PX = (50.0 / 23.0) * FT     # 50 ft paddock grid measured at 23 px
KAPPA_MAX = 1 / 4.5               # tightest physical FSAE corner ~4.5 m
WEAVE_A = 0.85                    # m, slalom lateral weave amplitude

def _extract_path(fn, closed_loop):
    from PIL import Image
    from skimage.morphology import skeletonize, closing, disk
    from skimage.measure import label
    from scipy.spatial import cKDTree
    img = np.array(Image.open(fn).convert('RGB')).astype(int)
    R, Gc, B = img[..., 0], img[..., 1], img[..., 2]
    mask = (R > 120) & (R - Gc > 50) & (R - B > 50)
    lab = label(closing(mask, disk(2)))
    sizes = np.bincount(lab.ravel()); sizes[0] = 0
    skel = skeletonize(lab == sizes.argmax())
    ys, xs = np.nonzero(skel)
    pts = np.column_stack([xs, ys]).astype(float)
    start = 0
    if not closed_loop:
        from scipy.ndimage import convolve
        nbr = convolve(skel.astype(int), np.ones((3, 3)), mode='constant')
        ends = np.column_stack(np.nonzero(skel & (nbr == 2)))
        if len(ends):
            ey, ex = ends[np.argmin(ends[:, 1])]
            start = int(np.argmin((pts[:, 0] - ex)**2 + (pts[:, 1] - ey)**2))
    tree = cKDTree(pts); used = np.zeros(len(pts), bool)
    order = [start]; used[start] = True; i = start
    for _ in range(len(pts) - 1):
        d, idx = tree.query(pts[i], k=14)
        nxt = next((j for dd, j in zip(d, idx) if not used[j] and dd < 6), None)
        if nxt is None: break
        order.append(nxt); used[nxt] = True; i = nxt
    return pts[np.array(order)] * M_PER_PX

def _smooth(path_m, closed_loop, smooth=0.2, ds=1.0):
    from scipy.interpolate import splprep, splev
    x, y = path_m[:, 0], path_m[:, 1]
    if closed_loop:
        x = np.append(x, x[0]); y = np.append(y, y[0])
    tck, _ = splprep([x, y], s=smooth * len(x), per=int(closed_loop))
    u = np.linspace(0, 1, 4000)
    xs, ys = splev(u, tck)
    dx, dy = splev(u, tck, der=1); ddx, ddy = splev(u, tck, der=2)
    kappa = np.minimum(np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5, KAPPA_MAX)
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))])
    sg = np.arange(0, s[-1], ds)
    return sg, np.interp(sg, s, xs), np.interp(sg, s, ys), np.interp(sg, s, kappa)

def _slalom_spans(fn, s, x, y):
    from PIL import Image
    from scipy.spatial import cKDTree
    img = np.array(Image.open(fn).convert('RGB')).astype(int)
    R, Gc, B = img[..., 0], img[..., 1], img[..., 2]
    ys_, xs_ = np.nonzero((R > 120) & (R - Gc > 50) & (R - B > 50))
    red_m = np.column_stack([xs_, ys_]) * M_PER_PX
    d, idx = cKDTree(np.column_stack([x, y])).query(red_m)
    off = (d > 1.8) & (d < 5.5)
    hist, edges = np.histogram(np.sort(s[idx[off]]), bins=np.arange(0, s[-1] + 10, 10))
    dense = hist > 8; spans = []; start = None
    for i, dn in enumerate(dense):
        if dn and start is None: start = edges[i]
        if not dn and start is not None: spans.append([start, edges[i]]); start = None
    if start is not None: spans.append([start, edges[-1]])
    merged = []
    for sp in spans:
        if merged and sp[0] - merged[-1][1] < 25: merged[-1][1] = sp[1]
        else: merged.append(sp)
    out = []
    for s0, s1 in merged:
        if s1 - s0 >= 20:
            im = np.searchsorted(s, 0.5 * (s0 + s1))
            out.append((s0, s1, (x[im], y[im])))
    return out

def _inject(s, kappa, spans, spacing_fn):
    kap = kappa.copy()
    for s0, s1, mid in spans:
        i0, i1 = np.searchsorted(s, s0), np.searchsorted(s, s1)
        d_cone = spacing_fn(mid) * FT
        k_peak = WEAVE_A * (np.pi / d_cone)**2
        ss = s[i0:i1 + 1] - s[i0]
        kap[i0:i1 + 1] = np.maximum(kap[i0:i1 + 1], k_peak * np.abs(np.sin(np.pi * ss / d_cone)))
    return kap

def _spacing_endur(c):
    return 50.0 if (480 < c[0] / M_PER_PX < 1024 and c[1] / M_PER_PX < 50) else 30.0

def digitize(name, fn, closed_loop, spacing_fn):
    """Image -> CSV (s,x,y,kappa with slaloms). Returns (s,x,y,kappa)."""
    path = _extract_path(fn, closed_loop)
    s, x, y, k0 = _smooth(path, closed_loop)
    k = _inject(s, k0, _slalom_spans(fn, s, x, y), spacing_fn)
    np.savetxt(os.path.join(OUT, f'track_{name}.csv'),
               np.column_stack([s, x, y, k]), delimiter=',',
               header='s_m,x_m,y_m,kappa_1perm', comments='')
    return s, x, y, k

def load_track(name):
    t = np.loadtxt(os.path.join(OUT, f'track_{name}.csv'), delimiter=',', skiprows=1)
    return t[:, 0], t[:, 1], t[:, 2], t[:, 3]

# ===========================================================================
# EVENTS
# ===========================================================================
def event_accel(veh, dist=75.0):
    s = np.linspace(0, dist, 751)
    v, t = solve_lap(veh, s, np.zeros_like(s), v0=0.0, closed=False)
    return t, v[-1]

def event_skidpad(veh, r=9.125):
    v = corner_speed(veh, 1 / r)
    return 2 * np.pi * r / v, v**2 / r / G

def event_times(veh, p_cap_endur=None, regen=True):
    s_ax, _, _, k_ax = load_track('autocross')
    s_en, _, _, k_en = load_track('endurance')
    t_acc, _ = event_accel(veh)
    t_skid, _ = event_skidpad(veh)
    _, t_ax = solve_lap(veh, s_ax, k_ax, v0=0.0, closed=False)
    ve = replace(veh, p_cap=(p_cap_endur or veh.p_cap), _mu=veh._mu)
    v_en, t_lap = solve_lap(ve, s_en, k_en, closed=True)
    return dict(accel=t_acc, skidpad=t_skid, autocross=t_ax, endur_lap=t_lap,
                endur_total=22 * t_lap, e_lap=lap_energy(ve, s_en, k_en, v_en, regen))

# ===========================================================================
# POINTS MODEL (standard FSAE dynamic-event formulas; relative to fastest car)
# ===========================================================================
def _tscore(t, tmin, tmax_f, pv, pm, squared=False):
    tmax = tmax_f * tmin; t = min(t, tmax)
    r = (tmax / t)**2 - 1 if squared else (tmax / t) - 1
    rmin = (tmax / tmin)**2 - 1 if squared else (tmax / tmin) - 1
    return pv * r / rmin + pm

def score_team(mine, ref, pack_usable):
    s = {}
    s['accel'] = _tscore(mine['accel'], min(mine['accel'], ref['accel']), 1.5, 95.5, 4.5)
    s['skidpad'] = _tscore(mine['skidpad'], min(mine['skidpad'], ref['skidpad']), 1.25, 71.5, 3.5, squared=True)
    s['autocross'] = _tscore(mine['autocross'], min(mine['autocross'], ref['autocross']), 1.45, 118.5, 6.5)
    e_need = 22 * mine['e_lap']
    tmin_en = min(mine['endur_total'], ref['endur_total'])
    if e_need <= pack_usable:
        s['endurance'] = _tscore(mine['endur_total'], tmin_en, 1.45, 250, 25)
        ef_mine = (tmin_en / mine['endur_total']) * (min(mine['e_lap'], ref['e_lap']) / mine['e_lap'])
        ef_ref = (tmin_en / ref['endur_total']) * (min(mine['e_lap'], ref['e_lap']) / ref['e_lap'])
        ef_max = max(ef_mine, ef_ref)
        s['efficiency'] = max(0.0, 100 * (ef_mine - 0.1) / (ef_max - 0.1))
    else:
        s['endurance'] = 25 * int(pack_usable / mine['e_lap']) / 22
        s['efficiency'] = 0.0
    s['total'] = sum(s.values())
    return s

def reference_competitor():
    """A strong top-10 car, run through the same model so bias cancels."""
    return replace(Vehicle(), mass=270.0, cl_a=4.2, cd_a=1.45, tire_F=0.70, _mu=None)

# ===========================================================================
# COMMANDS
# ===========================================================================
def cmd_tracks(veh, args):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    cfg = [('endurance', 'endurance.png', True, _spacing_endur),
           ('autocross', 'autocross.png', False, lambda c: 30.0)]
    fig, axes = plt.subplots(2, 1, figsize=(14, 7))
    for ax, (name, f, closed, sp) in zip(axes, cfg):
        fn = os.path.join(MAPS, f)
        if not os.path.exists(fn):
            print(f'  MISSING: {fn} — put the map PNG there.'); continue
        s, x, y, k = digitize(name, fn, closed, sp)
        v, t = solve_lap(veh, s, k, v0=None if closed else 0.0, closed=closed)
        print(f'{name:10s}: {s[-1]:.0f} m, sim {t:.1f} s, v_avg {s[-1]/t*MPH:.1f} mph, '
              f'min R {1/k.max():.1f} m  -> output/track_{name}.csv')
        pts = np.array([x, y]).T.reshape(-1, 1, 2)
        lc = LineCollection(np.concatenate([pts[:-1], pts[1:]], axis=1), cmap='RdYlGn', linewidths=3.5)
        lc.set_array(v * MPH); ax.add_collection(lc)
        fig.colorbar(lc, ax=ax, shrink=0.85, label='mph')
        ax.set_aspect('equal'); ax.autoscale(); ax.axis('off')
        ax.set_title(f'{name} — {s[-1]:.0f} m, {t:.1f} s')
    plt.tight_layout(); plt.savefig(os.path.join(OUT, 'tracks.png'), dpi=120)
    print(f'  figure -> output/tracks.png')

def cmd_events(veh, args):
    t = event_times(veh, regen=not args.no_regen)
    print(f'Vehicle: {veh.mass:.0f} kg ({veh.mass/0.4536:.0f} lb), {veh.p_cap/1e3:.0f} kW, tire_F {veh.tire_F}')
    print(f'  Accel 75 m   : {t["accel"]:6.2f} s')
    print(f'  Skidpad      : {t["skidpad"]:6.2f} s')
    print(f'  Autocross    : {t["autocross"]:6.2f} s')
    print(f'  Endurance    : {t["endur_lap"]:6.2f} s/lap  ->  {t["endur_total"]/60:.1f} min / 22 laps')
    print(f'  Energy       : {t["e_lap"]:.3f} kWh/lap  ->  {22*t["e_lap"]:.1f} kWh / 22 laps')

def cmd_strategy(veh, args):
    pack = args.pack_kwh
    print(f'Pack usable: {pack} kWh | regen {"OFF" if args.no_regen else "ON"} | mass {veh.mass/0.4536:.0f} lb')
    print(f'{"P_cap":>6} | {"lap":>6} | {"endur":>6} | {"kWh/lap":>8} {"22 laps":>8} | fits?')
    for p in [80, 70, 60, 50, 40, 30]:
        t = event_times(veh, p_cap_endur=p * 1e3, regen=not args.no_regen)
        need = 22 * t['e_lap']
        print(f'{p:6d} | {t["endur_lap"]:6.2f} | {t["endur_total"]/60:6.2f} | '
              f'{t["e_lap"]:8.3f} {need:8.2f} | {"YES" if need <= pack else "DNF"}')

def cmd_points(veh, args):
    pack = args.pack_kwh
    ref = reference_competitor()
    ref_t = event_times(ref, p_cap_endur=40e3, regen=True)
    print(f'TR26 {veh.mass/0.4536:.0f} lb, pack {pack} kWh, regen {"OFF" if args.no_regen else "ON"}')
    print(f'{"strategy":>16} | {"end":>6} {"eff":>5} {"acc":>5} {"skid":>5} {"ax":>6} | {"TOTAL":>6}')
    best = (None, -1)
    for p in [80, 60, 50, 40, 30]:
        t = event_times(veh, p_cap_endur=p * 1e3, regen=not args.no_regen)
        s = score_team(t, ref_t, pack)
        dnf = '' if 22 * t['e_lap'] <= pack else ' DNF'
        print(f'{str(p)+" kW":>16} | {s["endurance"]:6.1f} {s["efficiency"]:5.1f} '
              f'{s["accel"]:5.1f} {s["skidpad"]:5.1f} {s["autocross"]:6.1f} | {s["total"]:6.1f}{dnf}')
        if s['total'] > best[1]: best = (p, s['total'])
    print(f'\nBest: {best[0]} kW -> {best[1]:.1f} of 675 dynamic pts')
    t0 = event_times(veh, p_cap_endur=best[0] * 1e3, regen=not args.no_regen)
    s0 = score_team(t0, ref_t, pack)['total']
    print(f'\nDesign sensitivities in POINTS (at {best[0]} kW):')
    for name, vh in {
        '-10 kg mass': replace(veh, mass=veh.mass - 10, _mu=veh._mu),
        '+10% CL*A': replace(veh, cl_a=veh.cl_a * 1.1, _mu=veh._mu),
        '-10% CD*A': replace(veh, cd_a=veh.cd_a * 0.9, _mu=veh._mu),
        'tire F +0.05': replace(veh, tire_F=veh.tire_F + 0.05, _mu=None),
        'rev +1000 rpm': replace(veh, rev_limit_rpm=veh.rev_limit_rpm + 1000, _mu=veh._mu),
    }.items():
        s = score_team(event_times(vh, p_cap_endur=best[0] * 1e3, regen=not args.no_regen), ref_t, pack)['total']
        print(f'  {name:16s}: {s - s0:+6.1f} pts')

def cmd_animate(veh, args):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.collections import LineCollection
    s, x, y, k = load_track('endurance')
    v, t = solve_lap(veh, s, k, closed=True)
    vm = np.maximum(0.5 * (v[:-1] + v[1:]), 0.1)
    tc = np.concatenate([[0], np.cumsum(np.diff(s) / vm)])
    fig, ax = plt.subplots(figsize=(7, 5.5), dpi=80)
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    lc = LineCollection(np.concatenate([pts[:-1], pts[1:]], axis=1), cmap='RdYlGn', linewidths=4)
    lc.set_array(v * MPH); ax.add_collection(lc)
    fig.colorbar(lc, ax=ax, shrink=0.8, label='mph')
    car, = ax.plot([], [], 'o', ms=10, mfc='#1a1a2e', mec='white', mew=1.5, zorder=5)
    txt = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', family='monospace')
    ax.set_aspect('equal'); ax.axis('off'); ax.autoscale()
    ax.set_title(f'TR26 endurance lap — {t:.1f} s')
    fps = 12; frames = int(t * fps / 2.0)
    def upd(f):
        tt = f / fps * 2.0
        car.set_data([np.interp(tt, tc, x)], [np.interp(tt, tc, y)])
        txt.set_text(f't={tt:4.1f}s\nv={np.interp(tt, tc, v)*MPH:4.1f}mph')
        return car, txt
    FuncAnimation(fig, upd, frames=frames, blit=True).save(
        os.path.join(OUT, 'lap_animation.gif'), writer=PillowWriter(fps=fps))
    print(f'  -> output/lap_animation.gif ({t:.1f} s lap)')

def main():
    p = argparse.ArgumentParser(description='TR26 FSAE EV lap simulator')
    p.add_argument('command', choices=['tracks', 'events', 'strategy', 'points', 'animate', 'all'])
    p.add_argument('--mass-lb', type=float, default=650.0)
    p.add_argument('--pack-kwh', type=float, default=5.5)
    p.add_argument('--power-kw', type=float, default=80.0)
    p.add_argument('--tire-F', type=float, default=0.66)
    p.add_argument('--no-regen', action='store_true')
    args = p.parse_args()
    veh = Vehicle(mass=args.mass_lb * 0.4536, p_cap=args.power_kw * 1e3, tire_F=args.tire_F)
    cmds = dict(tracks=cmd_tracks, events=cmd_events, strategy=cmd_strategy,
                points=cmd_points, animate=cmd_animate)
    if args.command == 'all':
        for c in ['tracks', 'events', 'strategy', 'points']:
            print(f'\n{"="*60}\n{c.upper()}\n{"="*60}')
            cmds[c](veh, args)
    else:
        cmds[args.command](veh, args)

if __name__ == '__main__':
    main()
