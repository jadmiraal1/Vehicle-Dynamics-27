"""Digitize an FSAE course map PNG into a track CSV (s_m, x_m, y_m, kappa_1perm).

Usage (from the tracks/ folder):
    python digitize_track.py endurance
    python digitize_track.py autocross
    python digitize_track.py --all

Method: extract the bold red course line, skeletonize, order into a path, fit a
smoothing spline, compute curvature (clipped at 1/R_MIN), then inject slalom
weave curvature where dashed cone markings sit off the centerline.

SCALE IS AUTO-CALIBRATED from the paddock grid (FFT + peak-find on the grid
lines) -- it is NOT a hand-typed constant. The old GRID_PX = 11.6 was wrong;
the sheets measure 12.00 px, a 3.4% distance error that inflated every lap
time and energy number. Auto-calibration means that class of bug cannot recur.

Every run writes provenance into the CSV header: scale, source PNG + its md5,
course length, and whether slalom spacings were verified.

Requires: numpy, scipy, scikit-image, pillow.
"""
import sys
import os
import hashlib

import numpy as np
from PIL import Image
from skimage.morphology import skeletonize, closing, disk
from skimage.measure import label
from scipy.spatial import cKDTree
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.ndimage import convolve

FT = 0.3048
R_MIN = 4.5          # tightest physical FSAE corner [m] -> curvature clip
WEAVE_A = 0.85       # slalom lateral weave amplitude [m]
TOL_PX = 0.75        # spline residual tolerance [px]: the skeletonization
                     # noise floor. Scale-invariant by construction (see
                     # smooth()). 0.75 px is (a) physically right -- the
                     # skeleton of a ~5 px line locates the centreline to
                     # better than a pixel -- and (b) equal to the OLD code's
                     # effective tolerance (its s=0.2n in m^2 at 0.657 m/px
                     # works out to 0.68 px). So fixing the SCALE bug does not
                     # also silently re-characterise the tracks: one change at
                     # a time. Course length is robust to this (+/-2% over
                     # tol_px 0.5..2.0); slalom span DETECTION is not, which is
                     # one more reason those spacings stay Provisional.
MAX_SLALOM_M = 120.0  # a detected 'slalom' longer than this is not a slalom

# ----------------------------------------------------------------------------
# Per-track configuration. One place. No magic numbers buried in functions.
# ----------------------------------------------------------------------------
# slalom_zones: (x0_px, x1_px, y0_px, y1_px, cone_spacing_ft)
#
# !! SLALOM SPACINGS ARE UNVERIFIED (target-catalog status: Provisional) !!
# The 1024-px sheet exports are too coarse to read the cone-spacing
# annotations. Zooming to 9x, the autocross bottom-run annotation is
# legible only as "33'->30' spacing" or possibly "35'->50'". The previous
# version of this file hard-coded "35->53 ft, mean 44" -- a value carried
# over from an EARLIER YEAR'S sheet and never revalidated.
#
# This matters: k_peak = WEAVE_A * (pi/d_cone)^2, so d_cone = 44 ft vs 31 ft
# is a 2x difference in slalom curvature, and slaloms are a large share of
# autocross time.
#
# TO RESOLVE: re-export the sheets at >=3x resolution from the source PDF,
# read the annotations, and set these. Until then every slalom-influenced
# target stays Provisional.
TRACKS = {
    'endurance': dict(
        png='endurance_2026.png',
        closed=True,
        grid_ft=25.0,
        slalom_zones=[],
        spacing_default_ft=30.0,
        spacings_verified=False,
    ),
    'autocross': dict(
        png='skidpad_autocross_2026.png',
        closed=False,
        grid_ft=25.0,
        # y-boxes below are the two runs of the out-and-back. The course
        # occupies y 19..80 px; the midline is y=50. The OLD boxes (0-50 and
        # 60-122) left a dead band at y 50..60 that silently fell through to
        # the 30 ft default.
        slalom_zones=[
            (480, 1024,  0, 50, 32.0),   # top (outbound) run   -- UNVERIFIED
            (550, 1024, 50, 122, 31.5),  # bottom (return) run  -- UNVERIFIED
        ],
        spacing_default_ft=30.0,
        spacings_verified=False,
    ),
}


def md5(path):
    with open(path, 'rb') as fh:
        return hashlib.md5(fh.read()).hexdigest()


def calibrate_scale(img, grid_ft):
    """Measure the paddock-grid pitch in px from the image itself.

    The grid is thin grey/black ruling. Sum a greyness mask down the columns,
    peak-find, and take the median spacing. Returns (m_per_px, grid_px).
    """
    R, G, B = img[..., 0], img[..., 1], img[..., 2]
    grey = (abs(R - G) < 30) & (abs(G - B) < 30) & (R < 190)
    h = grey.shape[0]
    col = grey[int(0.12 * h):int(0.92 * h)].sum(axis=0).astype(float)
    pk, _ = find_peaks(col, height=col.max() * 0.45, distance=6)
    if len(pk) < 10:
        raise RuntimeError('grid calibration failed: only %d gridlines found' % len(pk))
    grid_px = float(np.median(np.diff(pk)))
    return grid_ft * FT / grid_px, grid_px


def extract_path(img, closed_loop):
    """Largest bold-red component -> skeleton -> ordered pixel path."""
    R, G, B = img[..., 0], img[..., 1], img[..., 2]
    # ONE mask for both extraction and slalom detection.
    #
    # Do NOT tighten this to "bold red only". The two sheets are rendered
    # differently: the autocross course is saturated red (median RGB ~233,38),
    # but the endurance course is a paler, more anti-aliased red (~192,82). A
    # bold-only threshold shatters the endurance line into 5 fragments and
    # yields a 125 m "course". The loose threshold recovers a single clean
    # component on BOTH sheets; the pale staging lanes and dashed cone
    # markings survive as separate, smaller components and are rejected by
    # the largest-component rule.
    mask_loose = (R > 120) & (R - G > 50) & (R - B > 50)

    lab = label(closing(mask_loose, disk(2)))
    sizes = np.bincount(lab.ravel())
    sizes[0] = 0
    skel = skeletonize(lab == sizes.argmax())

    ys, xs = np.nonzero(skel)
    pts = np.column_stack([xs, ys]).astype(float)

    nbr = convolve(skel.astype(int), np.ones((3, 3)), mode='constant')
    ends = np.column_stack(np.nonzero(skel & (nbr == 2)))
    branches = np.column_stack(np.nonzero(skel & (nbr >= 4)))
    if len(branches):
        print('  WARNING: %d skeleton branch points - path may be ambiguous'
              % len(branches))

    start = 0
    if not closed_loop:
        if not len(ends):
            raise RuntimeError('open course but skeleton has no endpoints')
        if len(ends) != 2:
            print('  WARNING: open course with %d endpoints (expected 2)' % len(ends))
        ey, ex = ends[np.argmin(ends[:, 1])]       # leftmost end = start
        start = int(np.argmin((pts[:, 0] - ex) ** 2 + (pts[:, 1] - ey) ** 2))

    tree = cKDTree(pts)
    used = np.zeros(len(pts), bool)
    order = [start]
    used[start] = True
    i = start
    for _ in range(len(pts) - 1):
        d, idx = tree.query(pts[i], k=14)
        nxt = next((j for dd, j in zip(d, idx) if not used[j] and dd < 6), None)
        if nxt is None:
            break
        order.append(nxt)
        used[nxt] = True
        i = nxt

    frac = len(order) / len(pts)
    print('  ordered %d/%d skeleton px (%.1f%%)' % (len(order), len(pts), 100 * frac))
    if frac < 0.98:
        print('  WARNING: walk did not consume the skeleton - broken line?')
    return pts[np.array(order)], mask_loose


def smooth(path_px, m_per_px, closed_loop, tol_px=TOL_PX, ds=1.0):
    """Spline-smooth the ordered path and take curvature.

    Fitted in PIXEL space, then scaled to metres. This matters: splprep's
    smoothing parameter `s` bounds the sum of squared residuals *in the units
    of the input coordinates*. The previous version fitted in METRES with
    s = 0.2*n, so the physical amount of smoothing silently depended on the
    m/px scale factor -- recalibrate the scale and you quietly change every
    curvature, and therefore every lap time. Exactly the class of hidden
    coupling this refactor exists to remove.

    Pixel space is also where the noise actually lives: the residual being
    smoothed out is skeletonization/quantization error of order 1 px. So
    s = n * tol_px^2 with tol_px ~ 1 px is both scale-invariant and physically
    motivated, instead of a tuned magic number.
    """
    x, y = path_px[:, 0].copy(), path_px[:, 1].copy()
    if closed_loop:
        x = np.append(x, x[0])
        y = np.append(y, y[0])
    n = len(x)
    tck, _ = splprep([x, y], s=n * tol_px ** 2, per=int(closed_loop))

    u = np.linspace(0, 1, 4000)
    xs, ys = splev(u, tck)
    dx, dy = splev(u, tck, der=1)
    ddx, ddy = splev(u, tck, der=2)

    # curvature in 1/px -> 1/m  (kappa scales as 1/length)
    kappa_px = np.abs(dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5
    kappa = np.minimum(kappa_px / m_per_px, 1.0 / R_MIN)

    xs, ys = xs * m_per_px, ys * m_per_px
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))])
    sg = np.arange(0, s[-1], ds)
    return (sg, np.interp(sg, s, xs), np.interp(sg, s, ys),
            np.interp(sg, s, kappa))


def inject_slaloms(sg, xg, yg, kg, mask, m_per_px, cfg):
    """Raise curvature to a sine weave where dashed cone markings sit off-line."""
    zones = cfg['slalom_zones']
    default_ft = cfg['spacing_default_ft']

    def spacing_ft(x_m, y_m):
        px, py = x_m / m_per_px, y_m / m_per_px
        for x0, x1, y0, y1, sp in zones:
            if x0 < px <= x1 and y0 <= py <= y1:
                return sp
        return default_ft

    ys, xs = np.nonzero(mask)
    red_m = np.column_stack([xs, ys]) * m_per_px
    d, idx = cKDTree(np.column_stack([xg, yg])).query(red_m)
    off = (d > 1.8) & (d < 5.5)
    if not off.any():
        print('  no off-centreline cone markings found - no slaloms injected')
        return kg

    hist, edges = np.histogram(np.sort(sg[idx[off]]),
                               bins=np.arange(0, sg[-1] + 10, 10))
    dense = hist > 8
    spans, start = [], None
    for i, dn in enumerate(dense):
        if dn and start is None:
            start = edges[i]
        if not dn and start is not None:
            spans.append([start, edges[i]])
            start = None
    if start is not None:
        spans.append([start, edges[-1]])

    merged = []
    for sp in spans:
        if merged and sp[0] - merged[-1][1] < 25:
            merged[-1][1] = sp[1]
        else:
            merged.append(list(sp))

    kap = kg.copy()
    for s0, s1 in merged:
        if s1 - s0 < 20:
            continue
        if s1 - s0 > MAX_SLALOM_M:
            print('  WARNING: %.0f-%.0f m span (%.0f m) is too long to be a real '
                  'slalom - cone-marking detection has over-merged. Injecting '
                  'anyway, but treat this track\'s slalom curvature as suspect.'
                  % (s0, s1, s1 - s0))
        im = np.searchsorted(sg, 0.5 * (s0 + s1))
        d_cone = spacing_ft(xg[im], yg[im]) * FT
        k_peak = WEAVE_A * (np.pi / d_cone) ** 2
        i0, i1 = np.searchsorted(sg, s0), np.searchsorted(sg, s1)
        ss = sg[i0:i1 + 1] - sg[i0]
        kap[i0:i1 + 1] = np.maximum(
            kap[i0:i1 + 1], k_peak * np.abs(np.sin(np.pi * ss / d_cone)))
        print('  slalom %.0f-%.0f m -> %.0f ft cones (k_peak %.3f, R %.1f m)'
              % (s0, s1, d_cone / FT, k_peak, 1 / k_peak))
    return kap


def digitize(name):
    cfg = TRACKS[name]
    png = cfg['png']
    if not os.path.exists(png):
        raise FileNotFoundError(png)

    img = np.array(Image.open(png).convert('RGB')).astype(int)
    m_per_px, grid_px = calibrate_scale(img, cfg['grid_ft'])
    print('%s -> track_%s.csv' % (png, name))
    print('  grid AUTO-CALIBRATED: %.2f px = %.0f ft -> %.4f m/px'
          % (grid_px, cfg['grid_ft'], m_per_px))

    path_px, mask = extract_path(img, cfg['closed'])
    sg, xg, yg, kg = smooth(path_px, m_per_px, cfg['closed'])
    kap = inject_slaloms(sg, xg, yg, kg, mask, m_per_px, cfg)

    clipped = 100 * (kap >= 0.999 / R_MIN).mean()
    print('  length %.1f m | min R %.2f m | %.1f%% of points at the curvature clip'
          % (sg[-1], 1 / kap.max(), clipped))
    if not cfg['spacings_verified']:
        print('  !! slalom cone spacings UNVERIFIED - targets stay Provisional')

    hdr = [
        'FSAE course centreline, digitized from %s' % png,
        'source_md5=%s' % md5(png),
        'grid_px=%.3f grid_ft=%.1f m_per_px=%.5f  (AUTO-CALIBRATED from the sheet)'
        % (grid_px, cfg['grid_ft'], m_per_px),
        'length_m=%.1f  closed=%s  R_min_m=%.2f'
        % (sg[-1], cfg['closed'], 1 / kap.max()),
        'slalom_spacings_verified=%s' % cfg['spacings_verified'],
    ]
    out = 'track_%s.csv' % name
    with open(out, 'w') as fh:
        for h in hdr:
            fh.write('# %s\n' % h)
        fh.write('s_m,x_m,y_m,kappa_1perm\n')
        for row in np.column_stack([sg, xg, yg, kap]):
            fh.write('%.4f,%.4f,%.4f,%.6f\n' % tuple(row))
    print('  wrote %s\n' % out)


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    if '--all' in sys.argv:
        args = list(TRACKS)
    if not args:
        print(__doc__)
        sys.exit(1)
    for nm in args:
        if nm not in TRACKS:
            print('unknown track %r; known: %s' % (nm, ', '.join(TRACKS)))
            sys.exit(1)
        digitize(nm)
