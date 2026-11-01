"""Scoring metrics. Pure functions: numpy arrays in, plain numbers/dicts out.

Nothing in here knows about bags, ROS, or files. That is deliberate: every metric can be
unit tested with a hand-made trajectory (see tests/test_metrics.py).

Conventions (sim_interfaces.md section 2):
  - all units SI, angles in radians
  - poses are in the `map` frame
  - `t` arrays are sim time in seconds, strictly increasing
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# --------------------------------------------------------------------------- helpers

def wrap_angle(a):
    """Wrap angle(s) to (-pi, pi]."""
    a = np.asarray(a, dtype=float)
    w = (a + np.pi) % (2.0 * np.pi) - np.pi
    # numpy gives [-pi, pi); flip -pi to +pi to match the contract
    return np.where(w == -np.pi, np.pi, w)


def interp_pose(t_query, t, x, y, yaw, max_gap_s=0.5):
    """Interpolate a pose track onto `t_query`.

    Yaw is interpolated on the circle (unwrap -> linear -> wrap), never linearly across ±pi.
    A query point is invalid (NaN) if it lies outside the track or inside a gap longer than
    `max_gap_s` between consecutive samples: interpolating across a dropout would hide it.

    Returns (x, y, yaw, valid_mask).
    """
    t_query = np.asarray(t_query, dtype=float)
    t = np.asarray(t, dtype=float)
    n = len(t_query)
    if len(t) < 2:
        nan = np.full(n, np.nan)
        return nan, nan.copy(), nan.copy(), np.zeros(n, dtype=bool)

    yaw_u = np.unwrap(np.asarray(yaw, dtype=float))
    xi = np.interp(t_query, t, x)
    yi = np.interp(t_query, t, y)
    yawi = wrap_angle(np.interp(t_query, t, yaw_u))

    # index of the sample at or before each query
    idx = np.searchsorted(t, t_query, side='right') - 1
    inside = (idx >= 0) & (t_query <= t[-1])
    idx_c = np.clip(idx, 0, len(t) - 2)
    gap = t[idx_c + 1] - t[idx_c]
    exact = np.isclose(t_query, t[np.clip(idx, 0, len(t) - 1)])
    valid = inside & ((gap <= max_gap_s) | exact)

    xi[~valid] = np.nan
    yi[~valid] = np.nan
    yawi[~valid] = np.nan
    return xi, yi, yawi, valid


def _rising_edges(flag):
    """Indices where a boolean array goes False -> True (index 0 counts if it starts True)."""
    flag = np.asarray(flag, dtype=bool)
    if flag.size == 0:
        return np.array([], dtype=int)
    prev = np.concatenate(([False], flag[:-1]))
    return np.nonzero(flag & ~prev)[0]


def _segments(flag, t):
    """(start_time, end_time) for each run of True in `flag`."""
    starts = _rising_edges(flag)
    flag = np.asarray(flag, dtype=bool)
    nxt = np.concatenate((flag[1:], [False]))
    ends = np.nonzero(flag & ~nxt)[0]
    return [(float(t[s]), float(t[e])) for s, e in zip(starts, ends)]


# --------------------------------------------------------------------------- 1. localization

def localization_error(t_gt, gt_x, gt_y, gt_yaw, t_est, est_x, est_y, est_yaw,
                       max_gap_s=0.5):
    """Estimate vs ground truth, evaluated at ground-truth timestamps.

    `coverage` is the fraction of ground-truth samples that had a valid estimate. Low
    coverage means the estimator dropped out or started late, which the mean would hide.
    """
    ex, ey, eyaw, valid = interp_pose(t_gt, t_est, est_x, est_y, est_yaw, max_gap_s)
    n = len(t_gt)
    if n == 0 or not valid.any():
        return {
            'coverage': 0.0, 'pos_err_mean_m': None, 'pos_err_rms_m': None,
            'pos_err_max_m': None, 'pos_err_max_t': None,
            'yaw_err_mean_rad': None, 'yaw_err_max_rad': None,
        }

    pos_err = np.hypot(ex - gt_x, ey - gt_y)[valid]
    yaw_err = np.abs(wrap_angle(eyaw - np.asarray(gt_yaw)))[valid]
    tv = np.asarray(t_gt)[valid]
    i_max = int(np.argmax(pos_err))
    return {
        'coverage': float(valid.mean()),
        'pos_err_mean_m': float(pos_err.mean()),
        'pos_err_rms_m': float(np.sqrt(np.mean(pos_err ** 2))),
        'pos_err_max_m': float(pos_err[i_max]),
        'pos_err_max_t': float(tv[i_max]),
        'yaw_err_mean_rad': float(yaw_err.mean()),
        'yaw_err_max_rad': float(yaw_err.max()),
    }


# --------------------------------------------------------------------------- 2. collisions

@dataclass(frozen=True)
class Footprint:
    """Rectangular footprint in base_link.

    `x_offset` is where the rectangle's center sits relative to base_link along +x
    (positive = footprint center is ahead of base_link). Get it wrong and every clearance
    is off by that amount.
    """
    length: float
    width: float
    x_offset: float = 0.0
    y_offset: float = 0.0

    def corners_body(self):
        hl, hw = self.length / 2.0, self.width / 2.0
        c = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
        return c + np.array([self.x_offset, self.y_offset])


def rect_circle_clearance(x, y, yaw, fp: Footprint, cx, cy, r):
    """Signed clearance between the footprint at each pose and one circle.

    > 0: gap in meters.  <= 0: touching / overlapping (value is not penetration depth when
    the circle center is inside the rectangle, only "<= 0").
    """
    x, y, yaw = (np.asarray(v, dtype=float) for v in (x, y, yaw))
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = cx - x, cy - y
    # circle center in the footprint's frame
    px = c * dx + s * dy - fp.x_offset
    py = -s * dx + c * dy - fp.y_offset
    hl, hw = fp.length / 2.0, fp.width / 2.0
    qx, qy = np.abs(px) - hl, np.abs(py) - hw
    outside = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
    inside = np.minimum(np.maximum(qx, qy), 0.0)  # <= 0 when center is inside the rectangle
    return outside + inside - r


def wall_clearance(x, y, yaw, fp: Footprint, length, width):
    """Smallest distance from any footprint corner to the arena walls (0..length, 0..width).

    Negative means a corner is outside the arena.
    """
    x, y, yaw = (np.asarray(v, dtype=float) for v in (x, y, yaw))
    corners = fp.corners_body()  # (4, 2)
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    wx = x[:, None] + c * corners[:, 0] - s * corners[:, 1]
    wy = y[:, None] + s * corners[:, 0] + c * corners[:, 1]
    d = np.minimum.reduce([wx, length - wx, wy, width - wy])
    return d.min(axis=1)


def obstacle_contacts(t, x, y, yaw, fp: Footprint, obstacles, arena_size=None,
                      contact_tol_m=0.0):
    """Collisions with boulders, crater entries, and wall contacts along a ground-truth track.

    `obstacles`: list of dicts {name, kind, x, y, diameter, ...} as in arena_<seed>.json.
    Boulders are collisions. Craters are separate "entered crater" events: the 2D sim will
    happily drive through one, so the scorer is the only thing that notices.

    Events are counted on *entering* contact, not per sample: sitting against a rock for
    2 s is one collision, not 100.
    """
    t = np.asarray(t, dtype=float)
    n = len(t)
    events = []
    per_obstacle = {}
    min_boulder = np.full(n, np.inf)

    for ob in obstacles:
        kind = str(ob.get('kind', '')).lower()
        r = float(ob['diameter']) / 2.0
        clr = rect_circle_clearance(x, y, yaw, fp, float(ob['x']), float(ob['y']), r)
        name = ob.get('name', f"{kind}_{len(per_obstacle)}")
        per_obstacle[name] = float(clr.min()) if n else None
        contact = clr <= contact_tol_m
        etype = 'crater_entry' if kind == 'crater' else 'collision'
        for (t0, t1) in _segments(contact, t):
            events.append({'type': etype, 'obstacle': name, 't_start': t0, 't_end': t1})
        if kind != 'crater':
            min_boulder = np.minimum(min_boulder, clr)

    min_wall = None
    if arena_size is not None and n:
        wc = wall_clearance(x, y, yaw, fp, *arena_size)
        min_wall = float(wc.min())
        for (t0, t1) in _segments(wc <= contact_tol_m, t):
            events.append({'type': 'wall_contact', 'obstacle': 'wall', 't_start': t0, 't_end': t1})

    events.sort(key=lambda e: e['t_start'])
    has_boulders = bool(n) and np.isfinite(min_boulder).any()
    i_min = int(np.argmin(min_boulder)) if has_boulders else None
    return {
        'collisions': sum(e['type'] == 'collision' for e in events),
        'crater_entries': sum(e['type'] == 'crater_entry' for e in events),
        'wall_contacts': sum(e['type'] == 'wall_contact' for e in events),
        'min_obstacle_clearance_m': float(min_boulder[i_min]) if has_boulders else None,
        'min_obstacle_clearance_t': float(t[i_min]) if has_boulders else None,
        'min_wall_clearance_m': min_wall,
        'per_obstacle_min_clearance_m': per_obstacle,
        'events': events,
    }


# --------------------------------------------------------------------------- 3. cycles

def cycles_from_dumps(t_mass, dumped_mass_kg, t_run_start, t_run_end, min_increase_kg=0.05):
    """Mining cycles from the ground-truth dumped-mass signal.

    `dumped_mass_kg` is berm + misplaced mass (total mass that left the hopper), sampled
    whenever mock_excavation published it. Every increase of at least `min_increase_kg`
    is one completed dump = the end of one cycle. Cycle 1 starts at `t_run_start`; cycle
    k starts where cycle k-1 ended.

    This needs only std_msgs/Float32 ground-truth topics, so it works before
    /mission/state exists. Switch to the mission state once luna_mission publishes it.
    """
    t_mass = np.asarray(t_mass, dtype=float)
    m = np.asarray(dumped_mass_kg, dtype=float)
    dump_times = []
    if len(m):
        last = m[0]
        for ti, mi in zip(t_mass[1:], m[1:]):
            if mi - last >= min_increase_kg:
                dump_times.append(float(ti))
                last = mi
            elif mi < last:  # sim reset; follow it down
                last = mi
    durations = np.diff(np.concatenate(([t_run_start], dump_times))) if dump_times else np.array([])
    return {
        'cycles_completed': len(dump_times),
        'cycle_end_times': dump_times,
        'cycle_time_mean_s': float(durations.mean()) if len(durations) else None,
        'cycle_time_max_s': float(durations.max()) if len(durations) else None,
        'run_duration_s': float(t_run_end - t_run_start),
    }


# --------------------------------------------------------------------------- 4. stuck

def time_stuck(t_cmd, cmd_lin, cmd_ang, t_gt, gt_speed, gt_yaw_rate,
               cmd_eps=0.02, move_eps=0.01, min_duration_s=1.0, cmd_hold_s=0.5):
    """Total time spent commanding motion while not moving.

    Commanded = |linear| > cmd_eps or |angular| > cmd_eps, using the latest command no older
    than `cmd_hold_s` (the contract's command timeout). Not moving = true speed and yaw
    rate both below `move_eps`. Only stretches longer than `min_duration_s` count, so a
    normal start from rest doesn't.
    """
    t_gt = np.asarray(t_gt, dtype=float)
    if len(t_gt) == 0 or len(t_cmd) == 0:
        return {'time_stuck_s': 0.0, 'stuck_episodes': []}
    t_cmd = np.asarray(t_cmd, dtype=float)
    idx = np.searchsorted(t_cmd, t_gt, side='right') - 1
    fresh = (idx >= 0) & (t_gt - t_cmd[np.clip(idx, 0, None)] <= cmd_hold_s)
    lin = np.where(fresh, np.asarray(cmd_lin)[np.clip(idx, 0, None)], 0.0)
    ang = np.where(fresh, np.asarray(cmd_ang)[np.clip(idx, 0, None)], 0.0)
    commanded = (np.abs(lin) > cmd_eps) | (np.abs(ang) > cmd_eps)
    still = (np.abs(gt_speed) < move_eps) & (np.abs(gt_yaw_rate) < move_eps)
    episodes = [(a, b) for a, b in _segments(commanded & still, t_gt) if b - a >= min_duration_s]
    return {
        'time_stuck_s': float(sum(b - a for a, b in episodes)),
        'stuck_episodes': [{'t_start': a, 't_end': b} for a, b in episodes],
    }
