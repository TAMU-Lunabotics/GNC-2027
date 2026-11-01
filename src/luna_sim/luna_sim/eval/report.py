"""Turn a loaded run + arena + config into a report dict (written as report_<seed>.json)."""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np

from .luna_scorer import __version__, gates
from .luna_scorer import metrics as m
from .load import Run


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git_commit(cwd=None):
    try:
        out = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=cwd,
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() or None
    except Exception:
        return None


def arena_size(layout, cfg):
    """(length, width) of the arena, from the layout JSON if it has it, else scorer.yaml."""
    for key in ('dimensions', 'arena'):
        d = layout.get(key)
        if isinstance(d, dict) and 'length' in d and 'width' in d:
            return float(d['length']), float(d['width'])
    a = cfg.get('arena', {})
    if 'length' in a and 'width' in a:
        return float(a['length']), float(a['width'])
    return None


def score(run: Run, layout: dict, cfg: dict, arena_path=None, oracles=None):
    gt = run.gt_pose
    warnings = list(run.warnings)
    fp_cfg = cfg['footprint']
    fp = m.Footprint(fp_cfg['length'], fp_cfg['width'],
                     fp_cfg.get('x_offset', 0.0), fp_cfg.get('y_offset', 0.0))
    loc_cfg = cfg.get('localization', {})
    col_cfg = cfg.get('collisions', {})
    stuck_cfg = cfg.get('stuck', {})
    cyc_cfg = cfg.get('cycles', {})

    out = {}
    if len(gt):
        t0, t1 = float(gt.t[0]), float(gt.t[-1])

        # 1. localization (skip the settle window so startup transients don't dominate max)
        settle = float(loc_cfg.get('settle_s', 1.0))
        keep = gt.t >= t0 + settle
        if len(run.est):
            out['localization'] = m.localization_error(
                gt.t[keep], gt['x'][keep], gt['y'][keep], gt['yaw'][keep],
                run.est.t, run.est['x'], run.est['y'], run.est['yaw'],
                max_gap_s=float(loc_cfg.get('max_gap_s', 0.5)))
        else:
            out['localization'] = None
            warnings.append('no localization estimate recorded')

        # 2. collisions / clearance (whole run, no settle window: a collision is a collision)
        size = arena_size(layout, cfg) if col_cfg.get('check_walls', True) else None
        out['collisions'] = m.obstacle_contacts(
            gt.t, gt['x'], gt['y'], gt['yaw'], fp, layout.get('obstacles', []),
            arena_size=size, contact_tol_m=float(col_cfg.get('contact_tol_m', 0.0)))

        # 3. cycles from ground-truth dumped mass
        if len(run.dumped_mass):
            out['cycles'] = m.cycles_from_dumps(
                run.dumped_mass.t, run.dumped_mass['kg'], t0, t1,
                min_increase_kg=float(cyc_cfg.get('min_dump_kg', 0.05)))
            out['cycles']['berm_mass_kg'] = float(run.berm_mass['kg'][-1])
            out['cycles']['misplaced_mass_kg'] = float(run.dumped_mass['kg'][-1]
                                                       - run.berm_mass['kg'][-1])
        else:
            out['cycles'] = {'cycles_completed': None, 'run_duration_s': t1 - t0}
            warnings.append('no excavation mass topics; cycles not scored')

        # 4. stuck: needs true velocity (or differentiate pose) + commands
        if len(run.cmd):
            if len(run.gt_twist):
                tw = run.gt_twist
                spd = np.interp(gt.t, tw.t, np.hypot(tw['vx'], tw['vy']))
                yr = np.interp(gt.t, tw.t, tw['wz'])
            else:
                dt_ = np.gradient(gt.t)
                spd = np.hypot(np.gradient(gt['x']), np.gradient(gt['y'])) / dt_
                yr = np.gradient(np.unwrap(gt['yaw'])) / dt_
            out['stuck'] = m.time_stuck(
                run.cmd.t, run.cmd['lin'], run.cmd['ang'], gt.t, spd, yr,
                cmd_eps=float(stuck_cfg.get('cmd_eps', 0.02)),
                move_eps=float(stuck_cfg.get('move_eps', 0.01)),
                min_duration_s=float(stuck_cfg.get('min_duration_s', 1.0)),
                cmd_hold_s=float(stuck_cfg.get('cmd_hold_s', 0.5)))
        else:
            out['stuck'] = None
            warnings.append('no /cmd_vel recorded; stuck time not scored')
    else:
        out = {'localization': None, 'collisions': None, 'cycles': None, 'stuck': None}

    passed, gate_results = gates.evaluate(out, cfg.get('gates', []))
    if not len(gt):
        passed = False

    return {
        'scorer_version': __version__,
        'scored_at': dt.datetime.now(dt.timezone.utc).isoformat(timespec='seconds'),
        'run': {
            'bag': str(run.path),
            'arena': str(arena_path) if arena_path else None,
            'arena_sha256': sha256(arena_path) if arena_path else None,
            'seed': layout.get('seed'),
            'oracles': sorted(oracles or []),
            'git_commit': git_commit(),
            'sim_time_start_s': float(gt.t[0]) if len(gt) else None,
            'sim_time_end_s': float(gt.t[-1]) if len(gt) else None,
            'topics_present': run.topics_present,
        },
        'passed': passed,
        'gates': gate_results,
        'metrics': out,
        'warnings': warnings,
    }


def write_json(obj, path):
    Path(path).write_text(json.dumps(obj, indent=2, default=float) + '\n')
