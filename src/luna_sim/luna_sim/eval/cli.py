"""luna_score: score one recorded sim run, or a folder of them.

  luna_score run  <bag> --arena arena_7.json [--config scorer.yaml] [--oracle localization]
  luna_score batch <runs_dir> [--arena-dir arenas/] [--config scorer.yaml]

Exit codes: 0 all gates passed, 1 a gate failed, 2 the scorer itself errored.
CI and the smoke test can use the exit code directly.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
import yaml

from .load import load_run
from .report import score, write_json

DEFAULT_CONFIG = Path(__file__).with_name('scorer.yaml')


def load_config(path):
    return yaml.safe_load(Path(path or DEFAULT_CONFIG).read_text())


def find_arena(bag: Path, arena_dir: Path | None):
    """Arena JSON for a bag: `arena.json` next to/inside the bag, else arena_<seed>.json
    in --arena-dir, where the seed is the last number in the bag's name (run_7 -> 7)."""
    base = bag if bag.is_dir() else bag.parent
    for cand in (base / 'arena.json', bag.with_suffix('.arena.json')):
        if cand.is_file():
            return cand
    match = re.findall(r'\d+', bag.stem)
    if arena_dir and match:
        cand = Path(arena_dir) / f'arena_{int(match[-1])}.json'
        if cand.is_file():
            return cand
    return None


def score_one(bag, arena, cfg, oracles):
    layout = json.loads(Path(arena).read_text()) if arena else {}
    run = load_run(bag, cfg.get('topics'))
    rep = score(run, layout, cfg, arena_path=arena, oracles=oracles)
    if not arena:
        rep['warnings'].append('no arena JSON: obstacles and walls not checked')
    return rep


def fmt(v, nd=3):
    if v is None:
        return '-'
    return f'{v:.{nd}f}' if isinstance(v, float) else str(v)


def summary_line(rep):
    mt = rep['metrics']
    loc, col, cyc, st = (mt.get(k) or {} for k in ('localization', 'collisions', 'cycles', 'stuck'))
    return {
        'seed': rep['run']['seed'],
        'passed': rep['passed'],
        'pos_err_mean_m': loc.get('pos_err_mean_m'),
        'pos_err_max_m': loc.get('pos_err_max_m'),
        'collisions': col.get('collisions'),
        'crater_entries': col.get('crater_entries'),
        'min_clearance_m': col.get('min_obstacle_clearance_m'),
        'cycles': cyc.get('cycles_completed'),
        'cycle_time_mean_s': cyc.get('cycle_time_mean_s'),
        'time_stuck_s': st.get('time_stuck_s'),
    }


def print_report(rep):
    s = summary_line(rep)
    print(f"{'PASS' if rep['passed'] else 'FAIL'}  {rep['run']['bag']}  (seed {s['seed']})")
    for k, v in s.items():
        if k not in ('seed', 'passed'):
            print(f'  {k:<20} {fmt(v)}')
    for g in rep['gates']:
        if not g['passed']:
            print(f"  gate FAILED: {g['name']}: {fmt(g['value'])} {g['op']} {g['threshold']}"
                  f"{'  (' + g['note'] + ')' if g['note'] else ''}")
    for w in rep['warnings']:
        print(f'  warning: {w}')


def cmd_run(a):
    cfg = load_config(a.config)
    bag = Path(a.bag)
    arena = Path(a.arena) if a.arena else find_arena(bag, None)
    rep = score_one(bag, arena, cfg, a.oracle)
    out = Path(a.out) if a.out else (bag if bag.is_dir() else bag.parent) / \
        f"report_{rep['run']['seed'] if rep['run']['seed'] is not None else bag.stem}.json"
    write_json(rep, out)
    print_report(rep)
    print(f'  report: {out}')
    return 0 if rep['passed'] else 1


def is_bag(p: Path):
    return (p.is_dir() and (p / 'metadata.yaml').is_file()) or p.suffix in ('.mcap', '.db3')


def cmd_batch(a):
    cfg = load_config(a.config)
    root = Path(a.runs_dir)
    bags = sorted(p for p in root.iterdir() if is_bag(p))
    if not bags:
        print(f'no bags found in {root}', file=sys.stderr)
        return 2
    rows, errors = [], []
    for bag in bags:
        try:
            rep = score_one(bag, find_arena(bag, a.arena_dir), cfg, a.oracle)
            write_json(rep, (bag if bag.is_dir() else bag.parent) /
                       f"report_{rep['run']['seed'] if rep['run']['seed'] is not None else bag.stem}.json")
            rows.append({'bag': bag.name, **summary_line(rep),
                         'failed_gates': [g['name'] for g in rep['gates'] if not g['passed']]})
        except Exception as e:  # one broken bag shouldn't kill the batch
            errors.append({'bag': bag.name, 'error': f'{type(e).__name__}: {e}'})

    def agg(key):
        vals = [r[key] for r in rows if isinstance(r.get(key), (int, float))]
        return {'mean': float(np.mean(vals)), 'max': float(np.max(vals))} if vals else None

    summary = {
        'runs': len(rows), 'passed': sum(r['passed'] for r in rows),
        'errors': errors,
        'aggregate': {k: agg(k) for k in ('pos_err_mean_m', 'pos_err_max_m', 'collisions',
                                          'min_clearance_m', 'cycle_time_mean_s',
                                          'time_stuck_s')},
        'per_run': rows,
    }
    write_json(summary, Path(a.out) if a.out else root / 'summary.json')

    cols = ['bag', 'passed', 'pos_err_max_m', 'collisions', 'min_clearance_m', 'cycles',
            'cycle_time_mean_s', 'time_stuck_s']
    print('  '.join(f'{c:>16}' for c in cols))
    for r in rows:
        print('  '.join(f"{('PASS' if r[c] else 'FAIL') if c == 'passed' else fmt(r[c]):>16}"
                        for c in cols))
    for e in errors:
        print(f"ERROR {e['bag']}: {e['error']}")
    print(f"\n{summary['passed']}/{summary['runs']} passed, {len(errors)} errored")
    if errors:
        return 2
    return 0 if summary['passed'] == summary['runs'] else 1


def main(argv=None):
    p = argparse.ArgumentParser(prog='luna_score', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    r = sub.add_parser('run', help='score one bag')
    r.add_argument('bag')
    r.add_argument('--arena', help='arena_<seed>.json (default: arena.json next to the bag)')
    r.add_argument('--config', help='scorer.yaml (default: the packaged one)')
    r.add_argument('--out', help='report path (default: report_<seed>.json next to the bag)')
    r.add_argument('--oracle', action='append', default=[],
                   help='oracle that was on for this run (repeatable): localization, perception')
    r.set_defaults(fn=cmd_run)

    b = sub.add_parser('batch', help='score every bag in a folder')
    b.add_argument('runs_dir')
    b.add_argument('--arena-dir', help='folder of arena_<seed>.json files')
    b.add_argument('--config')
    b.add_argument('--out', help='summary path (default: <runs_dir>/summary.json)')
    b.add_argument('--oracle', action='append', default=[])
    b.set_defaults(fn=cmd_batch)

    a = p.parse_args(argv)
    try:
        return a.fn(a)
    except Exception as e:
        print(f'luna_score error: {type(e).__name__}: {e}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
