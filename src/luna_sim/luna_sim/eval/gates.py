"""Pass/fail gates. Thresholds live in scorer.yaml, never in code.

Each gate names a metric (dotted path into the report's `metrics` dict), an operator and
a threshold. A gate whose metric is missing (None) fails unless it says `allow_missing`,
so a broken recording can't sneak through as a pass.
"""
from __future__ import annotations

import operator

OPS = {
    '<=': operator.le, '<': operator.lt, '>=': operator.ge, '>': operator.gt, '==': operator.eq,
}


def lookup(metrics, path):
    cur = metrics
    for key in path.split('.'):
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def evaluate(metrics, gates):
    """Returns (passed, results). `gates` is the list from scorer.yaml."""
    results = []
    for g in gates:
        value = lookup(metrics, g['metric'])
        op = g['op']
        if op not in OPS:
            raise ValueError(f"gate {g.get('name', g['metric'])}: unknown op {op!r}")
        if value is None:
            ok = bool(g.get('allow_missing', False))
            note = 'metric missing'
        else:
            ok = bool(OPS[op](value, g['value']))
            note = ''
        results.append({
            'name': g.get('name', g['metric']),
            'metric': g['metric'], 'op': op, 'threshold': g['value'],
            'value': value, 'passed': ok, 'note': note,
        })
    return all(r['passed'] for r in results), results
