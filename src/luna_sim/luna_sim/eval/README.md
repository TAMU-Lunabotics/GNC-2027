# luna_scorer: offline scorer for luna_sim runs

Reads a recorded sim run (ROS 2 bag) plus its `arena_<seed>.json`, and writes
`report_<seed>.json` with metrics and pass/fail gates. It's pure Python on top of
[`rosbags`](https://gitlab.com/ternaris/rosbags), so **no ROS install is needed**: it runs in
CI, on a laptop, anywhere.

The scorer only judges. Nothing here feeds back into the stack, and it is the reason
`/sim/ground_truth/*` exists (sim_interfaces.md §9).

```
run_<seed>/ (MCAP bag + arena.json) ──► luna_score ──► report_<seed>.json ──► summary.json
```

## Install

```bash
pip install -e tools/luna_scorer        # or: pip install -e '.[test]'
```

Not a colcon package on purpose. If you want it in the workspace, add a `COLCON_IGNORE`.

## Record a run

Record these topics, as MCAP, starting **with** the sim (the mass topics are transient local,
so the recorder picks up their latest values when it starts):

```bash
ros2 bag record -s mcap -o runs/run_7 \
  /clock /sim/ground_truth/pose /sim/ground_truth/twist \
  /sim/ground_truth/berm_mass_kg /sim/ground_truth/misplaced_mass_kg \
  /odometry/filtered /cmd_vel /tf /tf_static
cp <wherever the generator wrote it>/arena_7.json runs/run_7/arena.json
```

Best done in the launch file (`record:=true`) so every run uses the same topic list and the
arena JSON gets copied next to the bag automatically.

## Score

```bash
luna_score run runs/run_7                         # finds runs/run_7/arena.json
luna_score run runs/run_7 --arena maps/arena_7.json --oracle localization
luna_score batch runs/ --arena-dir maps/          # every bag in runs/, plus summary.json
```

Exit code **0** = every gate passed, **1** = a gate failed, **2** = scorer error (missing bag,
unreadable file). Use it directly in CI or after the smoke test.

Pass `--oracle localization` (etc.) for oracle runs. It goes into the report, because
results from oracle runs aren't comparable to real-stack runs.

## What it measures (v0)

| Section | Metrics | Source |
|---|---|---|
| `localization` | mean / RMS / max position error, time of max, yaw error, **coverage** | `/odometry/filtered` vs `/sim/ground_truth/pose`, at ground-truth stamps |
| `collisions` | boulder collisions, crater entries, wall contacts (each as **events**, not samples), min clearance + when, per-obstacle min clearance | footprint rectangle at each ground-truth pose vs arena JSON |
| `cycles` | cycles completed, time per cycle, berm / misplaced mass | each increase in berm + misplaced mass = one dump |
| `stuck` | total time commanding motion but not moving, episodes | `/cmd_vel` vs `/sim/ground_truth/twist` |

Details worth knowing:
- **Localization** ignores the first `settle_s` (filter startup) and treats estimate gaps
  longer than `max_gap_s` as dropouts instead of interpolating across them. Coverage reports
  how much of the run had an estimate; a low value means the mean is hiding a dropout.
- **Yaw** is interpolated and differenced on the circle, never linearly across ±π.
- **Time** is sim time throughout. Headerless topics (the Float32 masses) are mapped from the
  bag's receive time, which is wall time unless the recorder ran with `--use-sim-time`, to
  sim time using `/clock`, falling back to the ground-truth pose stamps. **Record `/clock`.**
- **Clearance** is signed. Negative means overlap. A 1 cm pass is still a pass, but the
  number tells you it was close.

## Configuration: `luna_scorer/scorer.yaml`

Footprint, thresholds and gates all live here. A gate is

```yaml
- name: localization_max_error
  metric: localization.pos_err_max_m     # path into the report's `metrics`
  op: "<="
  value: 0.15
  allow_missing: false                   # default: a metric that couldn't be computed FAILS
```

so a task's definition of done can be a gate file:
`luna_score batch runs/ --config planner_acceptance.yaml`.

## Code layout

```
luna_scorer/
  metrics.py   pure numpy: arrays in, numbers out. All the math is here.
  gates.py     thresholds -> pass/fail
  load.py      bag -> numpy time series (the only file that knows about rosbags)
  report.py    metrics + gates + run metadata -> report dict
  cli.py       luna_score run / batch
tests/
  test_metrics.py      hand-made trajectories, no bags
  synthetic_bag.py     writes a fake run as a real MCAP bag
  test_end_to_end.py   CLI on synthetic bags (clean run passes, collision run fails)
```

**Adding a metric:** write a function in `metrics.py` with a unit test on synthetic arrays,
call it from `report.score()`, and optionally add a gate. Because runs are recorded, you can
then re-score every old bag with the new metric without re-running the sim.

Run the tests with `pytest tests`.

## Open items

- [ ] Read `footprint` from `luna_description/config/robot_geometry.yaml` instead of
      duplicating it in `scorer.yaml`. **The current 1.20 × 0.80 m numbers are placeholders.**
- [ ] Confirm the arena JSON key for arena size. The scorer looks for `dimensions` or `arena`
      with `length`/`width`, falling back to `scorer.yaml`.
- [ ] Cycles from `/mission/state` once `luna_mission` publishes it (more precise cycle starts).
- [ ] Localization from TF (`map → odom → base_link`) as an alternative to
      `/odometry/filtered`, for checking what nav2 actually sees.
- [ ] Competition-points proxy (point table in YAML) and fault-response metrics.
- [ ] Batch plots / nightly trend tracking.
