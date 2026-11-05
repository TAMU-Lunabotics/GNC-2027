# mock_excavation and mock_dump

## What it is

These two nodes stand in for the robot's digging and dumping mechanisms. Each one runs an action server that the mission planner (the HFSM) calls when it enters the matching state:

```
                     ┌── /dig  (luna_interfaces/action/Dig)  ──► mock_excavation
mission planner ─────┤
   (HFSM)            └── /dump (luna_interfaces/action/Dump) ──► mock_dump
```

For now the mocks do almost nothing. When a goal arrives, they wait a configured time plus a small random amount, send feedback while they wait, and then report success. That's enough for the HFSM to run complete cycles in the Level 1 sim: navigate → dig → navigate → dump → repeat.

The real excavation and dump controllers will expose **the same action names and types**, so the mission planner won't change when we swap them in. That's the important part of this task: the interface, not the behavior.

These nodes live in `luna_sim` because they're sim stand-ins. They don't read ground truth yet; they may later, for zone checks.

---

## Interfaces

Both actions already exist in `luna_interfaces/action/`. Don't change them without asking me.

### `/dig` — `luna_interfaces/action/Dig` (served by `mock_excavation`)

| Part | Field | What the mock does |
|---|---|---|
| Goal | `max_duration_s` | If the sampled duration is longer than this (and it isn't 0), abort at `max_duration_s` with `failure_reason: "timeout"`. |
| Result | `success` | `true` unless canceled, timed out, or an injected failure. |
| Feedback | `elapsed_s` | Time since the goal started (sim time). |

### `/dump` — `luna_interfaces/action/Dump` (served by `mock_dump`)

| Part | Field | What the mock does |
|---|---|---|
| Goal | `max_duration_s` | Same timeout rule as dig. |
| Result | `success` | Same as dig. |
| Feedback | `elapsed_s` | Time since the goal started (sim time). |

---

## Required behavior

Both nodes behave the same way. Only the action type and field names differ.

1. **Accept or reject.**
   - Accept a goal if the server is idle.
   - Reject a new goal if one is already running. A real mechanism can't do two digs at once, and the mission planner should never send a second goal before the first finishes. Rejecting makes that bug visible instead of hiding it.

2. **Duration.** When a goal starts, sample `duration = duration_s + uniform(0, jitter_s)`. Log the sampled value.

3. **While running**, publish feedback at `feedback_hz` until the duration is up. Then succeed with the result fields above.

4. **Cancel.** If the client cancels, stop at the next feedback tick and return `success: false` with `failure_reason: "canceled"` and the partial mass. Accept every cancel request.

5. **Timeout.** Apply the `max_duration_s` rule from the tables.

6. **Injected failures (default off).**
   - With probability `failure_probability`, the goal fails partway through, at a random point between 20% and 80% of the duration, with the node's failure reason (`stall` or `jam`).
   - This lets the HFSM owner test retries and recovery. Keep it at 0 for normal runs.

7. **Use sim time only.**
   - All timing comes from the node clock (`use_sim_time` from launch). Never call `time.sleep()` or read wall time because the sim can run faster than real time, and wall time breaks that.
   - In rclpy, a blocking `execute_callback` that waits on the node clock needs a `MultiThreadedExecutor` and a `ReentrantCallbackGroup` for the action server. Otherwise it deadlocks, because the clock and timer callbacks can't run while it's waiting. Wait with `rate = self.create_rate(feedback_hz, self.get_clock())` and `rate.sleep()`.

8. **Reproducibility.** Seed the RNG from a `seed` parameter (launch passes the arena seed). Give each node its own RNG stream (for example, `make_rng(seed, "mock_dump")`) so the two don't share a random sequence.

---

## Parameters

`config/actuators/mock_excavation.yaml`:

| Parameter | Default | Meaning |
|---|---|---|
| `duration_s` | `20.0` | Base dig time (sim seconds). |
| `jitter_s` | `3.0` | Extra uniform random time, 0 to this. |
| `feedback_hz` | `2.0` | Feedback rate. |
| `failure_probability` | `0.0` | Chance a goal fails partway (`stall`). |
| `seed` | `0` | RNG seed. |

`config/actuators/mock_dump.yaml`:

| Parameter | Default | Meaning |
|---|---|---|
| `duration_s` | `10.0` | Base dump time (sim seconds). |
| `jitter_s` | `2.0` | Extra uniform random time, 0 to this. |
| `feedback_hz` | `2.0` | Feedback rate. |
| `failure_probability` | `0.0` | Chance a goal fails partway (`jam`). |
| `seed` | `0` | RNG seed. |

Keep every value in YAML, none in code.
---

## Structure (inside `luna_sim`)

```
luna_sim/
├── config/actuators/
│   ├── mock_excavation.yaml
│   └── mock_dump.yaml
├── luna_sim/actuators/
│   ├── __init__.py
│   ├── mock_excavation.py       ROS node: /dig action server
│   └── mock_dump.py             ROS node: /dump action server
├── scripts/
│   ├── mock_excavation.py       3-line wrappers, add both to PY_EXECUTABLES
│   └── mock_dump.py
└── test/test_timed_action.py    plain pytest
```

---
