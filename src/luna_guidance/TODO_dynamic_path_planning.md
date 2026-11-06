# dynamic_path_planning

## What this is

This is an upgrade to the path planner so it reacts to obstacles discovered while driving.

Right now the planner makes one plan per goal, using whatever is in `/map` (or `/map_truth`) at that moment. On the real arena, the map starts empty, and the obstacle detector adds boulders and craters as the robot gets close to them. That means the first path will go straight through obstacles the robot hasn't seen yet. The planner has to notice when the map changes in a way that blocks its path, and replan from wherever the robot is.

```
/map (full grid, republished as it changes) ──┐
/odometry/filtered ───────────────────────────┼──► path_planner ──► /path ──► guidance_controller
/goal_pose ───────────────────────────────────┘          └─► /planning/status
```

The obstacle detector is the only node that publishes `/map`. Each message is the complete current grid, the planner just has to notice when it changed.

This is still rover code: no `/sim/**` topics, no arena JSON.

Test against `/map` with the mock obstacle detector running (`obstacle_sensor:=true`), not `/map_truth`. `/map_truth` is still useful as a baseline, to compare "path on the full map" against "path with discovery".

---

## What changes from v1

- `/map` is no longer static. A new full grid arrives at the detector's rate (10 Hz), and most messages are identical to the previous one.
- The planner keeps the latest grid and its inflated version, and recomputes the inflation only when the grid actually changed.
- New replan triggers: path blocked, robot off the path, and optionally periodic. `replan_deviation_m` was already in the v1 parameters but never in the required behavior, so implement it now.
- Replanning happens in a timer and is rate-limited.
- If the robot's own cell ends up inside the inflated area, the planner can still plan out of it.

---

## Interfaces

### Inputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | `map` | reliable, transient local, depth 1 | The full current map from the detector: empty at startup, with obstacles added as they're detected. Republished at the detector's rate. Transient local means a planner that starts or restarts late gets the current map immediately. |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | pose in `map` | reliable, depth 10 | Same as v1. It will soon come from the EKF, so the pose can jump slightly. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `map` | reliable, depth 1 | Same as v1. |

### Outputs

Same as v1 (`/path`, `/planning/status`). Publish status on every plan attempt, including replans, and put the trigger in the reason string (for example `replan: path blocked`).

---

## Required behavior

1. **Keep the latest map.**
   - On each `/map` message, compare it with the stored grid using `np.array_equal` or similar. If it's identical, do nothing. Most messages will be identical, so this check is what keeps the planner from doing pointless work and replanning constantly.
   - If the geometry (`resolution`, `width`, `height`, `origin`) differs from the stored map, treat it as a brand-new map. Reset everything and replan if you have a goal.
   - Otherwise, store the new grid and mark the map as "changed".

2. **Recompute inflation when the map changed.**
   - Inflate the whole grid by `robot_radius_m + safety_margin_m`. You can use something like `scipy.ndimage.binary_dilation` with a disk-shaped structuring element, or threshold a distance transform. At our arena size (about 14k cells) this takes a few ms. Since it only runs when the map actually changed, it's cheap.
   - Recompute from scratch, don't accumulate. Cells can become free again, for example if the detector drops a false positive. OR-ing new obstacles into the old inflated grid would keep those cells blocked forever.
   - Add `<exec_depend>python3-scipy</exec_depend>` to `package.xml` if it isn't there already.

3. **Check the current path when the map changed.**
   - Check the remaining path, from the robot's current progress point to the goal, against the new inflated grid.
   - Check along each segment (Bresenham / line-of-sight), not only at the poses. After smoothing, segments can be long.
   - If any cell on the path is occupied, the path is blocked, so replan.
   - Cells becoming *free* never block the path, so they don't trigger a replan by themselves. They might allow a shorter path, and that's what the optional periodic replan is for.

4. **Replan triggers**, in priority order:
   - A new goal. Replan immediately; this is not rate-limited.
   - The current path is blocked.
   - The robot is farther than `replan_deviation_m` from the path.
   - *(Optional, off by default)* Every `replan_period_s`. Only adopt the new path if it's shorter than `improvement_ratio ×` the remaining length of the current path.

5. **Rate-limit replanning.**
   - Callbacks only store data and set flags ("map changed", "new goal").
   - A timer at `rate_hz` checks the triggers and plans.
   - Replan at most once per `min_replan_interval_s`, except for new goals.
   - If the path is blocked but you're inside the rate limit, wait. The guidance collision check is the backstop that stops the robot meanwhile.

6. **Plan from where the robot is.**
   - The start is the current pose from `/odometry/filtered`, not the original start.
   - As obstacles appear, the robot's own cell can end up inside the inflated area, either because an obstacle was detected close by or because of pose noise. A* from an occupied start finds nothing.
   - If the start cell is occupied in the inflated grid, BFS for the nearest free cell within `start_escape_radius_m`. Plan from that cell, and put the robot's actual pose as the first path point.
   - The escape segment may cross *inflated* cells, but never raw occupied cells.
   - If there's no free cell within the radius, report `FAILED` with reason `start in collision`.

7. **The goal becomes blocked.** If the goal cell becomes occupied in the inflated grid, publish an empty path (guidance stops) and `FAILED` with reason `goal in obstacle`. Don't move the goal silently, the mission planner will decide what to do later. If the goal becomes free again later (the obstacle was a false positive), replanning on the next trigger is fine.

8. **Fail clearly.** Same as v1: if there's no route, publish an empty path and `FAILED`. Never publish a path through an obstacle.

9. **Cell values.**
   - Occupied means a value `≥ occupied_threshold`.
   - Unknown (`-1`) counts as free when `unknown_is_free` is set. Planning optimistically through unexplored arena is how the robot explores.
   - The `OccupancyGrid` convention is 0 = for sure free and 1 = for sure occupied. 

---

## Parameters (`config/planner.yaml`, additions)

| Parameter | Default | Meaning |
|---|---|---|
| `rate_hz` | `10.0` | Replan-check timer rate. |
| `min_replan_interval_s` | `0.5` | Minimum time between replans (new goals exempt). |
| `replan_deviation_m` | `0.5` | Replan if the robot is this far off the path (from v1, now implemented). |
| `replan_period_s` | `0.0` | Periodic replan. `0` = off. |
| `improvement_ratio` | `0.9` | A periodic replan is only adopted if the new path is shorter than this × the remaining path. |
| `start_escape_radius_m` | `0.3` | How far to search for a free start cell. |
| `occupied_threshold` | `50` | Grid value at or above this counts as occupied. |
| `unknown_is_free` | `true` | Treat `-1` cells as free. |

---

## Package changes

```
luna_guidance/
├── config/planner.yaml              + new parameters
├── package.xml                      + exec_depend python3-scipy
├── luna_guidance/
│   ├── grid.py                      + same_geometry(), occupied_mask(), inflate(), nearest_free_cell()
│   ├── path_check.py                NEW. is_path_blocked(path, inflated_grid, from_index). NO ROS IMPORTS.
│   ├── replan_policy.py             NEW. Decides whether and why to replan. NO ROS IMPORTS.
│   └── planner_node.py              /map change detection, flags, replan timer
└── test/
    ├── test_grid.py                 + inflation, change detection
    ├── test_path_check.py
    └── test_replan_policy.py
```

`replan_policy.py` should be a pure function, something like `should_replan(now, last_replan_time, new_goal, blocked, deviation_m, params) -> reason | None`. Then you can test the rate limiting and priorities without ROS or a real clock.

---