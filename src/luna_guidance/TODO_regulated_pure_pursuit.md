# regulated_pure_pursuit

## What this is

This is an upgrade to the guidance controller. Plain pure pursuit drives at one speed and steers toward a point a fixed distance ahead. That's fine in theoretical and sim scenarios, but in the real world it can cause three problems:

- It takes tight turns at cruise speed, which can cause overshoot and slipping.
- It drives at full speed right past obstacles, even though the planner's path only clears them by the inflation margin, this can be risky and cause collisions.
- It reaches the goal at full speed and has to brake hard.

Regulated pure pursuit (RPP) keeps the same steering law and adds rules that slow the robot down in the above situations, when the path curves sharply, when the robot is close to an obstacle, and when it's approaching the goal. It also scales the lookahead distance with speed. The paper ("Regulated Pure Pursuit for Robot Path Tracking", Macenski et al.) is short and worth reading before you start.

```
/path ───────────────┐
/odometry/filtered ──┤
/map ────────────────┼──► guidance_controller ──► /cmd_vel ──► robot
/map_updates ────────┘          └─► /guidance/status, /guidance/lookahead, /guidance/collision_arc
```

It's the same node in the same package. Plain pure pursuit stays usable so we can compare the two in sim and show the difference at PDR. I recommend copying the `pure_pursuit.py` code into a new `regulated_pure_pursuit.py` file to get started. A lot of the logic is the same, just with velocity regulating added.

This is still rover code, so it can't use anything sim-only (no `/sim/**` topics, no arena JSON).

---

## What changes from v1

- New inputs: `/map` and `/map_updates`, used for obstacle proximity and the collision check.
- New file `regulated_pure_pursuit.py`. `pure_pursuit.py` stays as it is.
- New parameters (below).
- Outputs are unchanged, plus one new debug topic.
- New paths will now arrive while driving, because the planner replans when it discovers obstacles. Handling that cleanly is part of this task (see "New paths mid-run").

---

## Interfaces

### Inputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/guidance/path` | `nav_msgs/msg/Path` | `map` | reliable, transient local, depth 1 | Same as v1. List of waypoints representing desired path. |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | pose in `map`, twist in `base_link` | reliable, depth 10 | Same as v1. It will soon come from the EKF instead of the oracle, so expect small jumps when a tag corrects the pose. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `map` | reliable, transient local | Contains info about obstacles, initially empty. |
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | `map` | reliable, depth 10 | Updates the map with obstacle info as the mock_obstacle_detector finds them, and later on the LiDAR. |


### Outputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/cmd_vel/` | `geometry_msgs/msg/TwistStamped` | `base_link` | reliable, depth 1 | Only `linear.x` (m/s, + forward) and `angular.z` (rad/s, + counter-clockwise). Everything else 0. Published at a fixed rate (20 Hz), including zeros when stopped. Unchanged from previous version. |
| `/guidance/status` | `luna_interfaces/msg/GuidanceStatus` | — | reliable, transient local | `IDLE` / `FOLLOWING` / `GOAL_REACHED` / `STOPPED` + reason, cross-track error, distance to goal. Unchanged from previous version.|
| `/guidance/lookahead` | `geometry_msgs/msg/PointStamped` | `map` | best effort | For debugging/visualization. The point currently being steered toward. We can then see it in RViz. Unchanged from previous version. |
| `/guidance/collision_arc` | `nav_msgs/msg/Path` | `map` | best effort | **New.** The forward-simulated arc used by the collision check, for RViz. |

---

## The algorithm

Each control tick (timer, `rate_hz`):

1. **Find the carrot.**
   - Find the closest path point to the robot, searching *forward* from last tick's index. Never search backward, or the robot can snap to an earlier part of a path that loops near itself.
   - Lookahead distance: `L = clamp(|v_cmd| · lookahead_time_s, min_lookahead_m, max_lookahead_m)`, where `v_cmd` is the last commanded linear speed. The measured speed is too noisy to use here.
   - The carrot is the point where a circle of radius `L` around the robot crosses the path. **Interpolate** along the segment it crosses. Don't snap to the nearest waypoint, because snapping makes the steering twitch every time a waypoint is passed.
   - If the end of the path is closer than `L`, the carrot is the last pose.

2. **Curvature.** Same as v1: transform the carrot into `base_link` as `(x_c, y_c)`, then `κ = 2·y_c / (x_c² + y_c²)`.

3. **Rotate in place.** Same as v1. If the carrot's angle exceeds `rotate_in_place_angle_rad`, turn in place.

4. **Regulate linear speed.** This is the main difference from the standard pure pursuit. Start from `v = cruise_speed`, then apply each rule:
   - **Curvature:** `r = 1/|κ|`. If `r < regulated_min_radius_m`, then `v_curv = cruise_speed · r / regulated_min_radius_m`.
   - **Obstacle proximity:** `clearance = (distance from robot center to nearest occupied cell) − robot_radius_m`. If `clearance < proximity_clearance_m`, then `v_prox = cruise_speed · max(clearance, 0) / proximity_clearance_m`.
   - Take `v = max(min(v_curv, v_prox), min_regulated_speed)`. The floor keeps the robot from stalling in a tight spot where the scaling would go to ~0.
   - **Goal approach:** `d_goal` = remaining distance *along the path*. If `d_goal < approach_dist_m`, then `v = min(v, max(cruise_speed · d_goal / approach_dist_m, min_approach_speed))`.

5. **Angular velocity from curvature.** `ω = v · κ`.
   - Compute `ω` after regulating `v`. That way the robot stays on the same arc and only slows down. Scaling `v` and `ω` separately changes the arc and pulls the robot off the path.
   - If `|ω| > max_angular_speed`, scale `v` down so that `|ω| = max_angular_speed`. This keeps the arc too.

6. **Collision check.**
   - Forward-simulate(using calculus, dt * v = dx) the command `(v, ω)` for `collision_check_time_s`, in steps small enough that each step moves at most one grid cell. 
   - At each step, check a circle of `robot_radius_m` against the grid.
   - If anything collides, publish zero velocity and `STOPPED` with reason `collision ahead`. Don't try to steer around the obstacle, avoiding it is the planner's job so it will just make a new path in response.
   - Use the raw `robot_radius_m`, not the planner's inflated radius. Otherwise the robot stops next to every obstacle that the planner already accounted for.
7. **Limits.** Same as v1: clamp to the max speeds and apply the acceleration limits.

8. **Goal.** Same as v1: get within `goal_tolerance_m`, rotate to the final heading, then report `GOAL_REACHED`.


### New paths mid-run

The planner will now replan while the robot drives, so a new path during `FOLLOWING` is normal and should be handled smoothly. When one arrives:

- Reset the progress index, and search for the closest point on the new path starting from its beginning.
- Keep the current commanded velocity and let the acceleration limits smooth the change. Don't drop to zero between paths.
- Don't evaluate "goal reached" or "end of path" using state left over from the old path, this is can cause false-stop where the robot just stops moving for now reason.
- Only trigger rotate-in-place from the carrot's angle, after the closest-point search. The new path starts at the robot's pose, so the first point will often be slightly behind the robot.
- An empty path still means stop now.

---

## Parameters (`config/guidance.yaml`, additions)

| Parameter | Default | Meaning |
|---|---|---|
| `lookahead_time_s` | `1.5` | Lookahead = speed × this. |
| `min_lookahead_m` / `max_lookahead_m` | `0.3` / `0.9` | Lookahead bounds. The v1 `lookahead_m` is now only used by `pure_pursuit`. |
| `regulated_min_radius_m` | `0.9` | Slow down on turns tighter than this radius. |
| `robot_radius_m` | `0.60` | Same value as the planner. Later this will come from `robot_geometry.yaml`. |
| `proximity_clearance_m` | `0.4` | Slow down when the robot's *edge* is closer than this to an obstacle. |
| `min_regulated_speed` | `0.08` | Speed floor after the curvature and proximity rules. On the real robot it must be above the motor deadband. |
| `approach_dist_m` | `0.6` | Start slowing this far (along the path) from the goal. |
| `min_approach_speed` | `0.05` | Must be > 0, or the robot never arrives. |
| `collision_check_time_s` | `1.0` | How far ahead in time to check the commanded arc. |
| `occupied_threshold` | `50` | Grid value at or above this counts as occupied. |

These numbers are still guesses, so we can tune them in sim.

---

## Package changes

```
luna_guidance/
├── config/guidance.yaml                + new parameters
├── luna_guidance/
│   ├── pure_pursuit.py                 unchanged (still selectable)
│   ├── regulated_pure_pursuit.py       NEW. The control law above. NO ROS IMPORTS.
│   └── guidance_controller_node.py     ros node for the controller
└── test/
    ├── test_regulated_pure_pursuit.py
    └── test_obstacle_distance.py
```

- Both controllers keep the same interface: `get_control(pose, path, ...) -> (v, ω)`.
- RPP also needs obstacle distance. Pass it an `ObstacleDistance` object in the constructor; pure pursuit just doesn't get one.
- The node chooses the controller once at startup. There should be no `if controller == ...` checks scattered through the code.

---
