# guidance_controller

## What this is
This the code that determines how the rover should move so as to follow a given path. Note that while this code is currently being written for the simulation, this should be the code the rover will actually run.

```
/guidance/path ─────┐
/odometry/filtered ─┴──► guidance_controller ──► /cmd_vel/ ──► robot
                                            └─► /guidance/status, /guidance/lookahead
```

This code runs on the real rover, so it should not use anything sim-only(i.e no `/sim/**` topics, no arena JSON).

---

## Interfaces

### Inputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/guidance/path` | `nav_msgs/msg/Path` | `map` | reliable, transient local, depth 1 | The path to follow, robot -> goal. A new path replaces the old one, An empty path means stop now. |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | pose in `map`, twist in `base_link` | reliable, depth 10 | Robot pose and velocity, ≥ 30 Hz. This is an estimate, not the truth, so expect noise and occasional small jumps (see the mock localization spec). |

### Outputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/cmd_vel/` | `geometry_msgs/msg/TwistStamped` | `base_link` | reliable, depth 1 | Only `linear.x` (m/s, + forward) and `angular.z` (rad/s, + counter-clockwise). Everything else 0. Published at a fixed rate (20 Hz), including zeros when stopped. |
| `/guidance/status` | `luna_interfaces/msg/GuidanceStatus` | — | reliable, transient local | `IDLE` / `FOLLOWING` / `GOAL_REACHED` / `STOPPED` + reason, cross-track error, distance to goal. |
| `/guidance/lookahead` | `geometry_msgs/msg/PointStamped` | `map` | best effort | For debugging/visualization. The point currently being steered toward. We can then see it in RViz. |

---

Note: The 'cmd_vel' only uses 'linear.x' and 'angular.z' because it is in the reference frame of the robot, where the x-axis runs front to back, and rotation about z corresponds to rotating in the plane, not the reference frame of the map. These are the only two axes the robot can move on in its reference frame, so all the other components are always 0. Let me know if you need more clarification about this.

## Required behavior

1. **Path following.** It will probably be easier to implement pure pursuit first, and then once that works upgrade it to regulated pure pursuit, but it's up to you.
2. **Respect limits.** Clamp to `max_linear_speed` / `max_angular_speed` and limit acceleration, or the real robot will jerk and slip.

3. **Large heading error.** If the path starts behind the robot (|α| > ~60°), rotate in place first, then drive. Don't swing a wide arc.
4. **Stop safely.** Publish zero velocity and `STOPPED` with a reason when there's some problem.
5. **Fixed-rate loop.** Compute commands on a timer (20 Hz, whe can test different values later) using the latest pose, not inside the odometry callback.

### Parameters (`config/guidance.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `rate_hz` | `20.0` | Control loop rate. |
| `lookahead_m` | `0.5` | Pure-pursuit lookahead. |
| `cruise_speed` | `0.3` | m/s on straight path segments. |
| `max_linear_speed` / `max_angular_speed` | `0.5` / `1.0` | Hard limits (match the robot). |
| `max_linear_accel` / `max_angular_accel` | `0.5` / `1.5` | Per-second change limits. |
| `goal_tolerance_m` / `yaw_tolerance_rad` | `0.10` / `0.10` | When the goal counts as reached. |
| `rotate_in_place_angle_rad` | `1.05` | Turn in place first above this heading error. |
| `max_cross_track_m` | `0.75` | Give up and stop if this far off the path. |
---

These numbers are all kinda random, so feel free to change them if you feel the need.

## Package structure

```
luna_guidance/
├── CMakeLists.txt                 ament_cmake (same pattern as luna_sim)
├── package.xml
├── README.md                      
├── TODO.md                      This file
├── config/guidance.yaml         Holds the parameters for the guidance controller(all constants in your code should come from here so we have a single source of truth for them)
├── launch/guidance.launch.py
├── luna_guidance/
│   ├── __init__.py
│   ├── theta_star.py              The path planning algorithm. NO ROS IMPORTS.
│   ├── path_planner_node.py       ROS wrapper: subscriptions, timer, publishing, should be agnostic to the type of path planner used
│   ├── pure_pursuit.py            The control law. NO ROS IMPORTS.
│   └── guidance_controller_node.py    ROS wrapper: subscriptions, timer, publishing, should be agnostic to the type of guidance controller used
├── scripts/guidance_controller.py 3-line wrapper → `ros2 run luna_guidance guidance_controller` (see the luna/sim README.md for more info on scripts/ type directories or ask me)
```

**Keep the math out of the node.** `pure_pursuit.py` takes a pose `(x, y, yaw)`, a path as a list of `(x, y)`, and parameters, and returns `(v, ω)`. The controller node should
make an object from this file and call methods from it and then publish the output. So as example the node should do things like call `controller.get_control(...)` and get a tuple `(v, ω)`, which it then publishes.

Conventions: node clock only, `use_sim_time` from launch, full imports. See the `luna_sim` README.

---



# path_planner

## What this is

Given where the robot is, where it should go, and what's in the way, produce a collision-free path from the robot to the goal. The guidance controller (`guidance_controller_node.py`) then follows the path and produces velocity commands.

```
/map         ──────────┐ 
/odometry/filtered ────┼──► path_planner ──► /path ──► guidance_controller ──► /cmd_vel       
/goal_pose ────────────┘               └─► /planning/status
```

This code runs on the real rover, so it should not use anything sim-only(i.e no `/sim/**` topics, no arena JSON). This code runs on the real rover, so it should not use anything sim-only(i.e no `/sim/**` topics, no arena JSON). If it works in sim only because it can see things the real robot can't, it doesn't work.

---

## Interfaces

### Inputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | `map` | reliable, transient local | The robot's current understanding of the arena, including obstacles. |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | pose in `map` | reliable, depth 10 | Robot pose, ≥ 30 Hz. Use `pose.pose` as the start of the path. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `map` | reliable, depth 1 | Where to go. RViz's **2D Goal Pose** tool publishes exactly this, which is how you test. Later the mission planner sends goals instead. |

Note: For initial testing, use `/map_truth` instead of `/map`. This map has all the exact obstacles locations on it, where as for now `/map` will start empty and stay empty until obstacle detection is implemented.

### Outputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/path` | `nav_msgs/msg/Path` | `map` | reliable, transient local, depth 1 | Ordered poses from the robot's position to the goal. Spacing ≈ map resolution (0.05 m). Every pose's orientation = direction of travel at that point; the last pose's orientation = the goal's. An empty path means "stop". |
| `/planning/status` | `luna_interfaces/msg/PlannerStatus` | — | reliable, transient local | `SUCCEEDED` / `FAILED` + reason, path length, planning time. Publish on every plan attempt. |
---

## Required behavior

1. **Build a planning grid.** Start from `/map`, add every remembered obstacle as occupied cells, then **inflate** everything by the robot's radius plus a safety margin. After inflation the robot can be treated as a point: any free cell is a place the robot's center can be.
2. **Plan** from the robot's current pose to the goal on that grid.
3. **Smooth / simplify** the path enough that the controller isn't chasing a staircase. At minimum remove redundant collinear points; better, shortcut-smooth with line-of-sight checks.
4. **Replan when:**
   - a new goal arrives;
5. **Fail clearly.** Goal inside an obstacle, goal outside the arena, no route, or no pose yet → publish an empty `/plan` and `FAILED` with the reason. Never publish a path that goes through an obstacle.
### Parameters (`config/planner.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `robot_radius_m` | `0.60` | Half the robot's diagonal. Later: read from `luna_description/config/robot_geometry.yaml`. This will be used for obstacle-inflation later.|
| `safety_margin_m` | `0.10` | Extra inflation beyond the robot radius. |
| `replan_deviation_m` | `0.5` | Replan if the robot is farther than this from the path. |

---

## Package structure

```
luna_planning/
├── CMakeLists.txt                 ament_cmake (same pattern as luna_sim)
├── package.xml
├── README.md                      this file
├── config/planner.yaml
├── launch/planner.launch.py
├── luna_planning/                 Python package
│   ├── __init__.py
│   ├── grid.py                    occupancy grid helpers: world↔cell, inflate, add obstacles
│   ├── astar.py                   the algorithm. NO ROS IMPORTS.
│   ├── smoothing.py               path simplification. NO ROS IMPORTS.
│   └── planner_node.py            ROS wrapper: subscriptions, timers, publishing
├── scripts/path_planner.py        3-line wrapper → `ros2 run luna_planning path_planner`
└── test/
    ├── test_astar.py              plain pytest, no ROS
    └── test_grid.py
```

**Keep the algorithm out of the node.** `theta_star.py` takes a numpy grid, start cell and goal cell and returns a list of cells. It knows nothing about ROS. This makes testing easier
since the algorithm can be tested with pure python. `planner_node.py` only converts messages to arrays, calls the algorithm, and converts back, which will allow us to easily swap in other pathing algorithms if we want to switch or test others.

Conventions: node clock only, `use_sim_time` from launch, full imports. See the `luna_sim` README.

---
