# luna_guidance

Path planning and path following. This package holds code the real rover runs, so nothing in it may use `/sim/**` topics or the arena JSON.

Currently implemented: **guidance_controller** (pure pursuit path follower). The path planner lives here too and is documented in `TODO.md`.

```
/guidance/path ─────┐
/odometry/filtered ─┴──► guidance_controller ──► /cmd_vel
                                            ├─► /guidance/status
                                            └─► /guidance/lookahead
```

## Run it

```bash
colcon build --symlink-install --packages-select luna_guidance
source install/setup.bash

# with the sim already running (ros2 launch luna_sim level1.launch.py seed:=42)
ros2 launch luna_guidance guidance.launch.py use_sim_time:=true
```

Give it a path by hand (a straight 2 m line along +x, in the `map` frame). The QoS flags matter: the path topic is "transient local".

```bash
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /guidance/path nav_msgs/msg/Path \
  "{header: {frame_id: map}, poses: [
     {pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}},
     {pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}},
     {pose: {position: {x: 3.0, y: 1.0}, orientation: {w: 1.0}}}]}"
```

Stop it by publishing an empty path (`poses: []`). Watch it with `ros2 topic echo /guidance/status`, and in RViz add a PointStamped display for `/guidance/lookahead`.

## How it works

| File | Job |
|---|---|
| `luna_guidance/pure_pursuit.py` | The control law. **No ROS imports.** `get_control(x, y, yaw, dt) -> (v, w)`. |
| `luna_guidance/guidance_controller_node.py` | ROS wrapper: stores the latest path and pose, runs the controller on a 20 Hz timer, publishes. |
| `config/guidance.yaml` | Every tunable parameter. |
| `test/test_pure_pursuit.py` | Plain pytest, no ROS: `cd src/luna_guidance && pytest test/` |

Each control cycle:
1. Find the closest point on the path (only searching a few segments ahead of the last one, so a path that doubles back on itself can't confuse it).
2. Pick the lookahead point: `lookahead_m` further along the path.
3. If the heading error to it is over `rotate_in_place_angle_rad`, rotate in place; otherwise drive the circular arc through it (`curvature = 2 * lateral_offset / distance^2`, `w = v * curvature`).
4. Slow down for the goal and for tight turns, clamp to the speed limits, and limit acceleration.

States: `IDLE` (no path), `FOLLOWING`, `GOAL_REACHED` (stays until a new path), `STOPPED` (with a reason: lost localization, or off the path by more than `max_cross_track_m`).

## Parameters

All in `config/guidance.yaml`. The ones beyond the original spec: `odom_timeout_s`, `rotate_exit_ratio`, `rotate_gain`, `min_speed`, `min_lookahead_m`, `progress_window`, `goal_decel_ratio`.
