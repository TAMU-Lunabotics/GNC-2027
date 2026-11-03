# luna_guidance

Path planning and path following. This package holds code the real rover runs, so nothing in it may use `/sim/**` topics or the arena JSON.

Implemented on this branch: **guidance_controller** (pure pursuit path follower) and the **WORKING Stage 1 path planner** described below. `TODO.md` preserves the original task specification.

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

## Stage 1 path planner — WORKING — 2026–27

This branch implements a custom heading-aware search with Theta*-style
shortcuts. It is not the Nav2 Theta* implementation benchmarked in the original
trade study.

### Run the planner

From the workspace root after building and sourcing the workspace:

```bash
ros2 run luna_guidance path_planner --ros-args \
  --params-file "$(ros2 pkg prefix luna_guidance)/share/luna_guidance/config/planner.yaml"
```

For simulation using `/clock`, also pass `-p use_sim_time:=true`.
The existing `guidance.launch.py` launches the controller, not the planner.
The YAML selects hybrid mode; running without it defaults to circle mode.

See [INTERFACES.md](../../docs/INTERFACES.md) for topics, QoS, frames, and
failure behavior. The runtime planner consumes an occupancy grid and uses
no simulation-only inputs.

### Implementation

| File | Responsibility |
|---|---|
| `luna_guidance/grid.py` | Grid/world transforms and occupancy conversion. |
| `luna_guidance/clearance.py` | Circular point and full-segment clearance against occupied cells and map boundaries. |
| `luna_guidance/theta_star.py` | Cell-based Theta* and collinear waypoint removal for circle mode. |
| `luna_guidance/heading_clearance.py` | Circle-first clearance with rectangular straight-motion fallback and circular turn checks. |
| `luna_guidance/heading_theta_star.py` | Heading-aware search, exact endpoints, parent shortcuts, and bounded expansion count. |
| `luna_guidance/path_planner_node.py` | ROS input validation, search selection, final path checks, sampling, and publication. |
| `scripts/path_planner.py` | Executable entry point. |
| `config/planner.yaml` | Working footprint and search settings. |

Hybrid mode first checks each straight segment using an enclosing circle. If
that rejects the segment, it checks the full swept rectangle at the segment's
heading. Thus a narrow straight section can be accepted without requiring
rectangular clearance everywhere. Stationary turns still require circular
clearance. Both checks reject obstacle and boundary contact.

The search keeps incoming-direction states, checks turns before departure,
and independently considers parent shortcuts. Experimental state merging at
turnable positions is configurable as listed below. This modified search
has no completeness or optimality guarantee; `NO_ROUTE_IN_SEARCH` does not
prove that the physical rover cannot reach the goal.

### Footprint, parameters, and execution assumptions

The dimensions below describe the measured old test rover, including augers,
with its reference point at the footprint center. They are not confirmed
for the replacement rover.

| Parameter | Supplied YAML | Meaning |
|---|---|---|
| `footprint_mode` | `hybrid` | Circle-first straight checks with rectangular fallback; `circle` remains supported. |
| `robot_length_m`, `robot_width_m` | `1.2319`, `0.7239` | Driving footprint dimensions. |
| `robot_radius_m` | `0.715` | Enclosing radius for circle checks and stationary turns. |
| `safety_margin_m` | `0.10` | Provisional clearance allowance. |
| `heading_error_deg` | `5.0` | Provisional orientation allowance for rectangular checks. |
| `turn_weight` | `0.0` | Cost is distance plus this weight times turn angle in radians. Zero does not explicitly discourage turns. |
| `max_expansions` | `20000` | Search expansion limit, not a wall-clock deadline. |
| `merge_turnable` | `true` | Experimental state merging; requires zero turning weight. |
| `allow_straight_departure` | `false` | Optional checked travel along the exact start heading when the start cannot turn. |

Additional node defaults are `map_frame: map`, `occupied_threshold: 50`,
`unknown_is_blocked: true`, and `odom_timeout_s: 0.5`. Parameters are read at
node startup; restart the node after changing its configuration.

Heading allowance covers orientation error around the planned centerline.
It does not measure lateral tracking or localization error. The margin and
heading allowance require rover validation.

Search assumes forward straight motion and stationary turns, without reverse
travel. The existing pure-pursuit controller follows arcs and has not been
validated against this motion model. A published `Path` contains no stop/turn
markers and does not enforce these motions. Controller compatibility remains
a handoff item even when the planner's geometry checks pass.

### Map resolution and measured performance

Retain **0.05 m/cell** as the working input-map resolution because it
preserved nine successful routes lost at 0.10 m/cell in the measured batch,
despite the planning-time shortfall documented below. The planner reads
`OccupancyGrid.info.resolution`; it does not set or resample the map.
A future contributor can supply **0.10 m/cell** without changing the search
code. Regenerate the occupancy grid at the new resolution; changing metadata
alone changes its geometry and is not a valid conversion.

Performance measurements were collected on a laptop running Ubuntu under WSL2.
The pre-cleanup batch used 174 saved seed/start/goal cases, three
requests per case at each resolution, for 1,044 requests:

| Measurement | 0.10 m/cell | 0.05 m/cell |
|---|---:|---:|
| Unique cases returning paths | 16/174 | 25/174 |
| Maximum full goal-callback time | 0.346 s | 5.040 s |
| Requests exceeding 1.0 s | 0/522 | 19/522 |
| Maximum successful-request time | 0.0034 s | 1.041 s |
| Peak planner-process RSS, decimal MB | 61.40 | 76.86 |

Both resolutions stayed below the 500 MB budget in this workload. The
retained 0.05 m setting **does not consistently meet the 1.0 s planning
target**. This is a disclosed limitation, not an approved requirement waiver.
At 0.10 m, nine cases successful at 0.05 m were lost: six final-rotation
rejections and three `NO_ROUTE_IN_SEARCH` results. These selected diagnostic
cases do not establish a general success-rate estimate.

Measurements cover the installed node, loaded map, collision data, search,
final checks, and callback publication calls. Map generation ran outside the
measured process. Initial map loading was timed separately. ROS delivery
latency and a separate costmap process were outside the measurement scope.
The benchmark JSON and exploratory generator scripts are external review
artifacts, not runtime dependencies or part of this submission.

### Tests and validation scope

From the workspace root, after sourcing ROS and the built workspace:

```bash
/usr/bin/python3 -m pytest -q \
  src/luna_guidance/test/test_grid.py \
  src/luna_guidance/test/test_theta_star.py \
  src/luna_guidance/test/test_clearance_planning.py \
  src/luna_guidance/test/test_planner_pipeline.py \
  src/luna_guidance/test/test_path_planner_node.py \
  src/luna_guidance/test/test_hybrid_planner.py
```

These tests cover transforms, occupancy conversion, circle-mode search and
segment checks, hybrid detours, rectangle-only passages, turn rejection,
exact endpoints, expansion limits, and ROS wrapper behavior. Hand-built unit
fixtures isolate geometry cases; they are not competition-layout benchmarks.
The pipeline tests use the final clearance methods, not the superseded
whole-cell inflation helper.

Validation evidence:

- Before this final polish, the ROS package built and all 35 planner tests
  passed directly and through colcon. CMake, XML, and whitespace checks
  passed. Planner files had no reported lint findings; package-wide lint
  reported existing controller/test formatting and docstring findings.
- After the circle-mode helper extraction and style changes, the ROS
  package build and all 35 planner regression tests passed again.
- Earlier installed-YAML callback checks passed for rectangle-only passage
  planning, unsafe final rotation rejection, prior-path clearing, and
  invalid odometry.
- The 174-case footprint comparison retained all 15 circle successes and
  found 10 additional hybrid routes; all 25 passed geometry and cost rechecks.
- Physical-rover and real sensor-map validation remain outstanding.

### Stage 2 handoff: possible development paths

Starting from the snapshot behavior documented in
[INTERFACES.md](../../docs/INTERFACES.md), the next owner could pursue these
options against JP's Stage 2 requirements. These are not implemented features:

1. **Add update-driven replanning around the current search.** Retain the
   active goal, associate each search with a map version, and define which
   map changes invalidate the active path. Move long searches out of the
   subscription callback and cancel or discard obsolete results. Specify
   stop/hold behavior while no current validated route is available.
2. **Measure and improve response time before choosing another search.**
   Profile search and collision checks on target hardware, bound work and
   cache memory, and define the update-to-valid-path latency measurement.
   Revalidate any cached result against map and footprint changes. Incremental
   search such as D* Lite is a candidate to evaluate if repeated full searches
   miss the Stage 2 budget; it is not implemented here and would still need
   the hybrid heading and collision constraints.
3. **Make planner/controller motion assumptions agree.** Establish how the
   controller follows rectangle-only sections and performs checked turns.
   Validate tracking, localization error, and skid before selecting margins.
   If the controller follows arcs, those swept motions need clearance checks;
   straight-segment clearance alone does not validate them. If reverse travel
   or constrained arrival headings become necessary, evaluate a state-lattice
   or Hybrid A* approach against extending this search.

The Stage 2 0.20 s replanning target has not been demonstrated. Keep the
current interface and controller unchanged until coordinated changes are
reviewed.
