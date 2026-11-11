# luna_guidance

Path planning and path following. This package holds code the real rover runs, so nothing in it may use `/sim/**` topics or the arena JSON.

Implemented on this branch: **WORKING — 2026–27 Stage 2 replanning** and
**guidance_controller**, which supports pure pursuit and regulated pure pursuit
(RPP). Stage 2 reuses the Stage 1 search and clearance geometry. The original
task is preserved in `TODO_dynamic_path_planning.md`; differences from that
task are explained below and in [INTERFACES.md](../../docs/INTERFACES.md).

```
/guidance/path ─────┐
/odometry/filtered ─┴──► guidance_controller ──► /cmd_vel
                                            ├─► /guidance/status
                                            └─► /guidance/lookahead
```

## Run it

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to luna_guidance luna_sim luna_description
source install/setup.bash

# In one terminal, with the built workspace sourced:
ros2 launch luna_sim level1.launch.py seed:=5740 obstacle_sensor:=true \
  obstacle_detector_profile:=round2 rviz:=false

# In another terminal, with the same workspace and ROS domain sourced:
ros2 launch luna_guidance guidance.launch.py use_sim_time:=true use_map_updates:=true
```

The launch starts the hybrid planner and regulated pure pursuit by default.
`use_map_updates` defaults to `false` for the rover's complete `/map` input and
is enabled above for Level 1 patches. Use `footprint_mode:=circle` to reproduce
the earlier Round 2 regression trials. `controller_type:=pure_pursuit` selects
the original follower with the circle planner for controller comparisons.

After `/map_updates` has a publisher and a received patch, send a goal from
a third terminal on the same ROS domain:

```bash
ros2 topic pub --once --keep-alive 3 /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 5.85, y: 3.4}, orientation: {w: 1.0}}}'
```

For a controller-only demonstration, stop the planner before giving the
controller a path by hand. This example is a straight 2 m line along +x in
the `map` frame; the path topic is transient local.

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
| `luna_guidance/pure_pursuit.py` | Regular pure pursuit and the shared path state and command limits. |
| `luna_guidance/regulated_pure_pursuit.py` | Regulated pure pursuit with speed regulation and circular collision checks. |
| `luna_guidance/hybrid_path_follower.py` | Regulated following of hybrid planner legs with rectangular motion and circular turn checks. |
| `luna_guidance/guidance_controller_node.py` | ROS wrapper: stores the latest path and pose, runs the controller on a 20 Hz timer, publishes. |
| `config/guidance.yaml` | Every tunable parameter. |
| `test/test_pure_pursuit.py` | Plain pytest, no ROS: `cd src/luna_guidance && pytest test/` |

In circle mode each control cycle:
1. Find the closest point on the path (only searching a few segments ahead of the last one, so a path that doubles back on itself can't confuse it).
2. Pick the lookahead point: `lookahead_m` further along the path.
3. If the heading error to it is over `rotate_in_place_angle_rad`, rotate in place; otherwise drive the circular arc through it (`curvature = 2 * lateral_offset / distance^2`, `w = v * curvature`).
4. Slow down for the goal and for tight turns, clamp to the speed limits, and limit acceleration.

States: `IDLE` (no path), `FOLLOWING`, `GOAL_REACHED` (stays until a new path), `STOPPED` (with a reason: lost localization, or off the path by more than `max_cross_track_m`).

## Parameters

All in `config/guidance.yaml`. The ones beyond the original spec: `odom_timeout_s`, `rotate_exit_ratio`, `rotate_gain`, `min_speed`, `min_lookahead_m`, `progress_window`, `goal_decel_ratio`.

## Stage 1 search used by Stage 2 — WORKING — 2026–27

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
The current `guidance.launch.py` launches both planner and controller.
The supplied YAML selects hybrid mode. A standalone node started without a
parameter file defaults to circle.

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
| `footprint_mode` | `hybrid` | Circle-first straight checks with rectangular fallback. The Round 2 regression launch selects `circle` explicitly. |
| `robot_length_m`, `robot_width_m` | `1.2319`, `0.7239` | Driving footprint dimensions. |
| `robot_radius_m` | `0.715` | Enclosing radius for circle checks and stationary turns. |
| `safety_margin_m` | `0.10` | Provisional clearance allowance. |
| `heading_error_deg` | `5.0` | Provisional orientation allowance for rectangular checks. |
| `turn_weight` | `0.0` | Cost is distance plus this weight times turn angle in radians. Zero does not explicitly discourage turns. |
| `max_expansions` | `20000` | Search expansion limit, not a wall-clock deadline. |
| `merge_turnable` | `true` | Experimental state merging; requires zero turning weight. |
| `allow_straight_departure` | `true` | Hybrid option for checked travel along the exact start heading when the start cannot turn. |

Additional node defaults are `map_frame: map`, `occupied_threshold: 50`,
`unknown_is_blocked: true`, and `odom_timeout_s: 0.5`. Parameters are read at
node startup; restart the node after changing its configuration.

Heading allowance covers orientation error around the planned centerline.
It does not measure lateral tracking or localization error. The margin and
heading allowance require rover validation.

Hybrid search assumes forward straight motion and stationary turns. The
hybrid follower collapses collinear samples from the planner's `Path` and
keeps the original turn waypoints. It limits the lookahead to the active leg
and checks the full rectangular sweep of its steering corrections. At each
turn it stops translation and checks the enclosing circle before rotating.
The controller uses the measured footprint and heading allowance. The
planning margin provides space for tracking error, including on the approach
to a turn. A start escape may use the planner's relaxed clearance check.

The controller regression includes a simulated 1.2 m opening followed by
a turn. The earlier native multi-seed evidence below uses circle mode and
regulated pure pursuit. The hybrid configuration needs its own native
driving and timing measurements.

### Historical Stage 1 map resolution and performance

These earlier measurements predate the Stage 2 search optimizations and do
not describe the current Round 2 timing results. They are retained as the
Stage 1 baseline, including its disclosed limitations.

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

### Tests and historical Stage 1 validation

From the workspace root, after sourcing ROS and the built workspace:

```bash
colcon test --packages-select luna_guidance --event-handlers console_direct+
colcon test-result --test-result-base build/luna_guidance --verbose

# Or run every guidance test directly:
/usr/bin/python3 -m pytest -q src/luna_guidance/test
```

These tests cover transforms, occupancy conversion, circle-mode search and
segment checks, hybrid detours, rectangle-only passages, turn rejection,
exact endpoints, expansion limits, and ROS wrapper behavior. Hand-built unit
fixtures isolate geometry cases; they are not competition-layout benchmarks.
The pipeline tests use the final clearance methods, not the superseded
whole-cell inflation helper.

Historical Stage 1 validation evidence:

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

## Stage 2 replanning — WORKING — 2026–27

The planner retains the goal and checks the remaining route after meaningful
map changes. Identical maps and obstacles already behind the robot preserve
the active route. A blocked route, changed map geometry, excessive deviation,
or unavailable input clears the path and holds the controller. An empty path
immediately publishes a zero command and resets controller velocity state,
even if a replacement arrives before the next 20 Hz controller tick.

The planner advances progress within a forward window of `replan_deviation_m`
along the route for each distinct position. Repeated positions keep the same
projection. A corrected pose beyond the allowed deviation from that window
causes a hold and replan, even when a later leg is nearby.

A single search worker plans from a snapshot of the accepted map and pose.
A new goal, a changed map geometry, or invalid inputs cancel obsolete work.
Occupancy changes within the same geometry let the search finish. Before
publishing, the executor checks pending inputs and validates the candidate
against the latest map and pose. A rejected candidate leaves the goal active
for a rate-limited retry. A start inside the extra planning margin can use
a checked escape segment, but the
physical footprint must remain clear. Optional periodic improvement is off.
The implementation uses exact swept-clearance checks and a cached circular
center grid; it does not add a SciPy inflation dependency.

### Choices that differ from the original task

- **Replan cooldown:** a newly blocked active route bypasses the 0.5 s
  minimum interval once. Waiting for that interval could exceed the 0.20 s
  response target before search starts. Retries after cancellation or failure
  and optional improvement searches remain rate limited. Separate newly
  blocked replacement routes can each receive priority; this is not a fixed
  global CPU-utilization limit.
- **Unknown cells:** `unknown_is_blocked` retains its Stage 1 default of
  `true`. The existing RPP collision checker also blocks unknown cells.
  Setting the planner parameter to `false` permits optimistic global plans
  but does not make RPP follow them through unknown space. Exploration needs
  a mapper that marks observed free space and coordinated planner/controller
  behavior. The tested Level 1 base map starts with cells marked free, so
  these trials do not validate exploration through an unknown-valued map.
- **Interfaces:** the runtime output remains `/guidance/path`, matching the
  existing controller. Occupancy uses the ROS grid convention, `-1` or
  `0..100`, with occupied threshold 50 by default. The task's shorthand
  `/path` and its statement that value 1 means occupied are not the current
  runtime contract.

These are working implementation choices for review, not approved changes
to the team's requirements. The original task file remains unchanged.

### Measured Round 2 validation

The pre-review November 10, 2026 WSL2 / Ubuntu 22.04 / ROS 2 Humble run passed
21/21 selected arena-goal trials: seven generated arenas, three fresh launches
per arena, with detector noise seed fixed at 47659. All 42 blocked-route
replacements were within 200 ms: range 48–130 ms, median 82 ms. Every trial
passed path hold, zero command during hold, retained goal, arrival within
0.10 m, and driven truth-map clearance checks. Maximum final goal error was
0.0980814 m. The clearance check tests circular sweeps along straight
connections between recorded poses; it does not reconstruct the exact
curved trajectory between samples or prove hardware contact avoidance.

Replacement timing is measured from the triggering planner callback to the
path-publication call. It excludes sensing delay before that callback and
physical stopping response after a command.

This evidence uses the circle footprint (0.715 m radius plus 0.10 m planning
margin), RPP, simulated obstacle detection with far-detection refinement,
oracle localization, simulation time, `/map_updates`, and Cyclone DDS with
the test machine's local unicast configuration. The DDS configuration and
regression runner are test-environment artifacts, not rover runtime inputs.
The initial free-valued map is a simulation assumption, not proof that unseen
real space is free.

Use `obstacle_detector_profile:=round2` to reproduce the detector settings
for these trials. The profile enables far-detection refinement and fixes
the detector noise seed at 47659. The `seed` launch argument independently
selects the arena. The default profile loads the normal detector settings.

After the workspace cleanup, two native runs with circle selected built all
11 packages and passed 106 guidance tests, 104 simulator tests, and eight
repository tests. Both workspace reports had zero lint and XML findings.
Each run passed 21 drives and recorded 42 blocked-route replacements
within 200 ms. The combined observed range was 38–99 ms.

### Remaining validation

Target-computer timing and memory, other arena and detector-noise cases,
production full-map delivery, real perception and localization, and hybrid
footprint driving remain separate validation items. The old rover dimensions
and clearance margin require confirmation for the replacement rover. No
statistical reliability estimate or hard real-time guarantee follows from
the selected simulation trials.
