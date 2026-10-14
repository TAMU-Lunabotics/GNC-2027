# Mock Localization (Noisy Localizer)

**Owner:** _______ · **Status:** v1 in progress · **Package:** `luna_sim` (sim only) · **Node:** `noisy_localizer`

## What it does

Pretends to be the robot's localization system, with **realistic errors**. It publishes exactly what the real localization (the ESKF) will publish, but builds it from simulator ground truth plus a model of how localization goes wrong: noise, slow drift, sudden corrections when a tag is seen, and periods with no correction at all.

Why it exists: the current `oracle_localizer` is *perfect*, so the planner and controller have never been tested against the imperfect pose they'll get on the real robot. This node sits between the oracle and the real ESKF:

| Mode | Node | Use it to |
|---|---|---|
| Perfect | `oracle_localizer` | Develop planning/guidance logic. |
| **Realistic errors** | **`noisy_localizer`** | Prove planning/guidance survive real-world localization. |
| Real | ESKF (`luna_localization`, later) | The actual thing. |

All three publish the same interfaces. **Only one may run at a time**: they all publish `map → odom → base_link`.

```
/sim/ground_truth/pose ──┐
/sim/ground_truth/twist ─┴──► noisy_localizer ──► /odometry/filtered
                                               ├─► TF map → odom, odom → base_link
                                               └─► /localization/status
```

---

## Interfaces

### Inputs

| Topic | Type | Frame | Notes |
|---|---|---|---|
| `/sim/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | `map` | True pose, 50 Hz. |
| `/sim/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` | `base_link` | True velocity. |
| `arena_json` parameter *(optional)* | path | `map` | Only needed if fixes depend on where tags are (v2). |

### Outputs: identical to the oracle and the future ESKF

| Topic / TF | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | pose in `map`, twist in `base_link`, `child_frame_id: base_link` | reliable, depth 10 | ≥ 30 Hz. **Covariance must be filled** with your model's current uncertainty (diagonal: x, y, z, roll, pitch, yaw). Never all zeros. |
| TF `odom → base_link` | `tf2_msgs/TFMessage` | — | — | **Smooth.** Drifts slowly but never jumps. |
| TF `map → odom` | `tf2_msgs/TFMessage` | — | — | **Jumps** at corrections. Absorbs the drift. |
| `/localization/status` | `luna_interfaces/msg/LocalizationStatus` | — | reliable, transient local | `OK` / `DEGRADED` / `LOST`, position and yaw σ, seconds since the last fix. |

Message definition: `luna_interfaces/msg/LocalizationStatus.msg`.

---

## The error model (what makes this realistic)

Real localization has two layers, and this node should reproduce both, because **the planner and controller must handle both**:

1. **Odometry drifts.** Between absolute fixes, the estimate is dead reckoning, and its error grows continuously and smoothly. Model it as a drift state `(dx, dy, dθ)` that does a **random walk** while the robot moves (scale the step with distance traveled and with turning). The `odom → base_link` transform = truth + drift. Smooth, so the controller sees gradual error, never a jump.
2. **Fixes cause jumps.** Every so often the robot "sees a tag" and corrects. At a fix, set the `map`-frame estimate to truth + small fix noise, then put the correction in **`map → odom`** so `odom → base_link` stays continuous. This is exactly how REP-105 says it should look, and what the ESKF will do.

Plus:

3. **Measurement noise.** Small white noise on the published `map`-frame pose each message (`pose_noise_std_m`, `yaw_noise_std_rad`).
4. **Fix schedule.** A fix every `fix_period_s` on average (randomized). **Blackout windows:** with `blackout_probability`, skip fixes for `blackout_duration_s`. Drift grows unchecked, as when no tag is visible.
5. **Status follows the model.**
   - `OK` normally;
   - `DEGRADED` when `seconds_since_absolute_fix > degraded_after_s`;
   - `LOST` when the estimated σ exceeds `lost_std_m`.
   - Covariance and `position_std_m` grow with time since the last fix and shrink at each fix.
6. **Reproducible:** RNG seeded from a `seed` parameter.

### Parameters (`config/sensors/noisy_localizer.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `rate_hz` | `30.0` | Publish rate of `/odometry/filtered`. |
| `pose_noise_std_m` / `yaw_noise_std_rad` | `0.01` / `0.005` | White noise per message. |
| `drift_per_m` / `drift_yaw_per_rad` | `0.02` / `0.02` | Random-walk drift growth per meter traveled / radian turned. |
| `fix_period_s` | `3.0` | Mean time between absolute fixes. |
| `fix_noise_std_m` | `0.03` | Error left after a fix. |
| `blackout_probability` / `blackout_duration_s` | `0.05` / `10.0` | Chance per fix of entering a no-fix window, and its length. |
| `degraded_after_s` / `lost_std_m` | `5.0` / `0.5` | Status thresholds. |
| `seed` | `0` | RNG seed. |

All defaults are ASSUMPTIONS, to be replaced with real ESKF performance numbers once it exists.

---

## Structure (inside `luna_sim`)

```
luna_sim/
├── config/sensors/noisy_localizer.yaml
├── luna_sim/localization/
│   ├── __init__.py
│   ├── error_model.py             drift, fixes, noise, status. NO ROS IMPORTS.
│   └── noisy_localizer.py         ROS node (TF + odometry + status)
├── scripts/noisy_localizer.py     3-line wrapper; add to PY_EXECUTABLES
└── test/test_error_model.py       plain pytest
```

`error_model.py` should be a class with something like `update(true_x, true_y, true_yaw, dt) -> (map_pose, odom_pose, map_to_odom, sigma, status)`. All the interesting logic lives there and is testable without ROS. Use `oracle_localizer.py` as the template for the node side (TF broadcasting, odometry message, covariance layout).

Then change `launch/level1.launch.py`'s `oracle_localization` argument into `localization:=oracle|noisy|none` (default `oracle`) and start your node for `noisy`.

---

## Testing

**Unit tests (no ROS):** all noise parameters 0 → output equals truth exactly; robot standing still → no drift growth; drift grows with distance traveled; at a fix, `map` estimate snaps near truth **and** `odom → base_link` doesn't jump; status goes `OK → DEGRADED → LOST` during a long blackout; same seed → same sequence.

**In the sim:**
```bash
ros2 launch luna_sim level1.launch.py seed:=42 localization:=noisy
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
In RViz show **both** `/sim/ground_truth/pose` (Pose display, one color) and `/odometry/filtered` (Odometry display, another color). Drive around: the estimate should wander slowly away from the truth and snap back at fixes. Plot error over time in PlotJuggler: x of truth vs x of estimate. You should see a sawtooth (growing drift, sudden correction).

```bash
ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true   # should change only at fixes
ros2 topic echo /localization/status
```

## Definition of done (v1)

- [ ] Unit tests pass for the cases above.
- [ ] In RViz: estimate drifts smoothly, corrects at fixes; `map → odom` changes only at fixes.
- [ ] Covariance and status behave as specified during a forced blackout.
- [ ] Planner + guidance reach goals on 5 seeds with `localization:=noisy` (that's the point of this node).

## Later (v2)

Fixes only when a tag is actually in a camera's field of view (needs the tag layout in the arena JSON); latency between measurement and publish; occasional bad fixes (outliers) to test robustness.

# Mock Obstacle Detector

**Owner:** _______ · **Status:** v1 in progress · **Package:** `luna_sim` (sim only) · **Node:** `mock_obstacle_detector`

## What it does

Pretends to be the robot's obstacle detection. It knows the real arena layout (from the arena JSON) and the robot's true pose, and publishes **only what a real sensor would see**: obstacles within range and field of view, with realistic position/size errors and occasional misses.

It stands in for the real perception pipeline (lidar → `obstacle_detector`) so the **path planner can be developed now**. Later, Gazebo and the real lidar replace it, publishing the same topic. The planner must not notice the swap.

```
arena_<seed>.json ──────────┐
/sim/ground_truth/pose ─────┴──► mock_obstacle_detector ──► /perception/obstacles ──► path planner
                                                         └─► /perception/obstacle_markers (RViz)
```

This node lives in `luna_sim` **because it reads ground truth**, which only sim nodes may do. Its *output* is a normal perception topic.

---

## Interfaces

### Inputs

| Source | Type | Frame | Notes |
|---|---|---|---|
| `arena_json` parameter | path to `arena_<seed>.json` | `map` | Read once at startup with `luna_sim.arena.layout.load_layout()`. `layout['obstacles']` is a list of `{name, kind, x, y, diameter, height \| depth}`. |
| `/sim/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | `map` | True robot pose from `kinematic_base`, 50 Hz. |
| `luna_description/config/robot_geometry.yaml` | YAML | `base_link` | Sensor mounting pose (`sensors.lidar_link.xyz`), so range is measured from the sensor, not the robot center. Optional in v1. |

### Outputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/perception/obstacles` | `luna_interfaces/msg/ObstacleArray` | `map` | reliable, depth 10 | Published at `rate_hz` (10 Hz) **every cycle**, even if empty. `header.stamp` = time of the pose used. |
| `/perception/obstacle_markers` | `visualization_msgs/msg/MarkerArray` | `map` | reliable, depth 1 | **Debug.** Cylinders for detections (e.g. boulders orange, craters blue). |

Message definitions: `luna_interfaces/msg/ObstacleArray.msg`, `Obstacle.msg`. Fill every field:

| Field | Value |
|---|---|
| `id` | Stable per real obstacle (its index in the JSON list). The planner uses it to remember obstacles. |
| `kind` | `BOULDER` or `CRATER`, from the JSON `kind`. |
| `position` | True center + noise. `z = 0`. |
| `radius` | True `diameter / 2` + noise, never below a small positive minimum. |
| `height` | Boulders: `+height`. Craters: `-depth`. |
| `confidence` | 1.0 near the robot, falling toward `0.5` at max range. |

---

## Required behavior

1. **Visibility.** An obstacle is detected only if:
   - its distance from the sensor is within `[min_range_m, max_range_m]`, and
   - it is inside the horizontal field of view (`fov_deg` centered on the robot's heading; 360 = all around).

   Use the distance to the obstacle's **edge** (center distance − radius), so large obstacles appear when their edge comes into range.
2. **Noise.** Add Gaussian noise to position (`position_noise_std_m`) and radius (`radius_noise_std_m`). Scale it up with distance (e.g. ×(1 + d / max_range)). Far detections are worse.
3. **Misses.** Drop each visible obstacle with probability `miss_probability` per cycle. Real detectors flicker; the planner's memory must cope.
4. **(Optional) false positives.** With probability `false_positive_rate` per cycle, add a small fake boulder at a random visible spot, `confidence` ≤ 0.6, with a fresh `id` range (e.g. ≥ 10000).
5. **(Optional, v2) occlusion.** Hide obstacles fully behind a nearer boulder. Skip in v1.
6. **Reproducible.** Seed the random generator from a `seed` parameter (launch passes the arena seed). Same seed → same noise sequence.
7. **Fixed rate.** Publish on a timer at `rate_hz` using the latest pose, on the node clock (`use_sim_time`).

### Parameters (`config/sensors/mock_obstacle_detector.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `arena_json` | *(required)* | Path to the arena layout; set by `level1.launch.py`. |
| `rate_hz` | `10.0` | Publish rate. |
| `min_range_m` / `max_range_m` | `0.3` / `3.0` | Detection range. |
| `fov_deg` | `360.0` | Horizontal field of view (Mid-360 lidar ≈ 360). |
| `position_noise_std_m` | `0.03` | At zero range. |
| `radius_noise_std_m` | `0.02` | At zero range. |
| `miss_probability` | `0.05` | Per obstacle, per cycle. |
| `false_positive_rate` | `0.0` | Per cycle. Off by default. |
| `seed` | `0` | RNG seed. |

Every number here is an ASSUMPTION until we characterize the real sensor; keep them all in the YAML, none in code.

---

## Structure (inside `luna_sim`)

```
luna_sim/
├── config/sensors/mock_obstacle_detector.yaml
├── luna_sim/sensors/
│   ├── __init__.py
│   ├── visibility.py              in_range / in_fov / noise helpers. NO ROS IMPORTS.
│   └── mock_obstacle_detector.py  ROS node
├── scripts/mock_obstacle_detector.py   3-line wrapper; add to PY_EXECUTABLES
└── test/test_visibility.py        plain pytest
```

Then add the node to `launch/level1.launch.py` with `arena_json=paths['json']`, `seed=seed`, `use_sim_time=True`, behind an `obstacle_sensor` launch argument (default `true`).

Conventions: `load_layout()` for the JSON (never parse it yourself); node clock only; full imports. See the `luna_sim` README.

---

## Testing

**Unit tests (no ROS):** obstacle at 2 m straight ahead → visible; at 5 m → not; behind the robot with `fov_deg: 90` → not; with 360 → visible; edge-distance logic (big obstacle, center out of range, edge in range → visible); noise = 0 → exact values; same seed → same output.

**In the sim:**
```bash
ros2 launch luna_sim level1.launch.py seed:=42
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
In RViz show `/map_truth` (where obstacles really are) and `/perception/obstacle_markers` (what the robot sees). Drive around: markers should appear as the robot approaches obstacles, sit on top of the true ones with a little jitter, and disappear as it drives away.

```bash
ros2 topic hz /perception/obstacles      # ~10 Hz, steady
ros2 topic echo /perception/obstacles --once
```

## Definition of done (v1)

- [ ] Unit tests pass for the cases above.
- [ ] In RViz, detections line up with `/map_truth` within the configured noise and only appear in range/FOV.
- [ ] Publishes at a steady 10 Hz, including empty arrays.
- [ ] Same seed + same drive → same detections.
- [ ] Planner owner has used it to plan around obstacles.