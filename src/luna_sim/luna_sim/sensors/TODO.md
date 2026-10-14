# mock_localization_sensor


## What is it

Mocks sensor emulating the driver encoders to give sim data that we will use to test state-estimation in the sim. It should take the ground truth pose of the robot, and noise that realistically models the noise
you'd see from using encoder odometry in real life.

```
/sim/ground_truth/twist ──► mock_wheel_odometry ──► /wheel_odom     (integrated odometry)
robot_geometry.yaml ─────┘ 
```


---

## Interfaces

### Inputs

| Topic | Type | Frame | Notes |
|---|---|---|---|
| `/sim/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | `map` | True pose, 50 Hz. |
| `/sim/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` | `base_link` | True velocity. |
| `luna_description/config/robot_geometry.yaml` | YAML | — | Contains robot specification numbers to use for odometry calculations. Pass the path as the `geometry_yaml` parameter. |

### Outputs:

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/wheel_odom` | `nav_msgs/msg/Odometry` | `header.frame_id: odom`, `child_frame_id: base_link` | reliable, depth 10 | 50 Hz. Pose integrated from the encoders, starting at (0, 0, 0) when the node starts. Twist in `base_link`: `linear.x`, `angular.z` from the wheel speeds. Fill the covariance (see below). |

---

## Error Model

Real localization has two layers, and this node should reproduce both, because **the planner and controller must handle both**:

Odometry drifts since the estimate is dead reckoning, and its error grows continuously and smoothly, so it shouldn't jump around. Modeling it as a drift state `(dx, dy, dθ)` that does a random walk while the robot moves might be a good idea, but honestly I'm entirely sure what the best way for doing this is. Should also add a small white noise on the true pose each message (`pose_noise_std_m`, `yaw_noise_std_rad`).
For reproducibility, RNG should be seeded from a `seed` parameter.

### Parameters (`config/sensors/mock_wheel_odometry.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `rate_hz` | `30.0` | Publish rate of `/odometry/filtered`. |
| `pose_noise_std_m` / `yaw_noise_std_rad` | `0.01` / `0.005` | White noise per message. |
| `drift_per_m` / `drift_yaw_per_rad` | `0.02` / `0.02` | Random-walk drift growth per meter traveled / radian turned(assuming you go that route). |
| `seed` | `0` | RNG seed. This will come from level1.launch.py, but while testing you can use the default.|

---

## Structure (inside `luna_sim`)

```
luna_sim/
├── config/sensors/mock_wheel_odometry.yaml
├── luna_sim/localization/
│   ├── __init__.py
│   ├── mock_obstacle_detector.py            drift, fixes, noise, status. NO ROS IMPORTS.
│   └── mock_localization_sensor.py         ROS node (TF + odometry + status)
├── scripts/mock_localization_sensor.py     3-line wrapper, add to PY_EXECUTABLES in cmake
└── test/test_error_model.py       plain pytest
```
---

# mock_obstacle_detector

## What it is

Pretends to be the robot's obstacle detection. It knows the real arena layout (from the arena JSON) and the robot's true pose, and publishes what a real sensor(lidar) would see, obstacles within range and field of view, with realistic position/size errors.

It stands in for the real perception pipeline (lidar → `obstacle_detector`) so we can test path planning with dynamic obstacles. Later, Gazebo and the real lidar replace it, publishing the same topic. The planner must not notice the swap.

```
artemis_arena_<seed>.json ──┐
/sim/ground_truth/pose ─────┴──► mock_obstacle_detector ──► /map_updates ──► path planner
                                                         └─► /perception/obstacle_markers (RViz)
```

---

## Interfaces

### Inputs

| Source | Type | Frame | Notes |
|---|---|---|---|
| `arena_json` parameter | path to `artemis_arena_<seed>.json` | `map` | Read once at startup with `luna_sim.arena.layout.load_layout()`. `layout['obstacles']` is a list of `{name, kind, x, y, diameter, height \| depth}`. |
| `/sim/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | `map` | True robot pose from `kinematic_base`, 50 Hz. |

### Outputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | `map` | reliable, depth 10 | A rectangular patch of the map. See "Patch rules" below. RViz's Map display on `/map` applies these automatically. |


Upon detecting that the robot is close enough and facing the right way to detect an obstacle, this node should use that obstacle's center and radius to fill in a circle on the map's grid to represent that the obstacle has been detected, and then publish this new grid to `/map_updates`.

---

## Required behavior

1. **Visibility.** An obstacle is detected only if:
   - its distance from the sensor is within `[min_range_m, max_range_m]`, and
   - it is inside the horizontal field of view (`fov_deg` centered on the robot's heading; 360 = all around).
   Use the distance to the obstacle's edge (center distance − radius), so large obstacles appear when their edge comes into range.
2. **Noise.** Add Gaussian noise to position (`position_noise_std_m`) and radius (`radius_noise_std_m`). Scale it up with distance (e.g. ×(1 + d / max_range)). This more closely models the uncertainty of real sensors.
3. **Misses.** Drop each visible obstacle with probability `miss_probability` per cycle. Real detectors flicker; the planner's memory must cope.
6. **Reproducible.** Seed the random generator from a `seed` parameter (launch passes the arena seed), that way results are reproducible.
7. **Fixed rate.** Publish on a timer at `rate_hz` using the latest pose, on the node clock (`use_sim_time`).

### Parameters (`config/sensors/mock_obstacle_detector.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `arena_json` | *(required)* | Path to the arena layout; set by `level1.launch.py`. You can hard code this one when testing.|
| `rate_hz` | `10.0` | Publish rate. |
| `min_range_m` / `max_range_m` | `0.3` / `3.0` | Detection range. |
| `fov_deg` | `360.0` | Horizontal field of view. |
| `position_noise_std_m` | `0.03` | At zero range. |
| `radius_noise_std_m` | `0.02` | At zero range. |
| `seed` | `0` | RNG seed. If hard coding the .json file, use the same seed(its the string of numbers in the file name) |

---

## Structure (inside `luna_sim`)

```
luna_sim/
├── config/sensors/mock_obstacle_detector.yaml
├── luna_sim/sensors/
│   ├── __init__.py
│   ├── visibility.py
│   └── mock_obstacle_detector.py  ROS node
├── scripts/mock_obstacle_detector.py   3-line wrapper, add to PY_EXECUTABLES in cmake
└── test/test_visibility.py        plain pytest
```

Conventions: `load_layout()` from `layout.py` for the JSON, don't parse it yourself. See the `luna_sim` README.