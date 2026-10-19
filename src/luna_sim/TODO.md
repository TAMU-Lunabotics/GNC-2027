# mock_localization_sensor


## What it is

Pretends to be the robot's wheel encoders and the odometry computed from them. It takes the robot's true motion from the simulator and produces what the real encoders would report, including the ways encoders can be inaccurate, a skid-steer robot scrubs when it turns, and encoders count in discrete ticks.

On the real robot, we'll read the data from the encoder and publish odometry based off on this to `/wheel_odom`. This node publishes to the same topic, so the localization filter can be developed against realistic odometry now, and swap to real hardware later without changing.

```
/sim/ground_truth/twist ──► mock_wheel_odometry ──► /wheel_odom     (integrated odometry)
robot_geometry.yaml ─────┘
```

This node lives in `luna_sim` because it reads ground truth, which only sim nodes may do. Its*outputs are normal topics that will be used on the real robot.

---

## Interfaces

### Inputs

| Source | Type | Frame | Notes |
|---|---|---|---|
| `/sim/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` | `base_link` | True body velocity from `kinematic_base`, 50 Hz: `linear.x` (m/s), `angular.z` (rad/s). The motion that *actually* happened. |
| `luna_description/config/robot_geometry.yaml` | YAML | — | Used to get `wheels.radius`, `wheels.separation`. Pass the path as the `geometry_yaml` parameter. |

### Outputs:

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/wheel_odom` | `nav_msgs/msg/Odometry` | `header.frame_id: odom`, `child_frame_id: base_link` | reliable, depth 10 | 50 Hz. Pose integrated from the encoders, starting at (0, 0, 0) when the node starts. Twist in `base_link`: `linear.x`, `angular.z` from the wheel speeds. Fill the covariance (see below). |

**Publishes NO TF.** On our robot, localization owns `odom → base_link`, publishing TF here would fight the oracle / EKF.
---

## Error

Use a zero-mean gaussian distibution to add error to the ground truth data you're using before publishing to `/wheel_odom` in order to make this more realistic. Later on we'll add more complicated and realistic way of introducing error, like modeling slip. Make this error get worse when the rover is moving faster

Integrate each wheel's angle, then quantize to `2π / ticks_per_rev` before publishing. Wheel velocity = change in quantized angle / dt, since encoders can only count discrete rotations, so velocity is noisy at low speed.

Integrate `(x, y, θ)` in the `odom` frame (midpoint heading, like `kinematic_base`).

The mismatch between step 1 (true geometry + slip) and step 4 (nominal geometry) is what makes the odometry drift. **You don't add drift by hand; it falls out of the physics.** That's the point: errors that look like the real robot's, for the same reasons.

Fill `pose.covariance` and `twist.covariance` (6×6, row-major; x, y, z, roll, pitch, yaw) based off of the noise you generated and added to the odometry data. Never all zeroes.

**Reproduciblity.** Seed the RNG from a `seed` parameter (launch passes the arena seed).


### Parameters (`config/sensors/mock_wheel_odometry.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `geometry_yaml` | *(set by launch)* | Path to `robot_geometry.yaml` (nominal wheel radius and separation). |
| `rate_hz` | `50.0` | Publish rate. |
| `ticks_per_rev` | `4096` | Encoder ticks per wheel revolution (after gearing). |
| `pose_noise_std_m` | `4096` | Standard deviation for pose accuracy. |
| `yaw_noise_std_rad` | `0.05` | Standard deviation for rotation accuracy. |
| `drift_per_m` | `0.02` | Drift in pose accuracy per m. |
| `drift_yaw_per_rad` | `0.02` | Drift in rotation accuracy per rad. |
| `seed` | `0` | RNG seed. |



All defaults are ASSUMPTIONS until we measure the real drivetrain.

---

## Structure (inside `luna_sim`)

```
luna_sim/
├── config/sensors/mock_wheel_odometry.yaml
├── luna_sim/sensors/
│   ├── __init__.py
│   └── mock_wheel_odometry.py
├── scripts/mock_wheel_odometry.py 3-line wrapper, add to PY_EXECUTABLES
└── test/test_wheel_model.py       plain pytest
```

Then add the node to `launch/level1.launch.py` with `geometry_yaml`, `seed` and `use_sim_time` parameters.

Make sure to read the `luna_sim` README before working.

---

# mock_obstacle_detector

## What it is

Pretends to be the robot's obstacle detection. It knows the real arena layout (from the arena JSON) and the robot's true pose, and publishes only what a real sensor would see, obstacles within range and field of view, with realistic position/size errors.

It stands in for the real perception pipeline (lidar → `obstacle_detector`) so the path planner can be developed now. Later, Gazebo and the real lidar replace it, publishing the same topic. The planner must not notice the swap.

```
/map (walls, from map_server) ──────────┐
artemis_arena_<seed>.json ──────────────┤
/sim/ground_truth/pose ─────────────────┴──► mock_obstacle_detector ──► /map_updates ──► planner, RViz
```

This node lives in `luna_sim` because it reads ground truth, which only sim nodes may do. Its output is a normal perception topic.

---

## Interfaces

### Inputs

| Source | Type | Frame | Notes |
|---|---|---|---|
| `arena_json` parameter | path to `artemis_arena_<seed>.json` | `map` | Read once at startup with `luna_sim.arena.layout.load_layout()`. `layout['obstacles']` is a list of `{name, kind, x, y, diameter, height \| depth}`. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `map` | reliable, transient local | The initial(empty) map. Gives the grid geometry (`info.resolution`, `info.width`, `info.height`, `info.origin`) and the starting cell values. Published once, wait for it before publishing anything. |
| `/sim/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | `map` | True robot pose from `kinematic_base`, 50 Hz. |
| `luna_description/config/robot_geometry.yaml` | YAML | `base_link` | Sensor mounting pose (`sensors.lidar_link.xyz`), so range is measured from the sensor, not the robot center. |

### Outputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | `map` | reliable, depth 10 | This topic takes in new maps and updates the map from `/map` using it. |

## Required behavior

1. Each cycle (`rate_hz`) perform detection. An obstacle is visible if the distance from the sensor to its edge (center distance − radius) is within `[min_range_m, max_range_m]` and it's inside the horizontal field of view, `fov_deg`, centered on the robot's heading.

2. Add sensor error. Gaussian noise on position (`position_noise_std_m`) and radius (`radius_noise_std_m`), scaled up with distance (e.g. ×(1 + d / max_range)).

3. Rasterize each detection as a filled circle, set cells whose **center** is inside the circle to 1. Boulders and craters are both obstacles. Never set a cell back to free, you only add.

**Reproducibility**: Seed the RNG from a `seed` parameter (launch passes the arena seed).

### Parameters (`config/sensors/mock_obstacle_detector.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `arena_json` | *(set by launch)* | Arena layout path. |
| `rate_hz` | `10.0` | Detection / update rate. |
| `min_range_m` / `max_range_m` | `0.3` / `3.0` | Detection range (to the obstacle edge). |
| `fov_deg` | `360.0` | Horizontal field of view. |
| `position_noise_std_m` / `radius_noise_std_m` | `0.03` / `0.02` | At zero range. |
| `seed` | `0` | RNG seed. |

All values are ASSUMPTIONS until the real sensor is characterized, keep them in the YAML, none in code.

---

## Structure (inside `luna_sim`)

```
luna_sim/
├── config/sensors/mock_obstacle_detector.yaml
├── luna_sim/sensors/
│   ├── __init__.py
│   └── mock_obstacle_detector.py  ROS node
├── scripts/mock_obstacle_detector.py   3-line wrapper, add to PY_EXECUTABLES
└── test/
    ├── test_visibility.py
    └── test_grid_patch.py
```

Add the node to `launch/level1.launch.py` with `arena_json=paths['json']`, `seed=seed`, `use_sim_time=True`, behind an `obstacle_sensor` argument (default `true`).

Conventions: `load_layout()` for the JSON, node clock only, timer-driven publishing, full imports. See the `luna_sim` README.

---

# mock_tag_detector

## What it is

The real robot finds out where it is partly by using fiducial markers, AprilTags in our case, mounted at known spots on the arena walls. A camera sees a tag, a detector measures where that tag is relative to the camera, and since we know where the tag is in the arena, localization can work backward to where the robot must be.

This node pretends to be the camera + tag detector. It knows where every tag is using the arena_layout.json and where the robot truly is (from simulation ground truth), works out which tags each camera could see, and publishes what a real detector would report, each visible tag's pose in the camera frame, with realistic noise.


```
artemis_arena_<seed>.json (tag poses) ──┐
robot_geometry.yaml (cameras) ──────────┤
/sim/ground_truth/pose ─────────────────┴─► mock_tag_detector ──► /tags/detections ──► (tag_localizer → EKF, later)
                                                       └─► /tags/markers  (RViz debug)
```

This node lives in `luna_sim` because it reads ground truth, which only sim nodes may do. Its output is exactly what the real `tag_detector` will publish, so the real one can replace it later without anything downstream changing.

**Read first:** `luna_sim/README.md` (how the sim fits together, how to add a node).

---

## Interfaces

### Inputs

| Source | Type | Frame | Notes |
|---|---|---|---|
| `arena_json` parameter | path to `artemis_arena_<seed>.json` | `map` | Read once with `luna_sim.arena.layout.load_layout()`. `layout['tags']` is a list of `{id, size, x, y, z, yaw}`. (x,y,z) is the tag center position, and `yaw` is the direction the tag's face points, in `map`. Tags are vertical (mounted on walls). |
| `geometry_yaml` parameter | path to `luna_description/config/robot_geometry.yaml` | `base_link` | Camera mounting poses: `sensors.cam_front`, `sensors.cam_rear` (`xyz`, `rpy`). |
| `/sim/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | `map` | True robot pose, publishesd at 50 Hz. |


### Outputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/cam_front/detections` and `/cam_rear/detections` | `luna_interfaces/msg/TagDetectionArray` | `cam_front_optical` or `cam_rear_optical` | reliable, depth 10 | One message per camera per cycle (so 2 × 10 Hz with two cameras), each with its camera's optical frame in `header.frame_id`. Publish even when empty. "No tags in view" is information. |
| `/tags/markers` | `visualization_msgs/msg/MarkerArray` | `map` | reliable, depth 1 | For debugging, a square for every tag in the arena. Highlight the ones currently detected, and a line from the camera to each detected tag. |

Each `TagDetection` in the array:

| Field | Value |
|---|---|
| `id` | From the arena layout. |
| `pose.pose` | Pose of the tag frame in the camera optical frame, plus noise. |
| `pose.covariance` | 6×6 row-major (x, y, z, roll, pitch, yaw). Plug in the noise variances you used when adding noise to the tag location. Never all zeros. |
### Frame conventions

| Frame | Axes |
|---|---|
| `map` | x along the arena length, y along the width, z up. |
| `base_link` | x forward, y left, z up. |
| `cam_front` / `cam_rear` (camera body) | Same as `base_link` convention: x forward out of the lens, y left, z up. Its pose relative to `base_link` is in the geometry YAML. |
| `cam_front_optical` / `cam_rear_optical` | z forward (out of the lens), x right, y down. This is the standard for all camera and image data. |
| tag frame | Origin at the tag center. x right, y up, z out of the tag face (toward a viewer looking at it). |

So a tag straight ahead of the camera, facing it, has a pose in the optical frame of roughly `position = (0, 0, d)` with the tag's z axis pointing back toward the camera (−z of the optical frame).

---

## The model

For each camera, each cycle:

1. **Camera pose in `map`** = T(map → base_link, from ground truth) · T(base_link → camera, from the geometry YAML) · T(camera → optical).
2. **For each tag**, compute the tag's position in the optical frame, `p = (x, y, z)`. The tag is visible only if all of these hold:
   - in front: `z > 0`;
   - in range: `min_range_m ≤ |p| ≤ max_range_m`;
   - in the field of view: `|atan2(x, z)| ≤ hfov/2` and `|atan2(y, z)| ≤ vfov/2`;
   - facing the camera: the angle between the tag's normal (its z axis) and the direction from the tag to the camera is ≤ `max_view_angle_deg` (a tag seen edge-on, or from behind, can't be detected)
4. **Noise:** position std `= position_noise_base_m + position_noise_per_m2 · d²` (pose estimates get much worse with distance), rotation std `= rotation_noise_base_rad + rotation_noise_per_rad · view_angle`. Add Gaussian noise to the tag pose in the optical frame, put the same variances in the covariance matrix of the detection message.
5. **Publish** one `TagDetectionArray` per camera, stamped with the ground-truth pose's time.

Reproduciblity: seed the RNG from a `seed` parameter so that results are reproducible (launch passes the arena seed).


### Parameters (`config/sensors/mock_tag_detector.yaml`)


| Parameter | Default | Meaning |
|---|---|---|
| `arena_json` | *(required)* | Path to the arena layout, set by `level1.launch.py`. |
| `rate_hz` | `10.0` | Publish rate. |
| `min_range_m` / `max_range_m` | `0.2` / `4.0` | Detection range. |
| `hfov` /`vfov`| `90.0` / `60.0` | Horizontal and vertical field of view. |
| `max_view_angle_deg` | `70.0` | The maximum angle between the tag and the camera at which the tag can be viewed. |
| `position_noise_base_m` | `0.03` | Base standard deviation for position noise. |
| `position_noise_per_m2` | `0.02` | How much position noise to add for every m^2 of distance from the tag.|
| `rotation_noise_per_rad` | `0.05` | Base standard deviation for rotation noise. |
| `rotation_noise_per_rad` | `0.0` | How much rotation noise to add for every radian of rotation between the robot and the z-axis of the tag| 
| `seed` | `0` | RNG seed. |

---