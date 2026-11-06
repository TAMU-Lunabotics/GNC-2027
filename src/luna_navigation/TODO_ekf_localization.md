# global_ekf, local_ekf

## What it is

Right now `oracle_localizer` gives everyone the true pose, but of course on the real robot we don't know the true pose, we estimate it from sensors. This task builds the global pose estimate, meaning where the robot is in the arena, using `robot_localization`'s EKF. It fuses three things:

- wheel odometry: smooth and frequent, but it drifts (slip, and skid-steer scrub, especially in yaw)
- imu: good yaw rate, but it has a bias
- AprilTag detections: absolute position in the arena with no drift, but only when a tag is in view, and noisier

Odometry and the gyro carry the estimate between tag sightings, tags pull it back to the truth to reset drift.

```
/wheel_odom ────────────────────────────────────────┐
/imu/data_raw ──────────────────────────────────────┤
/cam_front/detections ─┐                            ├──► ekf_global ──► /odometry/filtered ──► planner, guidance
/cam_rear/detections ──┼──► tag_localizer ──► /tag_pose ┘          └─► TF map → odom
tags_yaml ─────────────┘          └─► /ekf_global/set_pose (first fix), /localization/initialized
```
```
/wheel_odom ───┬──────────────────────────────────────┐
/imu/data_raw ─┼──────────────────────────────────────┤
               │                                      │
               └──► ekf_local ──► /odometry/local     │
                         └─► TF odom → base_link      │
                                                      │
/cam_front/detections ─┐                              ├──► ekf_global ──► /odometry/filtered ──► planner, guidance
/cam_rear/detections ──┼──► tag_localizer ──► /tag_pose ┘          └─► TF map → odom
tags_yaml ─────────────┘          └─► /ekf_global/set_pose (first fix), /localization/initialized
```

Note, tag_localizer is a node I added. Since `robot_localization` can't read `TagDetectionArray`, this node converts each tag detection into a robot pose in `map`, with covariance.

**`ekf_global`**: a `robot_localization` `ekf_node` instance, set up with a YAML config. Here's the docs
https://docs.ros.org/en/melodic/api/robot_localization/html/index.html and a guide I used the first time I used the package https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/.

**`ekf_local`**: a `robot_localization` `ekf_node` instance, for handling local, smooth odometry. Theoreticallly, you should be able to use the same config file
just without the camera detection.

This is rover code: no `/sim/**` topics and no arena JSON. Tag positions come from the tag-layout YAML in config/.

Here's the local vs global explanation again. The global ekf will publish the `map->odom` transform, but the `odom->base_link` transform will be missing. A seperate local ekf will publish this transform. The reason we split it up this way is becasue having a global and local localization makes certain things easier. The global localization is where the robot is in the reference to the arena, it fuses all the sensors, notably including the camera.
The imu and encoders introduce drift into the estimate that the camera will fix periodically when it detects a tag, the problem is that this can cause the pose
estimate to jump around a few centimeters. This means the estimate coming out of the global ekf is not smooth and continuous. The local ekf only uses the imu and encoders, so it will drift, but the estimate stays continuous, and is usable for short-term situations.

Install with `sudo apt install ros-humble-robot-localization`.

---

## Interfaces

### Inputs

| Source | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/wheel_odom` | `nav_msgs/msg/Odometry` | `odom` / `base_link` | reliable, depth 10 | 50 Hz. |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | `imu_link` | reliable, depth 10 | 50 Hz in sim. The EKF fuses only the yaw rate. |
| `/tag_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `map` | reliable, depth 10 | From `tag_localizer`. One message per accepted tag detection, stamped with the detection's time. |
| `/ekf_global/set_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `map` | reliable, depth 1 | From `tag_localizer`. Initializes/resets the filter, published on the first good fix and on `/initialpose`. |

### Outputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | `map` / `base_link` | (robot_localization default) | From `ekf_global`. This is the topic the oracle publishes today, so the planner and guidance don't change. |
| `/odometry/local` | `nav_msgs/msg/Odometry` | `odom` / `base_link` | (robot_localization default) | From `ekf_local`.
| TF `map → odom` | | | | From `ekf_global`. |
| TF `odom → base_link` | | | | From `ekf_local`. |

---

## ekf_global config (`config/ekf_global.yaml`)

Start from this and tune it. The comments explain why each line is there.

```yaml
ekf_global:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true              # flat arena: ignore z, roll, pitch
    publish_tf: true              # publishes map → odom
    print_diagnostics: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: map              # makes this the GLOBAL filter

    # Wheel odometry: fuse twist only. The odom pose is the integral of the same twist,
    # so fusing both counts the same information twice.
    odom0: /wheel_odom
    odom0_config: [false, false, false,   # x, y, z
                   false, false, false,   # roll, pitch, yaw
                   true,  true,  false,   # vx, vy, vz
                   false, false, true,    # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
    odom0_differential: false
    odom0_queue_size: 10

    # Gyro: yaw rate only. There's no orientation, and the accelerometer adds mostly noise in 2D.
    imu0: /imu/data_raw
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,     # vyaw
                  false, false, false]
    imu0_differential: false
    imu0_queue_size: 10

    # Tags: absolute x, y, yaw from tag_localizer.
    pose0: /tag_pose
    pose0_config: [true,  true,  false,   # x, y
                   false, false, true,    # yaw
                   false, false, false,
                   false, false, false,
                   false, false, false]
    pose0_differential: false
    pose0_rejection_threshold: 5.0        # in standard deviations (Mahalanobis), rejects bad tags. This is kind of a random number, we can tune it if we see a lot of rejections in sim.
    pose0_queue_size: 10

    # Tag detections arrive late on the real robot (image processing).
    # This keeps a 1 s history and re-applies late measurements in the right place.
    smooth_lagged_data: true
    history_length: 1.0
```

Things to check:
- **`vy` covariance.** Fusing `vy = 0` only works if `/wheel_odom` puts a nonzero variance in `twist.covariance[7]`. Check with the odometry owner, or `ros2 topic echo /wheel_odom --once`.
- **QoS.** Check that each subscription actually connects: `ros2 topic info -v /imu/data_raw` should show `ekf_global` as a subscriber with compatible QoS. Do the same for the other inputs.
- **Process noise.** Leave `process_noise_covariance` at its default first, and only tune it after the error plots (see Testing) tell you to.
- **No gyro bias estimate.** `robot_localization` doesn't estimate sensor biases, so any gyro bias becomes heading drift between tags. Tags correct it, but expect some. If it's bad, tell me; we can subtract a bias measured while the robot sits still at startup.

---

## Parameters (`config/tag_localizer.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `tags_yaml` | *(set by launch)* | Known tag poses. |
| `map_frame` / `base_frame` | `map` / `base_link` | |
| `max_use_range_m` | `3.5` | Ignore detections of tags farther than this. |
| `init_max_range_m` | `2.5` | Only initialize from a tag this close. |
| `min_position_std_m` | `0.02` | Covariance floor (position). |
| `min_yaw_std_rad` | `0.02` | Covariance floor (yaw). |

All of these are ASSUMPTIONS; tune them in sim. Keep every value in YAML, none in code.

---

## Package structure (`luna_navigation`)

```
luna_navigation/
├── CMakeLists.txt                  ament_cmake (same pattern as luna_sim)
├── package.xml                     + exec_depend robot_localization, tf2_ros, tf2_geometry_msgs
├── README.md
├── TODO.md                         this file
├── config/
│   ├── ekf_global.yaml
│   └── ekf_local.yaml
├── launch/ekf.launch.py     ekf_global + ekf_local (with the set_pose remap for both), use sim_time
├── luna_navigation/
│   ├── __init__.py
│   └── tag_localizer_node.py       Publishes pose info from tag detections, with covariance.
├── scripts/tag_localizer.py        3-line wrapper → `ros2 run luna_localization tag_localizer`

```

I'll create the tag_localizer and handle `level1.launch.py` (the `localization:=oracle|ekf` argument). Your launch file just needs to start your two nodes with `use_sim_time` from launch.

---