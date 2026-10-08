# Working autonomy candidate

**Status: WORKING — 2026–27.** This is not the approved robot baseline or a measured 95% success claim. It replaces the [2025–26 timed central planner](https://github.com/TAMU-Lunabotics/GNC-2026/blob/main/src/central_planner/central_planner/central_planner.py) with measured material, actuator and berm checks, and addresses the [old planner's](https://github.com/TAMU-Lunabotics/GNC-2026/blob/main/src/pathing2/src/path_planner2.cpp) lack of unknown-space policy. It must not run alongside archived drive or mission nodes.

## Proposed GNC ↔ CDH interfaces (not approved)

| Input/output | ROS type | Frame and units | Rate / stop condition |
| --- | --- | --- | --- |
| `/odometry/filtered` | `nav_msgs/Odometry` | `map` → `base_link`; m, rad; finite x/y/yaw variances | ≥2 Hz; stop after 0.5 s or uncertain pose |
| `/unilidar/cloud` | `sensor_msgs/PointCloud2` | TF to `base_link` at message stamp; m | ≥2 Hz; stop after 0.5 s; unknown blocked |
| `/camera/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map`; m/rad, surveyed fiducials | timeout 60 s; tag pose is not fused into odometry here |
| `/<camera_name>/color/*` and `depth/*` | `sensor_msgs/Image`, `CameraInfo` | Orbbec-style optical frame; depth 16UC1 mm or 32FC1 m | reject malformed or dimension-mismatched data; optional depth stop after 0.5 s |
| `/drive/encoders` | `std_msgs/Int32MultiArray` | ≥2 measured counts | ≥2 Hz; stop after 0.5 s |
| `/hopper/mass_kg`, `/battery/voltage` | `std_msgs/Float32` | kg, V | stop after 1 s / 2 s or invalid limit |
| `/actuator/status` | `std_msgs/String` | raised, lowering, lowered, digging, raising, dumping, stopped | ≥1 Hz; stop on invalid or stale state |
| `/safety/estop` | `std_msgs/Bool` | true means stop | ≥2 Hz; stop on stale/asserted input |
| `/berm/baseline_ready`, `/berm/volume_l` | `std_msgs/Bool`, `Float32` | baseline; placed volume in L | no start without baseline; no completion without fresh volume |
| `/autonomy/cmd_vel`, `/autonomy/tool_command` | `geometry_msgs/Twist`, `std_msgs/String` | m/s, rad/s; stop/lower/dig/raise/dump | 20 Hz; CDH must independently stop on 250 ms expiry, E-stop, or process loss |
| `/autonomy/obstacle_grid`, `/autonomy/planned_path`, `/autonomy/state` | `nav_msgs/OccupancyGrid`, `Path`, `std_msgs/String` | `map`; -1 unknown/0 observed free/100 hit | map/path 2 Hz, state 20 Hz, empty path on fault |
| `/autonomy/start`, `/autonomy/reset` | `std_srvs/Trigger` | explicit operator transition | latched fault requires manual reset |

Map extents are used as a geofence, not a prior wall/column map. ROS plans only through cells observed free along LiDAR beams; unseen space is blocked. The idealized 2-D simulator assumes a surveyed free arena and cannot validate slopes, holes, dust, glare, occlusion, serial timing, motor signs, or Jetson load.

**Before use:** GNC and CDH must jointly approve topics and sensor identities; measure footprint, wheel and tool geometry, limit/RPM feedback, hopper capacity, regolith bulk density, tag positions, camera/LiDAR extrinsics, ROI and battery limits; check one TF authority and real bag replay; build on ROS 2 Humble; test rover-level simulation and supervised physical runs. The launch is disarmed by default. The current competition requires at least two hands-free full cycles, real-time obstacle/map/path display, and the berm-volume multiplier target of 25,000 cm³. No trial count here supports a 95% success estimate.
