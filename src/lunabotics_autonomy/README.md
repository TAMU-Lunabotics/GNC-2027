# 2026–27 autonomy candidate

**Status: WORKING.** This ROS 2 Humble package is a software candidate for GNC-2027. It is disarmed by default and requires a joint GNC/CDH interface decision and measured robot parameters before ROS or hardware trials.

The mission node executes at least two excavation, loaded travel, dump, and return cycles. It uses measured hopper mass and berm volume for completion, checks fresh sensor and safety feedback, and faults safely. It uses inflated A* and regulated lookahead tracking. In ROS, unseen map cells and unobserved rover footprint are blocked. Measured ground/range returns clear map cells only along actual LiDAR rays; reachable observed waypoints let the rover approach a goal beyond current sensor range. It publishes the obstacle grid and path for Mission Control.

The localization node owns `map → odom → base_link`. Calibrated signed wheel counts provide local motion. A surveyed fiducial initializes and corrects the map frame; bad tags and encoder discontinuities are rejected. Its uncertainty grows with travel, and the mission stops when that uncertainty exceeds its bound. The camera must be visible at startup, but tags need not remain in view throughout the field. The optional IMU rate consistency check is disabled until the IMU frame and covariance are verified. No other TF or odometry broadcaster may own these links.

GNC publishes `/autonomy/cmd_vel` and `/autonomy/tool_command` as logical commands. This package does not connect to motors or the Pico. CDH must supply independently guarded drivers and the input topics in `../../docs/WORKING_AUTONOMY.md`.

Run `python3 -m unittest discover -s tests -q` for pure Python tests, `python3 -m autonomy.sitl` for the idealized simulation, and `colcon build --packages-select lunabotics_autonomy` in ROS 2 Humble. The simulator assumes a surveyed free 2-D arena. Actual ROS 2 and Jetson timing were unavailable during this implementation.

For an **isolated synthetic ROS graph** on a Humble computer, use two terminals after building and sourcing the workspace:

```bash
ros2 launch lunabotics_autonomy graph_sitl.launch.py
ROS_DOMAIN_ID=213 ros2 run lunabotics_autonomy graph_acceptance
```

The launch selects domain 213 and never starts a CDH driver. Its synthetic LiDAR, camera pose, mass, berm and actuators exercise ROS topics, TF, mapping, localization and mission transitions. The acceptance runner checks two cycles, at least 25 L of simulated berm, and live map/path telemetry. It is **not** a physics, real camera, regolith, Pico or motor validation. Keep the graph separate from the rover network.

The `autonomy.launch.py` file remains disarmed until `armed:=true`, `geometry_confirmed:=true`, measured wheel radius, wheel track, encoder resolution, wheel speed, material and geometry settings, and correct CDH inputs are supplied. The LiDAR ground band and camera tag map/extrinsics must be measured. `camera_name` must match the separate Orbbec driver namespace, default `camera`. The synthetic graph's parameters must never be copied to the physical rover without measurement.
