# 2026–27 autonomy candidate

**Status: WORKING.** This ROS 2 Humble package is a software candidate for GNC-2027. It is disarmed by default and requires a joint GNC/CDH interface decision and measured robot parameters before ROS or hardware trials.

The mission node executes at least two excavation, loaded travel, dump, and return cycles. It uses measured hopper mass and berm volume for completion, checks fresh sensor and safety feedback, and faults safely. It uses inflated A* and regulated lookahead tracking. In ROS, unseen map cells are blocked; it publishes the observed obstacle grid and path for Mission Control.

GNC publishes `/autonomy/cmd_vel` and `/autonomy/tool_command` as logical commands. This package does not connect to motors or the Pico. CDH must supply independently guarded drivers and the input topics in `../../docs/WORKING_AUTONOMY.md`.

Run `python3 -m unittest discover -s tests -q` for pure Python tests, `python3 -m autonomy.sitl` for the idealized simulation, and `colcon build --packages-select lunabotics_autonomy` in ROS 2 Humble. The simulator assumes a surveyed free 2-D arena. Actual ROS 2 and Jetson timing were unavailable during this implementation.

The `autonomy.launch.py` file remains disarmed until `armed:=true`, `geometry_confirmed:=true`, measured material and geometry settings, and correct CDH inputs are supplied. `camera_name` must match the separate Orbbec driver namespace, default `camera`.
