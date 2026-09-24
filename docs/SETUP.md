# Development Setup

**Status: WORKING — 2026–27**

Use the shared Lunabotics software environment unless the team approves a change.

## Required
- Ubuntu 22.04 (native, dual boot, VM, or WSL2)
- ROS 2 Humble Desktop
- Git + GitHub SSH access
- VS Code with C++ / Python support
- colcon, rosdep, build-essential, CMake
- RViz2 and rqt

## Useful for GNC
- PlotJuggler for state/telemetry plots
- rosbag2 for repeatable sensor-data testing
- simulation/fake inputs before hardware is available

## Verify
```bash
source /opt/ros/humble/setup.bash
ros2 topic list

mkdir -p ~/lunabotics_ws/src
cd ~/lunabotics_ws
colcon build
source install/setup.bash
```

Current algorithms and sensor drivers should be tested against recorded/simulated data before hardware integration when possible.
