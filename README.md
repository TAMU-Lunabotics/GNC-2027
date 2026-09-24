# TAMU Lunabotics — GNC

Guidance, Navigation & Control (GNC) software for the Texas A&M Lunabotics team.

## Scope
GNC owns the robot's autonomy and motion intelligence:
- sensor interpretation and perception
- state estimation and localization
- path and motion planning
- drive and autonomous control
- digging/dumping motion logic
- autonomous and teleoperated motion behavior

## Current Platform
- Ubuntu 22.04
- ROS 2 Humble
- C++ / Python
- NVIDIA Jetson Orin Nano integration through CDH
- current architecture favors team-developed perception, localization, and navigation

Relevant robot sensing includes 4D lidar, encoders, and other subsystem sensors exposed through CDH interfaces.

## Repository Layout
```
src/perception/     perception and environment processing
src/localization/   state estimation and localization
src/planning/       path / trajectory planning
src/control/        motion and drive control
src/autonomy/       mission-level autonomy logic
simulation/         algorithm and integration testing
tests/              unit/integration tests
docs/               architecture and interface documentation
```

## Integration
Typical control flow:

```
Sensor Topics -> GNC Estimation / Planning / Control -> Command Output -> CDH / Motor Interface
```

Interfaces should explicitly define messages, timestamps, coordinate frames, units, update rates, health flags, and failure behavior.

## Workflow
Develop on feature branches and merge into `main` through pull requests. Keep `main` integration-ready.
