# TAMU Lunabotics — GNC (2026–27)

**Current development repository for the 2026–27 NASA Lunabotics season.**

Guidance, Navigation & Control (GNC) owns perception, localization/state estimation, planning, control, and autonomy behavior.

> **Version rule:** 2025–26 and 2024–25 algorithms/sensors are historical references only. They are not current 2026–27 selections unless explicitly promoted through a current design decision.

## 2026–27 Scope
- perception and obstacle detection
- state estimation / localization
- mapping and path planning
- drive / motion control
- excavation, travel, and dump autonomy logic
- autonomy-state outputs required for Mission Control visualization

## Current Working Development Baseline
As of September 2026:
- Ubuntu 22.04
- ROS 2 Humble
- C++ / Python
- modular boundaries for perception, localization, planning, control, and mission autonomy

**Current algorithms and sensor selections are not assumed from last year.** The 2025–26 A*, square-root-filter, PID/LQR, Unitree L1 LiDAR, BMI088 IMU, REV encoder, RGB-D camera, and mass-sensor choices are retained only as legacy design references until 2026–27 baselining.

## Repository Layout
```
src/perception/     current perception / obstacle processing
src/localization/   current state-estimation / localization
src/planning/       current mapping / path / trajectory planning
src/control/        current motion control
src/autonomy/       current mission-level autonomy
simulation/         algorithm and integration testing
tests/              unit / integration tests
docs/               architecture, interfaces, requirements, decisions
legacy/             only if old code is intentionally imported for reference
```

## Read First
- [Status & versioning](docs/STATUS_AND_VERSIONING.md)
- [2026–27 competition constraints](docs/COMPETITION_2026-27.md)
- [Interfaces](docs/INTERFACES.md)
- [2025–26 legacy baseline](docs/LEGACY_2025-26.md)
- [Decision log](docs/DECISIONS.md)

## Workflow
Develop on short-lived branches and merge through pull requests. Keep `main` representative of the **current 2026–27 system**, not a mixture of current and legacy implementations.
