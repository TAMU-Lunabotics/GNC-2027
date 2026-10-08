# GNC Interfaces — 2026–27

**Status: CURRENT DOCUMENT / INTERFACE BASELINE IN DEVELOPMENT**

GNC consumes robot-state/sensor data through CDH and publishes autonomy/motion outputs through explicit ROS 2 contracts.

## Important
No 2025–26 topic name, message schema, algorithm output, or command convention is automatically current. `/cmd_vel` and other prior conventions are **reference patterns only** until baselined here.

## Current Logical Data Flow
```
CDH Sensor / State Inputs
          |
          v
      Perception
          |
          v
 Localization / State Estimation
          |
          v
   Mapping / Planning
          |
          v
        Control
          |
          v
 Mission / Autonomy State
          |
          +----> CDH motion/actuator command path
          +----> Mission Control visualization
```

## Interface Contract
Every cross-team interface must define:
- status: CURRENT / WORKING / LEGACY
- producer / consumer
- ROS 2 topic/service/action and message type
- units and valid ranges
- coordinate frames and transforms
- timestamp source / synchronization assumptions
- update rate and latency
- covariance/quality/health fields where relevant
- timeout and stale-data behavior
- failure and fallback behavior

## 2026–27 Baseline Table
| Interface | Producer | Consumer | Status | Notes |
|---|---|---|---|---|
| Sensor data -> GNC | CDH | GNC | WORKING | Exact sensor set/topics TBD |
| Robot pose/state | Localization | Planning/Control | WORKING | Frame/covariance contract TBD |
| Obstacle/map representation | Perception/Mapping | Planning/MCC | CURRENT REQUIREMENT | Must support real-time judge-visible visualization |
| Planned path | Planning | Control/MCC | CURRENT REQUIREMENT | Resulting path must be visible in Mission Control |
| Motion command | Control | CDH | WORKING | Exact message/topic TBD |
| Autonomy state / cycle state | Autonomy | CDH/MCC | WORKING | Should expose hands-free/manual state and cycle phase |

Hardware drivers and low-level transport belong in CDH; GNC should operate against stable software interfaces.

## Working autonomy candidate

The new `src/lunabotics_autonomy` package is **WORKING**, not an approved current-season interface. Its proposed topic, frame, unit, rate, and timeout contracts are in [WORKING_AUTONOMY.md](WORKING_AUTONOMY.md). It publishes a measured obstacle grid and path for Mission Control, then sends logical motion and tool commands to CDH. It launches no motor controller. Joint GNC/CDH approval is required before hardware enablement.
