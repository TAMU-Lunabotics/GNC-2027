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

## Stage 1 path planner contract — WORKING — 2026–27

This section records the implementation on the path-planner branch; it does
not promote legacy topic names or hardware choices to an approved baseline.

| Direction | Topic | Type | QoS |
|---|---|---|---|
| Mapping -> planner | /map | nav_msgs/msg/OccupancyGrid | Reliable, transient local, depth 1 |
| Localization -> planner | /odometry/filtered | nav_msgs/msg/Odometry | Reliable, volatile, depth 10 |
| Mission/RViz -> planner | /goal_pose | geometry_msgs/msg/PoseStamped | Reliable, volatile, depth 1 |
| Planner -> controller/visualization | /guidance/path | nav_msgs/msg/Path | Reliable, transient local, depth 1 |
| Planner -> mission/visualization | /planning/status | luna_interfaces/msg/PlannerStatus | Reliable, transient local, depth 1 |


Positions and path lengths are meters; angles are radians; durations are
seconds. All input header frames must match map_frame (default map).
There is no TF transformation in the planner. Map origin translation and
planar rotation are supported. Hybrid planning requires valid planar
odometry orientation. The footprint is centered at the supplied robot pose.

Occupancy values are -1 or integers 0..100. Values >=50 are occupied by
default, and unknown cells are blocked by default. Map resolution is read
from the incoming message; the planner does not resample the grid.

Planning runs synchronously on a new goal, not at a periodic rate. Each goal
first clears the previous path and publishes PLANNING, followed by
SUCCEEDED or FAILED. Failure leaves an empty path. Startup publishes an
empty path and IDLE. Changed maps clear the path; valid changed maps produce
IDLE and require a new goal. Identical repeated maps are ignored.

Pose freshness is checked at goal processing against local receipt time
using the ROS node clock, with a default 0.5 s timeout. Input header timestamp
age and covariance are not assessed. Invalid odometry clears stored pose
validity but does not independently clear an already published path.
There is no periodic stale-pose watchdog in this planner.

Outputs use the ROS node clock; simulation requires use_sim_time with
a valid /clock source. Durations use a monotonic performance timer.
PlannerStatus.planning_time_s is measured before final success publication
and logging; external callback benchmarks include that remaining work.

Path samples preserve the planned straight segments with spacing no greater
than map resolution. The final pose carries the requested goal yaw.
No stop/turn action markers are encoded. Hybrid geometry assumes stationary
turns at checked locations; the current arc-following controller's
compatibility is unverified.

FAILED reasons include rejected inputs/starts, NO_ROUTE_IN_SEARCH,
EXPANSION_LIMIT, and GOAL_TURN_REJECTED. A search failure is not a proof of
physical impossibility. Expansion limits are distinct from no-route outcomes.
Final goal rotation is checked after search; alternative arrival headings
are not searched on rejection.

The map is treated as a snapshot. There is no automatic update-triggered
replan, deviation-triggered replan, or 0.20 s replanning guarantee.
See the [package README](../src/luna_guidance/README.md) for configuration,
measured performance, and validation scope.

Historical /path and /plan references in the task specification and
PlannerStatus comments are not the implemented output topic:
the implemented topic is /guidance/path. No message-schema change is made.
