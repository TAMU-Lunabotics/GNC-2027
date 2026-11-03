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

## Mock IMU contract — WORKING — 2026–27

This simulation sensor supplies raw readings for localization development.

| Direction | Topic | Type | Frame | QoS |
|---|---|---|---|---|
| Simulated base -> mock IMU | `/sim/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` | `base_link` | Reliable, volatile, depth 10 |
| Mock IMU -> localization | `/imu/data_raw` | `sensor_msgs/msg/Imu` | `imu_link` | Reliable, volatile, depth 10 |

Input fields are forward speed `linear.x` in m/s and yaw rate `angular.z`
in rad/s. The input source runs at 50 Hz. The output timer uses the ROS node
clock at `rate_hz`, default 50 Hz and constrained to `0 < rate_hz <= 50`.
When input is slower or interrupted, output rate can fall below this setting.

Output gyro units are rad/s and accelerometer units are m/s². An upright,
stationary ideal sensor reports +9.81 on accelerometer z. Sensor values
include the configured bias/noise and are clipped at the configured ranges.
The node publishes no TF; the robot description owns `base_link -> imu_link`.

`include_mounting_offset` is a startup-only switch, enabled in the supplied
YAML. It adds `alpha x r + omega x (omega x r)` to base acceleration before
the sensor-frame rotation, using `sensors.imu_link.xyz` in meters. Both
angular velocity and angular acceleration are along the base z axis; angular
acceleration comes from successive accepted yaw rates and source timestamps.
A vertical-only offset has no effect for this planar motion. Abrupt yaw-rate
changes can create large tangential terms, subject to the configured sensor
range. Turning the switch off retains the original translation-free model.
This extension does not supply roll, pitch, or vertical rover dynamics.

The first valid input establishes the acceleration baseline and produces
no measurement. Each later accepted input advances the model using its
header timestamp. The timer publishes the latest new reading once, copying
that source timestamp exactly; it does not repeatedly publish an old reading.
Intermediate readings may be skipped if several arrive between timer ticks.

Incorrect frames, nonfinite motion, and invalid timestamp fields are rejected
and clear any pending output. Duplicate or older timestamps are ignored
without advancing model state. A backward ROS-clock jump or clock-source
change resets the model, pending output, and seeded random stream.

Orientation is unavailable: all quaternion components are zero and
`orientation_covariance[0] = -1`. Consumers must ignore orientation.
Gyro and accelerometer covariances are row-major 3x3 diagonal matrices
containing white-noise variance, `noise_density² * rate_hz`, excluding bias.
`gyro_variance_override` and `accel_variance_override` default to `-1.0`
(automatic); a positive override supplies an explicit assumed diagonal
variance in `(rad/s)^2` or `(m/s²)^2`. Overrides change covariance metadata
only, leaving generated readings and the seeded random sequence unchanged.
Disabling white noise requires a positive override for that sensor; otherwise
startup fails rather than publishing an all-zero covariance that ROS would
interpret as unknown. The ROS-independent model retains zero mathematical
variance for ideal test readings. No arbitrary covariance floor is inserted.

Sensor settings, covariance overrides, and `geometry_yaml` are read-only
after initialization. YAML and command-line startup overrides remain
supported; restart the node to change these settings. `use_sim_time` keeps
its normal ROS-managed dynamic behavior.

See the [simulation README](../src/luna_sim/READEME.md#mock-imu---working---2026-27)
for configuration, launch usage, model assumptions, and software validation.
