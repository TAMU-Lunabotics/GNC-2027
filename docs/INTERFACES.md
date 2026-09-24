# GNC Interfaces

GNC consumes robot data through CDH and returns motion/autonomy commands through defined ROS 2 interfaces.

## Current Data Flow
```
CDH Sensor Topics
       |
       v
Perception / Estimation
       |
       v
Localization / Planning
       |
       v
     Control
       |
       v
Motion Command (current pattern: /cmd_vel)
       |
       v
CDH / Motor Interface
```

## Interface Rules
Every cross-team interface should define:
- ROS 2 topic/service/action and message type
- units and valid ranges
- coordinate frame
- timestamp source
- expected update rate
- health/status indication
- timeout and failure behavior

Use SI units unless explicitly documented otherwise.

## GNC Responsibilities
- perception and sensor interpretation
- state estimation and localization
- path / trajectory planning
- drive and motion control
- mission/autonomy behavior
- map/path/state outputs needed for debugging and visualization

Hardware drivers and low-level transport belong in CDH; GNC should operate against stable ROS 2 interfaces.
