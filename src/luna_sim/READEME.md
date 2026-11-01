# luna_sim

Simulation for the Lunabotics competition.

**What belongs here:** anything that only exists because we're the simulation is running: the arena generator, fake sensors, mock hardware, ground truth, the scorer.

**What doesn't:** anything the real rover also runs (path planning, localization, mission logic, drivers). That code lives in its own package, and the sim *launches* it. Nothing in the autonomy stack may import from or depend on `luna_sim`.


---

## Quick start

```bash
# from the workspace root(i.e GNC-2027)
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to luna_sim
source install/setup.bash

# generate an arena and serve its maps (add rviz:=true to view them)
ros2 launch luna_sim arena.launch.py seed:=42

# run Level 1 with the configured mock sensors and localization
ros2 launch luna_sim level1.launch.py seed:=42

# or just generate the files
ros2 run luna_sim arena_gen --seed 42
```

Omit `seed` to get a random arena. The chosen seed is printed so you can reproduce it, use this when testing to make sure that if your work failed in one situation, you get it to work in that situation.

### Level 1 launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `seed` | `random` | Arena seed. Same seed + same config = same arena. |
| `config` | `artemis_arena.yaml` | Arena rules file: a path, or a name in `config/arena/`. |
| `rtf` | `1.0` | Real-time factor. `2.0` = twice as fast as real time. |
| `oracle_localization` | `true` | `false` when testing a real localization node instead. |
| `imu_sensor` | `true` | Publish simulated gyro and accelerometer readings. |
| `rviz` | `true` | Open RViz with `rviz/sim.rviz`. |
| `out_dir` | `luna_sim/generated` | Where generated arena files go. |
---


## What's running in Level 1

```
sim_clock ──/clock──► (every node runs on sim time)

arena_gen ──► generated/artemis_arena_<seed>.json + known/truth maps
                 │                         │
                 │                    map_server ──► /map (only what the robot has detected)
                 │                    map_server_truth ──► /map_truth (everything)
                 ▼
        kinematic_base ◄── /cmd_vel (through command-line, guidance controller later)
                 │
                 └──► /sim/ground_truth/pose, /twist
                              │
                       oracle_localizer ──► /odometry/filtered
                                            TF map → odom → base_link

luna_description: robot_state_publisher ──► TF base_link → chassis, wheels, sensors
```

| Node | Does | Replaced on the real robot by |
|---|---|---|
| `sim_clock` | Publishes `/clock`; sets sim speed (`rtf`). | Wall time (no `/clock`). |
| `kinematic_base` | Integrates `/cmd_vel` into the true pose (diff drive, speed limits, 0.5 s command timeout). Starts at the arena's start pose. | The actual robot + `diff_drive_controller`. |
| `oracle_localizer` | Publishes ground truth as the localization estimate. | The ESKF (`luna_localization`). |
| `mock_imu` | Produces raw IMU readings with seeded sensor errors. | The physical IMU driver. |
| `map_server`, `map_server_truth` | Serve the generated maps. | `map_server` with the known arena map. |

---

## Layout

```
luna_sim/
├── config/
│   ├── arena/artemis_arena.yaml   arena rules: dimensions, zones, obstacle ranges
│   └── scenarios/                 (planned) fault-injection scenarios
├── launch/
│   └── arena.launch.py            generate an arena, serve /map and /map_truth
├── luna_sim/                      Python source code (importable as luna_sim.*)
│   ├── arena/
│   │   ├── arena_gen.py           Takes an arena YAML and a seed, then generates maps and a layout JSON
│   │   ├── layout.py              load_layout(): reads an arena .json file, all nodes needing arena data should import this
│   │   └── paths.py               helper functions for where generated files go and what they're named
│   └── eval/
│       └── scorer.py              (planned) scores sim runs
│   ├── kinematic_base.py          moves the robot following a differential drive model, using the /cmd_vel topic
│   ├── oracle_localizer.py        publishes perfect state-estimation
│   └── sim_clock.py
├── rviz/sim.rviz                  saved RViz layout (QoS already set correctly)
├── scripts/                       executables for `ros2 run` (thin wrappers)
├── generated/                     generated arenas (git-ignored)
├── worlds/                        arenas worth keeping (checked in)
├── src/                           C++ source (none yet)
├── CMakeLists.txt
└── package.xml
```

### What goes where

| Folder | Contents | In git? |
|---|---|---|
| `config/` | Hand-written settings: arena rules, scenarios. | Yes |
| `launch/` | Launch files. | Yes |
| `luna_sim/` | All Python logic. Every folder needs an `__init__.py`. | Yes |
| `rviz/` | RViz layouts. Edit in RViz, then File → Save Config. | Yes |
| `scripts/` | One 3-line wrapper per runnable command (see below). No logic. | Yes |
| `generated/` | Output of `arena_gen`: per seed, a `.json` layout plus `known` and `truth` maps (`.pgm` + `.yaml`). Disposable. | **No** |
| `worlds/` | Arenas we want to keep, e.g. a seed that caught a bug. Copy them here from `generated/`. Gazebo worlds later. | Yes |
| `src/` | C++ nodes, if we ever need them. | Yes |

---

## Generated arenas

Each run of `arena_gen` writes to `generated/`:

| File | What it is | Who uses it |
|---|---|---|
| `artemis_arena_<seed>.json` | Exact obstacle positions, zones, start pose. Ground truth. | Sim nodes and the scorer, via `load_layout()`. **Never the autonomy stack.** |
| `artemis_arena_<seed>_known.pgm` / `_known.yaml` | empty map, what the robot knows before a run. | `map_server` → `/map` (used for path planning) |
| `artemis_arena_<seed>_truth.pgm` / `_truth.yaml` | All obstacles | `map_server_truth` → `/map_truth` (RViz, debugging) |

File names and locations come from `luna_sim/arena/paths.py`. Don't build these paths by hand.

---

## Adding a runnable node or tool

`luna_sim` is an `ament_cmake` package, which has no `entry_points` like `ament_python`. So each command needs a small executable in `scripts/` that calls into the Python source:

1. Put the logic in `luna_sim/<area>/<name>.py` with a `main()` function.
2. Add `scripts/<name>.py`:
   ```python
   #!/usr/bin/env python3
   from luna_sim.<area>.<name> import main

   if __name__ == '__main__':
       main()
   ```
3. Make it executable: `chmod +x scripts/<name>.py`
4. Add `<name>` to `PY_EXECUTABLES` in `CMakeLists.txt`.
5. Rebuild. It now runs as `ros2 run luna_sim <name>`.

Naming: snake_case, script name = command name = node name.

---

## Rules

- **Ground truth stays in the sim.** `/sim/**`, `/map_truth` and `arena_<seed>.json` are for sim nodes, the scorer and debugging only.
- **Use the node clock.** Never `time.time()` or `time.sleep()`; everything runs on `use_sim_time`.
- **Use full imports.** `from luna_sim.arena.paths import ...`, never `from paths import ...`.
- **Nothing generated goes in git.** Promote arenas you want to keep into `worlds/`.

---

## Mock IMU - WORKING - 2026-27

`mock_imu` supports localization development against a simulated gyroscope
and accelerometer. The message contract is in
[INTERFACES.md](../../docs/INTERFACES.md#mock-imu-contract--working--202627).

### Run

The Level 1 launch enables the IMU by default and passes the arena seed,
geometry YAML, and simulation-time setting:

```bash
ros2 launch luna_sim level1.launch.py seed:=42 imu_sensor:=true
```

Use `imu_sensor:=false` to omit it. Inspect the output from another sourced
terminal with `ros2 topic echo /imu/data_raw --once` or
`ros2 topic hz /imu/data_raw`.

### Model and configuration

The model derives forward acceleration from successive stamped speeds and
lateral acceleration from speed times yaw rate. It includes +9.81 m/s²
specific force on an upright stationary sensor's z axis, then rotates into
the IMU mounting frame.

The working YAML enables `include_mounting_offset`. Before rotating the
accelerometer reading into the sensor frame, this adds the rigid-body terms
`alpha x r + omega x (omega x r)`, where `r` is the mounting position relative
to the base origin. Yaw acceleration is estimated from successive stamped
yaw rates. Set the flag to `false` to retain the original task model that
ignores mounting translation. See the
[rigid-body acceleration derivation](https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/befffaf20475c1a379c3ac52e91a78cb_MIT16_07F09_Lec21.pdf).

The current geometry places the IMU at `(0, 0, 0.20)` m. A purely vertical
offset produces no additional acceleration during planar yaw motion. Abrupt
changes in yaw rate can produce large finite-difference tangential terms
for horizontal offsets; configured sensor saturation still applies. This
extension assumes a fixed mounting position and planar motion. It does not
model rover roll, pitch, or vertical terrain motion.

Each axis has an independently initialized bias, a bias random walk scaled
by `sqrt(dt)`, and white noise with standard deviation
`noise_density * sqrt(rate_hz)`. Full-scale clipping follows bias and noise.
The same seed and accepted input sequence reproduce the model readings.
Stationary readings include the configured noise and bias.

Sensor settings are defined in
[config/sensors/mock_imu.yaml](config/sensors/mock_imu.yaml). The defaults
are development assumptions. Mounting RPY and XYZ come from
`luna_description/config/robot_geometry.yaml`, under `sensors.imu_link`.
XYZ is required when mounting-offset effects are enabled; the disabled
mode also accepts older RPY-only geometry configurations.
Sensor, mounting, and covariance parameters accept startup overrides and are
read-only afterward. Edit the configuration and restart the node to change
them. The ROS-managed `use_sim_time` parameter retains its normal behavior.

Published covariance normally equals the generated white-noise variance.
`gyro_variance_override` and `accel_variance_override` default to `-1.0`,
meaning automatic. Positive values explicitly override the reported diagonal
variance in `(rad/s)^2` and `(m/s²)^2`, respectively, without changing the
generated readings or random sequence. These values represent assumed
uncertainty for the consuming filter, not measured noise characteristics.

When disabling white noise, supply a positive variance override for that
sensor. Otherwise startup fails with a configuration error: ROS interprets
an all-zero covariance as unknown. The pure model still supports exact
zero-noise measurements and zero mathematical variance for unit tests.

| File | Responsibility |
|---|---|
| `luna_sim/sensors/imu_model.py` | ROS-independent physics, errors, covariance, and saturation. |
| `luna_sim/sensors/mock_imu.py` | Configuration, input validation, clock handling, and timed publication. |
| `scripts/mock_imu.py` | Executable entry point. |
| `test/test_imu_model.py` | 28 deterministic, offset-geometry, and statistical model tests. |
| `test/test_mock_imu_node.py` | 23 ROS configuration, parameter, covariance, mounting, clock, and topic-delivery tests. |

### Software validation

Ubuntu 22.04/WSL2 with ROS 2 Humble: all 51 functional tests passed,
along with the package build, CMake/XML checks, and IMU Python lint.
A headless Level 1 check using seed 42 verified the disabled launch flag,
then received at least 100 IMU samples with the expected seed,
simulation-time setting, frame, source timestamps, covariance, and
stationary gravity.

Offset tests cover centripetal and tangential acceleration, combined base
motion, sensor-frame rotation, baseline equivalence, rejected-input history,
configuration validation, and clock-reset behavior. Nonzero offsets in unit
fixtures are synthetic test cases, not measured rover mounting positions.

Run the registered functional tests after building and sourcing the workspace:

```bash
ctest --test-dir build/luna_sim \
  -R '^(test_imu_model|test_mock_imu_node)$' --output-on-failure
```

### Possible improvements

- **Characterize the physical IMU.** Use stationary recordings to estimate
  white-noise densities and supported bias random-walk parameters. Use
  repeated power-on recordings at a fixed orientation to estimate startup
  bias spread. Record the sensor model, ranges, sample rate, and filter
  settings with the data. Match the YAML to the tested hardware configuration;
  retain assumption labels for parameters without adequate measurements.
- **Support full 3D simulated rover motion.** Add synchronized ground-truth
  orientation and three-axis motion, then extend gravity projection and
  fixed-mount acceleration while retaining the planar mode. A Gazebo path
  needs a physical robot model, collision geometry, and suitable terrain,
  with its mechanical parameters measured or explicitly assumed. Coordinate
  the localization/planner interface for tilted poses before using this
  extension in the full autonomy stack.
