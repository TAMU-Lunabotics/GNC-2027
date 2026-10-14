# luna_description

The robot's physical description: its size, wheels, and where every sensor is mounted, as a URDF. Used by both the real rover and the sim.

```bash
ros2 launch luna_description description.launch.py        # add use_sim_time:=true in sim
```

This starts `robot_state_publisher`, which publishes the robot's transforms: `base_link → chassis`, the wheels, and every sensor frame. It does not publish where the robot is in the arena (`map → odom → base_link`); that comes from localization (or the oracle in sim).

---

## How it works

```
config/robot_geometry.yaml ──► urdf/luna.urdf.xacro ──(xacro)──► URDF ──► robot_state_publisher ──► TF
        all the numbers              the structure                                (base_link → sensors)
```

- **`robot_geometry.yaml`** holds every dimension and sensor pose. This is the only file you edit to change the robot's geometry.
- **`luna.urdf.xacro`** is the template. It loads the YAML with `xacro.load_yaml()` and builds links and joints from it. It contains no numbers of its own.
- **`description.launch.py`** runs xacro at launch time and hands the result to `robot_state_publisher`.

Other nodes read the same YAML instead of keeping their own copies of the number.

---

## Layout

```
luna_description/
├── config/
│   └── robot_geometry.yaml     dimensions + sensor poses (kinda random rn, will come from CAD eventually)
├── urdf/
│   └── luna.urdf.xacro         robot template
├── launch/
│   └── description.launch.py   xacro → robot_state_publisher
├── CMakeLists.txt              installs config/, urdf/, launch/
└── package.xml
```

---

## Frames

| Frame | Where it is |
|---|---|
| `base_link` | On the ground, midway between the wheels, x forward, y left, z up. The robot's "position" is this frame. |
| `chassis` | Center of the body box. |
| `*_wheel` | Wheel centers. Continuous joints: their transforms come from `/joint_states`. |
| `imu_link` | IMU body frame. |
| `lidar_link` | Lidar frame (mounted upside down: `rpy: [π, 0, 0]`). |
| `cam_front`, `cam_rear` | Camera bodies, x forward like every other link. |
| `cam_front_optical`, `cam_rear_optical` | Optical frames: z forward, x right, y down. Image and tag-detection data use these, never the camera body frames. |

---

## Editing geometry

1. Change the numbers in `config/robot_geometry.yaml`. Units are meters and radians; poses are `xyz` + `rpy` relative to `base_link`.
2. Relaunch. With `--symlink-install`, no rebuild needed for edits to existing files.
3. Check it in RViz (below).

Every value is currently marked `ASSUMPTION` until the mechanical team confirms it. When a number becomes real, remove the tag and note where it came from (CAD revision, measurement).

### Adding a sensor

1. Add its pose under `sensors:` in `robot_geometry.yaml`:
   ```yaml
   sensors:
     my_sensor: {xyz: [0.30, 0.0, 0.40], rpy: [0.0, 0.0, 0.0]}
   ```
2. Add one line to `luna.urdf.xacro`:
   ```xml
   <xacro:sensor_frame name="my_sensor"/>
   ```
3. If it's a camera, also add `<xacro:optical_frame camera="my_sensor"/>`.
4. Add the frame to the table above and to `docs/rover_interfaces.md`.

---

## Checking it

```bash
# without ROS running: does the xacro produce a valid URDF?
xacro src/luna_description/urdf/luna.urdf.xacro \
  geometry:=src/luna_description/config/robot_geometry.yaml > /tmp/luna.urdf
check_urdf /tmp/luna.urdf               # sudo apt install liburdfdom-tools

# in RViz
ros2 launch luna_description description.launch.py
rviz2
```

In RViz: set Fixed Frame to `base_link`, then add a RobotModel display (Description Topic `/robot_description`) and a **TF** display. Check that each sensor frame sits where it physically is on the robot, and that the camera optical frames have z pointing out of the lens.

---