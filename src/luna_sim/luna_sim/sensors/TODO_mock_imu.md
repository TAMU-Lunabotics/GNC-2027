# mock_imu

## What it is

This node pretends to be the robot's IMU. The EKF uses the gyro to track heading, because the gyro is much better than the wheels at measuring yaw rate on a skid-steer robot, especially while it's turning. Real MEMS gyros have noise and a bias that wanders slowly, and that bias is the main reason heading drifts. The mock needs both, or the EKF gets tested against a sensor that's better than the real one.


```
/sim/ground_truth/twist ──► mock_imu ──► /imu/data_raw ──► ekf_localization
robot_geometry.yaml ─────┘
```

This node lives in `luna_sim` because it reads ground truth, which only sim nodes may do. Its output is a normal sensor topic.

---

## Interfaces

### Inputs

| Source | Type | Frame | Notes |
|---|---|---|---|
| `/sim/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` | `base_link` | True body velocity from `kinematic_base`, 50 Hz: `linear.x` (m/s), `angular.z` (rad/s). |
| `geometry_yaml` parameter | path to `luna_description/config/robot_geometry.yaml` | `base_link` | IMU mounting pose: `sensors.imu_link` (`xyz`, `rpy`). |

### Outputs

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | `imu_link` | reliable, depth 10 | Published at `rate_hz`. `angular_velocity` in rad/s; `linear_acceleration` in m/s², including gravity(more on this below). |

Fields to fill:
- Orientation: this IMU doesn't estimate orientation. Leave `orientation` zeroed and set `orientation_covariance[0] = -1`, which is the ROS convention for "no orientation". (That's also why the topic is called `data_raw`.)
- Covariances: put the white-noise variances σ² on the diagonals of `angular_velocity_covariance` and `linear_acceleration_covariance`. Never leave them all zero. Don't include the bias here, the EKF estimates the bias separately.

**Publishes NO TF.** `base_link → imu_link` comes from the robot description.

---

## The model

The level 1 arena is flat, so roll = pitch = 0.

On each ground-truth message, take `dt` from the message stamps (skip the message if `dt ≤ 0`), then:

1. **True motion in `base_link`.**
   - Angular velocity: `ω_b = (0, 0, angular.z)`.
   - Forward acceleration: `a_x = (v_k − v_{k−1}) / dt`.
   - Sideways (centripetal) acceleration: `a_y = v · ω`.
   - Specific force: `f_b = (a_x, a_y, +g)` with `g = 9.81`.
   - An accelerometer measures acceleration minus gravity, so a robot sitting still reads +9.81 on z, not 0. Overlooking this is a usual cause for incorrect IMU usage.

2. **Rotate into `imu_link`** using the mounting `rpy`: `ω_i = Rᵀ ω_b`, `f_i = Rᵀ f_b`. Ignore the lever arm (the mounting `xyz`) for now. Its effect is small at our speeds, and the EKF doesn't use the accelerometer yet.

3. **Bias.** Keep a separate bias per axis, for the gyro and the accelerometer.
   - At startup: `b ~ N(0, bias_init_std²)`.
   - Every step, random walk: `b += N(0, bias_rw² · dt)`.

4. **White noise:** `σ = noise_density · sqrt(rate_hz)`. Datasheets give noise density (per sqrt(Hz)), and the per-sample standard deviation depends on the sample rate.

5. **Measure** = true + bias + noise.

6. **Saturate** at the full-scale range (`gyro_range_dps`, `accel_range_g`).

7. **Publish** on a timer at `rate_hz`, using the newest values, stamped with the ground-truth message's time.
   - For now `rate_hz` must be ≤ 50 Hz, the ground-truth rate.
   - The real IMU will run faster (≥ 100 Hz), so tell the EKF owner not to assume 50 Hz.

**Reproducibility:** seed the RNG from a `seed` parameter (launch passes the arena seed).

Setting any noise or bias parameter to 0 should turn that error off. This is useful for testing, both here and in the EKF.

---

## Parameters (`config/sensors/mock_imu.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `geometry_yaml` | *(set by launch)* | Path to `robot_geometry.yaml`. |
| `rate_hz` | `50.0` | Publish rate (≤ ground-truth rate for now). |
| `gyro_noise_density` | `7.0e-5` | rad/s/√Hz. |
| `gyro_bias_init_std` | `0.01` | rad/s. Turn-on bias spread. |
| `gyro_bias_rw` | `1.0e-4` | rad/s/√s. Bias random walk. |
| `accel_noise_density` | `8.0e-4` | m/s²/√Hz. |
| `accel_bias_init_std` | `0.05` | m/s². |
| `accel_bias_rw` | `1.0e-3` | m/s²/√s. |
| `gyro_range_dps` | `250.0` | Gyro full scale. |
| `accel_range_g` | `4.0` | Accelerometer full scale. |
| `seed` | `0` | RNG seed. |

All defaults are ASSUMPTIONS. The noise densities are ballpark datasheet numbers for this class of IMU, and on the real robot vibration will make things worse. Once the real IMU is running, we'll record a long static bag and do an Allan variance to get real numbers. Keep every value in the YAML, none in code.

---

## Structure (inside `luna_sim`)

```
luna_sim/
├── config/sensors/mock_imu.yaml
├── luna_sim/sensors/
│   ├── imu_model.py           the math above. NO ROS IMPORTS.
│   └── mock_imu.py            ROS node
├── scripts/mock_imu.py        3-line wrapper, add to PY_EXECUTABLES
└── test/test_imu_model.py     plain pytest
```

Add the node to `launch/level1.launch.py` with `geometry_yaml`, `seed` and `use_sim_time` parameters, behind an `imu_sensor` argument (default `true`). Other people are also adding nodes to `luna_sim`, so only touch your own files plus that one launch entry.

Make sure to read the `luna_sim` README before working.

---