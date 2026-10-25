"""
Differential-drive wheel encoder and odometry simulation.

This is the pure Python model with no ROS dependencies.
"""

import math
import random
from dataclasses import dataclass



def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))



@dataclass
class WheelOdometry:
    """One sample of reconstructed wheel odometry."""

    x: float
    y: float
    yaw: float
    linear_velocity: float
    angular_velocity: float
    left_ticks: int
    right_ticks: int

class WheelOdometryModel:
    def __init__(
        self,
        wheel_radius: float,
        wheel_separation: float,
        ticks_per_rev: int = 4096,
        noise_std: float = 0.0,
        noise_per_rad_s: float = 0.0,
        seed: int = 0
    ):
        # Validation
        if wheel_radius <= 0 or wheel_separation <= 0:
            raise ValueError("Wheel dimensions must be positive.")
        if ticks_per_rev <= 0:
            raise ValueError("ticks_per_rev must be positive.")
        if noise_std < 0 or noise_per_rad_s < 0:
            raise ValueError("Noise parameters cannot be negative.")
        
        self.radius = wheel_radius
        self.separation = wheel_separation
        self.ticks_per_rev = ticks_per_rev
        self.tick_angle = 2.0 * math.pi / ticks_per_rev

        # Noise in rad/s
        self.noise_std = noise_std
        self.noise_per_rad_s = noise_per_rad_s
        self.rng = random.Random(seed)

        self.reset()

    def reset(self):
        """Reset simulated encoders and odometry to zero."""
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.left_angle = 0.0
        self.right_angle = 0.0
        self.left_ticks = 0
        self.right_ticks = 0

        self.pose_covariance = [[0.0]*3 for _ in range(3)]
        self.twist_covariance = [[0.0]*2 for _ in range(2)]

    

    def update_covariance(self, dt: float, left_sigma: float, right_sigma: float, distance: float, delta_yaw: float, mid_yaw: float):
        """Propagate uncertainty through differential-drive odometry."""
        r = self.radius
        b = self.separation

        ### TWIST COVARIANCE

        # Variance of each wheel's measured angle increment.
        # Includes Gaussian wheel-speed noise and encoder quantization.
        q_var = self.tick_angle**2 / 6.0 # a common approximation of the variance for a quantized angle reading is q^2 / 12

        var_l = (left_sigma * dt)**2 + q_var
        var_r = (right_sigma * dt)**2 + q_var

        # Convert wheel-angle variance to wheel-speed variance
        speed_var_l = var_l / dt**2
        speed_var_r = var_r / dt**2

        # Twist covariance: [linear velocity, yaw rate]
        self.twist_covariance = [
            [
                r**2 / 4 * (speed_var_l + speed_var_r),
                r**2 / (2*b) * (speed_var_r - speed_var_l)
            ],
            [
                r**2 / (2*b) * (speed_var_r - speed_var_l),
                r**2 / b**2 * (speed_var_l + speed_var_r)
            ]
        ]



        ### POSE COVARIANCE

        # Jacobian w.r.t. previous pose
        F = [
            [1.0, 0.0, -distance * math.sin(mid_yaw)],
            [0.0, 1.0, distance * math.cos(mid_yaw)],
            [0.0, 0.0, 1.0]
        ]

        # Left wheel affects heading negatively; right positively
        s = math.sin(mid_yaw)
        c = math.cos(mid_yaw)

        # B maps the left and right encoder angle increments to changes in x, y, and heading.
        B = [
            [
                r/2 * c + distance*s*r / (2*b),
                r/2 * c - distance*s*r / (2*b)
            ],
            [
                r/2 * s - distance*c*r / (2*b),
                r/2 * s + distance*c*r / (2*b)
            ],
            [
                -r / b,
                r / b
            ]
        ]

        P = self.pose_covariance

        # First term: F * P * F^T
        FP = [
            [sum(F[i][k] * P[k][j] for k in range(3)) for j in range(3)] for i in range(3)
        ]
        new_P = [
            [sum(FP[i][k] * F[j][k] for k in range(3)) for j in range(3)] for i in range(3)
        ]

        # Second term: B * diag(var_l, var_r) * B^T
        for i in range(3):
            for j in range(3):
                new_P[i][j] += (B[i][0] * var_l * B[j][0]) + (B[i][1] * var_r * B[j][1])

        self.pose_covariance = new_P



    def step(self, true_v: float, true_w: float, dt: float) -> WheelOdometry:
        """Advance the simulated wheel encoders and odometry.

        Args:
            true_v: True forward body velocity (m/s).
            true_w: True yaw rate (rad/s).
            dt: Elapsed simulation time (s).
        """

        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        if not math.isfinite(true_v) or not math.isfinite(true_w):
            raise ValueError("True velocities must be finite")
        
        # Convert true body motion to ideal wheel velocities
        left_speed = (true_v - true_w * self.separation / 2.0) / self.radius
        right_speed = (true_v + true_w * self.separation / 2.0) / self.radius

        # Add Gaussian wheel-speed measurement noise
        left_sigma = self.noise_std + self.noise_per_rad_s * abs(left_speed)
        right_sigma = self.noise_std + self.noise_per_rad_s * abs(right_speed)

        noisy_left = left_speed + self.rng.gauss(0.0, left_sigma)
        noisy_right = right_speed + self.rng.gauss(0.0, right_sigma)

        # Integrate continuous wheel angles
        self.left_angle += noisy_left * dt
        self.right_angle += noisy_right * dt

        # Quantize to integer encoder ticks
        new_left_ticks = round(self.left_angle / self.tick_angle)
        new_right_ticks = round(self.right_angle / self.tick_angle)

        delta_left_ticks = new_left_ticks - self.left_ticks
        delta_right_ticks = new_right_ticks - self.right_ticks

        self.left_ticks = new_left_ticks
        self.right_ticks = new_right_ticks

        # Recover measured wheel angular velocities
        measured_left = delta_left_ticks * self.tick_angle / dt
        measured_right = delta_right_ticks * self.tick_angle / dt

        # Reconstruct body velocity
        self.linear_velocity = self.radius * (measured_right + measured_left) / 2.0
        self.angular_velocity = self.radius * (measured_right - measured_left) / self.separation

        # Integrate odometry using midpoint heading
        delta_yaw = self.angular_velocity * dt
        mid_yaw = self.yaw + delta_yaw / 2.0

        distance = self.linear_velocity * dt

        self.x += distance * math.cos(mid_yaw)
        self.y += distance * math.sin(mid_yaw)
        self.yaw = wrap_angle(self.yaw + delta_yaw)

        self.update_covariance(
            dt=dt,
            left_sigma=left_sigma,
            right_sigma=right_sigma,
            distance=distance,
            delta_yaw=delta_yaw,
            mid_yaw=mid_yaw
        )

        return WheelOdometry(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
            left_ticks=self.left_ticks,
            right_ticks=self.right_ticks
        )