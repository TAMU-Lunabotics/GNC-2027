"""Regulated Pure Pursuit control law, with no ROS dependencies.

The controller interface matches PurePursuit: set_path, stop, get_control.
The occupancy helper is supplied by the ROS wrapper (or by tests).
"""
import math
from dataclasses import dataclass

from luna_guidance.pure_pursuit import (
    PurePursuit, Params as BaseParams, clamp, approach, wrap_angle,
    IDLE, FOLLOWING, GOAL_REACHED, STOPPED,
)


@dataclass
class RegulatedParams(BaseParams):
    lookahead_time_s: float = 1.5
    min_lookahead_m: float = 0.3
    max_lookahead_m: float = 0.9
    regulated_min_radius_m: float = 0.9
    robot_radius_m: float = 0.715
    proximity_clearance_m: float = 0.4
    min_regulated_speed: float = 0.08
    approach_dist_m: float = 0.6
    min_approach_speed: float = 0.05
    collision_check_time_s: float = 1.0
    occupied_threshold: int = 50


class RegulatedPurePursuit(PurePursuit):
    def __init__(self, params=None, obstacles=None):
        super().__init__(params if params is not None else RegulatedParams())
        self.obstacles = obstacles
        self.collision_arc = []  # (map_x, map_y, map_yaw) points for RViz

    def set_path(self, path, goal_yaw=None):
        # The parent clears path progress/latches but preserves commanded velocity.
        super().set_path(path, goal_yaw)
        self.collision_arc = []
        if not self.has_path:
            self._zero(IDLE, 'path empty')  # Empty path means stop immediately.

    def _carrot_on_circle(self, x, y, seg, t, lookahead):
        """First forward path intersection with lookahead circle (interpolated)."""
        if len(self.path) == 1:
            return self.path[-1]
        for i in range(seg, len(self.path) - 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            frac = t if i == seg else 0.0
            ax += frac * (bx - ax)
            ay += frac * (by - ay)
            dx, dy = bx - ax, by - ay
            a = dx * dx + dy * dy
            if a < 1e-14:
                continue
            start_dist_sq = (ax - x) ** 2 + (ay - y) ** 2
            if start_dist_sq >= lookahead * lookahead:
                return ax, ay
            # Start inside circle; find segment's forward exit.
            b = 2.0 * ((ax - x) * dx + (ay - y) * dy)
            c = start_dist_sq - lookahead * lookahead
            disc = b * b - 4 * a * c
            if disc >= 0:
                u = (-b + math.sqrt(disc)) / (2 * a)
                if 0.0 <= u <= 1.0:
                    return ax + u * dx, ay + u * dy
        return self.path[-1]

    def _predict_arc(self, x, y, yaw, v, w):
        """Check actual acceleration-limited command at <= 1 grid-cell increments."""
        self.collision_arc = [(x, y, yaw)]
        if self.obstacles is None or not self.obstacles.ready:
            return True  # fail closed: map missing
        radius = self.p.robot_radius_m
        if self.obstacles.collides(x, y, radius):
            return True
        horizon = max(0.0, self.p.collision_check_time_s)
        if horizon == 0.0:
            return False
        cell = self.obstacles.resolution
        # Along the robot-center trajectory, never move more than 1 cell per step.
        n = max(1, math.ceil(abs(v) * horizon / cell),
                math.ceil(abs(w) * horizon / 0.1),
                math.ceil(horizon / 0.05))
        dt = horizon / n
        xx, yy, theta = x, y, yaw
        for _ in range(n):
            if abs(w) < 1e-10:
                xx += v * math.cos(theta) * dt
                yy += v * math.sin(theta) * dt
            else:
                new_theta = theta + w * dt
                xx += (v / w) * (math.sin(new_theta) - math.sin(theta))
                yy -= (v / w) * (math.cos(new_theta) - math.cos(theta))
                theta = new_theta
            self.collision_arc.append((xx, yy, wrap_angle(theta)))
            if self.obstacles.collides(xx, yy, radius):
                return True
        return False

    def get_control(self, x, y, yaw, dt):
        p = self.p
        dt = max(dt, 1e-3)
        self.collision_arc = []
        if not self.has_path:
            return self._zero(IDLE, 'no path')
        if self._latched:
            return self._zero(self.state, self.reason)
        if self.obstacles is None or not self.obstacles.ready:
            return self._zero(STOPPED, 'no map yet')

        goal_x, goal_y = self.path[-1]
        self.dist_to_goal = math.hypot(goal_x - x, goal_y - y)
        dist, seg, t = self._closest(x, y)
        self._idx, self.cross_track = seg, dist
        if dist > p.max_cross_track_m:
            self._latched = True
            return self._zero(STOPPED, f'cross-track error {dist:.2f} m is over the '
                                       f'{p.max_cross_track_m:.2f} m limit')

        if self.dist_to_goal <= p.goal_tolerance_m:
            if self.goal_yaw is not None:
                yaw_err = wrap_angle(self.goal_yaw - yaw)
                if abs(yaw_err) > p.yaw_tolerance_rad:
                    self.state, self.reason = FOLLOWING, 'aligning with goal heading'
                    self.lookahead_point = (goal_x, goal_y)
                    target_w = clamp(p.rotate_gain * yaw_err,
                                     -p.max_angular_speed, p.max_angular_speed)
                    v, w = self._limit(0.0, target_w, dt)
                    if self._predict_arc(x, y, yaw, v, w):
                        return self._zero(STOPPED, 'collision ahead')
                    return v, w
            self._latched = True
            return self._zero(GOAL_REACHED, 'goal reached')

        lookahead = clamp(abs(self._v) * p.lookahead_time_s,
                          p.min_lookahead_m, p.max_lookahead_m)
        lx, ly = self._carrot_on_circle(x, y, seg, t, lookahead)
        self.lookahead_point = lx, ly
        dx, dy = lx - x, ly - y
        xr = math.cos(yaw) * dx + math.sin(yaw) * dy
        yr = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(yr, xr)
        self.state, self.reason = FOLLOWING, ''
        if self._rotating:
            if abs(alpha) < p.rotate_in_place_angle_rad * p.rotate_exit_ratio:
                self._rotating = False
        elif abs(alpha) > p.rotate_in_place_angle_rad:
            self._rotating = True
        if self._rotating:
            self.reason = 'rotating in place'
            target_w = clamp(p.rotate_gain * alpha, -p.max_angular_speed, p.max_angular_speed)
            v, w = self._limit(0.0, target_w, dt)
            if self._predict_arc(x, y, yaw, v, w):
                return self._zero(STOPPED, 'collision ahead')
            return v, w

        reach_sq = max(xr * xr + yr * yr, p.min_lookahead_m ** 2)
        curvature = 2 * yr / reach_sq
        v_curv = p.cruise_speed
        if abs(curvature) > 1e-10 and p.regulated_min_radius_m > 0:
            radius = 1.0 / abs(curvature)
            if radius < p.regulated_min_radius_m:
                v_curv *= radius / p.regulated_min_radius_m
        clearance = self.obstacles.clearance(x, y, p.robot_radius_m)
        if clearance is None:
            return self._zero(STOPPED, 'no map yet')
        v_prox = p.cruise_speed
        if p.proximity_clearance_m > 0 and clearance < p.proximity_clearance_m:
            v_prox *= max(clearance, 0.0) / p.proximity_clearance_m
        v = min(p.cruise_speed, max(min(v_curv, v_prox), p.min_regulated_speed))
        remaining = max(self._dist_along_path[-1] - self._arc_position(seg, t), 0.0)
        if p.approach_dist_m > 0 and remaining < p.approach_dist_m:
            v = min(v, max(p.cruise_speed * remaining / p.approach_dist_m,
                           p.min_approach_speed))
        if abs(curvature) * v > p.max_angular_speed:
            v = p.max_angular_speed / abs(curvature)
        # Clamp speed before deriving omega so the commanded curvature is preserved.
        v = min(v, p.max_linear_speed)
        w = v * curvature
        v, w = self._limit(v, w, dt)
        if self._predict_arc(x, y, yaw, v, w):
            return self._zero(STOPPED, 'collision ahead')
        return v, w
