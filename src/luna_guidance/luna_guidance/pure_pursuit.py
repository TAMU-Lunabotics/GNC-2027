"""Pure pursuit path following. No ROS imports.

    controller = PurePursuit(Params())
    controller.set_path([(0, 0), (1, 0), (2, 0.5)], goal_yaw=0.0)
    v, w = controller.get_control(x, y, yaw, dt)
"""
import bisect
import math
from dataclasses import dataclass

IDLE = 'IDLE'
FOLLOWING = 'FOLLOWING'
GOAL_REACHED = 'GOAL_REACHED'
STOPPED = 'STOPPED'


@dataclass
class Params:
    lookahead_m: float = 0.5
    cruise_speed: float = 0.3
    goal_tolerance_m: float = 0.10
    yaw_tolerance_rad: float = 0.10
    rotate_in_place_angle_rad: float = 1.05
    max_cross_track_m: float = 0.75
    max_linear_speed: float = 0.5
    max_angular_speed: float = 1.0
    max_linear_accel: float = 0.5
    max_angular_accel: float = 1.5
    rotate_exit_ratio: float = 0.5
    rotate_gain: float = 1.5
    min_speed: float = 0.05
    min_lookahead_m: float = 0.15
    progress_window: int = 20
    goal_decel_ratio: float = 0.8


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, low, high):
    return max(low, min(high, value))


def approach(current, target, max_step):
    return current + clamp(target - current, -max_step, max_step)


class PurePursuit:
    def __init__(self, params=None):
        self.p = params if params is not None else Params()

        self.state = IDLE
        self.reason = 'no path'
        self.cross_track = 0.0
        self.dist_to_goal = 0.0
        self.lookahead_point = None

        self.path = []
        self.goal_yaw = None
        self._dist_along_path = []
        self._idx = 0
        self._rotating = False
        self._latched = False
        self._v = 0.0
        self._w = 0.0

    @property
    def has_path(self):
        return len(self.path) > 0

    def set_path(self, path, goal_yaw=None):
        self.path = [(float(x), float(y)) for x, y in path]
        self.goal_yaw = goal_yaw
        self._idx = 0
        self._rotating = False
        self._latched = False
        self.lookahead_point = None

        self._dist_along_path = [0.0]
        for i in range(1, len(self.path)):
            ax, ay = self.path[i - 1]
            bx, by = self.path[i]
            self._dist_along_path.append(self._dist_along_path[-1] + math.hypot(bx - ax, by - ay))

        if self.has_path:
            self.state, self.reason = FOLLOWING, ''
        else:
            self.state, self.reason = IDLE, 'path empty'

    def stop(self, reason):
        if self._latched:
            return self._zero(self.state, self.reason)
        return self._zero(STOPPED, reason)

    def get_control(self, x, y, yaw, dt):
        p = self.p
        dt = max(dt, 1e-3)

        if not self.has_path:
            return self._zero(IDLE, 'no path')
        if self._latched:
            return self._zero(self.state, self.reason)

        goal_x, goal_y = self.path[-1]
        self.dist_to_goal = math.hypot(goal_x - x, goal_y - y)

        dist, seg, t = self._closest(x, y)
        self._idx = seg
        self.cross_track = dist
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
                    w = clamp(p.rotate_gain * yaw_err, -p.max_angular_speed, p.max_angular_speed)
                    return self._limit(0.0, w, dt)
            self._latched = True
            return self._zero(GOAL_REACHED, 'goal reached')

        lx, ly = self._point_at(self._arc_position(seg, t) + p.lookahead_m)
        self.lookahead_point = (lx, ly)

        # lookahead point as seen from the robot: x ahead, y to the left
        dx, dy = lx - x, ly - y
        xr = math.cos(yaw) * dx + math.sin(yaw) * dy
        yr = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(yr, xr)
        self.state, self.reason = FOLLOWING, ''

        # leave rotate mode at a smaller angle than we entered it, so noise can't flip it back and forth
        if self._rotating:
            if abs(alpha) < p.rotate_in_place_angle_rad * p.rotate_exit_ratio:
                self._rotating = False
        elif abs(alpha) > p.rotate_in_place_angle_rad:
            self._rotating = True
        if self._rotating:
            self.reason = 'rotating in place'
            w = clamp(p.rotate_gain * alpha, -p.max_angular_speed, p.max_angular_speed)
            return self._limit(0.0, w, dt)

        # arc through the lookahead point: curvature = 2y / L^2, turn rate = v * curvature
        reach = max(math.hypot(xr, yr), p.min_lookahead_m)
        curvature = 2.0 * yr / (reach * reach)

        v = p.cruise_speed
        decel = p.goal_decel_ratio * p.max_linear_accel
        v_goal = p.min_speed + math.sqrt(2.0 * decel * max(self.dist_to_goal - p.goal_tolerance_m, 0.0))
        v = min(v, v_goal)
        if abs(curvature) * v > p.max_angular_speed:
            v = p.max_angular_speed / abs(curvature)

        return self._limit(v, v * curvature, dt)

    def _zero(self, state, reason):
        self.state, self.reason = state, reason
        self._v = 0.0
        self._w = 0.0
        return 0.0, 0.0

    def _limit(self, v, w, dt):
        p = self.p
        v = approach(self._v, v, p.max_linear_accel * dt)
        w = approach(self._w, w, p.max_angular_accel * dt)
        v = clamp(v, 0.0, p.max_linear_speed)
        w = clamp(w, -p.max_angular_speed, p.max_angular_speed)
        self._v, self._w = v, w
        return v, w

    def _closest(self, x, y):
        if len(self.path) == 1:
            return 0.0, 0, 0.0

        best = None
        last = min(self._idx + self.p.progress_window, len(self.path) - 1)
        for i in range(self._idx, last):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            abx, aby = bx - ax, by - ay
            length_sq = abx * abx + aby * aby
            t = 0.0 if length_sq == 0.0 else clamp(((x - ax) * abx + (y - ay) * aby) / length_sq, 0.0, 1.0)
            d = math.hypot(x - (ax + t * abx), y - (ay + t * aby))
            if best is None or d < best[0]:
                best = (d, i, t)
        return best

    def _arc_position(self, seg, t):
        if len(self.path) == 1:
            return 0.0
        return self._dist_along_path[seg] + t * (self._dist_along_path[seg + 1] - self._dist_along_path[seg])

    def _point_at(self, s):
        if len(self.path) == 1 or s >= self._dist_along_path[-1]:
            return self.path[-1]
        k = min(bisect.bisect_right(self._dist_along_path, s) - 1, len(self.path) - 2)
        seg = self._dist_along_path[k + 1] - self._dist_along_path[k]
        if seg <= 1e-9:
            return self.path[k + 1]
        t = (s - self._dist_along_path[k]) / seg
        ax, ay = self.path[k]
        bx, by = self.path[k + 1]
        return ax + t * (bx - ax), ay + t * (by - ay)
