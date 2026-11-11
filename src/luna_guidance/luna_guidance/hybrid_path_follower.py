"""Follow hybrid planner segments with rectangular clearance and stationary turns."""

from dataclasses import dataclass
import math

from luna_guidance.grid import GridGeometry, occupancy_to_blocked
from luna_guidance.heading_clearance import CircleFirstHeadingClearance
from luna_guidance.path_check import heading_changes, project_path, segment_yaw
from luna_guidance.pure_pursuit import FOLLOWING, GOAL_REACHED, IDLE, STOPPED, clamp, wrap_angle
from luna_guidance.regulated_pure_pursuit import (
    RegulatedParams, RegulatedPurePursuit, project_twist,
)

STOP_SPEED_MPS = 0.02
STOP_TURN_RATE_RAD_S = 0.05


@dataclass
class HybridParams(RegulatedParams):
    robot_length_m: float = 1.2319
    robot_width_m: float = 0.7239
    heading_error_deg: float = 5.0


class HybridPathFollower(RegulatedPurePursuit):
    def __init__(self, params=None, obstacles=None):
        super().__init__(params if params is not None else HybridParams(), obstacles)
        if not math.isfinite(self.p.collision_check_time_s) or self.p.collision_check_time_s <= 0:
            raise ValueError('Hybrid following requires a positive collision_check_time_s.')
        self._clearance = None
        self._map_revision = None

    def set_path(self, path, goal_yaw=None):
        # Collapse the planner's collinear samples while preserving turns.
        if goal_yaw is not None and not math.isfinite(goal_yaw):
            raise ValueError('Goal heading must be finite.')
        vertices = []
        for x, y in path:
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError('Path positions must be finite.')
            point = (x, y)
            if vertices and point == vertices[-1]:
                continue
            if len(vertices) >= 2 and not heading_changes(
                segment_yaw(*vertices[-2:]), segment_yaw(vertices[-1], point)
            ):
                vertices[-1] = point
            else:
                vertices.append(point)
        super().set_path(vertices, goal_yaw)
        self._rotating = self.has_path

    def _refresh_clearance(self):
        obstacles = self.obstacles
        if obstacles is None or not obstacles.ready:
            return False
        if self._map_revision != obstacles.revision:
            geometry = GridGeometry(
                obstacles.width, obstacles.height, obstacles.resolution,
                obstacles.origin_x, obstacles.origin_y, obstacles.origin_yaw,
            )
            blocked = occupancy_to_blocked(obstacles.data, geometry, obstacles.occupied_threshold)
            p = self.p
            # Use the physical footprint with heading allowance. The planner's
            # extra buffer can be waived on an approved start escape segment.
            self._clearance = CircleFirstHeadingClearance(
                blocked, geometry, p.robot_length_m, p.robot_width_m, 0.0,
                p.heading_error_deg, turn_radius_m=p.robot_radius_m,
            )
            self._map_revision = obstacles.revision
        return True

    def _predict_arc(self, x, y, yaw, v, w):
        """Check the entire projected sweep of the acceleration-limited command."""
        self.collision_arc = [(x, y, yaw)]
        clearance = self._clearance
        horizon = self.p.collision_check_time_s
        if v == 0.0:
            if w != 0.0:
                self.collision_arc.append((x, y, wrap_angle(yaw + w * horizon)))
                return not clearance.turn_is_free((x, y))
            return not clearance.pose_is_free((x, y), yaw)
        if abs(w) < 1e-10:
            end = project_twist(x, y, yaw, v, w, horizon)
            self.collision_arc.append(end)
            if end[:2] == (x, y):
                return not clearance.pose_is_free((x, y), yaw)
            return not clearance.straight_is_free((x, y), end[:2])

        steps = max(1, math.ceil(horizon / 0.05))
        step = horizon / steps
        # Each corner travels no more than this distance from the midpoint
        # of a sample interval, including translation and rotation.
        padding = (abs(v) + self.p.robot_radius_m * abs(w)) * step / 2
        for _ in range(steps):
            mid_x, mid_y, mid_yaw = project_twist(x, y, yaw, v, w, step / 2)
            x, y, yaw = project_twist(x, y, yaw, v, w, step)
            self.collision_arc.append((x, y, wrap_angle(yaw)))
            if not clearance.pose_is_free((mid_x, mid_y), mid_yaw, motion_padding_m=padding):
                return True
        return False

    def _turn(self, x, y, yaw, error, dt, measured_speed, reason):
        if self._v > 0.0 or measured_speed > STOP_SPEED_MPS:
            return self._zero(FOLLOWING, 'stopping for turn')
        self.state, self.reason = FOLLOWING, reason
        target = clamp(self.p.rotate_gain * error, -self.p.max_angular_speed,
                       self.p.max_angular_speed)
        v, w = self._limit(0.0, target, dt)
        if self._predict_arc(x, y, yaw, v, w):
            return self._zero(STOPPED, 'turn clearance blocked')
        return v, w

    def get_control(self, x, y, yaw, dt, *, measured_twist=(0.0, 0.0)):
        self.collision_arc = []
        if not self.has_path:
            return self._zero(IDLE, 'no path')
        if self._latched:
            return self._zero(self.state, self.reason)
        if not all(math.isfinite(value) for value in (x, y, yaw, dt, *measured_twist)):
            return self._zero(STOPPED, 'invalid localization')
        try:
            if not self._refresh_clearance():
                return self._zero(STOPPED, 'no map yet')
        except ValueError:
            return self._zero(STOPPED, 'invalid footprint or map')

        p = self.p
        dt = max(dt, 1e-3)
        measured_speed, measured_rate = map(abs, measured_twist)
        self.dist_to_goal = math.dist((x, y), self.path[-1])
        segment = self.path[self._idx:self._idx + 2]
        endpoint = segment[-1]
        final_leg = self._idx + 2 >= len(self.path)
        goal_error = wrap_angle(self.goal_yaw - yaw) if self.goal_yaw is not None else 0.0
        align_goal = final_leg and abs(goal_error) > p.yaw_tolerance_rad
        tolerance = p.goal_tolerance_m
        if len(segment) == 2 and (not final_leg or align_goal):
            tolerance = min(tolerance, self._clearance.geometry.resolution / 2)
        distance = math.dist((x, y), endpoint)
        projection = project_path(segment, (x, y))
        self.cross_track = projection.deviation
        if self.cross_track > p.max_cross_track_m:
            self._latched = True
            return self._zero(STOPPED, f'cross-track error {self.cross_track:.2f} m is over the '
                                       f'{p.max_cross_track_m:.2f} m limit')

        if distance <= tolerance:
            self.lookahead_point = endpoint
            if self._v > 0.0 or measured_speed > STOP_SPEED_MPS:
                return self._zero(FOLLOWING, 'stopping at waypoint')
            if not final_leg:
                self._idx += 1
                self._rotating = True
                return self._zero(FOLLOWING, 'at turn waypoint')
            if align_goal:
                return self._turn(x, y, yaw, goal_error, dt, measured_speed,
                                  'aligning with goal heading')
            if self._w != 0.0 or measured_rate > STOP_TURN_RATE_RAD_S:
                return self._zero(FOLLOWING, 'settling at goal')
            self._latched = True
            return self._zero(GOAL_REACHED, 'goal reached')
        if len(segment) == 1:
            return self._zero(STOPPED, 'outside single-point goal tolerance')

        along = self._arc_position(self._idx, projection.fraction)
        target = self._point_at(min(
            along + self._lookahead_distance(), self._dist_along_path[self._idx + 1],
        ))
        self.lookahead_point = target
        dx, dy = target[0] - x, target[1] - y
        xr = math.cos(yaw) * dx + math.sin(yaw) * dy
        yr = -math.sin(yaw) * dx + math.cos(yaw) * dy
        error = math.atan2(yr, xr)
        self._rotating = self._rotating or abs(error) > p.rotate_in_place_angle_rad
        if self._rotating:
            if abs(error) > p.yaw_tolerance_rad:
                return self._turn(x, y, yaw, error, dt, measured_speed, 'aligning with segment')
            if self._w != 0.0 or measured_rate > STOP_TURN_RATE_RAD_S:
                return self._zero(FOLLOWING, 'settling after turn')
            self._rotating = False

        self.state, self.reason = FOLLOWING, 'following segment'
        speed_cap = math.sqrt(2.0 * p.max_linear_accel * distance)
        return self._follow_carrot(x, y, yaw, xr, yr, distance, dt, speed_cap=speed_cap)
