"""Run the existing searches on a fixed map and pose snapshot."""

from dataclasses import dataclass
from functools import lru_cache
import math

from luna_guidance.clearance import CircularClearance
from luna_guidance.grid import GridGeometry, nearby_cell_centers
from luna_guidance.heading_clearance import CircleFirstHeadingClearance
from luna_guidance.heading_theta_star import heading_theta_star
from luna_guidance.path_check import path_rejection, remaining_path, segment_yaw
from luna_guidance.theta_star import simplify_path, theta_star


@dataclass(frozen=True)
class SearchSettings:
    footprint_mode: str
    radius_m: float
    margin_m: float
    length_m: float
    width_m: float
    heading_error_deg: float
    turn_weight: float
    max_expansions: int
    merge_turnable: bool
    allow_straight_departure: bool
    start_escape_radius_m: float

    def checker(self, blocked, geometry, *, escape=False):
        margin = 0.0 if escape else self.margin_m
        if self.footprint_mode == 'hybrid':
            return CircleFirstHeadingClearance(
                blocked, geometry, self.length_m, self.width_m, margin,
                self.heading_error_deg, turn_radius_m=self.radius_m,
            )
        return CircularClearance(blocked, geometry, self.radius_m, margin)


@dataclass
class PlanningMap:
    geometry: GridGeometry
    raw_blocked: list[list[bool]]
    checker: CircularClearance | CircleFirstHeadingClearance
    circular_grid: list[list[bool]] | None = None

    def cell_grid(self, should_cancel):
        # Only the search worker builds and caches the circular center grid.
        if self.circular_grid is None:
            self.circular_grid = self.checker.center_blocked_grid(should_cancel)
        if self.circular_grid is None:
            raise ValueError('Search cancelled.')
        return self.circular_grid


@dataclass(frozen=True)
class Route:
    points: tuple[tuple[float, float], ...]
    escape: bool = False


def pose_rejection(checker, position, yaw):
    if isinstance(checker, CircularClearance):
        return checker.point_reason(position)
    if not checker.pose_is_free(position, yaw):
        return 'Insufficient rectangular footprint clearance.'
    return None


def remaining_rejection(snapshot, settings, route, projection, goal_yaw, start_yaw=None):
    """Validate the remaining straight sweeps and turns against a newer map."""
    points = remaining_path(route.points, projection)
    yaw = start_yaw
    if yaw is None and len(route.points) > 1:
        i = projection.segment
        yaw = segment_yaw(route.points[i], route.points[i + 1])
    if route.escape and projection.segment == 0 and projection.fraction < 1.0:
        relaxed = settings.checker(snapshot.raw_blocked, snapshot.geometry, escape=True)
        departure = segment_yaw(*points[:2])
        reason = path_rejection(relaxed, points[:2], departure, yaw)
        if reason is not None:
            return reason
        points = points[1:]
        yaw = departure
    return path_rejection(snapshot.checker, points, goal_yaw, yaw)


def _escape_start(snapshot, settings, position, yaw, should_cancel):
    """Find a nearby full-clearance center with a checked escape sweep."""
    relaxed = settings.checker(snapshot.raw_blocked, snapshot.geometry, escape=True)
    if pose_rejection(relaxed, position, yaw) is not None:
        return None

    for point in nearby_cell_centers(snapshot.geometry, position, settings.start_escape_radius_m):
        if should_cancel():
            raise ValueError('Search cancelled.')
        if math.dist(position, point) <= 1e-12:
            continue
        departure = segment_yaw(position, point)
        if pose_rejection(snapshot.checker, point, departure) is not None:
            continue
        if path_rejection(relaxed, [position, point], departure, yaw) is None:
            return point, departure
    return None


def _circle_points(snapshot, start, goal, should_cancel):
    geometry, checker = snapshot.geometry, snapshot.checker
    if checker.segment_is_free(start, goal):
        return [start, goal]

    blocked = snapshot.cell_grid(should_cancel)
    start_cell = geometry.world_to_cell(*start)
    goal_cell = geometry.world_to_cell(*goal)
    for name, cell, position in (('Start', start_cell, start), ('Goal', goal_cell, goal)):
        x, y = cell
        if blocked[y][x]:
            raise ValueError(
                f'{name} position is valid, but its grid-cell center lacks '
                'clearance. Grid attachment failed.'
            )
        if not checker.segment_is_free(position, geometry.cell_to_world(cell)):
            raise ValueError(
                f'{name} cannot connect safely to its grid-cell center. '
                'Grid attachment failed.'
            )

    # Theta* revisits some connections within this fixed map snapshot.
    segment_is_free = lru_cache(maxsize=4096)(checker.cell_segment_is_free)
    cells = theta_star(blocked, start_cell, goal_cell, segment_is_free, should_cancel)
    if not cells:
        raise ValueError('Search found no route on the circular-clearance grid.')
    return [start, *(geometry.cell_to_world(cell) for cell in simplify_path(cells)), goal]


def find_route(snapshot, settings, position, yaw, goal, goal_yaw, should_cancel,
               *, allow_escape=False):
    """Plan with the Stage 1 footprint rules and recheck the returned geometry."""
    if should_cancel():
        raise ValueError('Search cancelled.')
    checker = snapshot.checker
    goal_cell = snapshot.geometry.world_to_cell(*goal)
    if not snapshot.geometry.contains(goal_cell):
        raise ValueError('Goal: Position lies outside the map.')
    goal_x, goal_y = goal_cell
    if snapshot.raw_blocked[goal_y][goal_x]:
        raise ValueError('Goal in obstacle: the goal cell is occupied.')
    if isinstance(checker, CircularClearance):
        reason = checker.point_reason(goal)
        if reason is not None:
            raise ValueError(f'Goal in obstacle or without clearance: {reason}')
    elif yaw is None:
        raise ValueError('No valid starting orientation received.')

    start, start_yaw = position, yaw
    reason = pose_rejection(checker, start, start_yaw)
    escape = reason is not None and allow_escape
    if escape:
        candidate = _escape_start(snapshot, settings, start, start_yaw, should_cancel)
        if candidate is None:
            raise ValueError(f'Start in collision: {reason}')
        start, start_yaw = candidate
    elif reason is not None:
        raise ValueError(f'Start: {reason}')

    if isinstance(checker, CircularClearance):
        points = _circle_points(snapshot, start, goal, should_cancel)
        points = [p for i, p in enumerate(points) if i == 0 or p != points[i - 1]]
    else:
        result = heading_theta_star(
            checker, start, goal, start_yaw,
            turn_weight=settings.turn_weight,
            max_expansions=settings.max_expansions,
            merge_turnable=settings.merge_turnable,
            allow_straight_departure=settings.allow_straight_departure,
            should_cancel=should_cancel,
        )
        if result['status'] != 'PATH_FOUND':
            raise ValueError(f"Hybrid search: {result['status']}")
        points = result['path']

    if not points or points[0] != start or points[-1] != goal:
        raise ValueError('Search returned incorrect endpoints.')
    reason = path_rejection(checker, points, goal_yaw, start_yaw)
    if reason is not None:
        raise ValueError(reason)
    if escape:
        points.insert(0, position)
    return Route(tuple(points), escape)
