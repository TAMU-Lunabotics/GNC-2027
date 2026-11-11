"""Progress and swept-clearance checks for a planned polyline."""

from dataclasses import dataclass
import math

from luna_guidance.clearance import CircularClearance


@dataclass(frozen=True)
class Projection:
    segment: int
    fraction: float
    point: tuple[float, float]
    deviation: float


def project_path(points, position, previous=None, *, max_advance_m=math.inf):
    """Find the closest point within a forward distance along the polyline."""
    if len(points) == 1:
        return Projection(0, 0.0, points[0], math.dist(position, points[0]))

    first = previous.segment if previous else 0
    best = None
    for i in range(first, len(points) - 1):
        a, b = points[i:i + 2]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length_sq = dx * dx + dy * dy
        lower = previous.fraction if previous and i == first else 0.0
        upper = 1.0
        fraction = lower
        if length_sq:
            length = math.sqrt(length_sq)
            upper = min(1.0, lower + max_advance_m / length)
            fraction = ((position[0] - a[0]) * dx + (position[1] - a[1]) * dy) / length_sq
            fraction = max(lower, min(upper, fraction))
            max_advance_m -= (upper - lower) * length
        point = b if fraction == 1.0 else (a[0] + fraction * dx, a[1] + fraction * dy)
        distance = math.dist(position, point)
        if best is None or distance < best.deviation - 1e-9:
            best = Projection(i, fraction, point, distance)
        if upper < 1.0 or max_advance_m <= 0.0:
            break
    return best


def remaining_path(points, projection):
    """Return the projected start and the subsequent distinct waypoints."""
    remaining = [projection.point]
    for point in points[projection.segment + 1:]:
        if point != remaining[-1]:
            remaining.append(point)
    return remaining


def segment_yaw(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def heading_changes(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b))) > 1e-9


def path_rejection(checker, points, goal_yaw, start_yaw=None):
    """Return a rejection reason, checking full segments and hybrid turns."""
    if isinstance(checker, CircularClearance):
        if checker.point_reason(points[0]) is not None:
            return 'Final recheck rejected the starting footprint.'
        for a, b in zip(points, points[1:]):
            if not checker.segment_is_free(a, b):
                return 'Final path segment failed circular clearance.'
        return None

    yaw = start_yaw
    if yaw is None:
        yaw = segment_yaw(*points[:2]) if len(points) > 1 else goal_yaw
    if not checker.pose_is_free(points[0], yaw):
        return 'Final recheck rejected the starting footprint.'

    for a, b in zip(points, points[1:]):
        if math.dist(a, b) <= 1e-12:
            return 'Hybrid path contains a zero-length segment.'
        departure = segment_yaw(a, b)
        if heading_changes(yaw, departure) and not checker.turn_is_free(a):
            return 'Final recheck rejected a stationary turn.'
        if not checker.straight_is_free(a, b):
            return 'Final recheck rejected a straight segment.'
        yaw = departure

    if heading_changes(yaw, goal_yaw) and not checker.turn_is_free(points[-1]):
        return ('GOAL_TURN_REJECTED: requested final rotation lacks clearance; '
                'alternative arrival headings were not searched.')
    return None


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))
