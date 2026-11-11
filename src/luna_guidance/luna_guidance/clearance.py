"""Geometric distances for circular-footprint collision checking."""

from itertools import groupby
import math


def point_segment_distance(point, start, end):
    """Return the shortest distance from a point to a closed segment."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy

    if length_squared == 0.0:
        return math.hypot(
            point[0] - start[0],
            point[1] - start[1],
        )

    fraction = (
        (point[0] - start[0]) * dx
        + (point[1] - start[1]) * dy
    ) / length_squared
    fraction = max(0.0, min(1.0, fraction))

    closest_x = start[0] + fraction * dx
    closest_y = start[1] + fraction * dy
    return math.hypot(point[0] - closest_x, point[1] - closest_y)


def point_rectangle_distance(point, rectangle):
    """Return distance to a filled rectangle: xmin, ymin, xmax, ymax."""
    xmin, ymin, xmax, ymax = rectangle
    gap_x = max(xmin - point[0], 0.0, point[0] - xmax)
    gap_y = max(ymin - point[1], 0.0, point[1] - ymax)
    return math.hypot(gap_x, gap_y)


def segment_intersects_rectangle(start, end, rectangle):
    """Return whether a closed segment touches or crosses a rectangle."""
    xmin, ymin, xmax, ymax = rectangle
    enter = 0.0
    leave = 1.0

    for origin, delta, lower, upper in (
        (start[0], end[0] - start[0], xmin, xmax),
        (start[1], end[1] - start[1], ymin, ymax),
    ):
        if delta == 0.0:
            if origin < lower or origin > upper:
                return False
            continue

        first = (lower - origin) / delta
        second = (upper - origin) / delta
        if first > second:
            first, second = second, first

        enter = max(enter, first)
        leave = min(leave, second)
        if enter > leave:
            return False

    return True


def segment_rectangle_distance(start, end, rectangle):
    """
    Return minimum distance from a complete segment to a filled rectangle.

    Inputs must be finite and use the same coordinate system and units.
    The rectangle must satisfy xmin <= xmax and ymin <= ymax.
    Intersection or boundary contact returns zero.
    """
    if segment_intersects_rectangle(start, end, rectangle):
        return 0.0

    xmin, ymin, xmax, ymax = rectangle
    corners = (
        (xmin, ymin),
        (xmin, ymax),
        (xmax, ymin),
        (xmax, ymax),
    )

    distances = [
        point_rectangle_distance(start, rectangle),
        point_rectangle_distance(end, rectangle),
    ]
    distances.extend(
        point_segment_distance(corner, start, end)
        for corner in corners
    )
    return min(distances)


class CircularClearance:
    """Check a circular footprint against occupied cells and map edges."""

    def __init__(self, blocked, geometry, robot_radius_m, safety_margin_m):
        for value in (robot_radius_m, safety_margin_m):
            if not math.isfinite(value) or value < 0:
                raise ValueError("Radius and margin must be finite and nonnegative.")

        self.clearance = robot_radius_m + safety_margin_m
        if not math.isfinite(self.clearance):
            raise ValueError("Combined clearance must be finite.")

        if len(blocked) != geometry.height or any(
            len(row) != geometry.width for row in blocked
        ):
            raise ValueError("Blocked grid shape does not match geometry.")

        self.geometry = geometry
        self.width_m = geometry.width * geometry.resolution
        self.height_m = geometry.height * geometry.resolution
        self.cosine = math.cos(geometry.origin_yaw)
        self.sine = math.sin(geometry.origin_yaw)

        # A tiny numerical allowance, separate from the physical margin.
        self.limit = self.clearance + 1e-9
        self.obstacles = []
        self.occupied_spans = []

        resolution = geometry.resolution
        for y, row in enumerate(blocked):
            bottom, top = y * resolution, (y + 1) * resolution
            for occupied, run in groupby(enumerate(row), key=lambda entry: bool(entry[1])):
                if not occupied:
                    continue
                cells = [
                    ((x, y), (x * resolution, bottom, (x + 1) * resolution, top))
                    for x, _ in run
                ]
                self.obstacles.extend(cells)
                # A contiguous run is exactly one filled rectangle. Retain
                # its cells for the existing point rejection diagnostics.
                span = (cells[0][1][0], bottom, cells[-1][1][2], top)
                self.occupied_spans.append((span, cells))

    def to_local(self, point):
        """Convert a finite world position into grid-local meters."""
        x, y = point
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Position must contain finite values.")

        dx = x - self.geometry.origin_x
        dy = y - self.geometry.origin_y
        return (
            self.cosine * dx + self.sine * dy,
            -self.sine * dx + self.cosine * dy,
        )

    def _boundary_reason(self, point):
        x, y = point
        if not (0 <= x < self.width_m and 0 <= y < self.height_m):
            return "Position lies outside the map."

        gap = min(x, y, self.width_m - x, self.height_m - y)
        if gap <= self.limit:
            return (
                f"Insufficient boundary clearance: {gap:.4f} m; "
                f"must exceed {self.clearance:.4f} m."
            )
        return None

    def point_reason(self, world_point):
        """Return None if a position is free, otherwise explain rejection."""
        point = self.to_local(world_point)
        reason = self._boundary_reason(point)
        if reason is not None:
            return reason

        for span, cells in self.occupied_spans:
            if point_rectangle_distance(point, span) > self.limit:
                continue
            for cell, rectangle in cells:
                gap = point_rectangle_distance(point, rectangle)
                if gap <= self.limit:
                    return (
                        f"Insufficient obstacle clearance near cell {cell}: "
                        f"{gap:.4f} m; must exceed {self.clearance:.4f} m."
                    )
        return None

    def segment_is_free(self, world_start, world_end):
        """Check the complete segment, including both endpoints."""
        start = self.to_local(world_start)
        end = self.to_local(world_end)

        if (
            self._boundary_reason(start) is not None
            or self._boundary_reason(end) is not None
        ):
            return False

        # The permitted map interior is convex: endpoint checks suffice
        # for boundary clearance along a straight segment.
        lower_x = min(start[0], end[0]) - self.limit
        upper_x = max(start[0], end[0]) + self.limit
        lower_y = min(start[1], end[1]) - self.limit
        upper_y = max(start[1], end[1]) + self.limit

        for rectangle, _ in self.occupied_spans:
            xmin, ymin, xmax, ymax = rectangle

            # Skip obstacles outside the segment's expanded bounding box.
            if (
                xmax < lower_x or xmin > upper_x
                or ymax < lower_y or ymin > upper_y
            ):
                continue

            if segment_rectangle_distance(start, end, rectangle) <= self.limit:
                return False

        return True

    def cell_segment_is_free(self, start_cell, end_cell):
        """Check a connection between two grid-cell centers."""
        if (
            not self.geometry.contains(start_cell)
            or not self.geometry.contains(end_cell)
        ):
            return False

        return self.segment_is_free(
            self.geometry.cell_to_world(start_cell),
            self.geometry.cell_to_world(end_cell),
        )

    def center_blocked_grid(self, should_cancel=None):
        """Return blocked flags for cell centers, or None if cancelled."""
        grid = []
        width = self.geometry.width
        for y in range(self.geometry.height):
            if should_cancel is not None and should_cancel():
                return None
            row = [True] * width
            centers = []
            for x in range(width):
                point = self.to_local(self.geometry.cell_to_world((x, y)))
                if self._boundary_reason(point) is None:
                    centers.append((x, point))

            if centers:
                min_y = min(point[1] for _, point in centers)
                max_y = max(point[1] for _, point in centers)
                # A span is a filled rectangle. Only spans within vertical
                # clearance of this row can block any of its centers.
                spans = [
                    span for span, _ in self.occupied_spans
                    if span[1] - max_y <= self.limit and min_y - span[3] <= self.limit
                ]
                for x, point in centers:
                    row[x] = any(
                        span[0] - point[0] <= self.limit
                        and point[0] - span[2] <= self.limit
                        and point_rectangle_distance(point, span) <= self.limit
                        for span in spans
                    )
            grid.append(row)
        return grid
