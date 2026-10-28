"""Geometric distances for circular-footprint collision checking."""

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

        resolution = geometry.resolution
        for y, row in enumerate(blocked):
            for x, occupied in enumerate(row):
                if occupied:
                    rectangle = (
                        x * resolution,
                        y * resolution,
                        (x + 1) * resolution,
                        (y + 1) * resolution,
                    )
                    self.obstacles.append(((x, y), rectangle))

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

        for cell, rectangle in self.obstacles:
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

        for _, rectangle in self.obstacles:
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

    def center_blocked_grid(self):
        """Return blocked flags for cell centers, not whole free cells."""
        return [
            [
                self.point_reason(
                    self.geometry.cell_to_world((x, y))
                ) is not None
                for x in range(self.geometry.width)
            ]
            for y in range(self.geometry.height)
        ]
