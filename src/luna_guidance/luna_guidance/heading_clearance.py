"""Rectangular straight-motion and circular stationary-turn clearance."""

import math

from luna_guidance.clearance import CircularClearance


class HeadingClearance:
    """
    World-coordinate checks for forward driving and stationary turns.

    Heading uncertainty covers orientation error around the planned center
    trajectory. It does not represent lateral tracking or localization error.
    """

    def __init__(
        self,
        blocked,
        geometry,
        length_m,
        width_m,
        margin_m,
        heading_error_deg,
        turn_radius_m=None,
    ):
        values = (length_m, width_m, margin_m, heading_error_deg)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Footprint parameters must be finite.")
        if length_m <= 0 or width_m <= 0 or margin_m < 0:
            raise ValueError("Dimensions must be positive; margin nonnegative.")
        if not 0 <= heading_error_deg <= 180:
            raise ValueError("Heading allowance must be between 0 and 180.")

        self.half_length = length_m / 2
        self.half_width = width_m / 2
        corner_radius = math.hypot(self.half_length, self.half_width)

        if turn_radius_m is None:
            turn_radius_m = corner_radius
        if (
            not math.isfinite(turn_radius_m)
            or turn_radius_m < corner_radius
        ):
            raise ValueError("Turn radius must enclose the physical footprint.")

        self.circle = CircularClearance(
            blocked, geometry, turn_radius_m, margin_m
        )
        self.padding = margin_m + 2 * corner_radius * math.sin(
            math.radians(heading_error_deg) / 2
        )
        self.geometry = geometry

    def _rectangle_free(self, center, yaw, half_length, half_width):
        """Check an oriented rectangle against full occupied cells."""
        x, y = center
        cosine, sine = math.cos(yaw), math.sin(yaw)
        epsilon = 1e-9
        half_length += epsilon
        half_width += epsilon

        extent_x = abs(cosine) * half_length + abs(sine) * half_width
        extent_y = abs(sine) * half_length + abs(cosine) * half_width

        if min(
            x - extent_x,
            y - extent_y,
            self.circle.width_m - x - extent_x,
            self.circle.height_m - y - extent_y,
        ) <= 0:
            return False

        for _, (xmin, ymin, xmax, ymax) in self.circle.obstacles:
            obstacle_x = (xmin + xmax) / 2
            obstacle_y = (ymin + ymax) / 2
            half_x = (xmax - xmin) / 2
            half_y = (ymax - ymin) / 2
            dx, dy = obstacle_x - x, obstacle_y - y

            # Separation on any of the four axes proves no overlap.
            if abs(dx) > extent_x + half_x:
                continue
            if abs(dy) > extent_y + half_y:
                continue
            if abs(cosine * dx + sine * dy) > (
                half_length + abs(cosine) * half_x + abs(sine) * half_y
            ):
                continue
            if abs(-sine * dx + cosine * dy) > (
                half_width + abs(sine) * half_x + abs(cosine) * half_y
            ):
                continue
            return False

        return True

    def pose_is_free(self, position, world_yaw):
        if not math.isfinite(world_yaw):
            raise ValueError("Yaw must be finite.")
        return self._rectangle_free(
            self.circle.to_local(position),
            world_yaw - self.geometry.origin_yaw,
            self.half_length + self.padding,
            self.half_width + self.padding,
        )

    def straight_is_free(self, start, end):
        """Check the full sweep while facing from start toward end."""
        a = self.circle.to_local(start)
        b = self.circle.to_local(end)
        distance = math.dist(a, b)
        if distance == 0:
            raise ValueError("Use pose_is_free for zero-length movements.")

        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        center = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

        # Forward translation sweeps a rectangle elongated along its length.
        return self._rectangle_free(
            center,
            yaw,
            self.half_length + self.padding + distance / 2,
            self.half_width + self.padding,
        )

    def turn_is_free(self, position):
        """Conservatively permit any stationary rotation at this position."""
        return self.circle.point_reason(position) is None


class CircleFirstHeadingClearance(HeadingClearance):
    """
    Accept circular clearance first, then rectangular straight clearance.

    Stationary turns still require the enclosing circle.
    """

    def pose_is_free(self, position, world_yaw):
        if not math.isfinite(world_yaw):
            raise ValueError("Yaw must be finite.")
        if self.circle.point_reason(position) is None:
            return True
        return super().pose_is_free(position, world_yaw)

    def straight_is_free(self, start, end):
        # Validate endpoints before trying either footprint check.
        a = self.circle.to_local(start)
        b = self.circle.to_local(end)
        if math.dist(a, b) == 0:
            raise ValueError("Use pose_is_free for zero-length movements.")

        if self.circle.segment_is_free(start, end):
            return True
        return super().straight_is_free(start, end)
