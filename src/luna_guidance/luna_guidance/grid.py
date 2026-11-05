"""ROS-independent grid coordinate helpers."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GridGeometry:
    """
    Grid dimensions and its origin in the map frame.

    Resolution and origin coordinates are in meters.
    Origin yaw is in radians.
    """

    width: int
    height: int
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0

    def __post_init__(self):
        if (
            type(self.width) is not int
            or type(self.height) is not int
            or self.width <= 0
            or self.height <= 0
        ):
            raise ValueError("Grid dimensions must be positive integers.")

        values = (
            self.resolution,
            self.origin_x,
            self.origin_y,
            self.origin_yaw,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Grid geometry must contain finite values.")
        if self.resolution <= 0:
            raise ValueError("Resolution must be positive.")

    def contains(self, cell: tuple[int, int]) -> bool:
        """Return whether a cell lies within the grid."""
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """Convert map coordinates to a cell without clamping bounds."""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Position must contain finite values.")

        dx = x - self.origin_x
        dy = y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)

        # Inverse rotation: map frame to grid frame.
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy

        return (
            math.floor(local_x / self.resolution),
            math.floor(local_y / self.resolution),
        )

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        """Return a cell center in map coordinates."""
        if not self.contains(cell):
            raise ValueError("Cell is outside the grid.")

        x, y = cell
        local_x = (x + 0.5) * self.resolution
        local_y = (y + 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)

        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )


def occupancy_to_blocked(
    data,
    geometry: GridGeometry,
    occupied_threshold: int = 50,
    unknown_is_blocked: bool = True,
) -> list[list[bool]]:
    """
    Convert row-major occupancy values into grid[y][x].

    Expected values: -1 for unknown, or integers from 0 to 100.
    True means blocked; False means free.
    """
    if type(occupied_threshold) is not int:
        raise ValueError("Occupancy threshold must be an integer.")
    if not 1 <= occupied_threshold <= 100:
        raise ValueError("Occupancy threshold must be between 1 and 100.")
    if type(unknown_is_blocked) is not bool:
        raise ValueError("Unknown-cell policy must be Boolean.")

    expected_size = geometry.width * geometry.height
    if len(data) != expected_size:
        raise ValueError("Occupancy data length does not match grid size.")

    blocked = []
    for value in data:
        # Accept integer types such as those in NumPy arrays.
        try:
            value = value.__index__()
        except (AttributeError, TypeError):
            raise ValueError("Occupancy values must be integers.") from None

        if not -1 <= value <= 100:
            raise ValueError("Occupancy values must be -1 or 0 through 100.")

        if value == -1:
            blocked.append(unknown_is_blocked)
        else:
            blocked.append(value >= occupied_threshold)

    return [
        blocked[row * geometry.width:(row + 1) * geometry.width]
        for row in range(geometry.height)
    ]
