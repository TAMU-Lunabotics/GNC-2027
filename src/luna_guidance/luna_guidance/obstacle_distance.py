"""
ROS-independent occupancy-grid proximity and circular-footprint collision checks.

A -1 occupancy value is treated as blocked (unknown space is not safe).
Grid origin yaw is supported. Occupied cells are axis-aligned in grid coordinates.
"""
import math


class ObstacleDistance:
    def __init__(self, occupied_threshold=50):
        if not 1 <= occupied_threshold <= 100:
            raise ValueError('occupied_threshold must be in [1, 100]')
        self.occupied_threshold = occupied_threshold
        self.ready = False
        self.revision = 0
        self.data = []
        self.width = self.height = 0
        self.resolution = 0.0
        self.origin_x = self.origin_y = self.origin_yaw = 0.0
        self._occupied = []

    def set_map(self, width, height, resolution, origin_x, origin_y, origin_yaw, data):
        if width <= 0 or height <= 0 or resolution <= 0 or len(data) != width * height:
            raise ValueError('invalid occupancy grid dimensions or data length')
        if not all(math.isfinite(v) for v in (resolution, origin_x, origin_y, origin_yaw)):
            raise ValueError('invalid occupancy grid geometry')
        self.width, self.height, self.resolution = int(width), int(height), float(resolution)
        self.origin_x, self.origin_y, self.origin_yaw = (
            float(origin_x), float(origin_y), float(origin_yaw)
        )
        self.data = list(data)
        self._rebuild()
        self.ready = True

    def apply_update(self, x, y, width, height, data):
        if not self.ready:
            return False
        if (x < 0 or y < 0 or width <= 0 or height <= 0 or
                x + width > self.width or y + height > self.height or
                len(data) != width * height):
            raise ValueError('occupancy update outside grid bounds')
        for j in range(height):
            start = (y + j) * self.width + x
            self.data[start:start + width] = list(data[j * width:(j + 1) * width])
        self._rebuild()
        return True

    def _rebuild(self):
        self._occupied = [(i % self.width, i // self.width)
                          for i, value in enumerate(self.data)
                          if value < 0 or value >= self.occupied_threshold]
        self.revision += 1

    def _local(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def clearance(self, x, y, robot_radius=0.0):
        """Nearest occupied *cell boundary* or map boundary, minus footprint radius."""
        if not self.ready:
            return None
        u, v = self._local(x, y)
        size_x, size_y = self.width * self.resolution, self.height * self.resolution
        # The grid boundary is also an obstacle, even for an otherwise empty map.
        if not (0 <= u <= size_x and 0 <= v <= size_y):
            return -robot_radius
        best = min(u, v, size_x - u, size_y - v)
        r = self.resolution
        for col, row in self._occupied:
            dx = max(col * r - u, 0.0, u - (col + 1) * r)
            dy = max(row * r - v, 0.0, v - (row + 1) * r)
            best = min(best, math.hypot(dx, dy))
            if best == 0.0:
                break
        return best - robot_radius

    def collides(self, x, y, robot_radius):
        clearance = self.clearance(x, y, robot_radius)
        return clearance is None or clearance <= 0.0
