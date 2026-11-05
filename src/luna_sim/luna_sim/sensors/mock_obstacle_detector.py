"""
Mock lidar-based obstacle detection for the Level 1 sim.

Knows the TRUE arena layout and the TRUE robot pose, and publishes only what a real lidar
would see: obstacles within range and field of view, with range-scaled position and radius
error, rasterized as filled circles into a map_msgs/OccupancyGridUpdate on /map_updates.

Stands in for lidar -> obstacle_detector so the planner can be developed now. Gazebo and the
real lidar publish the same topic later; the planner must not notice the swap.

Accumulate-only: a cell marked occupied is never set back to free.
"""
import math
import os
import random

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

from luna_sim.arena.layout import load_layout

WARN_PERIOD_S = 5.0


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x, y, z, w):
    """Yaw about z. All the 2D sim needs."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def sensor_origin(x, y, yaw, mount_x, mount_y):
    """Sensor XY in the map frame, from the robot pose and its base_link mount offset."""
    return (x + mount_x * math.cos(yaw) - mount_y * math.sin(yaw),
            y + mount_x * math.sin(yaw) + mount_y * math.cos(yaw))


def edge_distance(sx, sy, ox, oy, radius):
    """Sensor to the obstacle's near edge. Negative means the sensor is inside the footprint."""
    return math.hypot(ox - sx, oy - sy) - radius


def in_fov(sx, sy, heading, ox, oy, fov_deg):
    """Check whether the obstacle CENTER bearing lies in the FOV wedge, centered on heading."""
    if fov_deg >= 360.0:
        return True
    return abs(wrap_angle(math.atan2(oy - sy, ox - sx) - heading)) <= math.radians(fov_deg) / 2.0


def in_range(edge_dist, min_range_m, max_range_m):
    """
    Range gate, measured to the obstacle edge.

    edge_dist <= 0.0            -> visible, the sensor is inside or on the footprint
    0.0 < edge_dist < min_range -> hidden, minimum-range blind spot
    min_range <= d <= max_range -> visible, normal detection band
    edge_dist > max_range       -> hidden, beyond maximum range
    """
    if edge_dist <= 0.0:
        return True
    return min_range_m <= edge_dist <= max_range_m


def is_visible(sx, sy, heading, ox, oy, radius, min_range_m, max_range_m, fov_deg):
    """Full visibility test: the range gate on edge distance, plus the FOV wedge."""
    if not in_range(edge_distance(sx, sy, ox, oy, radius), min_range_m, max_range_m):
        return False
    return in_fov(sx, sy, heading, ox, oy, fov_deg)


def noise_scale(center_dist, max_range_m):
    """Error multiplier: 1x at the sensor, 2x at max range."""
    if max_range_m <= 0.0:
        return 1.0
    return 1.0 + center_dist / max_range_m


def apply_noise(rng, ox, oy, radius, center_dist, max_range_m, pos_std, rad_std):
    """Gaussian error on position and radius, scaled with distance. Radius stays >= 0."""
    scale = noise_scale(center_dist, max_range_m)
    return (ox + rng.gauss(0.0, pos_std * scale),
            oy + rng.gauss(0.0, pos_std * scale),
            max(radius + rng.gauss(0.0, rad_std * scale), 0.0))


def rasterize_circle(cx, cy, radius, origin_x, origin_y, resolution, width, height):
    """Cells whose CENTER is inside the circle, clipped to the grid, as (col, row)."""
    if radius <= 0.0 or resolution <= 0.0:
        return []

    col_lo = max(int(math.floor((cx - radius - origin_x) / resolution)), 0)
    row_lo = max(int(math.floor((cy - radius - origin_y) / resolution)), 0)
    col_hi = min(int(math.ceil((cx + radius - origin_x) / resolution)), width - 1)
    row_hi = min(int(math.ceil((cy + radius - origin_y) / resolution)), height - 1)

    r_sq = radius * radius
    cells = []
    for row in range(row_lo, row_hi + 1):
        dy = origin_y + (row + 0.5) * resolution - cy
        for col in range(col_lo, col_hi + 1):
            dx = origin_x + (col + 0.5) * resolution - cx
            if dx * dx + dy * dy <= r_sq:
                cells.append((col, row))
    return cells


def cells_bbox(cells):
    """Bounding box of a cell list as (x, y, width, height). None if empty."""
    if not cells:
        return None
    cols = [c for c, _ in cells]
    rows = [r for _, r in cells]
    x0, y0 = min(cols), min(rows)
    return x0, y0, max(cols) - x0 + 1, max(rows) - y0 + 1


def extract_patch(grid, grid_width, x0, y0, patch_width, patch_height):
    """Row-major copy of a sub-rectangle of grid, for OccupancyGridUpdate.data."""
    data = []
    for row in range(y0, y0 + patch_height):
        start = row * grid_width + x0
        data.extend(grid[start:start + patch_width])
    return data


def obstacle_radius(obstacle):
    """Radius from the outer rim diameter. Boulders and craters are both obstacles."""
    return float(obstacle["diameter"]) / 2.0


def load_mount_xy(path, sensor_link):
    """Read sensors.<sensor_link>.xyz from robot_geometry.yaml. XY only; z is irrelevant in 2D."""
    with open(path) as f:
        geometry = yaml.safe_load(f)
    xyz = geometry["sensors"][sensor_link]["xyz"]
    return float(xyz[0]), float(xyz[1])


class MockObstacleDetector(Node):
    """Fakes the lidar -> obstacle_detector pipeline from arena ground truth."""

    def __init__(self) -> None:
        super().__init__("mock_obstacle_detector")

        layout_path = self.declare_parameter("arena_json", "").value
        if not layout_path:
            raise RuntimeError("parameter 'arena_json' is required (path to arena_<seed>.json)")

        rate_hz = float(self.declare_parameter("rate_hz", 10.0).value)
        self.min_range_m = float(self.declare_parameter("min_range_m", 0.3).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 3.0).value)
        self.fov_deg = float(self.declare_parameter("fov_deg", 90.0).value)
        self.pos_std = float(self.declare_parameter("position_noise_std_m", 0.03).value)
        self.rad_std = float(self.declare_parameter("radius_noise_std_m", 0.02).value)
        self.occupied = int(self.declare_parameter("occupied_value", 100).value)
        seed = int(self.declare_parameter("seed", 0).value)

        sensor_link = self.declare_parameter("sensor_link", "lidar_link").value
        geometry_path = self.declare_parameter("robot_geometry_yaml", "").value or os.path.join(
            get_package_share_directory("luna_description"), "config", "robot_geometry.yaml")
        self.mount_x, self.mount_y = load_mount_xy(geometry_path, sensor_link)

        # Ground truth, read once at startup. Only sim nodes may do this.
        self.obstacles = [(float(o["x"]), float(o["y"]), obstacle_radius(o))
                          for o in load_layout(layout_path)["obstacles"]]

        self.rng = random.Random(seed)
        self.pose = None            # true robot pose, from kinematic_base at 50 Hz
        self.grid = None            # persistent accumulated grid, seeded from /map
        self.meta = None            # (resolution, width, height, origin_x, origin_y)
        self.last_warn_t = None
        self.estimates = {} # list of obstacles that have been detected

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        update_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(OccupancyGridUpdate, "/map_updates", update_qos)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(PoseStamped, "/sim/ground_truth/pose", self._on_pose, 10)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"{len(self.obstacles)} obstacles from {layout_path}, detecting at {rate_hz:.0f} Hz, "
            f"range [{self.min_range_m:.2f}, {self.max_range_m:.2f}] m to edge, "
            f"fov {self.fov_deg:.0f} deg, seed {seed}, "
            f"{sensor_link} mount offset ({self.mount_x:.2f}, {self.mount_y:.2f})")

    def _on_map(self, msg) -> None:
        """Seed the accumulated grid from the wall map, preserving cells we already marked."""
        info = msg.info
        meta = (info.resolution, info.width, info.height,
                info.origin.position.x, info.origin.position.y)

        if self.meta is None or meta != self.meta:
            self.grid = list(msg.data)
            self.meta = meta
            for estimate in self.estimates.values():
                self._mark(estimate)
            self.get_logger().info(
                f"/map seeded: {info.width}x{info.height} at {info.resolution:.3f} m/cell, "
                f"origin ({meta[3]:.2f}, {meta[4]:.2f})")
        else:
            for i, v in enumerate(msg.data):
                if self.grid[i] != self.occupied:
                    self.grid[i] = v

    def _mark(self, estimate):
        """Mark an estimated obstacle's cells as occupied and return the cells that changed."""
        resolution, width, height, origin_x, origin_y = self.meta
        changed = []
        for col, row in rasterize_circle(*estimate, origin_x, origin_y,
                                         resolution, width, height):
            i = row * width + col
            if self.grid[i] != self.occupied:
                self.grid[i] = self.occupied
                changed.append((col, row))
        return changed

    def _on_pose(self, msg) -> None:
        pose = msg.pose
        self.pose = (pose.position.x, pose.position.y,
                     yaw_from_quaternion(pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z, pose.orientation.w))

    def _warn_throttled(self, text) -> None:
        """Warn at most once per WARN_PERIOD_S, on the node clock like everything else."""
        now = self.get_clock().now()
        if self.last_warn_t is not None:
            if (now - self.last_warn_t).nanoseconds * 1e-9 < WARN_PERIOD_S:
                return
        self.last_warn_t = now
        self.get_logger().warn(text)

    def _tick(self) -> None:
        if self.grid is None:
            self._warn_throttled("waiting for /map; publishing nothing")
            return
        if self.pose is None:
            self._warn_throttled("waiting for /sim/ground_truth/pose; publishing nothing")
            return

        resolution, width, height, origin_x, origin_y = self.meta
        x, y, yaw = self.pose
        sx, sy = sensor_origin(x, y, yaw, self.mount_x, self.mount_y)

        new_cells = []
        for idx, (ox, oy, radius) in enumerate(self.obstacles):
            if idx in self.estimates:
                continue  # already detected; its estimate is fixed
            if not is_visible(sx, sy, yaw, ox, oy, radius,
                              self.min_range_m, self.max_range_m, self.fov_deg):
                continue

            center_dist = math.hypot(ox - sx, oy - sy)
            estimate = apply_noise(self.rng, ox, oy, radius, center_dist,
                                   self.max_range_m, self.pos_std, self.rad_std)
            self.estimates[idx] = estimate
            new_cells.extend(self._mark(estimate))

        bbox = cells_bbox(new_cells)
        if bbox is None:
            return

        x0, y0, patch_w, patch_h = bbox
        update = OccupancyGridUpdate()
        update.header.stamp = self.get_clock().now().to_msg()
        update.header.frame_id = "map"
        update.x = x0
        update.y = y0
        update.width = patch_w
        update.height = patch_h
        update.data = extract_patch(self.grid, width, x0, y0, patch_w, patch_h)
        self.pub.publish(update)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
