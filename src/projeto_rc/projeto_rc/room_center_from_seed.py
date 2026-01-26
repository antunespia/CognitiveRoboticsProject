#!/usr/bin/env python3
import math
from dataclasses import dataclass
from collections import deque
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

from scipy.ndimage import distance_transform_edt, binary_dilation


@dataclass
class Seed:
    name: str
    x: float
    y: float
    z: float
    yaw: float
    time_sec: float  # original time in file


def parse_seeds(path: str) -> List[Seed]:
    seeds: List[Seed] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = [p.strip() for p in s.split(",")]
            if len(parts) != 6:
                continue
            seeds.append(
                Seed(
                    name=parts[0],
                    x=float(parts[1]),
                    y=float(parts[2]),
                    z=float(parts[3]),
                    yaw=float(parts[4]),
                    time_sec=float(parts[5]),
                )
            )
    return seeds


def world_to_grid(x: float, y: float, origin_x: float, origin_y: float, res: float) -> Tuple[int, int]:
    gx = int(math.floor((x - origin_x) / res))
    gy = int(math.floor((y - origin_y) / res))
    return gx, gy


def grid_to_world(gx: int, gy: int, origin_x: float, origin_y: float, res: float) -> Tuple[float, float]:
    x = origin_x + (gx + 0.5) * res
    y = origin_y + (gy + 0.5) * res
    return x, y


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def flood_fill(free: np.ndarray, sx: int, sy: int) -> np.ndarray:
    """Connected component of free space containing (sx,sy). 8-connected."""
    H, W = free.shape
    if not (0 <= sx < W and 0 <= sy < H) or not free[sy, sx]:
        return np.zeros_like(free, dtype=bool)

    comp = np.zeros_like(free, dtype=bool)
    q = deque([(sx, sy)])
    comp[sy, sx] = True

    nbrs = [(-1,-1),(0,-1),(1,-1),(-1,0),(1,0),(-1,1),(0,1),(1,1)]
    while q:
        x, y = q.popleft()
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and free[ny, nx] and not comp[ny, nx]:
                comp[ny, nx] = True
                q.append((nx, ny))
    return comp


class RoomCenterInflated(Node):
    def __init__(self):
        super().__init__("room_center_inflated")

        self.declare_parameter("seeds_file", "waypoints_input.txt")
        self.declare_parameter("output_file", "waypoints_output.txt")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("unknown_as_occupied", True)

        # Tune to close doors
        self.declare_parameter("inflation_radius_m", 0.30)

        # Time proportional to room area (seconds per m^2)
        self.declare_parameter("time_gain_sec_per_m2", 10.0)

        # If True, overwrite time only when center succeeds; if False, always overwrite time when area is computed
        self.declare_parameter("overwrite_time_on_success_only", True)

        self.seeds_file = self.get_parameter("seeds_file").value
        self.output_file = self.get_parameter("output_file").value
        self.occ_thr = int(self.get_parameter("occupied_threshold").value)
        self.unk_occ = bool(self.get_parameter("unknown_as_occupied").value)
        self.inflation_radius_m = float(self.get_parameter("inflation_radius_m").value)
        self.k_time = float(self.get_parameter("time_gain_sec_per_m2").value)
        self.overwrite_time_on_success_only = bool(self.get_parameter("overwrite_time_on_success_only").value)

        self.seeds: List[Seed] = parse_seeds(self.seeds_file)
        self.get_logger().info(f"Loaded {len(self.seeds)} seeds from {self.seeds_file}")

        self.done = False
        self.pose_stable = False
        self._last_pose = None  # (x, y, z, yaw)
        self._last_change_ns = self.get_clock().now().nanoseconds
        self._stable_duration_sec = 30.0
        self._pos_tol = 0.05
        self._yaw_tol = 0.1
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, "/odom", self.odometry_callback, 10)

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        current = (x, y, z, yaw)
        if self._last_pose is None:
            self._last_pose = current
            self._last_change_ns = self.get_clock().now().nanoseconds
            return
        def angle_diff(a, b):
            return abs(math.atan2(math.sin(a - b), math.cos(a - b)))
        dx = abs(current[0] - self._last_pose[0])
        dy = abs(current[1] - self._last_pose[1])
        dz = abs(current[2] - self._last_pose[2])
        dyaw = angle_diff(current[3], self._last_pose[3])
        if dx > self._pos_tol or dy > self._pos_tol or dz > self._pos_tol or dyaw > self._yaw_tol:
            self._last_pose = current
            self._last_change_ns = self.get_clock().now().nanoseconds
            self.pose_stable = False
        else:
            now_ns = self.get_clock().now().nanoseconds
            elapsed = (now_ns - self._last_change_ns) / 1e9
            if elapsed >= self._stable_duration_sec:
                if not self.pose_stable:
                    self.get_logger().info(f"Pose estável há {elapsed:.1f}s, pronto para processar mapa.")
                self.pose_stable = True
            else:
                self.pose_stable = False

    def on_map(self, msg: OccupancyGrid):
        if self.done:
            return
        if not self.pose_stable:
            self.get_logger().info("A aguardar que o robô fique parado durante 30 segundos antes de processar o mapa...")
            return
        self.done = True

        info = msg.info
        W, H = info.width, info.height
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape((H, W))

        # Occupied mask
        if self.unk_occ:
            occupied = (data == -1) | (data > self.occ_thr)
        else:
            occupied = (data > self.occ_thr)

        # Inflate occupied
        rad_cells = int(math.ceil(self.inflation_radius_m / res))
        self.get_logger().info(
            f"Inflation: {self.inflation_radius_m:.2f} m -> {rad_cells} cells (res={res:.2f} m/cell)"
        )

        if rad_cells > 0:
            structure = np.ones((2 * rad_cells + 1, 2 * rad_cells + 1), dtype=bool)
            occupied_infl = binary_dilation(occupied, structure=structure)
        else:
            occupied_infl = occupied

        free_infl = ~occupied_infl
        dist_m = distance_transform_edt(free_infl) * res

        # Start output as a copy of input (guarantees all rooms exist)
        out_rows: List[Seed] = [Seed(s.name, s.x, s.y, s.z, s.yaw, s.time_sec) for s in self.seeds]

        for i, s in enumerate(self.seeds):
            gx, gy = world_to_grid(s.x, s.y, origin_x, origin_y, res)
            gx = clamp(gx, 0, W - 1)
            gy = clamp(gy, 0, H - 1)

            if not free_infl[gy, gx]:
                self.get_logger().warn(
                    f"[{s.name}] seed not free AFTER inflation; keeping original x,y,time in output. "
                    f"(Try lowering inflation_radius_m) seed_grid=({gx},{gy})"
                )
                continue

            comp = flood_fill(free_infl, gx, gy)
            comp_cells = int(comp.sum())
            if comp_cells < 20:
                self.get_logger().warn(
                    f"[{s.name}] component too small ({comp_cells}); keeping original x,y,time in output."
                )
                continue

            area_m2 = comp_cells * (res * res)

            masked = np.where(comp, dist_m, -1.0)
            idx = int(np.argmax(masked))
            cy, cx = np.unravel_index(idx, masked.shape)
            cx_w, cy_w = grid_to_world(cx, cy, origin_x, origin_y, res)

            time_sec_new = float(self.k_time * area_m2)

            # Update output row (x,y always updated on success)
            out_rows[i].x = cx_w
            out_rows[i].y = cy_w

            # Update time depending on policy
            if self.overwrite_time_on_success_only:
                out_rows[i].time_sec = time_sec_new
            else:
                out_rows[i].time_sec = time_sec_new

            self.get_logger().info(
                f"[{s.name}] UPDATED center=({cx_w:.3f},{cy_w:.3f}) area={area_m2:.2f}m^2 time={out_rows[i].time_sec:.1f}s"
            )

        # Write output identical format
        with open(self.output_file, "w", encoding="utf-8") as f:
            f.write("# Waypoints list for projeto_rc\n")
            f.write("# Format: name,x,y,z,yaw,disinfection_time_sec (yaw in radians, disinfection_time in seconds)\n\n")
            for w in out_rows:
                f.write(f"{w.name},{w.x:.3f},{w.y:.3f},{w.z:.3f},{w.yaw:.4f},{w.time_sec:.1f}\n")

        self.get_logger().info(f"Wrote output waypoints to {self.output_file}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = RoomCenterInflated()
    rclpy.spin(node)


if __name__ == "__main__":
    main()