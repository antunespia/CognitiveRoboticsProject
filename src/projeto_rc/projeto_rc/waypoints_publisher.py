#Deteta se o slam parou (robot parou de mexer) e depois envia waypoints para o robot seguir

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from pathlib import Path
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


def quaternion_to_yaw(q):
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)


class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')

        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)  # Waypoint publisher goal pose
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        # Create a bool to see if the waypoint logic is ready to run (has the slam finished?)
        self.is_slam_finished = False

        # For detecting stable pose
        self._last_pose = None  # (x,y,z,yaw)
        self._last_change_ns = self.get_clock().now().nanoseconds
        self._stable_duration_sec = 40.0
        self._pos_tol = 1e-3
        self._yaw_tol = 0.01

        # Waypoint mission state
        self._waypoints = []
        self._current_goal_idx = None
        self._missions_started = False
        self._goal_pos_tol = 0.2  # meters to consider reached
        self._goal_yaw_tol = 0.2  # radians
        self._goal_stable_required_s = 2.0
        self._goal_reached_since_ns = None
        self._goal_published = False

        # Load waypoints from config file
        try:
            self._load_waypoints()
            self.get_logger().info(f'Loaded {len(self._waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Periodic check: if pose remained stable for required duration
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self._last_change_ns) / 1e9
        if (not self.is_slam_finished) and (elapsed >= self._stable_duration_sec):
            self.is_slam_finished = True
            self.get_logger().info(f'SLAM considered finished: pose stable for {elapsed:.1f}s')

        # If SLAM just finished, start mission
        if self.is_slam_finished and (not self._missions_started):
            if len(self._waypoints) == 0:
                self.get_logger().warn('No waypoints to publish')
            else:
                self._missions_started = True
                self._current_goal_idx = 0
                self._publish_current_goal()

        # If mission active, only publish goal once per waypoint
        if self._missions_started and (self._current_goal_idx is not None) and (not self._goal_published):
            self._publish_current_goal()
            self._goal_published = True

    def odometry_callback(self, msg):  # deteta se a pose mudou
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q)

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

        if (dx > self._pos_tol) or (dy > self._pos_tol) or (dz > self._pos_tol) or (dyaw > self._yaw_tol):
            self._last_pose = current
            self._last_change_ns = self.get_clock().now().nanoseconds
            if self.is_slam_finished:
                self.is_slam_finished = False
                self.get_logger().info('Pose changed: SLAM not finished')

        # If a mission is active, check whether current goal was reached
        if self._missions_started and (self._current_goal_idx is not None) and self._current_goal_idx < len(self._waypoints):
            gx, gy, gz, gyaw = self._waypoints[self._current_goal_idx]['coords']
            dist = math.sqrt((x - gx) ** 2 + (y - gy) ** 2 + (z - gz) ** 2)
            ang = abs(math.atan2(math.sin(yaw - gyaw), math.cos(yaw - gyaw)))
            now_ns = self.get_clock().now().nanoseconds
            if (dist <= self._goal_pos_tol) and (ang <= self._goal_yaw_tol):
                if self._goal_reached_since_ns is None:
                    self._goal_reached_since_ns = now_ns
                else:
                    held = (now_ns - self._goal_reached_since_ns) / 1e9
                    if held >= self._goal_stable_required_s:
                        self.get_logger().info(f'Goal {self._waypoints[self._current_goal_idx]["name"]} reached')
                        # advance to next goal
                        self._current_goal_idx += 1
                        self._goal_reached_since_ns = None
                        self._goal_published = False
                        if self._current_goal_idx >= len(self._waypoints):
                            self.get_logger().info('All waypoints completed')
                            self._current_goal_idx = None
                        else:
                            self._publish_current_goal()
                            self._goal_published = True
            else:
                # not within tolerance
                self._goal_reached_since_ns = None

    # --- Waypoint helpers ---
    def _load_waypoints(self):
        # Try multiple candidate locations for config/waypoints.txt.
        file_dir = Path(__file__).resolve().parent
        candidates = []
        # package share (installed location) via ament_index
        if get_package_share_directory is not None:
            try:
                share = get_package_share_directory('projeto_rc')
                candidates.append(Path(share) / 'config' / 'waypoints.txt')
            except Exception:
                pass
        # search package dir and its ancestors (source layout)
        for p in [file_dir] + list(file_dir.parents):
            candidates.append(p / 'config' / 'waypoints.txt')
            candidates.append(p / 'projeto_rc' / 'config' / 'waypoints.txt')
        # check current working dir as last resort
        candidates.append(Path.cwd() / 'config' / 'waypoints.txt')

        path = None
        for c in candidates:
            if c.exists():
                path = c
                break

        if path is None:
            raise FileNotFoundError('Waypoints file not found. Searched: ' + ','.join(str(p) for p in candidates))

        waypoints = []
        with open(path, 'r') as f:
            for line in f:
                s = line.split('#', 1)[0].strip()
                if not s:
                    continue
                parts = [p.strip() for p in s.split(',')]
                if len(parts) < 5:
                    continue
                name = parts[0]
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                yaw = float(parts[4])
                waypoints.append({'name': name, 'coords': (x, y, z, yaw)})
        self._waypoints = waypoints

    def _yaw_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def _publish_current_goal(self):
        if (self._current_goal_idx is None) or (self._current_goal_idx >= len(self._waypoints)):
            return
        wp = self._waypoints[self._current_goal_idx]
        x, y, z, yaw = wp['coords']
        msg = PoseStamped()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published goal {wp["name"]} (idx {self._current_goal_idx})')


def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    try:
        rclpy.spin(waypoint_publisher)
    except KeyboardInterrupt:
        pass

    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


