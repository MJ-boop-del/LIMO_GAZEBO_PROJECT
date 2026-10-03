import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

import tf2_ros
from tf2_ros import TransformException


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # ==========================================
        # Parameters
        # ==========================================

        self.lookahead_distance = 0.3
        self.linear_speed = 0.1
        self.max_angular_speed = 1.0

        # ==========================================
        # Global Path
        # ==========================================

        self.path = None

        self.path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )

        # ==========================================
        # Command publisher
        # ==========================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==========================================
        # TF
        # map -> base_link
        # ==========================================

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ==========================================
        # Control loop
        # ==========================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Pure Pursuit controller started'
        )

    # ==========================================
    # Path callback
    # ==========================================

    def path_callback(self, msg):

        self.path = msg

    # ==========================================
    # Angle normalization
    # ==========================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ==========================================
    # Get current LIMO pose from TF
    # ==========================================

    def get_robot_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation

            siny_cosp = 2.0 * (
                q.w * q.z +
                q.x * q.y
            )

            cosy_cosp = 1.0 - 2.0 * (
                q.y * q.y +
                q.z * q.z
            )

            yaw = math.atan2(
                siny_cosp,
                cosy_cosp
            )

            return x, y, yaw

        except TransformException:

            return None

    # ==========================================
    # Find nearest path point
    # ==========================================

    def find_nearest_index(self, x, y):

        if self.path is None:
            return None

        min_distance = float('inf')
        nearest_index = None

        for i, pose in enumerate(self.path.poses):

            px = pose.pose.position.x
            py = pose.pose.position.y

            distance = math.hypot(
                px - x,
                py - y
            )

            if distance < min_distance:

                min_distance = distance
                nearest_index = i

        return nearest_index

    # ==========================================
    # Find lookahead point
    # ==========================================

    def find_lookahead_index(
        self,
        x,
        y,
        nearest_index
    ):

        if nearest_index is None:
            return None

        number_of_points = len(
            self.path.poses
        )

        # 폐곡선이므로 전체 path를 순환하면서 검색
        for offset in range(
            number_of_points
        ):

            index = (
                nearest_index +
                offset
            ) % number_of_points

            pose = self.path.poses[index]

            px = pose.pose.position.x
            py = pose.pose.position.y

            distance = math.hypot(
                px - x,
                py - y
            )

            if distance >= self.lookahead_distance:

                return index

        return nearest_index

    # ==========================================
    # Control
    # ==========================================

    def control_loop(self):

        # ------------------------------------------
        # Path가 아직 없으면 정지
        # ------------------------------------------

        if self.path is None:

            self.publish_stop()
            return

        # ------------------------------------------
        # TF에서 현재 위치 가져오기
        # ------------------------------------------

        robot_pose = self.get_robot_pose()

        if robot_pose is None:

            self.publish_stop()
            return

        x, y, yaw = robot_pose

        # ------------------------------------------
        # 가장 가까운 path point
        # ------------------------------------------

        nearest_index = self.find_nearest_index(
            x,
            y
        )

        if nearest_index is None:

            self.publish_stop()
            return

        # ------------------------------------------
        # Lookahead point
        # ------------------------------------------

        lookahead_index = self.find_lookahead_index(
            x,
            y,
            nearest_index
        )

        if lookahead_index is None:

            self.publish_stop()
            return

        target = self.path.poses[
            lookahead_index
        ]

        target_x = target.pose.position.x
        target_y = target.pose.position.y

        # ------------------------------------------
        # Map 좌표 → LIMO 좌표
        # ------------------------------------------

        dx = target_x - x
        dy = target_y - y

        local_x = (
            math.cos(yaw) * dx +
            math.sin(yaw) * dy
        )

        local_y = (
            -math.sin(yaw) * dx +
            math.cos(yaw) * dy
        )

        # ------------------------------------------
        # Lookahead point가 뒤에 있으면 정지
        # ------------------------------------------
       

        # ------------------------------------------
        # Pure Pursuit curvature
        # ------------------------------------------

        distance_squared = (
            local_x * local_x +
            local_y * local_y
        )

        if distance_squared < 1e-6:

            self.publish_stop()
            return

        curvature = (
            2.0 * local_y /
            distance_squared
        )

        angular_z = (
            self.linear_speed *
            curvature
        )

        self.get_logger().info(
            f"robot=({x:.3f}, {y:.3f}) "
            f"nearest={nearest_index} "
            f"lookahead={lookahead_index} "
            f"target=({target_x:.3f}, {target_y:.3f}) "
            f"local=({local_x:.3f}, {local_y:.3f}) "
            f"curvature={curvature:.3f} "
            f"angular={angular_z:.3f}"
        )

        # ------------------------------------------
        # Angular velocity 제한
        # ------------------------------------------

        angular_z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                angular_z
            )
        )

        # ------------------------------------------
        # Command
        # ------------------------------------------

        cmd = Twist()

        cmd.linear.x = self.linear_speed
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)

    # ==========================================
    # Stop
    # ==========================================

    def publish_stop(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = PurePursuitController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.publish_stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()