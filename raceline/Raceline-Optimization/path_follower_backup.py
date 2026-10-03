import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # ==============================
        # Parameters
        # ==============================
        self.lookahead_distance = 0.5
        self.linear_speed = 0.2
        self.max_angular_speed = 1.5

        # LIMO wheelbase의 대략적인 값
        self.wheelbase = 0.2

        self.path = None
        self.odom = None

        # ==============================
        # Subscribers
        # ==============================
        self.path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ==============================
        # Publisher
        # ==============================
        self.cmd_pub = self.create_publisher(
            Twist,
            '/ackermann_controller/cmd_vel',
            10
        )

        # 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Pure Pursuit controller started'
        )

    def path_callback(self, msg):
        self.path = msg

    def odom_callback(self, msg):
        self.odom = msg

    def get_yaw(self, q):
        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def find_nearest_index(self, x, y):

        if self.path is None:
            return None

        min_dist = float('inf')
        nearest_index = None

        for i, pose in enumerate(self.path.poses):

            px = pose.pose.position.x
            py = pose.pose.position.y

            dist = math.hypot(
                px - x,
                py - y
            )

            if dist < min_dist:
                min_dist = dist
                nearest_index = i

        return nearest_index

    def find_lookahead_index(
        self,
        x,
        y,
        nearest_index
    ):

        if nearest_index is None:
            return None

        for i in range(
            nearest_index,
            len(self.path.poses)
        ):

            px = self.path.poses[i].pose.position.x
            py = self.path.poses[i].pose.position.y

            distance = math.hypot(
                px - x,
                py - y
            )

            if distance >= self.lookahead_distance:
                return i

        # 폐곡선의 경우 처음으로 돌아감
        for i in range(0, nearest_index):

            px = self.path.poses[i].pose.position.x
            py = self.path.poses[i].pose.position.y

            distance = math.hypot(
                px - x,
                py - y
            )

            if distance >= self.lookahead_distance:
                return i

        return nearest_index

    def control_loop(self):

        # 경로 또는 odom이 없으면 정지
        if self.path is None or self.odom is None:

            self.publish_stop()
            return

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y

        yaw = self.get_yaw(
            self.odom.pose.pose.orientation
        )

        nearest_index = self.find_nearest_index(
            x,
            y
        )

        if nearest_index is None:

            self.publish_stop()
            return

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

        # --------------------------------
        # 목표점을 차량 좌표계로 변환
        # --------------------------------

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

        # 차량 뒤쪽에 있는 점은 사용하지 않음
        if local_x <= 0.0:

            self.publish_stop()
            return

        # --------------------------------
        # Pure Pursuit
        # --------------------------------

        ld2 = (
            local_x * local_x +
            local_y * local_y
        )

        curvature = (
            2.0 * local_y / ld2
        )

        angular_z = (
            self.linear_speed *
            curvature
        )

        angular_z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                angular_z
            )
        )

        cmd = Twist()

        cmd.linear.x = self.linear_speed
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)

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