#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('speed', 0.3)

        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic').value
        self.speed = float(self.get_parameter('speed').value)

        self.lookahead_goal = None

        self.cmd_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)

        self.goal_sub = self.create_subscription(
            Marker,
            '/lookahead_goal',
            self.goal_callback,
            10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self)

        self.timer = self.create_timer(
            0.05, self.control_loop)

        self.get_logger().info(
            'Pure Pursuit node started')

    def goal_callback(self, msg):
        self.lookahead_goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )

    def control_loop(self):

        if self.lookahead_goal is None:
            self.publish_stop()
            return

        try:
            if not self.tf_buffer.can_transform(
                'map',
                'base_link',
                rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.5)
            ):
                return

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(seconds=0)
            )

        except Exception as e:
            self.get_logger().warn(
                f'TF unavailable: {e}')
            self.publish_stop()
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        q = transform.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        target_x, target_y = self.lookahead_goal

        dx = target_x - robot_x
        dy = target_y - robot_y

        local_x = (
            math.cos(yaw) * dx
            + math.sin(yaw) * dy
        )

        local_y = (
            -math.sin(yaw) * dx
            + math.cos(yaw) * dy
        )

        distance_squared = (
            local_x * local_x
            + local_y * local_y
        )

        if distance_squared < 1e-6:
            self.publish_stop()
            return

        curvature = (
            2.0 * local_y
            / distance_squared
        )

        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = curvature * self.speed

        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = PurePursuit()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.publish_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
