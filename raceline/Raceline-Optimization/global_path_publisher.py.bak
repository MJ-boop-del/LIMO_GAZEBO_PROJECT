import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import os


class GlobalPathPublisher(Node):

    def __init__(self):
        super().__init__('global_path_publisher')

        self.publisher = self.create_publisher(
            Path,
            '/global_path',
            10
        )

        csv_file = 'outputs/traj_race_cl.csv'

        if not os.path.exists(csv_file):
            self.get_logger().error(
                f'CSV file not found: {csv_file}'
            )
            return

        data = np.loadtxt(
            csv_file,
            delimiter=';',
            comments='#'
        )

        self.path = Path()

        # 중요: 현재 localization map의 좌표계
        self.path.header.frame_id = 'map'

        # -------------------------------------------------
        # Global Path 진행 방향 및 시작점 설정
        #
        # 현재 LIMO 출발점은 path index 301 부근이며,
        # 실제 주행 방향은 301 -> 300 -> 299 -> ...
        # -------------------------------------------------

        start_index = 301

        ordered_indices = list(
            range(start_index, -1, -1)
        ) + list(
            range(len(data) - 1, start_index, -1)
        )

        for i in ordered_indices:

            row = data[i]

            pose = PoseStamped()

            pose.header.frame_id = 'map'

            pose.pose.position.x = float(row[1])
            pose.pose.position.y = float(row[2])
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            self.path.poses.append(pose)
        self.get_logger().info(
            f'Global path loaded: {len(self.path.poses)} points'
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_path
        )

    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()

        for pose in self.path.poses:
            pose.header.stamp = self.path.header.stamp

        self.publisher.publish(self.path)
        self.get_logger().info('Global path published')


def main(args=None):

    rclpy.init(args=args)

    node = GlobalPathPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
