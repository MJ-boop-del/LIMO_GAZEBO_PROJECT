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

        # 현재 localization map의 좌표계
        self.path.header.frame_id = 'map'

        # LIMO 출발점은 path index 301 부근
        # 실제 주행 방향은 301 -> 300 -> 299 -> ...
        start_index = 301

        ordered_indices = list(
            range(start_index, -1, -1)
        ) + list(
            range(len(data) - 1, start_index, -1)
        )

        for i in ordered_indices:

            row = data[i]

            x = float(row[1])
            y = float(row[2])

            # ==========================================
            # 문제 구간에서 path를 안쪽으로 12 cm 이동
            # ==========================================

            if 55 <= i <= 80:

                next_i = (i + 1) % len(data)

                nx = float(data[next_i][1])
                ny = float(data[next_i][2])

                dx = nx - x
                dy = ny - y

                length = np.hypot(dx, dy)

                if length > 1e-6:

                    # 진행 방향의 오른쪽 법선
                    right_x = dy / length
                    right_y = -dx / length

                    # 왼쪽으로 12 cm 이동
                    offset = 0.12

                    x += right_x * offset
                    y += right_y * offset

            pose = PoseStamped()

            pose.header.frame_id = 'map'

            pose.pose.position.x = x
            pose.pose.position.y = y
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

        self.path.header.stamp = (
            self.get_clock().now().to_msg()
        )

        for pose in self.path.poses:
            pose.header.stamp = self.path.header.stamp

        self.publisher.publish(self.path)

        self.get_logger().info(
            'Global path published'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GlobalPathPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()