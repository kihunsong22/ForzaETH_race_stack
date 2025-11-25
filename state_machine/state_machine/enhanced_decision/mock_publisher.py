"""
Mock publisher for /overtake_opportunity topic

This simulates Package 1's output so Package 2 can be developed in parallel.
Teammate 1 will replace this with real curvature-based detection.
"""

import rclpy
from rclpy.node import Node
from f110_msgs.msg import OvertakeOpportunity


class MockOvertakePublisher(Node):
    """
    Publishes mock /overtake_opportunity messages

    For testing: Always publishes is_safe_zone=True
    """

    def __init__(self):
        super().__init__('mock_overtake_publisher')

        self.publisher = self.create_publisher(
            OvertakeOpportunity,
            '/overtake_opportunity',
            10
        )

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_mock)

        self.get_logger().info('Mock overtake opportunity publisher started')

    def publish_mock(self):
        """Publish simple mock data"""
        msg = OvertakeOpportunity()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Simple mock: always in safe zone
        msg.is_safe_zone = True
        msg.zone_start_s = 0.0
        msg.zone_end_s = 100.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockOvertakePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
