import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration


class PrecisionLandVizNode(Node):
    """
    Subscribes to /target_pose_world and publishes RViz Marker messages.

    Replicates the green box visualization from the original C++ Gazebo implementation.
    Marker dimensions: 0.5 x 0.5 x 0.02 m (matches the ArUco target).
    """

    def __init__(self):
        super().__init__('precision_land_viz')

        self.declare_parameter('enable_gz_viz', False)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose_world',
            self._pose_callback,
            qos,
        )
        self._marker_pub = self.create_publisher(Marker, '/target_pose_marker', 10)

        self.get_logger().info('PrecisionLandViz started — publishing to /target_pose_marker')

    def _pose_callback(self, msg):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'aruco_target'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = msg.pose

        # Match C++ Gazebo marker dimensions
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.02

        # Green with slight transparency, matching C++ Gazebo colors
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

        # Marker expires after 0.5 s if no new pose arrives
        marker.lifetime = Duration(sec=0, nanosec=500_000_000)

        self._marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
