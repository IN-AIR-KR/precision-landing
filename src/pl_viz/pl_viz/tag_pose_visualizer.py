"""Display the estimated target pose in the Gazebo scene."""
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TagPoseVisualizer(Node):
    def __init__(self):
        super().__init__('tag_pose_visualizer')
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.marker_pb2 import Marker
        from gz.msgs10.marker_v_pb2 import Marker_V
        from gz.transport13 import Node as GazeboNode

        self.transport = GazeboNode()
        self.response_type = Boolean
        self.request = Marker_V()
        self.marker = self.request.marker.add(
            ns='aruco_target', id=1, action=Marker.ADD_MODIFY,
            type=Marker.BOX, visibility=Marker.GUI,
        )
        self.marker.scale.x = self.marker.scale.y = 0.5
        self.marker.scale.z = 0.02
        for color in (self.marker.material.ambient, self.marker.material.diffuse):
            color.g = 1.0
            color.a = 0.8
        self.marker.lifetime.nsec = 500_000_000
        self.subscription = self.create_subscription(
            PoseStamped, '/target_pose_world', self.on_pose,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def on_pose(self, message):
        position = message.pose.position
        orientation = message.pose.orientation
        pose = self.marker.pose
        pose.position.x, pose.position.y, pose.position.z = position.x, position.y, position.z
        pose.orientation.x, pose.orientation.y = orientation.x, orientation.y
        pose.orientation.z, pose.orientation.w = orientation.z, orientation.w
        success, response = self.transport.request(
            '/marker_array', self.request, type(self.request), self.response_type, 500,
        )
        if not success or not response.data:
            self.get_logger().debug('Gazebo marker request failed or timed out')


def main():
    rclpy.init()
    node = None
    try:
        node = TagPoseVisualizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ImportError as exc:
        raise SystemExit(
            'Gazebo markers require python3-gz-transport13 and python3-gz-msgs10. '
            'Install them or set enable_gazebo_viz:=false.'
        ) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
