"""Relay Gazebo model positions for the flight recorder."""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


class SimGroundTruth(Node):
    def __init__(self):
        super().__init__('sim_ground_truth')
        self.declare_parameter('world', 'aruco_moving')
        self.declare_parameter('vehicle_model', 'x500_mono_cam_down_0')
        world = self.get_parameter('world').value
        marker_model, self.marker_height = {
            'aruco': ('arucotag', 0.0),
            'aruco_moving': ('aruco_moving_target', 0.103),
        }[world]
        self.marker_model = marker_model
        self.vehicle_model = self.get_parameter('vehicle_model').value
        self.marker_pub = self.create_publisher(
            PoseStamped, '/pl/sim/marker_pose', qos_profile_sensor_data)
        self.vehicle_pub = self.create_publisher(
            PoseStamped, '/pl/sim/vehicle_pose', qos_profile_sensor_data)
        self.last_stamp = None
        self.create_subscription(TFMessage, '/pl/sim/model_poses', self.on_poses,
                                 qos_profile_sensor_data)

    def on_poses(self, msg):
        stamp = self.get_clock().now()
        if self.last_stamp is not None and 0 <= (stamp - self.last_stamp).nanoseconds < 50_000_000:
            return
        models = {tf.child_frame_id: tf.transform for tf in msg.transforms}
        marker = models.get(self.marker_model)
        vehicle = models.get(self.vehicle_model)
        if marker is None or vehicle is None:
            return
        self.last_stamp = stamp
        self.publish_pose(self.marker_pub, marker, stamp, self.marker_height)
        self.publish_pose(self.vehicle_pub, vehicle, stamp, 0.0)

    @staticmethod
    def publish_pose(publisher, transform, stamp, height):
        msg = PoseStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'gazebo_enu'
        msg.pose.orientation = q = transform.rotation
        p = transform.translation
        # The moving model's tag is on top of the pad, along its local +Z.
        msg.pose.position.x = p.x + height * 2 * (q.x * q.z + q.w * q.y)
        msg.pose.position.y = p.y + height * 2 * (q.y * q.z - q.w * q.x)
        msg.pose.position.z = p.z + height * (1 - 2 * (q.x * q.x + q.y * q.y))
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimGroundTruth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
