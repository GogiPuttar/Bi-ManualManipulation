import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import tf_transformations
import cv2
from cv_bridge import CvBridge
from bimanual_msgs.msg import GraspState

class ObjectSensor(Node):
    def __init__(self):
        super().__init__('object_sensor')

        self.declare_parameter('cv_params_file', '')

        self.timestep = 0.2 # seconds

        self.grasp_state = GraspState()
        self.grasp_state.current_holder = "world"
        self.grasp_state.expected_holder = "world"

        self.last_world_tf = None
        self.locked_palm_tf = None
        self.locked_palm_parent = None

        self.bridge = CvBridge()
        self.br = TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(GraspState, '/grasp_state', self.grasp_cb, 10)
        self.create_subscription(Image, '/left/camera/color/image_raw', self.left_color_cb, 10)
        self.create_subscription(Image, '/right/camera/color/image_raw', self.right_color_cb, 10)
        self.create_subscription(Image, '/left/camera/depth/image_raw', self.left_depth_cb, 10)
        self.create_subscription(Image, '/right/camera/depth/image_raw', self.right_depth_cb, 10)

        # Publisher
        self.marker_pub = self.create_publisher(Marker, '/object_sensed_marker', 10)

        # Internal image buffers
        self.latest_left_color = None
        self.latest_right_color = None
        self.latest_left_depth = None
        self.latest_right_depth = None

        # Timer to process and broadcast
        self.timer = self.create_timer(self.timestep, self.process)

    def grasp_cb(self, msg):
        self.grasp_state = msg

    def left_color_cb(self, msg):
        self.latest_left_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def right_color_cb(self, msg):
        self.latest_right_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def left_depth_cb(self, msg):
        self.latest_left_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def right_depth_cb(self, msg):
        self.latest_right_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def process(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.child_frame_id = "object_sensed"

        ch = self.grasp_state.current_holder
        eh = self.grasp_state.expected_holder

        if ch == "world" and eh == "world":
            # Object is unknown – run CV or publish dummy value
            tf_msg.header.frame_id = "world"
            tf_msg.transform.translation.x = 0.0  # dummy
            tf_msg.transform.translation.y = 0.0
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.w = 1.0

            self.last_world_tf = tf_msg  # Save for future

        elif ch == "world" and eh.startswith(("left", "right")):
            # Reaching toward object – use last known
            if self.last_world_tf:
                tf_msg = self.last_world_tf
            else:
                return  # Nothing to publish yet

        elif ch.startswith(("left", "right")) and eh.startswith(("left", "right")):
            # Grasp handoff in progress – lock palm-relative TF once
            if self.locked_palm_tf is None or self.locked_palm_parent != ch:
                self.locked_palm_tf = TransformStamped()
                self.locked_palm_tf.header.stamp = self.get_clock().now().to_msg()
                self.locked_palm_tf.header.frame_id = ch
                self.locked_palm_tf.child_frame_id = "object_sensed"
                self.locked_palm_tf.transform.translation.x = 0.0
                self.locked_palm_tf.transform.translation.y = 0.0
                self.locked_palm_tf.transform.translation.z = 0.0
                self.locked_palm_tf.transform.rotation.w = 1.0
                self.locked_palm_parent = ch

            tf_msg = self.locked_palm_tf

        elif ch.startswith(("left", "right")) and eh == "bin":
            # Object held in final hand – use previously locked TF
            if self.locked_palm_tf:
                tf_msg = self.locked_palm_tf
            else:
                return

        else:
            # Unknown state – publish safe default
            tf_msg.header.frame_id = "world"
            tf_msg.transform.translation.x = 0.0
            tf_msg.transform.translation.y = 0.0
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.w = 1.0

        self.br.sendTransform(tf_msg)

        # Marker logic
        marker = Marker()
        marker.header = tf_msg.header
        marker.ns = "object_sensed"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = tf_msg.transform.translation.x
        marker.pose.position.y = tf_msg.transform.translation.y
        marker.pose.position.z = tf_msg.transform.translation.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.r = 0.7
        marker.color.g = 1.0
        marker.color.b = 0.7
        marker.color.a = 1.0

        if not marker.pose.position == Point():
            self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSensor()
    rclpy.spin(node)
    rclpy.shutdown()
