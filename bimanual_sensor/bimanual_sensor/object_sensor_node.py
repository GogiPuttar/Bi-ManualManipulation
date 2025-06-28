import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import tf_transformations
import cv2
from cv_bridge import CvBridge

class ObjectSensor(Node):
    def __init__(self):
        super().__init__('object_sensor')

        self.declare_parameter('cv_params_file', '')

        self.grasp_state = "none"
        self.bridge = CvBridge()
        self.br = TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(String, '/grasp_state', self.grasp_cb, 10)
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
        self.timer = self.create_timer(0.2, self.process)

    def grasp_cb(self, msg):
        self.grasp_state = msg.data

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

        # Decision based on grasp state
        if self.grasp_state == "none":
            # TODO: Run CV to detect object and compute transform
            # If found:
            tf_msg.header.frame_id = "world"
            tf_msg.transform.translation.x = 0.3
            tf_msg.transform.translation.y = 0.2
            tf_msg.transform.translation.z = 0.1
            tf_msg.transform.rotation.w = 1.0
        elif self.grasp_state in ("left", "right"):
            tf_msg.header.frame_id = f"{self.grasp_state}_palm"
            tf_msg.transform.translation.x = 0.0
            tf_msg.transform.translation.y = 0.0
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.w = 1.0
        else:
            tf_msg.header.frame_id = "world"
            tf_msg.transform.translation.x = 0.0
            tf_msg.transform.translation.y = 0.0
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.w = 1.0

        self.br.sendTransform(tf_msg)

        # Also publish marker
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
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSensor()
    rclpy.spin(node)
    rclpy.shutdown()
