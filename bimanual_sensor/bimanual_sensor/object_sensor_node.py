import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import tf_transformations
import cv2
from cv_bridge import CvBridge
from bimanual_msgs.msg import GraspState
from tf2_ros import Buffer, TransformListener
import yaml
import numpy as np
import os

class ObjectSensor(Node):
    def __init__(self):
        super().__init__('object_sensor')

        self.timestep = 0.2  # seconds

        self.declare_parameter('cv_params_file', '')
        cv_path = self.get_parameter('cv_params_file').get_parameter_value().string_value

        if not os.path.exists(cv_path):
            self.get_logger().error(f"cv_params.yaml not found at: {cv_path}")
            rclpy.shutdown()
            return

        self.bridge = CvBridge()
        self.br = TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(GraspState, '/grasp_state', self.grasp_cb, 10)
        self.create_subscription(Image, '/right/camera/color/image_raw', self.right_color_cb, 10)
        self.create_subscription(Image, '/right/camera/depth/image_raw', self.right_depth_cb, 10)

        # Publisher
        self.marker_pub = self.create_publisher(Marker, '/object_sensed_marker', 10)

        # Internal image buffers
        self.latest_color = None
        self.latest_depth = None

        # Timer
        self.timer = self.create_timer(self.timestep, self.process)

        # Load intrinsics
        with open(cv_path, 'r') as f:
            cv_params = yaml.safe_load(f)

        self.fx = cv_params['fx']
        self.fy = cv_params['fy']
        self.cx = cv_params['cx']
        self.cy = cv_params['cy']
        self.cam_frame = cv_params['camera_frame']
        self.hue_min = cv_params['hue_min']
        self.hue_max = cv_params['hue_max']
        self.saturation_min = cv_params['saturation_min']
        self.value_min = cv_params['value_min']

        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.grasp_state = GraspState()

    def grasp_cb(self, msg):
        self.grasp_state = msg

    def right_color_cb(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def right_depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def process(self):

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "object_sensed"
        tf_msg.transform.rotation.w = 1.0

        if self.latest_color is None or self.latest_depth is None:
            self.get_logger().warn("Waiting for camera data...")
            return

        hsv = cv2.cvtColor(self.latest_color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           (self.hue_min, self.saturation_min, self.value_min),
                           (self.hue_max, 255, 255))

        M = cv2.moments(mask)
        if M['m00'] == 0:
            self.get_logger().info("No object detected. broadcasting identity transform")
            self.br.sendTransform(tf_msg)
            return

        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])
        z = self.latest_depth[v, u]
        if z == 0.0 or np.isnan(z):
            self.get_logger().warn("Invalid depth at centroid.")
            return

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        try:
            tf_cam_world = self.tf_buffer.lookup_transform("world", self.cam_frame, rclpy.time.Time())
            trans = tf_cam_world.transform.translation
            rot = tf_cam_world.transform.rotation
            T = tf_transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
            T[0, 3] = trans.x
            T[1, 3] = trans.y
            T[2, 3] = trans.z
            cam_coords = np.array([x, y, z, 1.0])
            world_coords = T @ cam_coords

            tf_msg.transform.translation.x = world_coords[0]
            tf_msg.transform.translation.y = world_coords[1]
            tf_msg.transform.translation.z = world_coords[2]

            self.br.sendTransform(tf_msg)
            self.publish_marker()

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {str(e)}")

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "object_sensed"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_sensed"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
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
