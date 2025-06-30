import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from bimanual_msgs.action import MoveArmToPose, GraspUntilContact, ReleaseGrasp
from bimanual_msgs.msg import GraspState
from rclpy.action import ActionClient
import tf2_ros
import tf_transformations
from tf2_ros import Buffer, TransformListener
import yaml
import os

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # Load plan_params.yaml
        plan_path = self.declare_parameter('plan_params', '').get_parameter_value().string_value
        self.get_logger().info(f"Loading plan params from: {plan_path}")
        with open(plan_path, 'r') as f:
            self.plan_params = yaml.safe_load(f)

        # TF Buffer & Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action clients
        self.left_move_client = ActionClient(self, MoveArmToPose, '/left/move_arm_to_pose')
        self.right_move_client = ActionClient(self, MoveArmToPose, '/right/move_arm_to_pose')
        self.left_grasp_client = ActionClient(self, GraspUntilContact, '/left/grasp_until_contact')
        self.right_grasp_client = ActionClient(self, GraspUntilContact, '/right/grasp_until_contact')
        self.left_release_client = ActionClient(self, ReleaseGrasp, '/left/release')
        self.right_release_client = ActionClient(self, ReleaseGrasp, '/right/release')

        # Grasp state publisher
        self.grasp_pub = self.create_publisher(
            GraspState,
            '/grasp_state',
            10
        )

        # Start high-level task logic
        self.timer = self.create_timer(0.1, self.run_task_logic)

    def run_task_logic(self):
        # TODO: Implement object-sensed based task sequencing
        self.get_logger().info_once("Task manager node is running...")

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
