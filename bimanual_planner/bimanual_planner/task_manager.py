import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from bimanual_msgs.action import MoveArmToPose, GraspUntilContact, ReleaseGrasp
from bimanual_msgs.msg import GraspState
from rclpy.action import ActionClient
import yaml
import os
from enum import Enum, auto
from tf2_ros import Buffer, TransformListener
import tf2_ros
import tf_transformations

class RightArmState(Enum):
    WAIT = auto()
    MOVE_TO_STANDOFF = auto()
    MOVE_TO_GRASP = auto()
    GRASP = auto()
    MOVE_TO_HANDOFF_STANDOFF = auto()
    MOVE_TO_HANDOFF = auto()
    RELEASE = auto()
    WATCH = auto()

class LeftArmState(Enum):
    WAIT = auto()
    MOVE_TO_HANDOFF_STANDOFF = auto()
    MOVE_TO_HANDOFF = auto()
    GRASP = auto()
    MOVE_TO_BIN = auto()
    RELEASE = auto()

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # Load plan_params.yaml
        plan_path = self.declare_parameter('plan_params_file', '').get_parameter_value().string_value
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
        self.left_release_client = ActionClient(self, ReleaseGrasp, '/left/release_grasp')
        self.right_release_client = ActionClient(self, ReleaseGrasp, '/right/release_grasp')

        # Grasp state publisher
        self.grasp_pub = self.create_publisher(
            GraspState,
            '/grasp_state',
            10
        )

        self.timestep = 0.2

        # Start high-level task logic
        self.timer = self.create_timer(self.timestep, self.update_fsm)

        self.right_state = RightArmState.WAIT
        self.left_state = LeftArmState.WAIT

        self.grasp_state = GraspState()
        self.grasp_state.current_holder = "world"
        self.grasp_state.expected_holder = "world"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_futures = {}
        self.goal_handles = {}
        self.result_futures = {}

        self.object_pose = PoseStamped()

    def update_fsm(self):
        self.get_logger().debug(f"Right state: {self.right_state}, Left state: {self.left_state}") 
        self.get_logger().debug(f"current holder: {self.grasp_state.current_holder}, expected holder: {self.grasp_state.expected_holder}") 

        # Optionally log or act on this
        
        # === RIGHT ARM FSM ===
        if self.right_state == RightArmState.WAIT:
            
            self.object_pose = self.get_object_pose()
            if self.object_pose is None:
                return
            
            self.get_logger().debug(
                f"Object Pose: x={self.object_pose.pose.position.x:.2f}, y={self.object_pose.pose.position.y:.2f}, z={self.object_pose.pose.position.z:.2f}"
            )
            
            if not self.is_identity_pose(self.object_pose):
                self.right_state = RightArmState.MOVE_TO_STANDOFF
                self.grasp_state.expected_holder = "right"
                self.send_move_goal(self.right_move_client, self.get_relative_pose(self.object_pose, 'standoff_relative_pose'), 2.0)

        elif self.right_state == RightArmState.MOVE_TO_STANDOFF:
            if self.action_done(self.right_move_client):
                self.right_state = RightArmState.MOVE_TO_GRASP
                self.send_move_goal(self.right_move_client, self.get_relative_pose(self.object_pose, 'grasp_relative_pose'), 0.5)

        elif self.right_state == RightArmState.MOVE_TO_GRASP:
            if self.action_done(self.right_move_client):
                self.right_state = RightArmState.GRASP
                self.send_grasp_goal(self.right_grasp_client)

        elif self.right_state == RightArmState.GRASP:
            if self.action_done(self.right_grasp_client):
                self.grasp_state.current_holder = "right"
                self.right_state = RightArmState.MOVE_TO_HANDOFF_STANDOFF
                self.send_move_goal(self.right_move_client, self.get_named_pose('handoff_standoff_pose_right'), 2.0)

        elif self.right_state == RightArmState.MOVE_TO_HANDOFF_STANDOFF:
            if self.action_done(self.right_move_client):
                self.right_state = RightArmState.MOVE_TO_HANDOFF
                self.send_move_goal(self.right_move_client, self.get_named_pose('handoff_pose_right'), 0.5)

        elif self.right_state == RightArmState.MOVE_TO_HANDOFF:
            if self.action_done(self.right_move_client) and self.left_state == LeftArmState.GRASP and self.action_done(self.left_grasp_client):
                self.right_state = RightArmState.RELEASE
                self.send_release_goal(self.right_release_client)

        elif self.right_state == RightArmState.RELEASE:
            if self.action_done(self.right_release_client):
                self.grasp_state.current_holder = "left"
                self.right_state = RightArmState.WATCH
                self.grasp_state.expected_holder = "world"
                self.send_move_goal(self.right_move_client, self.get_named_pose('watch_pose_right'), 1.0)

        elif self.right_state == RightArmState.WATCH:
            if self.action_done(self.right_release_client) and self.left_state == LeftArmState.WAIT:
                self.right_state = RightArmState.WAIT
                self.object_pose = PoseStamped()

        # === LEFT ARM FSM ===
        if self.left_state == LeftArmState.WAIT:
            if self.grasp_state.current_holder == "right":
                self.left_state = LeftArmState.MOVE_TO_HANDOFF_STANDOFF
                self.grasp_state.expected_holder = "left"
                self.send_move_goal(self.left_move_client, self.get_named_pose('handoff_standoff_pose_left'), 2.0)

        if self.left_state == LeftArmState.MOVE_TO_HANDOFF_STANDOFF:
            if self.grasp_state.current_holder == "right" and self.action_done(self.left_move_client):
                self.left_state = LeftArmState.MOVE_TO_HANDOFF
                self.grasp_state.expected_holder = "left"
                self.send_move_goal(self.left_move_client, self.get_named_pose('handoff_pose_left'), 0.8)

        elif self.left_state == LeftArmState.MOVE_TO_HANDOFF:
            if self.action_done(self.left_move_client) and self.right_state == RightArmState.MOVE_TO_HANDOFF and self.action_done(self.right_move_client):
                self.left_state = LeftArmState.GRASP
                self.send_grasp_goal(self.left_grasp_client)

        elif self.left_state == LeftArmState.GRASP:
            if self.action_done(self.left_grasp_client) and self.right_state == RightArmState.WATCH:
                self.grasp_state.current_holder = "left"
                self.left_state = LeftArmState.MOVE_TO_BIN
                self.send_move_goal(self.left_move_client, self.get_named_pose('bin_pose'), 3.0)

        elif self.left_state == LeftArmState.MOVE_TO_BIN:
            if self.action_done(self.left_move_client):
                self.left_state = LeftArmState.RELEASE
                self.send_release_goal(self.left_release_client)

        elif self.left_state == LeftArmState.RELEASE:
            if self.action_done(self.left_release_client):
                self.grasp_state.current_holder = "world"
                self.left_state = LeftArmState.WAIT
                self.grasp_state.expected_holder = "world"

        self.publish_grasp_state()

    def is_identity_pose(self, pose):
        return pose.pose.position.x == 0.0 and pose.pose.position.y == 0.0 and pose.pose.position.z == 0.0

    def get_relative_pose(self, reference_pose, relative_key):
        rel = self.plan_params[relative_key]

        # Reference transform
        ref_pos = reference_pose.pose.position
        ref_ori = reference_pose.pose.orientation
        T_ref = tf_transformations.quaternion_matrix([ref_ori.x, ref_ori.y, ref_ori.z, ref_ori.w])
        T_ref[0, 3] = ref_pos.x
        T_ref[1, 3] = ref_pos.y
        T_ref[2, 3] = ref_pos.z

        # Relative transform
        rel_pos = [rel['x'], rel['y'], rel['z']]
        rel_quat = [rel.get('qx', 0.0), rel.get('qy', 0.0), rel.get('qz', 0.0), rel.get('qw', 1.0)]
        T_rel = tf_transformations.quaternion_matrix(rel_quat)
        T_rel[0, 3] = rel_pos[0]
        T_rel[1, 3] = rel_pos[1]
        T_rel[2, 3] = rel_pos[2]

        # Combined transform
        T_combined = T_ref @ T_rel
        combined_pos = T_combined[:3, 3]
        combined_quat = tf_transformations.quaternion_from_matrix(T_combined)

        # Output PoseStamped
        out = PoseStamped()
        out.header.frame_id = reference_pose.header.frame_id
        out.pose.position.x = combined_pos[0]
        out.pose.position.y = combined_pos[1]
        out.pose.position.z = combined_pos[2]
        out.pose.orientation.x = combined_quat[0]
        out.pose.orientation.y = combined_quat[1]
        out.pose.orientation.z = combined_quat[2]
        out.pose.orientation.w = combined_quat[3]
        return out

    def get_named_pose(self, key):
        val = self.plan_params[key]
        out = PoseStamped()
        out.header.frame_id = "world"
        out.pose.position.x = val['x']
        out.pose.position.y = val['y']
        out.pose.position.z = val['z']
        out.pose.orientation.x = val['qx']
        out.pose.orientation.y = val['qy']
        out.pose.orientation.z = val['qz']
        out.pose.orientation.w = val['qw']
        return out

    def send_move_goal(self, client, pose, duration):
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"{client._action_name} not available")
            return

        goal = MoveArmToPose.Goal()
        goal.target_pose = pose
        goal.duration = duration

        send_goal_future = client.send_goal_async(goal)

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"{client._action_name} goal rejected")
                return
            self.result_futures[client] = goal_handle.get_result_async()

        send_goal_future.add_done_callback(goal_response_callback)

    def send_grasp_goal(self, client):
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"{client._action_name} not available")
            return

        goal = GraspUntilContact.Goal()

        send_goal_future = client.send_goal_async(goal)

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"{client._action_name} goal rejected")
                return
            self.result_futures[client] = goal_handle.get_result_async()

        send_goal_future.add_done_callback(goal_response_callback)

    def send_release_goal(self, client):
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"{client._action_name} not available")
            return

        goal = ReleaseGrasp.Goal()

        send_goal_future = client.send_goal_async(goal)

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"{client._action_name} goal rejected")
                return
            self.result_futures[client] = goal_handle.get_result_async()

        send_goal_future.add_done_callback(goal_response_callback)

    def action_done(self, client):
        if client not in self.result_futures:
            return False

        result_future = self.result_futures[client]
        if not result_future.done():
            return False

        result = result_future.result()
        return result.status == 4  # STATUS_SUCCEEDED
    
    def get_object_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "world", "object_sensed", rclpy.time.Time())
            
            pose = PoseStamped()
            pose.header = tf.header
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            return pose
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"[TF] Could not get transform: {ex}")
            return None

    def publish_grasp_state(self):
        msg = GraspState()
        msg.current_holder = self.grasp_state.current_holder
        msg.expected_holder = self.grasp_state.expected_holder
        self.grasp_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
