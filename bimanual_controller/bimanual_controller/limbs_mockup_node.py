#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from bimanual_msgs.msg import Tactile
from bimanual_msgs.action import MoveArmToPose, GraspUntilContact, ReleaseGrasp
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile
from threading import Lock
import numpy as np
import time
import yaml
import PyKDL
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from lxml import etree

class LimbState:
    def __init__(self, name, node, motion_params, grasp_params, robot_description):
        self.name = name  # 'left' or 'right'
        self.node = node
        self.motion_params = motion_params
        self.grasp_params = grasp_params
        self.robot_description = robot_description
        self.tactile_contact = [False, False, False, False]  # index, middle, pinky, thumb

        # Track joint positions
        self.joint_names = [f'{name}_joint_{i+1}' for i in range(7)] + \
                           [f'{name}_joint_{i}_0' for i in range(16)]
        self.joint_positions = np.zeros(len(self.joint_names))
        self.joint_lock = Lock()

        self.finger_release_speed = self.motion_params.get('finger_release_speed', 0.0)
        self.finger_grasp_speed = self.motion_params.get('finger_grasp_speed', 0.0)
        self.grasp_position = np.array(self.grasp_params.get(f"{self.name}_hand_grasp_position", np.zeros(16)))
        self.release_position = np.array(self.grasp_params.get(f"{self.name}_hand_release_position", np.zeros(16)))

        # Tactile subscription
        node.create_subscription(
            Tactile,
            f'/{name}/tactile',
            self.tactile_callback,
            10
        )

        # Action servers
        self.move_action = ActionServer(
            node, MoveArmToPose, f'/{name}/move_arm_to_pose', self.execute_move
        )
        self.grasp_action = ActionServer(
            node, GraspUntilContact, f'/{name}/grasp_until_contact', self.execute_grasp
        )
        self.release_action = ActionServer(
            node, ReleaseGrasp, f'/{name}/release_grasp', self.execute_release
        )
        
        self.move_to_home()

    # Simple clip to home function
    def move_to_home(self):
        home_arm_positions = self.motion_params.get('home_position', {}).get(f"{self.name}_arm", [])
        home_hand_positions = self.motion_params.get('home_position', {}).get(f"{self.name}_hand", [])

        if len(home_arm_positions) == 7 and len(home_hand_positions) == 16:
            with self.joint_lock:
                self.joint_positions[:7] = home_arm_positions
                self.joint_positions[7:23] = home_hand_positions
            self.node.get_logger().info(f'[{self.name}] Moved to home position: Arm: {home_arm_positions}, Hand:{home_hand_positions}')
        else:
            self.node.get_logger().warn(f'[{self.name}] Home position not defined or malformed.')

    def tactile_callback(self, msg: Tactile):
        # Expecting msg.name: [..., left_index_contact, ..., left_middle_contact, ...]
        for i in range(4):
            self.tactile_contact[i] = bool(msg.contacts[i])

    def remove_unsupported_tags(self, urdf_string):
        root = etree.fromstring(urdf_string.encode())
        for tag in root.xpath('//origin_xyz | //ros2_control'):
            tag.getparent().remove(tag)
        return etree.tostring(root).decode()
    
    def solve_ik(self, target_pose: PoseStamped):

        # 1. Load KDL tree and chain only once
        if not hasattr(self, 'ik_chain'):

            # Clean URDF
            cleaned_urdf = self.remove_unsupported_tags(self.robot_description)

            self.node.get_logger().warn(f'{cleaned_urdf}')

            robot = URDF.from_xml_string(cleaned_urdf)

            success, tree = treeFromUrdfModel(robot)
            if not success:
                self.node.get_logger().error("Failed to parse URDF into KDL tree.")
                return None

            base_link = self.motion_params.get('ik_base_link', 'world')
            tip_link = self.motion_params.get(f'{self.name}_ik_tip_link', f'{self.name}_palm')

            self.ik_chain = tree.getChain(base_link, tip_link)
            self.node.get_logger().info(f"KDL chain has {self.ik_chain.getNrOfJoints()} joints.")
            self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.ik_chain)

        # 2. Convert goal pose into PyKDL Frame
        pos = target_pose.pose.position
        ori = target_pose.pose.orientation
        kdl_goal = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(ori.x, ori.y, ori.z, ori.w),
            PyKDL.Vector(pos.x, pos.y, pos.z)
        )

        # 3. Current joint estimate
        with self.joint_lock:
            current = self.joint_positions[:7]
        kdl_seed = PyKDL.JntArray(len(current))
        for i in range(len(current)):
            kdl_seed[i] = current[i]

        # 4. IK solve
        kdl_result = PyKDL.JntArray(self.ik_chain.getNrOfJoints())
        ret = self.ik_solver.CartToJnt(kdl_seed, kdl_goal, kdl_result)

        if ret >= 0:
            return np.array([kdl_result[i] for i in range(self.ik_chain.getNrOfJoints())])
        else:
            return None

    def execute_move(self, goal_handle):
        self.node.get_logger().info(f'[{self.name}] Executing MoveArmToPose...')
        
        target_pose = goal_handle.request.target_pose
        duration = goal_handle.request.duration or self.motion_params.get("default_move_duration", 2.0)

        # Step 1: Solve IK
        target_joints = self.solve_ik(target_pose)
        if target_joints is None:
            self.node.get_logger().warn(f'[{self.name}] IK failed.')
            goal_handle.abort()
            return MoveArmToPose.Result(success=False)

        # Step 2: Get current joint positions (arm only)
        with self.joint_lock:
            current = self.joint_positions[:7].copy()

        update_rate = 50.0
        dt = 1.0 / update_rate
        # threshold = 0.01

        # Step 3: Interpolation setup
        steps = int(duration * update_rate)  # 50 Hz update rate
        for i in range(steps):
            alpha = (i + 1) / steps
            new_joint_pos = (1 - alpha) * current + alpha * target_joints

            with self.joint_lock:
                self.joint_positions[:7] = new_joint_pos

            # Step 4: Feedback
            feedback = MoveArmToPose.Feedback(progress=alpha)
            goal_handle.publish_feedback(feedback)

            time.sleep(dt)

        goal_handle.succeed()
        return MoveArmToPose.Result(success=True)

    def execute_grasp(self, goal_handle):
        self.node.get_logger().info(f'[{self.name}] Executing GraspUntilContact...')
        target_fingers = goal_handle.request.target_closure or [1.0] * 4

        # Compute the 16-joint target using grasp_position
        target = np.zeros(16)
        target[0:4] = target_fingers[0] * self.grasp_position[0:4]
        target[4:8] = target_fingers[1] * self.grasp_position[4:8]
        target[8:12] = target_fingers[2] * self.grasp_position[8:12]
        target[12:16] = target_fingers[3] * self.grasp_position[12:16]
        target = np.clip(target, 0.0, 1.0)

        # Tactile contact flags: [index, middle, pinky, thumb]
        contact = [False, False, False, False]

        update_rate = 50.0
        dt = 1.0 / update_rate
        threshold = 0.01

        while not all(contact):
            with self.joint_lock:
                current = self.joint_positions[7:23].copy()

            error = target - current
            delta = np.zeros(16)

            # Update each finger independently
            for i, contacted in enumerate(contact):
                if contacted:
                    continue  # freeze this finger

                finger_start = i * 4
                finger_end = finger_start + 4
                finger_error = error[finger_start:finger_end]

                # If already close enough, mark as contacted (even without tactile)
                if np.linalg.norm(finger_error) < threshold:
                    contact[i] = True
                    continue

                # If tactile contact exists for this finger
                if self.tactile_contact[i]:
                    contact[i] = True
                    continue

                # Otherwise, update movement for this finger
                delta[finger_start:finger_end] = np.clip(
                    finger_error,
                    -self.finger_grasp_speed * dt,
                    self.finger_grasp_speed * dt
                )

            new_pos = current + delta

            with self.joint_lock:
                self.joint_positions[7:23] = new_pos

            # Publish feedback
            feedback = GraspUntilContact.Feedback()
            feedback.contact = contact
            feedback.current_closure = self.joint_positions[7:23].tolist()
            goal_handle.publish_feedback(feedback)

            time.sleep(dt)

        goal_handle.succeed()
        return GraspUntilContact.Result(
            success=True,
            final_closure=self.joint_positions[7:23].tolist()
        )

    def execute_release(self, goal_handle):
        self.node.get_logger().info(f'[{self.name}] Executing ReleaseGrasp...')
        target_fingers = goal_handle.request.target_opening or [0.0] * 4

        target = np.zeros(16)
        target[0:4] = target_fingers[0] * self.release_position[0:4]
        target[4:8] = target_fingers[1] * self.release_position[4:8]
        target[8:12] = target_fingers[2] * self.release_position[8:12]
        target[12:16] = target_fingers[3] * self.release_position[12:16]

        target = np.clip(np.array(target), 0.0, 1.0)

        with self.joint_lock:
            current = self.joint_positions[7:23].copy()

        update_rate = 50.0
        dt = 1.0 / update_rate
        threshold = 0.01

        while True:
            with self.joint_lock:
                current = self.joint_positions[7:23].copy()

            error = target - current
            if np.linalg.norm(error) < threshold:
                break

            delta = np.clip(error, -self.finger_release_speed * dt, self.finger_release_speed * dt)
            new_pos = current + delta

            with self.joint_lock:
                self.joint_positions[7:23] = new_pos

            feedback = ReleaseGrasp.Feedback()
            feedback.current_closure = self.joint_positions[7:23].tolist()
            goal_handle.publish_feedback(feedback)

            time.sleep(dt)

        goal_handle.succeed()
        return ReleaseGrasp.Result(
            success=True,
            final_closure=self.joint_positions[7:23].tolist()
        )

    def get_joint_state_msg(self):
        with self.joint_lock:
            return JointState(
                header=Header(stamp=self.node.get_clock().now().to_msg()),
                name=self.joint_names,
                position=self.joint_positions.tolist()
            )

class LimbsMockupNode(Node):
    def __init__(self):
        super().__init__('limbs_mockup_node')
        self.declare_parameters(namespace='',
            parameters=[
                ('motion_params_file', ''),
                ('grasp_params_file', ''),
                ('robot_description', ''),
            ]
        )

        motion_path = self.get_parameter('motion_params_file').get_parameter_value().string_value
        grasp_path = self.get_parameter('grasp_params_file').get_parameter_value().string_value
        self.robot_description = self.get_parameter('robot_description').get_parameter_value().string_value

        self.motion_params = self.load_yaml(motion_path)
        self.grasp_params = self.load_yaml(grasp_path)

        # One joint publisher for both limbs
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Each limb handles its own actions
        self.left = LimbState('left', self, self.motion_params, self.grasp_params, self.robot_description)
        self.right = LimbState('right', self, self.motion_params, self.grasp_params, self.robot_description)

        # Timer to publish joint states
        self.create_timer(0.02, self.publish_joint_states)  # 50Hz

    def load_yaml(self, path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def publish_joint_states(self):
        left_msg = self.left.get_joint_state_msg()
        right_msg = self.right.get_joint_state_msg()

        # Combine both
        combined = JointState()
        combined.header.stamp = self.get_clock().now().to_msg()
        combined.name = left_msg.name + right_msg.name
        combined.position = left_msg.position + right_msg.position
        self.joint_pub.publish(combined)

def main(args=None):
    rclpy.init(args=args)
    node = LimbsMockupNode()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

