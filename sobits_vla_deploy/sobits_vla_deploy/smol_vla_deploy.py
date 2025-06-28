from lerobot.common.datasets.utils import build_dataset_frame, hw_to_dataset_features
from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.common.utils.control_utils import predict_action
from lerobot.common.utils.utils import get_safe_torch_device

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from cv_bridge import CvBridge 

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy, JointState, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SmolVLADeploy(Node):
    def __init__(self):
        super().__init__('smolvla_deploy')

        # Declare parameters for robot configuration
        self.declare_parameter('robot.name', 'sobit_light')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.declare_parameter('robot.parts', ['body', 'mobile_base'])
        self.robot_parts = self.get_parameter('robot.parts').get_parameter_value().string_array_value
        self.robot_joints = {}
        for part in self.robot_parts:
            self.declare_parameter(f'robot.{part}.joints', [])
            self.declare_parameter(f'robot.{part}.joints_ros', [])
            self.declare_parameter(f'robot.{part}.obs_topic', '') 
            self.declare_parameter(f'robot.{part}.act_topic', '')

            self.robot_joints[part] = {
                'joints': self.get_parameter(f'robot.{part}.joints').get_parameter_value().string_array_value,
                'joints_ros': self.get_parameter(f'robot.{part}.joints_ros').get_parameter_value().string_array_value,
                'obs_topic': self.get_parameter(f'robot.{part}.obs_topic').get_parameter_value().string_value,
                'act_topic': self.get_parameter(f'robot.{part}.act_topic').get_parameter_value().string_value
            }

        self.declare_parameter('robot.cameras.types', ['front', 'back', 'head', 'wrist.top'])
        self.camera_types = self.get_parameter('robot.cameras.types').get_parameter_value().string_array_value
        self.cameras = {}
        for camera_type in self.camera_types:
            self.declare_parameter(f'robot.cameras.{camera_type}.width', 1280)
            self.declare_parameter(f'robot.cameras.{camera_type}.height', 720)
            self.declare_parameter(f'robot.cameras.{camera_type}.fps', 10)
            self.declare_parameter(f'robot.cameras.{camera_type}.color_mode', 'rgb')
            self.declare_parameter(f'robot.cameras.{camera_type}.topic', f'/sobit_light/{camera_type}/camera/image_raw')

            self.cameras[camera_type] = {
                'width': self.get_parameter(f'robot.cameras.{camera_type}.width').get_parameter_value().integer_value,
                'height': self.get_parameter(f'robot.cameras.{camera_type}.height').get_parameter_value().integer_value,
                'fps': self.get_parameter(f'robot.cameras.{camera_type}.fps').get_parameter_value().integer_value,
                'color_mode': self.get_parameter(f'robot.cameras.{camera_type}.color_mode').get_parameter_value().string_value,
                'topic': self.get_parameter(f'robot.cameras.{camera_type}.topic').get_parameter_value().string_value
            }

        # Declare parameters for gamepad configuration
        self.declare_parameter('gamepad.topic', '/joy')
        self.gamepad_topic = self.get_parameter('gamepad.topic').get_parameter_value().string_value
        self.declare_parameter('gamepad.play_button', 0)  # Assuming the play button is at index 0
        self.play_button = self.get_parameter('gamepad.play_button').get_parameter_value().integer_value

        # Declare parameters for model configuration
        self.declare_parameter('model.repo_id', 'team-sobits/sobit_light_smolvla')
        model_id = self.get_parameter('repo_id').get_parameter_value().string_value
        self.declare_parameter('model.device', 'cuda')
        model_device = self.get_parameter('model.device').get_parameter_value().string_value
        self.declare_parameter('model.use_amp', True)
        model_use_amp = self.get_parameter('model.use_amp').get_parameter_value().bool_value

        # Load policy
        self.policy = SmolVLAPolicy.from_pretrained(
            pretrained_name_or_path = model_id,
            config= {
                'device': model_device,
                'use_amp': model_use_amp
            }
        )
        self.policy.reset()

        # Declare values
        self.play_button_pressed = False

        self.state_ft = {part_joint: float for part in self.robot_parts for part_joint in self.robot_joints[part]['joints']}
        self.cam_ft = {camera_type: (self.cameras[camera_type]['height'], self.cameras[camera_type]['width'], 3) for camera_type in self.camera_types}
        self.obs_ft = hw_to_dataset_features({**self.state_ft, **self.cam_ft}, "observation")
        self.state_vector = {part_joint: None for part in self.robot_parts for part_joint in self.robot_joints[part]['joints']}
        self.cam_vector = {camera_type: None for camera_type in self.camera_types}
        self.obs = None
        self.bridge = CvBridge()

        self.body_joint_key2ros = {joint: ros_joint for joint, ros_joint in zip(self.robot_joints['body']['joints'], self.robot_joints['body']['joints_ros'])}

        self.joint_states = None  # Placeholder for joint states
        self.odom_states = None  # Placeholder for odometry data

        # Set QoS profile
        qos_profile = QoSProfile(
            # reliability=ReliabilityPolicy.BEST_EFFORT,
            # history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscription for joystick input
        self.gamepad_sub = self.create_subscription(
            Joy,
            self.gamepad_topic,
            self.gamepad_callback,
            qos_profile
        )

        # TODO: Create subscription for joint states (generic for all robot parts)
        self.create_subscription(
            JointState,
            self.robot_joints['body']['obs_topic'],
            self.obs_joint_callback,
            qos_profile
        )

        # TODO: Create subscription for odometry (generic for all robot parts)
        self.create_subscription(
            Odometry,
            self.robot_joints['mobile_base']['obs_topic'],
            self.obs_odometry_callback,
            qos_profile
        )

        # Create subscriptions for each camera type
        for camera_type, camera_info in self.cameras.items():
            # Create subscription to camera images
            self.create_subscription(
                Image,
                camera_info['topic'],
                lambda msg, ct=camera_type: self.obs_cam_callback(msg, ct),
                qos_profile
            )

        # Create publisher for joint trajectory
        self.act_joint_pub = self.create_publisher(
            JointTrajectory,
            self.robot_joints['body']['act_topic'],
            qos_profile
        )

        # Create publisher for mobile base actions
        self.act_mobile_base_pub = self.create_publisher(
            Twist,
            self.robot_joints['mobile_base']['act_topic'],
            qos_profile
        )

        # Create timer to apply actions when play button is pressed
        self.timer = self.create_timer(0.1, self.apply_action_callback)

    def apply_action_callback(self):
        if not self.play_button_pressed:
            return
        if self.joint_states is None:
            self.get_logger().warn("Joint states not received yet")
            return
        if any(image is None for image in self.cam_vector.values()):
            self.get_logger().warn("Not all camera images received yet")
            return
        
        # Get observation
        self.obs = {**self.state_vector, **self.cam_vector}
        self.get_logger().debug(f"Current observation: {self.obs}")

        self.obs_frame = build_dataset_frame(self.obs, self.obs_ft, prefix="observation")
        self.get_logger().debug(f"Observation frame: {self.obs_frame}")

        # Predict action using the policy
        self.act_values = predict_action(
                self.obs_frame, self.policy, get_safe_torch_device(self.policy.config.device), self.policy.config.use_amp
            )
        self.get_logger().debug(f"Predicted action values: {self.act_values}")

        # Create joint trajectory message
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = self.robot_joints['body']['joints_ros']
        joint_trajectory_point = JointTrajectoryPoint()
        
        # Joint key to ROS joint name mapping
        joint_key_values = {
            ros_joint: self.act_values[joint_key]
            for joint_key, ros_joint in self.body_joint_key2ros.items()
        }

        joint_trajectory_point.positions = [joint_key_values[ros_joint] for ros_joint in joint_trajectory.joint_names]
        joint_trajectory_point.time_from_start.sec = 0
        joint_trajectory_point.time_from_start.nanosec = 100000000  # 100ms
        joint_trajectory.points.append(joint_trajectory_point)

        # Publish joint trajectory
        self.act_joint_pub.publish(joint_trajectory)
        self.get_logger().debug(f"Published joint trajectory: {joint_trajectory}")

        # Create and publish mobile base action
        mobile_base_action = Twist()
        mobile_base_action.linear.x = self.act_values['x.vel']
        mobile_base_action.angular.z = self.act_values['theta.vel']
        self.act_mobile_base_pub.publish(mobile_base_action)
        self.get_logger().debug(f"Published mobile base action: linear.x={mobile_base_action.linear.x}, angular.z={mobile_base_action.angular.z}")


    def obs_joint_callback(self, msg: JointState):
        self.get_logger().info(f"Received joint states: {msg.name}")
        self.joint_states = msg

        for joint_name, joint_value in zip(msg.name, msg.position):
            if joint_name in self.body_joint_key2ros:
                ros_joint_name = self.body_joint_key2ros[joint_name]
                self.state_vector[ros_joint_name] = joint_value
                self.get_logger().info(f"Joint {ros_joint_name} value: {joint_value}")


    def obs_odometry_callback(self, msg: Odometry):
        self.get_logger().info(f"Received odometry data: {msg.twist.twist.linear.x}, {msg.twist.twist.angular.z}")
        self.odom_states = msg.twist.twist

        self.state_vector['x.vel'] = msg.twist.twist.linear.x
        self.state_vector['theta.vel'] = msg.twist.twist.angular.z

    def obs_cam_callback(self, msg: Image, camera_type: str):
        self.get_logger().info(f"Received image from {camera_type} camera")
        self.cam_vector[camera_type] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def gamepad_callback(self, msg: Joy):
        if msg.buttons[self.play_button] == 1:
            self.play_button_pressed = True
