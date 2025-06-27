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

        self.declare_parameter('model_path', 'team-sobits/sobit_light_smolvla')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # Declare parameters for gamepad configuration
        self.declare_parameter('gamepad.topic', '/joy')
        self.gamepad_topic = self.get_parameter('gamepad.topic').get_parameter_value().string_value
        self.declare_parameter('gamepad.play_button', 0)  # Assuming the play button is at index 0
        self.play_button = self.get_parameter('gamepad.play_button').get_parameter_value().integer_value

        # Load policy
        self.policy = SmolVLAPolicy.from_pretrained(
            model_path,
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
        
        



    def obs_joint_callback(self, msg: JointState):
        self.get_logger().info(f"Received joint states: {msg.name}")

    def obs_odometry_callback(self, msg: Odometry):
        self.get_logger().info(f"Received odometry data: {msg.twist.twist.linear.x}, {msg.twist.twist.angular.z}")

    def obs_cam_callback(self, msg: Image, camera_type: str):
        self.get_logger().info(f"Received image from {camera_type} camera")
        self.cam_vector[camera_type] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def gamepad_callback(self, msg: Joy):
        if msg.buttons[self.play_button] == 1:
            self.play_button_pressed = True
