#!/usr/bin/env python3
"""
Multi-camera ros2bag to AV1 MP4 conversion for LeRobot dataset structure.
Updated to operate as a ROS 2 Node parsing launch properties.
"""
import os
import cv2
import yaml
import numpy as np
import torch
import torch.nn.functional as F
from pathlib import Path

import rclpy
from rclpy.node import Node

from rosbags.highlevel import AnyReader
from rosbags.image import message_to_cvimage
from lerobot.datasets.lerobot_dataset import LeRobotDataset

def extract_episode_data(bag_folder, camera_topics, primary_camera, joint_states_topic, cmd_vel_topic, sync_thres, has_mobile_base, has_cmd_vel_y, action_features, logger):
    """
    Iterate over the bag and return a list of frames.
    """
    frames = []
    latest_images = {}
    latest_joint_state = None
    latest_joint_time = 0.0
    
    if has_cmd_vel_y:
        latest_cmd_vel = (0.0, 0.0, 0.0)
    else:
        latest_cmd_vel = (0.0, 0.0)
    
    topic_to_cam = {v: k for k, v in camera_topics.items()}
    primary_topic = camera_topics.get(primary_camera)
    
    with AnyReader([Path(bag_folder)]) as reader:
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            
            if topic == joint_states_topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                joint_pos = dict(zip(msg.name, msg.position))
                t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                latest_joint_state = [
                    joint_pos.get(feat, 0.0) for feat in action_features
                ]
                latest_joint_time = t_sec
                
            if topic == cmd_vel_topic and has_mobile_base:
                msg = reader.deserialize(rawdata, connection.msgtype)
                t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 if hasattr(msg, 'header') else t_bag
                
                if has_cmd_vel_y:
                    latest_cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
                else:
                    latest_cmd_vel = [msg.linear.x, msg.angular.z]
                    
                latest_cmd_vel_time = t_sec
                
            elif topic in topic_to_cam:
                cam_name = topic_to_cam[topic]
                msg = reader.deserialize(rawdata, connection.msgtype)
                img = message_to_cvimage(msg)
                
                # Check format and convert to correct RGB if necessary
                if len(img.shape) == 3 and img.shape[2] == 3:
                     # Usually message_to_cvimage returns BGR natively from rosbags
                     if msg.encoding in ['bgr8', 'bgra8']:
                         img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                images[cam_name] = img
                image_times[cam_name] = t_sec
                
                # Snapshot on primary camera
                if cam_name == primary_camera and None not in images.values() and latest_joint_state is not None:
                # Proceed only if cmd_vel is satisfied (either not needed, or available)
                    if not has_mobile_base or latest_cmd_vel is not None:
                        # Sync checks
                        img_times = list(image_times.values())
                        max_camera_diff = max(img_times) - min(img_times)
                        
                        # For joint state sync, compare against the primary camera time
                        joint_diff = abs(image_times[primary_camera] - latest_joint_time)
                        
                        cmd_vel_diff = 0.0
                        if has_mobile_base:
                            cmd_vel_diff = abs(image_times[primary_camera] - latest_cmd_vel_time)
                            
                        if max_camera_diff > sync_thres or joint_diff > sync_thres or cmd_vel_diff > sync_thres:
                            logger.warn(f"Sync threshold exceeded at {t_bag:.2f}s: max_cam_diff={max_camera_diff:.3f}s, joint_diff={joint_diff:.3f}s, cmd_vel_diff={cmd_vel_diff:.3f}s")
                            
                        action = latest_joint_state + (latest_cmd_vel if has_mobile_base else [])
                        state = latest_joint_state + (latest_cmd_vel if has_mobile_base else [])
                        
                        frame = {
                            "action": torch.tensor(action, dtype=torch.float32),
                            "observation.state": torch.tensor(state, dtype=torch.float32), 
                        }
                        
                        # Ensure all requested cameras are present
                        missing_camera = False
                        for c_name in camera_topics.keys():
                            if images[c_name] is None: # This check should be redundant due to `None not in images.values()`
                                missing_camera = True
                                break
                            img_t = torch.from_numpy(images[c_name]).permute(2, 0, 1).contiguous()
                            frame[c_name] = img_t
                            
                        if not missing_camera:
                            frames.append((t_sec, frame))
                            # Reset images to None after snapshot to wait for new set
                            images = {cam_name: None for cam_name in camera_topics.keys()}

    return frames

class RosbagConversionNode(Node):
    def __init__(self):
        super().__init__('rosbag_conversion_node')
        
        self.declare_parameter('rosbag_directory', '')
        self.declare_parameter('recorded_bags_meta_file', '')
        self.declare_parameter('dataset_name', 'MyDataset')
        
        # Extract parameters
        self.rosbag_directory = self.get_parameter('rosbag_directory').get_parameter_value().string_value
        self.recorded_bags_meta_file = self.get_parameter('recorded_bags_meta_file').get_parameter_value().string_value
        self.dataset_name = self.get_parameter('dataset_name').get_parameter_value().string_value
        
    def convert(self):
        self.get_logger().info("Starting dataset conversion...")
        self.get_logger().info(f"Target dataset name: {self.dataset_name}")
        self.get_logger().info(f"Reading from rosbags root: {self.rosbag_directory}")
        
        if not os.path.exists(self.recorded_bags_meta_file):
            self.get_logger().error(f"Meta file not found: {self.recorded_bags_meta_file}")
            return
            
        with open(self.recorded_bags_meta_file, "r") as f:
            meta = yaml.safe_load(f)
            
        task_list = list(meta.get("recorded_bags", {}).items())
        
        convert_info = meta.get("convert_info", {})
        fps = convert_info.get("fps", 10)
        sync_threshold = convert_info.get("sync_threshold", 0.1)
        
        robot_info = meta.get("robot_info", {})
        try:
            has_mobile_base = robot_info.get("morphology", {}).get("has_mobile_base", True)
            has_cmd_vel_y = robot_info.get("morphology", {}).get("has_cmd_vel_y", False)
            camera_data = robot_info.get("sensors", {}).get("rgbd", {}).get("properties", {})
            joint_states_topic = robot_info.get("morphology", {}).get("joint_states_topic", "/joint_states")
            cmd_vel_topic = robot_info.get("morphology", {}).get("cmd_vel_topic", "/cmd_vel")
        except KeyError as e:
            self.get_logger().error(f"Missing required key in recorded_bags_meta: {e}")
            return

        if not camera_data:
            self.get_logger().error("No camera configurations found in yaml properties. Exiting.")
            return

        # Extract dynamic action features
        action_features = []
        try:
            morphology = robot_info.get("morphology", {})
            parts = morphology.get("parts", [])
            for part in parts:
                part_info = morphology.get(part, {})
                if part_info.get("is_actionable", False):
                    action_features.extend(part_info.get("joint_names", []))
                    
            if not action_features:
                self.get_logger().warn("No actionable joints found in morphology. The action space will only consist of cmd_vel if activated.")
        except Exception as e:
            self.get_logger().error(f"Error extracting action features from morphology: {e}")
            return

        camera_topics = {}
        for cam, vals in camera_data.items():
            camera_topics[cam] = vals["topic"]
            
        primary_camera = list(camera_topics.keys())[0]

        cmd_vel_keys = []
        if has_mobile_base:
            cmd_vel_keys = ["cmd_vel_x", "cmd_vel_y", "cmd_vel_theta"] if has_cmd_vel_y else ["cmd_vel_x", "cmd_vel_theta"]
        
        features = {
            "action": {
                "dtype": "float32",
                "shape": (len(action_features) + len(cmd_vel_keys),),
                "names": action_features + cmd_vel_keys
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (len(action_features) + len(cmd_vel_keys),),
                "names": action_features + cmd_vel_keys
            }
        }
        
        for cam_name, vals in camera_data.items():
            features[cam_name] = {
                "dtype": "video",
                "shape": (3, vals["height"], vals["width"]),
                "names": ["channels", "height", "width"]
            }

        dataset = LeRobotDataset.create(
            repo_id=self.dataset_name,
            fps=fps,
            features=features,
            video_backend="auto",
            streaming_encoding=True,
        )

        if hasattr(dataset, "info"):
            dataset.info["robot_info"] = robot_info
        
        for chunk_idx, (task_name, task_info) in enumerate(task_list):
            if task_name == "tasks":
                continue
                
            bag_group = task_info.get("bag_path", task_info.get("bag_dir", task_name))
            
            # Use rosbag_directory to locate the group properly
            group_dir = os.path.join(self.rosbag_directory, bag_group) if not bag_group.startswith("/") else bag_group
            if not os.path.isdir(group_dir):
                group_dir = os.path.join(self.rosbag_directory, task_name)
                if not os.path.isdir(group_dir):
                    self.get_logger().warn(f"Directory not found: {group_dir}")
                    continue

            episodes_dict = task_info.get("episodes", {})
            instruction = task_info.get("instructions", "No instruction provided")
            
            for ep in sorted(os.listdir(group_dir)):
                ep_path = os.path.join(group_dir, ep)
                if not os.path.isdir(ep_path):
                    continue
                    
                db3_files = [f for f in os.listdir(ep_path) if f.endswith(".db3") or f.endswith(".mcap")]
                if not db3_files:
                    continue
                bagfile = os.path.join(ep_path, db3_files[0])
                
                self.get_logger().info(f"Processing bag: {bagfile}")
                
                frames = extract_episode_data(
                    os.path.dirname(bagfile), 
                    camera_topics, 
                    primary_camera, 
                    joint_states_topic, 
                    cmd_vel_topic,
                    sync_thres=sync_threshold,
                    has_mobile_base=has_mobile_base,
                    has_cmd_vel_y=has_cmd_vel_y,
                    action_features=action_features,
                    logger=self.get_logger()
                )
                
                if not frames:
                    self.get_logger().warn(f"No frames extracted from {bagfile}")
                    continue
                    
                ep_meta = episodes_dict.get(ep, {})
                subtasks = ep_meta.get("subtasks", [])
                
                for i, (t_sec, frame) in enumerate(frames):
                    dataset.add_frame(frame)
                    
                dataset.save_episode(task=instruction)
                self.get_logger().info(f"Saved episode from {bagfile} with {len(frames)} frames.")

        dataset.finalize()
        self.get_logger().info(f"Dataset creation completed!")

def main(args=None):
    rclpy.init(args=args)
    node = RosbagConversionNode()
    
    # Run conversion logic once
    node.convert()
    
    # Do not spin, just finish
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
