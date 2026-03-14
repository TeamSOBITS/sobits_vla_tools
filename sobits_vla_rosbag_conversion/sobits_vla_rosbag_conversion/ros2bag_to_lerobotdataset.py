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

        # Configuration attributes (populated from YAML)
        self.camera_topics = {}
        self.primary_camera = ""
        self.joint_states_topic = ""
        self.cmd_vel_topic = ""
        self.sync_threshold = 0.1
        self.has_mobile_base = False
        self.has_cmd_vel_y = False
        self.has_cmd_vel_z = False
        self.action_features = []
        self.all_subtasks_list = []
        self.has_subtasks = False

        # Create a one-shot timer to start conversion after the node is ready
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """One-shot timer callback to trigger the conversion."""
        self.timer.cancel()
        try:
            self.convert()
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")
        finally:
            self.get_logger().info("Shutting down node...")
            # Trigger shutdown
            raise SystemExit

    def _extract_episode_data(self, bag_folder, subtasks_map):
        """
        Iterate over the bag and return a list of frames.
        """
        frames = []
        images = {cam_name: None for cam_name in self.camera_topics.keys()}
        image_times = {cam_name: 0.0 for cam_name in self.camera_topics.keys()}
        latest_joint_state = None
        latest_joint_time = 0.0
        
        latest_cmd_vel = None
        latest_cmd_vel_time = 0.0
        
        if self.has_mobile_base:
            if self.has_cmd_vel_y:
                latest_cmd_vel = [0.0, 0.0, 0.0]
            else:
                latest_cmd_vel = [0.0, 0.0]
        
        topic_to_cam = {v: k for k, v in self.camera_topics.items()}
        
        with AnyReader([Path(bag_folder)]) as reader:
            for connection, timestamp, rawdata in reader.messages():
                topic = connection.topic
                t_bag = timestamp * 1e-9
                
                if topic == self.joint_states_topic:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    joint_pos = dict(zip(msg.name, msg.position))
                    t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    latest_joint_state = [
                        joint_pos.get(feat, 0.0) for feat in self.action_features
                    ]
                    latest_joint_time = t_sec
                    
                if topic == self.cmd_vel_topic and self.has_mobile_base:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 if hasattr(msg, 'header') else t_bag
                    
                    if self.has_cmd_vel_y and self.has_cmd_vel_z:
                        latest_cmd_vel = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
                    elif self.has_cmd_vel_y:
                        latest_cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
                    elif self.has_cmd_vel_z:
                        latest_cmd_vel = [msg.linear.x, msg.linear.z, msg.angular.z]
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
                    if cam_name == self.primary_camera and None not in images.values() and latest_joint_state is not None:
                        # Proceed only if cmd_vel is satisfied (either not needed, or available)
                        if not self.has_mobile_base or latest_cmd_vel is not None:
                            # Sync checks
                            img_times = list(image_times.values())
                            max_camera_diff = max(img_times) - min(img_times)
                            
                            # For joint state sync, compare against the primary camera time
                            joint_diff = abs(image_times[self.primary_camera] - latest_joint_time)
                            
                            cmd_vel_diff = 0.0
                            if self.has_mobile_base:
                                cmd_vel_diff = abs(image_times[self.primary_camera] - latest_cmd_vel_time)
                                
                            if max_camera_diff > self.sync_threshold or joint_diff > self.sync_threshold or cmd_vel_diff > self.sync_threshold:
                                self.get_logger().warn(f"Sync threshold exceeded: max_cam_diff={max_camera_diff:.3f}s, joint_diff={joint_diff:.3f}s, cmd_vel_diff={cmd_vel_diff:.3f}s")
                                
                            action = latest_joint_state + (latest_cmd_vel if self.has_mobile_base else [])
                            state = latest_joint_state + (latest_cmd_vel if self.has_mobile_base else [])
                            
                            frame = {
                                "action": torch.tensor(action, dtype=torch.float32),
                                "observation.state": torch.tensor(state, dtype=torch.float32), 
                            }
                            
                            # Ensure all requested cameras are present
                            missing_camera = False
                            for c_name in self.camera_topics.keys():
                                if images[c_name] is None:
                                    missing_camera = True
                                    break
                                img_t = torch.from_numpy(images[c_name]).permute(2, 0, 1).contiguous()
                                frame[c_name] = img_t
                                
                            if not missing_camera:
                                if self.all_subtasks_list:
                                    current_subtask_idx = 0 # Default to index 0 ("No Subtask")
                                    if subtasks_map:
                                        for st_key, st_info in subtasks_map.items():
                                            start_time = st_info.get("start_timestamp", -1.0)
                                            end_time = st_info.get("end_timestamp", float('inf'))
                                            
                                            if end_time == 0.0:
                                                end_time = float('inf')
                                                
                                            if start_time <= t_sec <= end_time:
                                                label = st_info.get("label")
                                                if label in self.all_subtasks_list:
                                                    current_subtask_idx = self.all_subtasks_list.index(label)
                                                break
                                            
                                    frame["subtask_index"] = torch.tensor([current_subtask_idx], dtype=torch.int64)

                                frames.append((t_sec, frame))
                                # Reset images to None after snapshot to wait for new set
                                images = {cam_name: None for cam_name in self.camera_topics.keys()}

        return frames
        
    def convert(self):
        self.get_logger().info("Starting dataset conversion...")
        self.get_logger().info(f"Target dataset name: {self.dataset_name}")
        self.get_logger().info(f"Reading from rosbags root: {self.rosbag_directory}")
        
        self.get_logger().info(f"Searching for metadata files in: {self.rosbag_directory}")
        
        meta_files = []
        for root, _, files in os.walk(self.rosbag_directory):
            for file in files:
                if file == "recorded_bags_meta.yaml":
                    meta_files.append(os.path.join(root, file))
                    
        if not meta_files:
            self.get_logger().error(f"No recorded_bags_meta.yaml files found in {self.rosbag_directory} or its subdirectories.")
            return
            
        self.get_logger().info(f"Found {len(meta_files)} metadata files.")
        
        all_tasks = []
        robot_ref_name = None
        robot_ref_version = None
        
        for idx, meta_file in enumerate(meta_files):
            with open(meta_file, "r") as f:
                meta = yaml.safe_load(f)
                
            robot_info = meta.get("robot_info", {})
            r_name = robot_info.get("name")
            r_vers = robot_info.get("version")
            
            if idx == 0:
                robot_ref_name = r_name
                robot_ref_version = r_vers
                # Save the base configurations from the first valid YAML
                convert_info = meta.get("convert_info", {})
                fps = convert_info.get("fps", 10)
                self.sync_threshold = convert_info.get("sync_threshold", 0.1)
                
                try:
                    self.joint_states_topic = robot_info.get("morphology", {}).get("joint_states_topic", "/joint_states")
                    
                    # Extract morphology
                    morphology = robot_info.get("morphology", {})
                    parts = morphology.get("parts", [])
                    
                    self.action_features = []
                    for part in parts:
                        part_info = morphology.get(part, {})
                        if part_info.get("is_actionable", False):
                            self.action_features.extend(part_info.get("joint_names", []))
                            
                            # Check if the part is a mobile base module
                            if part in ["mobile_base", "legs"]:
                                self.has_mobile_base = True
                                self.has_cmd_vel_y = part_info.get("has_cmd_vel_y", False)
                                self.has_cmd_vel_z = part_info.get("has_cmd_vel_z", False)
                                self.cmd_vel_topic = part_info.get("cmd_vel_topic", "/cmd_vel")

                    if not self.action_features and not self.has_mobile_base:
                        self.get_logger().warn("No actionable joints and no active mobile base found in morphology.")
                        
                except Exception as e:
                    self.get_logger().error(f"Error extracting features from morphology in primary yaml: {e}")
                    return
            else:
                if r_name != robot_ref_name or r_vers != robot_ref_version:
                    self.get_logger().error(f"Found conflicting robot identities! Expected {robot_ref_name} v{robot_ref_version}, but found {r_name} v{r_vers} in {meta_file}")
                    return
                    
            tasks_list_data = list(meta.get("recorded_bags", {}).items())
            
            # Extend tasks with the path to the original meta_file to resolve local groups properly
            for t_name, t_info in tasks_list_data:
                if t_name == "tasks": continue
                t_info["_meta_source_dir"] = os.path.dirname(meta_file)
                all_tasks.append((t_name, t_info))

        camera_data = robot_info.get("sensors", {}).get("rgbd", {})
        if not camera_data or "names" not in camera_data or "topics" not in camera_data:
            self.get_logger().error("No precise 'topics' and 'names' arrays found for 'rgbd' sensor in yaml. Exiting.")
            return

        self.camera_topics = {}
        for cam_name, cam_topic in zip(camera_data.get("names", []), camera_data.get("topics", [])):
            self.camera_topics[cam_name] = cam_topic
            
        if not self.camera_topics:
            self.get_logger().error("No camera topics loaded.")
            return

        self.primary_camera = list(self.camera_topics.keys())[0]

        cmd_vel_keys = []
        if self.has_mobile_base:
            if self.has_cmd_vel_y and self.has_cmd_vel_z:
                cmd_vel_keys = ["cmd_vel_x", "cmd_vel_y", "cmd_vel_z", "cmd_vel_theta"]
            elif self.has_cmd_vel_y:
                cmd_vel_keys = ["cmd_vel_x", "cmd_vel_y", "cmd_vel_theta"]
            elif self.has_cmd_vel_z:
                cmd_vel_keys = ["cmd_vel_x", "cmd_vel_z", "cmd_vel_theta"]
            else:
                cmd_vel_keys = ["cmd_vel_x", "cmd_vel_theta"]
        
        # Extract all unique subtasks from the metadata
        all_subtasks_set = set()
        for task_name, task_info in all_tasks:
            episodes_dict = task_info.get("episodes", {})
            for ep_key, ep_meta in episodes_dict.items():
                subtasks_map = ep_meta.get("subtasks", {})
                for st_key, st_info in subtasks_map.items():
                    if "label" in st_info:
                        all_subtasks_set.add(st_info["label"])
                        
        self.all_subtasks_list = sorted(list(all_subtasks_set))
        if self.all_subtasks_list:
            self.all_subtasks_list.insert(0, "No Subtask")
            
        self.has_subtasks = len(self.all_subtasks_list) > 0

        features = {
            "action": {
                "dtype": "float32",
                "shape": (len(self.action_features) + len(cmd_vel_keys),),
                "names": self.action_features + cmd_vel_keys
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (len(self.action_features) + len(cmd_vel_keys),),
                "names": self.action_features + cmd_vel_keys
            }
        }
        
        if self.has_subtasks:
            features["subtask_index"] = {
                "dtype": "int64",
                "shape": (1,),
                "names": ["subtask_index"]
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

        if self.has_subtasks:
            import pandas as pd
            dataset.meta.subtasks = pd.DataFrame({"subtask": self.all_subtasks_list})

        if hasattr(dataset, "info"):
            dataset.info["robot_info"] = robot_info
        
        for chunk_idx, (task_name, task_info) in enumerate(all_tasks):
            meta_src = task_info.get("_meta_source_dir", self.rosbag_directory)
            bag_group = task_info.get("bag_path", task_info.get("bag_dir", task_name))
            
            # Use metadata source directory to locate the group properly
            group_dir = os.path.join(meta_src, bag_group) if not bag_group.startswith("/") else bag_group
            if not os.path.isdir(group_dir):
                group_dir = os.path.join(meta_src, task_name)
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
                
                ep_meta = episodes_dict.get(ep, {})
                subtasks = ep_meta.get("subtasks", {})
                
                frames = self._extract_episode_data(
                    os.path.dirname(bagfile), 
                    subtasks_map=subtasks
                )
                
                if not frames:
                    self.get_logger().warn(f"No frames extracted from {bagfile}")
                    continue
                                        
                for i, (t_sec, frame) in enumerate(frames):
                    dataset.add_frame(frame)
                    
                dataset.save_episode(task=instruction)
                self.get_logger().info(f"Saved episode from {bagfile} with {len(frames)} frames.")

        dataset.finalize()
        self.get_logger().info(f"Dataset creation completed!")

def main(args=None):
    rclpy.init(args=args)
    node = RosbagConversionNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
