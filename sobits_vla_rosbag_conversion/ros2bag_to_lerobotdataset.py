#!/usr/bin/env python3
"""
Convert ROS2 bags to a structured dataset for the LeRobot platform.
"""
import os
import cv2
import yaml
import pandas as pd
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.image import message_to_cvimage
import subprocess
import json
from lerobot.common.datasets.compute_stats import compute_episode_stats

# TODO: Create a settings YAML file to configure paths, FPS, etc.
RECORDED_BAGS_ROOT = "rosbags"
DATASET_ROOT = "MyDataset"
SETTINGS_YAML = "config/convert_settings.yaml"
RECORDED_BAGS_META = os.path.join(RECORDED_BAGS_ROOT, "recorded_bags_meta.yaml")
DEFAULT_FPS = 10

ACTION_FEATURES = [
    "arm_shoulder_roll_joint",
    "arm_shoulder_pitch_joint",
    "arm_shoulder_pitch_sub_joint",
    "arm_elbow_pitch_joint",
    "arm_forearm_roll_joint",
    "arm_wrist_pitch_joint",
    "arm_wrist_roll_joint",
    "hand_joint",
    "head_yaw_joint",
    "head_pitch_joint"
]

def load_camera_topics(yaml_file):
    """
    Load camera topic information from a YAML configuration file.

    Args:
        yaml_file (str): Path to the YAML file containing camera topics.

    Returns:
        Dict[str, str]: Mapping from camera names to ROS topic strings.
    """
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)
    return data["camera_info"]

def safe_makedirs(path):
    """
    Create a directory path if it does not already exist.

    Args:
        path (str): Directory path to create.
    """
    os.makedirs(path, exist_ok=True)

def sync_data_to_video_frame(video_frame_timestamps, data_timestamps, data):
    """
    Synchronize recorded data to video frame timestamps by nearest timestamp matching.

    Args:
        video_frame_timestamps (List[float]): List of timestamps for video frames.
        data_timestamps (List[float]): List of timestamps for the data to synchronize.
        data (List[Any]): List of data entries corresponding to data_timestamps.

    Returns:
        List[Any]: List of data entries synchronized to each video frame.
    """
    synced_obs = []
    obs_ts_np = np.array(data_timestamps)
    for t in video_frame_timestamps:
        idx = np.argmin(np.abs(obs_ts_np - t))
        synced_obs.append(data[idx])
    return synced_obs

def convert_images_to_video(bag_folder, topic, out_mp4, fps):
    """
    Convert image messages from a ROS bag topic to an MP4 video file.

    Args:
        bag_folder (str): Path to the directory containing the ROS bag.
        topic (str): ROS topic containing image messages.
        out_mp4 (str): Path to save the output MP4 video.
        fps (float): Frames per second for the output video.

    Returns:
        Tuple[bool, List[float]]:
            - Success flag indicating whether video was written.
            - List of timestamps corresponding to each frame in the video.
    """
    writer = None
    nframes = 0
    frame_timestamps = []
    # Create a directory to save frames
    frames_dir = os.path.splitext(out_mp4)[0] + "_frames"
    safe_makedirs(frames_dir)
    with AnyReader([Path(bag_folder)]) as reader:
        topics = [conn.topic for conn in reader.connections]
        if topic not in topics:
            print(f"[WARN] Topic {topic} not in bag {bag_folder}.")
            return False, []
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic != topic:
                continue
            msg = reader.deserialize(rawdata, connection.msgtype)
            img = message_to_cvimage(msg)
            cv_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # Save frame as image
            frame_path = os.path.join(frames_dir, f"frame_{nframes:06d}.jpg")
            cv2.imwrite(frame_path, cv_img)
            if writer is None:
                h, w, _ = cv_img.shape
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(out_mp4, fourcc, fps, (w, h))
            writer.write(cv_img)
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_timestamps.append(t_sec)
            nframes += 1
    if writer:
        writer.release()
        print(f"Video written: {out_mp4} ({nframes} frames)")
        print(f"Frames saved to: {frames_dir}")
        return True, frame_timestamps
    else:
        print("[FAIL] No frames written.")
        return False, []

def convert_mp4_to_av1(mp4_path):
    """
    Convert an MP4 video file to AV1 codec using ffmpeg.

    Args:
        mp4_path (str): Path to the MP4 file to convert.

    Returns:
        str: Path to the converted AV1 MP4 file.
    """
    av1_path = mp4_path
    temp_av1 = mp4_path + ".av1tmp.mp4"
    cmd = [
        "ffmpeg", "-y",
        "-i", mp4_path,
        "-c:v", "libaom-av1",
        "-cpu-used", "8",
        "-crf", "30",
        "-strict", "experimental",
        "-pix_fmt", "yuv420p",
        temp_av1
    ]
    subprocess.run(cmd, check=True)
    os.remove(mp4_path)
    os.replace(temp_av1, av1_path)
    print(f"[DONE] AV1 written and original removed: {av1_path}")
    return av1_path

def extract_actions_from_joint_states(bag_folder, joint_states_topic="/sobit_light/joint_trajectory_controller/controller_state", cmd_vel_topic="/sobit_light/manual_control/cmd_vel"):
    """
    Extract action vectors from joint states and command velocities in a ROS bag.

    Args:
        bag_folder (str): Path to the directory containing the ROS bag.
        joint_states_topic (str): ROS topic for joint trajectory controller state.
        cmd_vel_topic (str): ROS topic for command velocity.

    Returns:
        Tuple[List[int], List[float], List[List[float]]]:
            - Frame indices.
            - Timestamps of the actions.
            - List of action vectors per frame.
    """   
    frame_indices = []
    timestamps = []
    actions = []

    # Extract mobile base velocities from cmd_vel topic
    vel_timestamps, xvels, thetavels = extract_velocities_from_cmd_vel(bag_folder, cmd_vel_topic)

    with AnyReader([Path(bag_folder)]) as reader:
        joint_states_topics = [conn.topic for conn in reader.connections]
        if joint_states_topic not in joint_states_topics:
            print(f"[WARN] Topic {joint_states_topic} not in bag.")
            return [], [], []
        for idx, (connection, timestamp, rawdata) in enumerate(reader.messages()):
            if connection.topic != joint_states_topic:
                continue
            msg = reader.deserialize(rawdata, connection.msgtype)
            # Map joint names to positions
            joint_pos = dict(zip(msg.joint_names, msg.reference.positions))
            # timestamp in seconds: sec + nanosec * 1e-9
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Sync velocities using timestamp from joint_states
            if vel_timestamps:
                idx_vel = np.argmin(np.abs(np.array(vel_timestamps) - t_sec))
                x_vel = xvels[idx_vel]
                theta_vel = thetavels[idx_vel]
            else:
                x_vel = 0.0
                theta_vel = 0.0

            # Create the observation vector in the required order
            obs_vec = [
                joint_pos.get("arm_shoulder_roll_joint", 0.0),
                joint_pos.get("arm_shoulder_pitch_joint", 0.0),
                joint_pos.get("arm_shoulder_pitch_sub_joint", 0.0),
                joint_pos.get("arm_elbow_pitch_joint", 0.0),
                joint_pos.get("arm_forearm_roll_joint", 0.0),
                joint_pos.get("arm_wrist_pitch_joint", 0.0),
                joint_pos.get("arm_wrist_roll_joint", 0.0),
                joint_pos.get("hand_joint", 0.0),
                joint_pos.get("head_yaw_joint", 0.0),
                joint_pos.get("head_pitch_joint", 0.0),
                x_vel,
                theta_vel,
            ]
            actions.append(obs_vec)
            timestamps.append(t_sec)
            frame_indices.append(idx)
    return frame_indices, timestamps, actions

def extract_observations_from_joint_states(bag_folder, joint_states_topic="/sobit_light/joint_states", odom_topic="/sobit_light/odometry/odometry"):
    """
    Extract observation vectors from joint states and odometry in a ROS bag.

    Args:
        bag_folder (str): Path to the directory containing the ROS bag.
        joint_states_topic (str): ROS topic for joint states.
        odom_topic (str): ROS topic for odometry.

    Returns:
        Tuple[List[int], List[float], List[List[float]]]:
            - Frame indices.
            - Timestamps of the observations.
            - List of observation vectors per frame.
    """
    frame_indices = []
    timestamps = []
    observations = []

    # Extract mobile base velocities from cmd_vel topic
    vel_timestamps, xvels, thetavels = extract_velocities_from_odom(bag_folder, odom_topic)

    with AnyReader([Path(bag_folder)]) as reader:
        joint_states_topics = [conn.topic for conn in reader.connections]
        if joint_states_topic not in joint_states_topics:
            print(f"[WARN] Topic {joint_states_topic} not in bag.")
            return [], [], []
        for idx, (connection, timestamp, rawdata) in enumerate(reader.messages()):
            if connection.topic != joint_states_topic:
                continue
            msg = reader.deserialize(rawdata, connection.msgtype)
            # Map joint names to positions
            joint_pos = dict(zip(msg.name, msg.position))
            # timestamp in seconds: sec + nanosec * 1e-9
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Sync velocities using timestamp from joint_states
            if vel_timestamps:
                idx_vel = np.argmin(np.abs(np.array(vel_timestamps) - t_sec))
                x_vel = xvels[idx_vel]
                theta_vel = thetavels[idx_vel]
            else:
                x_vel = 0.0
                theta_vel = 0.0

            # Create the observation vector in the required order
            obs_vec = [
                joint_pos.get("arm_shoulder_roll_joint", 0.0),
                joint_pos.get("arm_shoulder_pitch_joint", 0.0),
                joint_pos.get("arm_shoulder_pitch_sub_joint", 0.0),
                joint_pos.get("arm_elbow_pitch_joint", 0.0),
                joint_pos.get("arm_forearm_roll_joint", 0.0),
                joint_pos.get("arm_wrist_pitch_joint", 0.0),
                joint_pos.get("arm_wrist_roll_joint", 0.0),
                joint_pos.get("hand_joint", 0.0),
                joint_pos.get("head_yaw_joint", 0.0),
                joint_pos.get("head_pitch_joint", 0.0),
                x_vel,
                theta_vel,
            ]
            observations.append(obs_vec)
            timestamps.append(t_sec)
            frame_indices.append(idx)
    return frame_indices, timestamps, observations

def extract_velocities_from_cmd_vel(bag_folder, topic):
    """
    Extract linear and angular velocities from a cmd_vel ROS topic.

    Args:
        bag_folder (str): Path to the directory containing the ROS bag.
        topic (str): ROS topic for command velocity.

    Returns:
        Tuple[List[float], List[float], List[float]]:
            - Timestamps of the messages.
            - List of linear x velocities.
            - List of angular z velocities.
    """
    timestamps = []
    xvels = []
    thetavels = []
    with AnyReader([Path(bag_folder)]) as reader:
        topics = [conn.topic for conn in reader.connections]
        if topic not in topics:
            return [], [], []
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic != topic:
                continue
            msg = reader.deserialize(rawdata, connection.msgtype)
            t_sec = timestamp * 1e-9
            xvels.append(msg.linear.x)
            thetavels.append(msg.angular.z)
            timestamps.append(t_sec)
    return timestamps, xvels, thetavels

def extract_velocities_from_odom(bag_folder, topic):
    """
    Extract linear and angular velocities from an odometry ROS topic.

    Args:
        bag_folder (str): Path to the directory containing the ROS bag.
        topic (str): ROS topic for odometry.

    Returns:
        Tuple[List[float], List[float], List[float]]:
            - Timestamps of the messages.
            - List of linear x velocities.
            - List of angular z velocities.
    """
    timestamps = []
    xvels = []
    thetavels = []
    with AnyReader([Path(bag_folder)]) as reader:
        topics = [conn.topic for conn in reader.connections]
        if topic not in topics:
            return [], [], []
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic != topic:
                continue
            msg = reader.deserialize(rawdata, connection.msgtype)
            t_sec = timestamp * 1e-9
            xvels.append(msg.twist.twist.linear.x)
            thetavels.append(msg.twist.twist.angular.z)
            timestamps.append(t_sec)
    return timestamps, xvels, thetavels


def write_parquet_for_episode(
    out_path,
    frame_indices,
    timestamps,
    actions,
    observations,
    episode_index,
    task_index
):
    """
    Write synchronized action and observation data to a Parquet file.

    Args:
        out_path (str): Path to the output Parquet file.
        frame_indices (List[int]): Frame indices.
        timestamps (List[float]): Frame timestamps.
        actions (List[List[float]]): List of action vectors.
        observations (List[List[float]]): List of observation vectors.
        episode_index (int): Episode index.
        task_index (int): Task index.
    """
    action_states = np.array(actions, dtype=np.float32)
    observation_states = np.array(observations, dtype=np.float32)
    df = pd.DataFrame({
        "action": action_states.tolist(),
        "observation.state": observation_states.tolist(),
        "timestamp": timestamps,
        "frame_index": frame_indices,
        "episode_index": [episode_index] * len(frame_indices),
        "index": list(range(len(frame_indices))),
        "task_index": [task_index] * len(frame_indices),
    })
    df.to_parquet(out_path, index=False)
    print(f"[DONE] Parquet written: {out_path}")

def write_info_json_from_template(template_path, out_path, updates):
    """
    Generate info.json metadata file by updating a template with dataset information.

    Args:
        template_path (str): Path to the template info.json file.
        out_path (str): Path to save the updated info.json.
        updates (Dict): Dictionary of updates to apply (e.g., total_episodes, fps).
    """
    with open(template_path, "r") as f:
        info = json.load(f)

    for k, v in updates.items():
        info[k] = v

    # Load camera keys dynamically from SETTINGS_YAML
    with open(SETTINGS_YAML, "r") as f:
        config = yaml.safe_load(f)
    camera_keys = list(config["camera_info"].keys())

    if "fps" in updates:
        fps = updates["fps"]
        for key in camera_keys:
            try:
                info["features"][key]["info"]["video.fps"] = fps
            except Exception:
                print(f"[WARN] Could not set FPS for {key} in info.json.")

    # Write the updated info.json
    safe_makedirs(os.path.dirname(out_path))
    with open(out_path, "w") as f:
        json.dump(info, f, indent=3)
    print(f"[DONE] meta/info.json written: {out_path}")

def write_tasks_jsonl(meta_yaml_path, out_jsonl_path):
    """
    Write the tasks metadata to a JSON Lines (JSONL) file.

    Args:
        meta_yaml_path (str): Path to the recorded_bags_meta.yaml file.
        out_jsonl_path (str): Path to save the JSONL file.
    """
    # Load the meta YAML
    with open(meta_yaml_path, "r") as f:
        meta = yaml.safe_load(f)
    tasks = []
    task_list = meta["recorded_bags"]["tasks"]
    for idx, task_name in enumerate(task_list):
        task_dict = {
            "task_index": idx,
            "task": meta["recorded_bags"][task_name]["label"]
        }
        tasks.append(task_dict)
    # Write as JSONL
    safe_makedirs(os.path.dirname(out_jsonl_path))
    with open(out_jsonl_path, "w") as f:
        for task in tasks:
            f.write(json.dumps(task, ensure_ascii=False) + "\n")
    print(f"[DONE] meta/tasks.jsonl written: {out_jsonl_path}")

def write_episodes_jsonl(
    meta_yaml_path,
    dataset_root,
    chunks_size,
    episode_lengths
):
    """
    Write episode metadata to a JSON Lines (JSONL) file.

    Args:
        meta_yaml_path (str): Path to the recorded_bags_meta.yaml file.
        dataset_root (str): Root directory of the dataset.
        chunks_size (List[int]): List of episode counts per chunk.
        episode_lengths (Dict[Tuple[int, int], int]): Mapping of (chunk_idx, episode_idx) to episode length.
    """
    # Gather all task instructions
    with open(meta_yaml_path, "r") as f:
        meta = yaml.safe_load(f)
    # task_list = list(meta["recorded_bags"].items())
    task_list = meta["recorded_bags"]["tasks"]

    lines = []
    episode_global_idx = 0
    for chunk_idx, task_name in enumerate(task_list):
        instruction = meta["recorded_bags"][task_name]["label"]
        n_episodes = chunks_size[chunk_idx]
        for episode_index in range(n_episodes):
            length = episode_lengths.get((chunk_idx, episode_index), 0)
            entry = {
                "episode_index": episode_global_idx,
                "tasks": [instruction],
                "length": length
            }
            lines.append(entry)
            episode_global_idx += 1

    out_jsonl_path = os.path.join(dataset_root, "meta", "episodes.jsonl")
    safe_makedirs(os.path.dirname(out_jsonl_path))
    with open(out_jsonl_path, "w") as f:
        for entry in lines:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    print(f"[DONE] meta/episodes.jsonl written: {out_jsonl_path}")


def compute_stats(array):
    """
    Compute statistics (min, max, mean, std, count) over a numpy array.

    Args:
        array (Union[List, np.ndarray]): Input array to compute statistics.

    Returns:
        Dict[str, Any]: Dictionary containing computed statistics.
    """
    array = np.array(array)
    return {
        "min"  : np.min(array, axis=0).tolist(),
        "max"  : np.max(array, axis=0).tolist(),
        "mean" : np.mean(array, axis=0).tolist(),
        "std"  : np.std(array, axis=0).tolist(),
        "count": np.array([len(array)]),
    }

class NumpyEncoder(json.JSONEncoder):
    """
    Custom JSON encoder that converts numpy arrays to lists.
    """
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)

def write_episodes_stats_jsonl(dataset_root, chunks_size, camera_info):
    """
    Compute and write episode statistics to a JSON Lines (JSONL) file.

    Args:
        dataset_root (str): Root directory of the dataset.
        chunks_size (List[int]): List of episode counts per chunk.
        camera_info (Dict[str, str]): Mapping of camera names to ROS topics.
    """
    stats_lines = []
    episode_global_idx = 0
    camera_keys = list(camera_info.keys())
    for chunk_idx, n_episodes in enumerate(chunks_size):
        chunk_name = f"chunk-{chunk_idx:03d}"
        data_dir = os.path.join(dataset_root, "data", chunk_name)
        frame_dir = os.path.join(dataset_root, "videos", chunk_name)
        for ep_idx in range(n_episodes):
            parquet_path = os.path.join(data_dir, f"episode_{ep_idx:06d}.parquet")
            if not os.path.exists(parquet_path):
                continue
            df = pd.read_parquet(parquet_path)
            episode_data = {
                "action": np.array(df["action"].tolist()),
                "observation.state": np.array(df["observation.state"].tolist()),
                "timestamp": np.array(df["timestamp"].tolist()),
                "frame_index": np.array(df["frame_index"].tolist()),
                "episode_index": np.array(df["episode_index"].tolist()),
                "index": np.array(df["index"].tolist()),
                "task_index": np.array(df["task_index"].tolist()),
            }
            features = {
                "action": {"dtype": "float"},
                "observation.state": {"dtype": "float"},
                "timestamp": {"dtype": "float"},
                "frame_index": {"dtype": "int"},
                "episode_index": {"dtype": "int"},
                "index": {"dtype": "int"},
                "task_index": {"dtype": "int"},
            }
            # --------- IMAGE STATS ---------
            frame_dirs_to_delete = []
            for cam_key in camera_keys:
                frames_dir = os.path.join(frame_dir, cam_key.replace("=", "").strip(), f"episode_{ep_idx:06d}_frames")
                if os.path.isdir(frames_dir):
                    frame_paths = sorted([
                        os.path.join(frames_dir, fname)
                        for fname in os.listdir(frames_dir)
                        if fname.lower().endswith(('.jpg', '.jpeg', '.png'))
                    ])
                else:
                    frame_paths = []
                # If no frames found, skip this camera
                if not frame_paths:
                    print(f"[WARN] No frames found for {cam_key} in episode {ep_idx}.")
                    continue

                if len(frame_paths) > 0:
                    frame_paths = [p for p in frame_paths if isinstance(p, str)]
                    if len(frame_paths) == 0:
                        print(f"[WARN] No valid image paths for {cam_key}, skipping.")
                    else:
                        try:
                            episode_data[cam_key] = frame_paths
                            features[cam_key] = {"dtype": "image"}
                            frame_dirs_to_delete.append(frames_dir)
                        except Exception as e:
                            print(f"[ERROR] sample_images failed for {cam_key}: {e}")

            # --------- END IMAGE STATS ---------
            # Load meta/info.json to get feature keys
            meta_info_path = os.path.join(dataset_root, "meta", "info.json")
            with open(meta_info_path, "r") as f:
                info_meta = json.load(f)

            feature_keys = list(info_meta["features"].keys())

            # Compute stats
            stats = compute_episode_stats(episode_data, features)

            # Build stats dict
            ordered_stats = {k: stats[k] for k in feature_keys if k in stats}

            # Append to stats_lines
            stats_lines.append({
                "episode_index": episode_global_idx,
                "stats": ordered_stats
            })

            episode_global_idx += 1

            # Clean up frames
            import shutil
            for frames_dir in frame_dirs_to_delete:
                shutil.rmtree(frames_dir, ignore_errors=True)

    out_jsonl_path = os.path.join(dataset_root, "meta", "episodes_stats.jsonl")
    safe_makedirs(os.path.dirname(out_jsonl_path))
    with open(out_jsonl_path, "w") as f:
        for entry in stats_lines:
            f.write(json.dumps(entry, ensure_ascii=False, cls=NumpyEncoder) + "\n")
    print(f"[DONE] meta/episodes_stats.jsonl written: {out_jsonl_path}")

def compute_fps_for_videos(video_frame_timestamps_dict):
    """
    Compute FPS per camera from frame timestamps.

    Args:
        video_frame_timestamps_dict: Dict[str, List[float]]
            A dictionary where keys are camera names and values are lists of timestamps for each frame.

    Returns:
        Dict[str, float]: mapping of camera name -> computed FPS
    """
    fps_per_camera = {}

    for cam_name, timestamps in video_frame_timestamps_dict.items():
        if not timestamps or len(timestamps) < 2:
            fps_per_camera[cam_name] = 0.0
            continue
        duration = timestamps[-1] - timestamps[0]
        nframes = len(timestamps)
        fps = nframes / duration if duration > 0 else 0.0
        fps_per_camera[cam_name] = fps

    return fps_per_camera


def main():
    """
    Main function that converts ROS2 bags into the LeRobot dataset format.

    It processes video conversion, action and observation extraction, parquet writing,
    metadata generation, and statistics computation.
    """
    total_episodes = 0
    total_frames = 0
    total_videos = 0
    chunks_size = []
    episode_lengths = {}
    
    camera_info = load_camera_topics(SETTINGS_YAML)
    primary_camera = "observation.images.hand" if "observation.images.hand" in camera_info else list(camera_info.keys())[0]

    # Load task/chunk mapping from recorded_bags_meta.yaml
    with open(RECORDED_BAGS_META, "r") as f:
        meta = yaml.safe_load(f)
    # task_list = list(meta["recorded_bags"].items())
    task_list = meta["recorded_bags"]["tasks"]
    total_tasks = len(task_list)
    total_chunks = len(task_list)
    # Assign chunks in order
    for chunk_idx, task_name in enumerate(task_list):
        bag_group = meta["recorded_bags"][task_name]["bag_dir"]
        chunk_name = f"chunk-{chunk_idx:03d}"
        group_dir = os.path.join(RECORDED_BAGS_ROOT, bag_group)
        if not os.path.isdir(group_dir):
            print(f"[WARN] Directory not found: {group_dir}")
            continue
        # Each episode is a folder inside the group
        for ep in sorted(os.listdir(group_dir)):
            ep_path = os.path.join(group_dir, ep)
            if not os.path.isdir(ep_path):
                continue
            # Find bag file (*.mcap) inside episode folder
            mcap_files = [f for f in os.listdir(ep_path) if f.endswith(".mcap")]
            if not mcap_files:
                print(f"[WARN] No .mcap found in {ep_path}")
                continue
            bagfile = os.path.join(ep_path, mcap_files[0])
            # Name episode file as e.g. episode_000000.mp4 (index from 0 in all chunks)
            ep_idx = int(ep.split('_')[-1]) - 1  # e.g. episode_1 -> 0-based index
            episode_name = f"episode_{ep_idx:06d}.mp4"

            video_frame_timestamps_dict = {}
            for cam_name, topic in camera_info.items():
                out_dir = os.path.join(
                    DATASET_ROOT, "videos", chunk_name, cam_name.replace("=", "").strip()
                )
                safe_makedirs(out_dir)
                out_mp4 = os.path.join(out_dir, episode_name)
                print(f"[{chunk_name}][{cam_name}] {topic} → {out_mp4}")

                if os.path.exists(out_mp4):
                    print(f"[SKIP] {out_mp4} already exists. Skipping video encoding.")
                    ts_path = out_mp4.replace('.mp4', '_frame_times.npy')
                    if os.path.exists(ts_path):
                        video_frame_timestamps = np.load(ts_path).tolist()
                    else:
                        print(f"[ERROR] Frame timestamps file missing for {out_mp4}, cannot sync.")
                        video_frame_timestamps = None
                else:
                    is_success, video_frame_timestamps = convert_images_to_video(os.path.dirname(bagfile), topic, out_mp4, DEFAULT_FPS)  # FIXME: Use lowest fps of all cameras
                    if is_success:
                        convert_mp4_to_av1(out_mp4)
                        np.save(out_mp4.replace('.mp4', '_frame_times.npy'), np.array(video_frame_timestamps))

                video_frame_timestamps_dict[cam_name] = video_frame_timestamps

            fps_per_camera = compute_fps_for_videos(video_frame_timestamps_dict)
            print("Computed FPS per camera:")
            for cam, fps in fps_per_camera.items():
                print(f"  {cam}: {fps:.2f}")
            # lowest FPS from all cameras
            lowest_fps = min(fps_per_camera.values())
            # FPS = int(lowest_fps) if lowest_fps > 0 else int(DEFAULT_FPS) 
            FPS = DEFAULT_FPS  # Use default FPS for now FIXME: Use lowest FPS of all cameras

            # Pick primary camera for action sync
            video_frame_timestamps = video_frame_timestamps_dict.get(primary_camera)

            # Write parquet for this episode
            data_out_dir = os.path.join(DATASET_ROOT, "data", chunk_name)
            safe_makedirs(data_out_dir)
            parquet_path = os.path.join(data_out_dir, f"episode_{ep_idx:06d}.parquet")
            
            _, obs_timestamps, observations = extract_observations_from_joint_states(os.path.dirname(bagfile))
            _, act_timestamps, actions = extract_actions_from_joint_states(os.path.dirname(bagfile))
            if video_frame_timestamps and obs_timestamps and act_timestamps:
                # Normalize both to start at zero
                t0 = video_frame_timestamps[0]
                video_frame_timestamps = [t - t0 for t in video_frame_timestamps]
                obs_timestamps = [t - t0 for t in obs_timestamps]
                act_timestamps = [t - t0 for t in act_timestamps]
                frame_indices = list(range(len(video_frame_timestamps)))
                # Synchronize data to video frames
                synced_actions = sync_data_to_video_frame(video_frame_timestamps, act_timestamps, actions)
                synced_observations = sync_data_to_video_frame(video_frame_timestamps, obs_timestamps, observations)
            else:
                frame_indices = []
                video_frame_timestamps = []
                synced_actions = []
                synced_observations = []

            write_parquet_for_episode(
                parquet_path,
                frame_indices,
                video_frame_timestamps,
                synced_actions,
                synced_observations,
                ep_idx,
                chunk_idx
            )
            episode_lengths[(chunk_idx, ep_idx)] = len(frame_indices)

            # collect statistics for the info.json
            total_episodes += 1
            total_videos += len(camera_info)
            if len(frame_indices) > 0:
                total_frames += len(frame_indices)

        chunk_episode_count = len([ep for ep in os.listdir(group_dir) if os.path.isdir(os.path.join(group_dir, ep))])
        chunks_size.append(chunk_episode_count)

    updates = {
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": total_tasks,
        "total_videos": total_videos,
        "total_chunks": total_chunks,
        "chunks_size": chunks_size,
        "fps": FPS,
        "splits": {
            "train": f"0:{total_episodes}"
        }
    }

    # Get the robot_type from the first task (assume all same in recorded_bags_meta.yaml)
    if task_list:
        robot_type = meta["robot_info"]["name"]
    else:
        robot_type = "default_robot"
        print(f"[ERROR] Could not recognize robot type from tasks. Using '{robot_type}'.")
    
    meta_dir = os.path.join(DATASET_ROOT, "meta")
    safe_makedirs(meta_dir)

    info_json_path = os.path.join(meta_dir, "info.json")
    tasks_jsonl_path = os.path.join(meta_dir, "tasks.jsonl")
    episodes_jsonl_path = os.path.join(meta_dir, "episodes.jsonl")
    episodes_stats_jsonl_path = os.path.join(meta_dir, "episodes_stats.jsonl")

    if not os.path.exists(info_json_path):
        write_info_json_from_template(f"template_info/{robot_type}_info.json", info_json_path, updates)
    else:
        print(f"[SKIP] {info_json_path} already exists.")

    if not os.path.exists(tasks_jsonl_path):
        write_tasks_jsonl(RECORDED_BAGS_META, tasks_jsonl_path)
    else:
        print(f"[SKIP] {tasks_jsonl_path} already exists.")

    if not os.path.exists(episodes_jsonl_path):
        write_episodes_jsonl(RECORDED_BAGS_META, DATASET_ROOT, chunks_size, episode_lengths)
    else:
        print(f"[SKIP] {episodes_jsonl_path} already exists.")

    if not os.path.exists(episodes_stats_jsonl_path):
        write_episodes_stats_jsonl(DATASET_ROOT, chunks_size, camera_info)
    else:
        print(f"[SKIP] {episodes_stats_jsonl_path} already exists.")

if __name__ == "__main__":
    main()
