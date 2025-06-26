#!/usr/bin/env python3
"""
Multi-camera ros2bag to AV1 MP4 conversion for LeRobot dataset structure.
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

# TODO: Create a settings YAML file to configure paths, FPS, etc.
RECORDED_BAGS_ROOT = "rosbags"
DATASET_ROOT = "MyDataset"
SETTINGS_YAML = "config/convert_settings.yaml"
RECORDED_BAGS_META = os.path.join(RECORDED_BAGS_ROOT, "recorded_bags_meta.yaml")
FPS = 10

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
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)
    return data["camera_info"]

def safe_makedirs(path):
    os.makedirs(path, exist_ok=True)

def sync_data_to_video_frame(video_frame_timestamps, data_timestamps, data):
    # For each video frame timestamp, find the closest data timestamp
    synced_obs = []
    obs_ts_np = np.array(data_timestamps)
    for t in video_frame_timestamps:
        idx = np.argmin(np.abs(obs_ts_np - t))
        synced_obs.append(data[idx])
    return synced_obs

def convert_images_to_video(bag_folder, topic, out_mp4, fps):
    writer = None
    nframes = 0
    frame_timestamps = []
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
        return True, frame_timestamps
    else:
        print("[FAIL] No frames written.")
        return False, []

def convert_mp4_to_av1(mp4_path):
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

def extract_observations_from_joint_states(bag_folder, joint_states_topic="/sobit_light/joint_states", cmd_vel_topic="/sobit_light/manual_control/cmd_vel"):
    frame_indices = []
    timestamps = []
    observations = []

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

def write_parquet_for_episode(
    out_path,
    frame_indices,
    timestamps,
    actions,
    observations,
    episode_index,
    task_index
):
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
    with open(template_path, "r") as f:
        info = json.load(f)

    for k, v in updates.items():
        info[k] = v

    if "fps" in updates:
        fps = updates["fps"]
        for key in ["observation.images.front", "observation.images.back", "observation.images.head", "observation.images.wrist.top"]:
            try:
                info["features"][key]["info"]["video.fps"] = fps
            except Exception:
                pass

    # Write the updated info.json
    safe_makedirs(os.path.dirname(out_path))
    with open(out_path, "w") as f:
        json.dump(info, f, indent=3)
    print(f"[DONE] meta/info.json written: {out_path}")

def write_tasks_jsonl(meta_yaml_path, out_jsonl_path):
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
    array = np.array(array)
    return {
        "min"  : np.min(array, axis=0).tolist(),
        "max"  : np.max(array, axis=0).tolist(),
        "mean" : np.mean(array, axis=0).tolist(),
        "std"  : np.std(array, axis=0).tolist(),
    }

def write_episodes_stats_jsonl(dataset_root, chunks_size):
    stats_lines = []
    episode_global_idx = 0
    for chunk_idx, n_episodes in enumerate(chunks_size):
        chunk_name = f"chunk-{chunk_idx:03d}"
        data_dir = os.path.join(dataset_root, "data", chunk_name)
        for ep_idx in range(n_episodes):
            parquet_path = os.path.join(data_dir, f"episode_{ep_idx:06d}.parquet")
            if not os.path.exists(parquet_path):
                continue
            df = pd.read_parquet(parquet_path)
            stats = {}
            # For "action" and "observation.state"
            for key in ["action", "observation.state"]:
                arr = np.array(df[key].tolist())
                stats[key] = compute_stats(arr)
            stats_lines.append({
                "episode_index": episode_global_idx,
                "stats": stats
            })
            episode_global_idx += 1

    out_jsonl_path = os.path.join(dataset_root, "meta", "episodes_stats.jsonl")
    safe_makedirs(os.path.dirname(out_jsonl_path))
    with open(out_jsonl_path, "w") as f:
        for entry in stats_lines:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    print(f"[DONE] meta/episodes_stats.jsonl written: {out_jsonl_path}")

def main():
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
                    is_success, video_frame_timestamps = convert_images_to_video(os.path.dirname(bagfile), topic, out_mp4, FPS)
                    if is_success:
                        convert_mp4_to_av1(out_mp4)
                        np.save(out_mp4.replace('.mp4', '_frame_times.npy'), np.array(video_frame_timestamps))

                video_frame_timestamps_dict[cam_name] = video_frame_timestamps

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
        write_episodes_stats_jsonl(DATASET_ROOT, chunks_size)
    else:
        print(f"[SKIP] {episodes_stats_jsonl_path} already exists.")

if __name__ == "__main__":
    main()
