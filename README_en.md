<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS VLA Tools

<!-- INTRODUCTION -->
## Introduction

SOBITS VLA Tools is a monorepo providing the full pipeline for controlling SOBITS-developed robots with Vision-Language-Action (VLA) models — from data collection through training to real-time deployment, all integrated via ROS 2.

### Package Overview

| Package | Description |
| ------- | ----------- |
| `sobits_vla_rosbag_collection` | Gamepad-triggered multi-modal rosbag recording with live quality monitoring |
| `sobits_vla_rosbag_conversion` | Converts rosbags into [LeRobot](https://github.com/huggingface/lerobot) dataset format |
| `sobits_vla_training` | Model training utilities (TBD) |
| `sobits_vla_deploy` | Real-time VLA inference node for robot control (TBD) |
| `sobits_vla_visualization` | Dataset and inference visualization (TBD) |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

| System | Version |
| ------ | ------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Jazzy Jalisco           |
| Python | ≥3.10                  |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to the `src` folder of your ROS workspace.
    ```sh
    $ cd ~/colcon_ws/src/
    ```
2. Clone this repository.
    ```sh
    $ git clone https://github.com/TeamSOBITS/sobits_vla_tools
    ```
3. Install the required dependencies.
    ```sh
    $ cd sobits_vla_tools/
    $ bash install.sh
    ```
4. Compile the packages.
    ```sh
    $ cd ~/colcon_ws
    $ rosdep update
    $ rosdep install --from-paths src -y --ignore-src
    $ colcon build
    $ source install/setup.bash
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- PACKAGES -->
## Packages

### 1. Data Collection

**Package:** [sobits_vla_rosbag_collection](./sobits_vla_rosbag_collection/)

Records multi-modal sensor data (cameras, joint states, odometry, LiDAR, TF) as rosbag episodes using a gamepad controller.

#### Launch

```bash
ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py \
  robot_name:=sobit_light \
  record_directory:=/path/to/rosbags
```

| Argument | Default | Description |
| -------- | ------- | ----------- |
| `robot_name` | (required) | Robot name — must match a `record_settings_<robot_name>.yaml` config file |
| `record_directory` | `<package_share>/rosbags` | Absolute path where rosbag episodes are saved |

#### Gamepad Controls

| Button | Action |
| ------ | ------ |
| Record/Pause | Start recording / Pause / Resume |
| Save/Delete | Save current episode / Delete last saved episode (if not recording) |

Button mappings are configured in [gamepad_settings.yaml](./sobits_vla_rosbag_collection/config/gamepad_settings.yaml).
Supported controller profiles currently include `quest`, `dualshock4`, and `keyboard`.

#### Recording Quality Monitors

The collection node monitors data quality in real-time during recording:

| Monitor | Description |
| ------- | ----------- |
| **FPS monitoring** | Warns if camera publish rate drops below configured threshold |
| **Disk space** | Warns when free disk space falls below threshold; stops recording at critical level |
| **Minimum episode duration** | Rejects episodes shorter than configured duration |
| **Timestamp jumps** | Detects ROS clock discontinuities vs. wall time |
| **Bag integrity** | Verifies bag file is readable and non-empty after save |
| **Config consistency** | On resume, validates current config matches existing `recorded_bags_meta.yaml` |

#### Configuration

Robot-specific config: `config/record_settings_<robot_name>.yaml`

For this repository, the main presets are:
- `config/record_settings_sobit_home.yaml`
- `config/record_settings_sobit_light.yaml`

| Group | Key Parameters |
| ----- | -------------- |
| Robot morphology | `parts`, `joint_names`, `is_actionable`, `joint_states_topic` |
| Sensors | Camera topics, LiDAR, IMU |
| Recording | `topics_to_record`, compression format/mode |
| Monitoring | `expected_sensor_fps`, `min_disk_space_warning_gb`, `min_episode_duration` |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### 2. Dataset Conversion

**Package:** [sobits_vla_rosbag_conversion](./sobits_vla_rosbag_conversion/)

Converts raw rosbag recordings into [LeRobot](https://github.com/huggingface/lerobot) dataset format with time-synchronized multi-modal frames.

#### Launch

```bash
ros2 launch sobits_vla_rosbag_conversion rosbag_conversion.launch.py \
  config_file:=conversion_settings.yaml \
  rosbag_directory:=/path/to/rosbags \
  dataset_name:=MyDataset
```

| Argument | Default | Description |
| -------- | ------- | ----------- |
| `config_file` | `conversion_settings.yaml` | Conversion config file (switch per robot profile) |
| `rosbag_directory` | (from collection package) | Path to recorded rosbag episodes |
| `recorded_bags_meta_file` | `<rosbag_directory>/recorded_bags_meta.yaml` | Metadata file from collection |
| `dataset_name` | (from config) | Output dataset name |

#### Key Features

- **Frame synchronization**: Aligns camera, joint state, and command data using a primary camera trigger with configurable sync threshold
- **Downsampling**: Configurable target FPS — bags that cannot deliver the requested rate are skipped
- **Delta actions**: Computes `action.delta` (commanded - measured position) per frame
- **End-effector pose**: Optional 6-DOF pose extraction via TF tree
- **Static frame filtering**: Optionally skips frames where joints are not moving
- **Conversion stats**: Generates a YAML report with per-episode quality metrics, skipped bags, and reasons

#### Configuration

Config file: [conversion_settings.yaml](./sobits_vla_rosbag_conversion/config/conversion_settings.yaml)

Robot-specific preset example: [conversion_settings_sobit_home.yaml](./sobits_vla_rosbag_conversion/config/conversion_settings_sobit_home.yaml)

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `fps` | `10` | Target dataset frame rate |
| `sync_threshold` | `0.1` | Max temporal gap (seconds) between synced sensors |
| `primary_camera` | `head_camera` | Camera used as sync trigger |
| `cameras` | `head_camera, hand_left_camera, hand_right_camera` | Cameras included in the dataset |
| `ee_pose.enabled` | `false` | Enable end-effector pose extraction |
| `skip_static_threshold` | `0.0` | Joint movement threshold for static frame filtering (0 = disabled) |
| `push_to_hub` | `false` | Push resulting dataset to HuggingFace Hub |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### 3. Training

**Package:** [sobits_vla_training](./sobits_vla_training/)

> [!NOTE]
> TBD — Training utilities are under development.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### 4. Deployment

**Package:** [sobits_vla_deploy](./sobits_vla_deploy/)

Runs real-time VLA inference on the robot with async chunk execution and RTC-enabled chunk smoothing when supported by the policy.

#### Run

```bash
ros2 run sobits_vla_deploy sobits_vla_deploy.py --ros-args \
  --params-file $(ros2 pkg prefix sobits_vla_deploy)/share/sobits_vla_deploy/config/robot_config.yaml
```

To use a robot-specific setup, pass a `robot_config_<robot_name>.yaml` file (for example, `robot_config_sobit_home.yaml`).

#### Launch

```bash
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py
```

With a robot-specific config:

```bash
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py \
  config_file:=$(ros2 pkg prefix sobits_vla_deploy)/share/sobits_vla_deploy/config/robot_config_sobit_home.yaml
```

#### Config layout

- Generic template: [robot_config.yaml](./sobits_vla_deploy/config/robot_config.yaml)
- Robot-specific preset example: [robot_config_sobit_home.yaml](./sobits_vla_deploy/config/robot_config_sobit_home.yaml)

The deploy node uses a flat robot config under `robot` (selected by `robot.name`) and supports:
- joint state and odom topics
- multiple joint trajectory controller groups
- optional mobile base command features
- camera topics and encodings

#### Multi-controller gamepad mapping

Deploy now supports controller-specific mappings:

```yaml
gamepad:
  topic: /joy
  name: quest
  controllers: [quest, dualshock4]
  quest:
    button_mapping:
      play: 4
      stop: 5
  dualshock4:
    button_mapping:
      play: 7
      stop: 6
```

This allows multiple controllers to trigger play/stop in the same runtime.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### 5. Visualization

**Package:** [sobits_vla_visualization](./sobits_vla_visualization/)

> [!NOTE]
> TBD — Visualization tools are under development.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- WORKFLOW -->
## Workflow

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  1. Collection   │────▶│  2. Conversion   │────▶│  3. Training     │────▶│  4. Deployment   │
│  (rosbag_        │     │  (rosbag_        │     │  (training)      │     │  (deploy)        │
│   collection)    │     │   conversion)    │     │                  │     │                  │
│                  │     │                  │     │                  │     │                  │
│  Gamepad-driven  │     │  Rosbags →      │     │  Fine-tune VLA   │     │  Real-time       │
│  episode         │     │  LeRobot dataset │     │  model           │     │  inference on    │
│  recording       │     │  with sync &     │     │                  │     │  robot           │
│                  │     │  quality stats   │     │                  │     │                  │
└─────────────────┘     └─────────────────┘     └─────────────────┘     └─────────────────┘
```

1. **Collect** demonstration data by teleoperating the robot with a gamepad
2. **Convert** recorded rosbags into a LeRobot-compatible dataset
3. **Train** a VLA model on the collected dataset
4. **Deploy** the trained model for autonomous robot control

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

- [LeRobot](https://github.com/huggingface/lerobot) — Dataset format and training framework
<!-- - [SmolVLA](https://huggingface.co/HuggingFaceTB/SmolVLA-256) — VLA model architecture -->
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — Robot middleware

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobits_vla_tools/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobits_vla_tools/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobits_vla_tools/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobits_vla_tools/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[license-url]: LICENSE
