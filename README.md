<a name="readme-top"></a>

[JA](README_ja.md) | [EN](README.md)

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
| `sobits_vla_common` | Shared library: robot descriptor schema/loader, policy registry, lerobot 0.6.0 compat patches, `new_robot` scaffolder, and the `GamepadClient` node |
| `sobits_vla_rosbag_collection` | Gamepad-triggered multi-modal rosbag recording with live quality monitoring |
| `sobits_vla_rosbag_conversion` | Converts rosbags into [LeRobot](https://github.com/huggingface/lerobot) dataset format |
| `sobits_vla_training` | Trains/fine-tunes VLA policies (pi05, pi0, pi0_fast, smolvla, ACT, GR00T) via lerobot |
| `sobits_vla_deploy` | Real-time VLA inference node for robot control (async chunking + RTC) |
| `sobits_vla_visualization` | Dataset and inference visualization (TBD) |

All four pipeline stages read robot morphology from a single **robot descriptor** (`sobits_vla_common/robots/<robot_id>.robot.yaml`) — the one source of truth for joint groups, command topics, sensors, and mobile base. See [Robot Descriptor](#robot-descriptor) below.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ROBOT DESCRIPTOR -->
## Robot Descriptor

A robot descriptor (`sobits_vla_common/robots/<robot_id>.robot.yaml`) is the single source of truth for a robot's morphology. Collection (C++), conversion, training, and deploy all load it instead of duplicating joint/topic lists across per-stage configs.

It defines:
- `groups` — joint groups, each with `command_topic`, `command_action`, `max_joint_delta`, `active`, and ordered `joints` (`ros_name` → dataset `feature`)
- `mobile_base` — optional `cmd_vel`/`odom` interface with `has_vel_*` / `max_vel_*` / `features`
- `sensors.cameras` — name, compressed/raw/info topics, encoding, active flag
- `ee_poses` — optional TF end-effector poses
- `excluded_joints` — mimic/wheel/passive joints to drop from feature vectors

Each pipeline config references it by `descriptor_id` (deploy/training) or `robot_descriptor_id` (collection/conversion), then trims a subset via `robot.exclude.groups` / `exclude.cameras` / `exclude.mobile_base` (unknown names raise). Adding a new robot + N policies = **1 descriptor + N model-only configs** instead of editing every stage.

### Scaffold a new robot

```bash
ros2 run sobits_vla_common new_robot \
  --robot_id sobit_mini --dof 7 --cameras head,hand_left --mobile_base diff \
  --gen_collection_config
```

Generates a commented `<robot_id>.robot.yaml` (and optional collection config) with `# TODO:` markers on every topic/joint field. Validate after editing:

```bash
ros2 run sobits_vla_common new_robot --robot_id sobit_mini --validate_only
```

Validation fails (exit 1) while `# TODO` placeholders or unresolved `arm_joint<N>` names remain.

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
| `robot_name` | (required) | Robot name — must match a `collection_config_<robot_name>.yaml` config file |
| `record_directory` | `<package_share>/rosbags` | Absolute path where rosbag episodes are saved |

#### Gamepad Controls

| Button | Action |
| ------ | ------ |
| Record/Pause | Start recording / Pause / Resume |
| Save/Delete | Save current episode / Delete last saved episode (if not recording) |

Button mappings are configured in [gamepad_config.yaml](./sobits_vla_rosbag_collection/config/gamepad_config.yaml).
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

Robot-specific config: `config/collection_config_<robot_name>.yaml`

For this repository, the main presets are:
- `config/collection_config_sobit_home.yaml`
- `config/collection_config_sobit_light.yaml`

Morphology (joint groups, command topics, sensors, mobile base) is **not** here — it is loaded from the [robot descriptor](#robot-descriptor) via `robot_descriptor_id`. The collection config only holds recording params:

| Group | Key Parameters |
| ----- | -------------- |
| Descriptor | `robot_descriptor_id` (selects `<id>.robot.yaml`) |
| User info | `user_info.name` / `location` / `email` |
| Recording | `additional_topics`, `conversion_format`, compression format/mode |
| Monitoring | `expected_sensor_fps`, `min_disk_space_mb`, `min_episode_duration`, `max_episode_duration`, `timestamp_jump_threshold` |

#### Launching SOBIT HOME

Launch the robot **before** starting the teleop node or the Quest app.

**Real robot:**

```bash
ros2 launch sobit_home_bringup real_minimal.launch.py \
  enable_teleop:=true
```

**Simulation (Gazebo):**

```bash
ros2 launch sobit_home_bringup gz_minimal.launch.py \
  world_model:=simple_data_collection \
  enable_teleop:=true
```

Available `world_model` values: `empty`, `wrs`, `small_house`, `rcjo2025_arena`, `rcjo2026_arena`, `simple_data_collection`.

**Teleop node (Quest, real robot):**

```bash
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=quest \
  use_moveit:=true \
  ros_ip:=127.0.0.1
```

**Teleop node (Quest, simulation):**

```bash
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=quest \
  use_moveit:=true \
  ros_ip:=127.0.0.1 \
  use_sim_time:=true
```

#### Collecting Data with SOBIT HOME and Meta Quest

SOBIT HOME teleoperation uses the Meta Quest headset via [sobits_teleop](https://github.com/TeamSOBITS/sobits_teleop). The Quest app communicates with the PC over TCP port 10000. Two connection methods are supported:

| Method | When to use |
| ------ | ----------- |
| **Wired (ADB)** | Quest connected to PC via USB cable — most reliable, no network required |
| **Wireless (Wi-Fi)** | Cable-free operation; Quest and PC must be on the same network |

---

##### Option A — Wired connection (ADB)

The launch file automatically runs `adb reverse tcp:10000 tcp:10000`, which tunnels the app's connection through USB. No IP configuration is needed on the Quest side.

1. Turn on the Quest by pressing the button on the **left side** of the headset.
2. Connect the Quest to the PC with a USB cable and run the following command **once** in the computer. (You should have runned the installer for sobits_teleop already).
   ```bash
   sudo adb kill-server
   sudo adb start-server
   ```
3. Accept the **"Allow USB debugging"** prompt inside the headset by clicking on the **"Allow always from this computer"**
4. Verify the device is detected:
   ```bash
   adb devices
   ```
5. Launch the teleop node on the PC:
   ```bash
   ros2 launch sobits_teleop sobits_teleop.launch.py \
     robot_name:=sobit_home \
     device:=quest \
     use_moveit:=true \
     ros_ip:=127.0.0.1
   ```
6. Inside the headset, open the library window (**Meta button**, right controller), go to **Menu->Unknown Sources**, and launch the **Quest Teleoperation** app.
7. Press the **three-lines button on the left controller** to open settings, set the IP to `127.0.0.1`, and press **OK** — the robot's camera feeds should appear.

---

##### Option B — Wireless connection (Wi-Fi)

Quest and PC must be on the same Wi-Fi network.

**On the Quest headset:**

1. Turn on the Quest by pressing the button on the **left side** of the headset.
2. Using the **right controller**, press the **Meta button** to open the library window.
3. Open **Quick Controls** (the icon with two dots and three lines) and press **Wi-Fi**.
4. Select your network, enter the password, and wait until **"Connected"** appears.
5. Press the back arrow, then **Done** to finish.

**On the PC:**

7. Launch the teleop node, passing your PC's IP on the shared network:
   ```bash
   ros2 launch sobits_teleop sobits_teleop.launch.py \
     robot_name:=sobit_home \
     device:=quest \
     use_moveit:=true \
     ros_ip:=<PC_IP_ADDRESS>
   ```
   Replace `<PC_IP_ADDRESS>` with your PC's IP (e.g. `192.168.11.10`).

**Back on the Quest:**

8. From the library window, go to **Menu->Unknown Sources** and launch the **Quest Teleoperation** app.
9. Press the **three-lines button on the left controller**, enter your PC's IP address, and press **OK** — the robot's camera feeds should appear.

---

##### Teleop launch arguments

| Argument | Default | Description |
| -------- | ------- | ----------- |
| `robot_name` | `sobit_home` | Robot configuration profile to load |
| `device` | `quest` | Input device — Quest controller |
| `ros_ip` | `127.0.0.1` | PC IP the Quest app connects to (`127.0.0.1` for wired ADB). Check your your PC's IP if wireless. |
| `use_moveit` | `true` | Set `true` to enable MoveIt-based arm control via Quest |

> [!TIP]
> Launch the robot bringup on the PC **before** opening the Quest app, so camera topics are already available when the app connects.

##### Set the task name

The collection node organises episodes into per-task directories. Before recording, set the task name via the `/vla_task_update` service:

```bash
ros2 service call /rosbag_collection/vla_task_update sobits_interfaces/srv/VlaUpdateTask "{label: 'pick up the bottle'}"
```

Changing the task name mid-session will save subsequent episodes into a new directory.

##### Launch the rosbag collection node and record episodes

In a separate terminal, launch the rosbag collection node:

```bash
ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py \
  robot_name:=sobit_home \
  record_directory:=/path/to/rosbags
```

Once the node is running, use the Quest controllers to manage episodes:

| Button | State | Action |
| ------ | ----- | ------ |
| **A button** (right controller) | Idle | Start recording |
| **A button** (right controller) | Recording | Pause / Resume |
| **B button** (right controller) | Recording | Save episode and stop |
| **B button** (right controller) | Idle | Delete the last saved episode |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### 2. Dataset Conversion

**Package:** [sobits_vla_rosbag_conversion](./sobits_vla_rosbag_conversion/)

Converts raw rosbag recordings into [LeRobot](https://github.com/huggingface/lerobot) dataset format with time-synchronized multi-modal frames.

#### Launch

```bash
ros2 launch sobits_vla_rosbag_conversion rosbag_conversion.launch.py \
  config_file:=conversion_config.yaml \
  rosbag_directory:=/path/to/rosbags \
  dataset_name:=MyDataset
```

| Argument | Default | Description |
| -------- | ------- | ----------- |
| `config_file` | `conversion_config.yaml` | Conversion config file (switch per robot profile) |
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

Config file: [conversion_config.yaml](./sobits_vla_rosbag_conversion/config/conversion_config.yaml)

Robot-specific preset example: [conversion_config_sobit_home.yaml](./sobits_vla_rosbag_conversion/config/conversion_config_sobit_home.yaml)

Set `robot_descriptor_id` to drive `excluded_joints` and camera selection from the [robot descriptor](#robot-descriptor); otherwise the legacy inline `excluded_joints` / `cameras` keys are used.

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `robot_descriptor_id` | `""` | Selects `<id>.robot.yaml` for joint/camera selection (empty = use inline keys) |
| `fps` | `10` | Target dataset frame rate |
| `sync_threshold` | `0.1` | Max temporal gap (seconds) between synced sensors |
| `downsample_tolerance` | `0.015` | Tolerance margin (seconds) to accept frames arriving early due to scheduling jitter |
| `primary_camera` | `head_camera` | Camera used as sync trigger |
| `cameras` | `head_camera, hand_left_camera, hand_right_camera` | Cameras included in the dataset |
| `ee_pose.enabled` | `false` | Enable end-effector pose extraction |
| `skip_static_threshold` | `0.0` | Joint movement threshold for static frame filtering (0 = disabled) |
| `push_to_hub` | `false` | Push resulting dataset to HuggingFace Hub |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### 3. Training

**Package:** [sobits_vla_training](./sobits_vla_training/)

Trains/fine-tunes a VLA policy on a LeRobot dataset via lerobot 0.6.0. Supported policies: `pi05`, `pi0`, `pi0_fast`, `smolvla`, `act`, `groot`. PEFT/LoRA, Hub push, and W&B logging are configured per YAML.

#### Launch

```bash
ros2 launch sobits_vla_training sobits_vla_training.launch.py robot:=sobit_home_left_pi05
```

`robot:=<name>` selects `training_config_<name>.yaml` from the package `config/`.

#### Config layout

Each `training_config_*.yaml` carries:
- `policy` — policy type (one of the supported six)
- `robot` — `descriptor_id` + `exclude.groups` / `exclude.cameras` / `exclude.mobile_base`; the trainer derives `max_state_dim` / `max_action_dim` from the descriptor's active joints + mobile-base features (so they need not be hand-set)
- `dataset` / `training` / `checkpoint` / `wandb` / `hub` — standard lerobot knobs
- `peft` — LoRA method/targets (empty `method_type` = full fine-tune)
- `policy_overrides` — any field of the policy's lerobot config (introspected; unknown keys warn)

> [!NOTE]
> GR00T (`groot`) keeps explicit `max_state_dim: 64` / `max_action_dim: 32` and uses its own `tune_*` freezing flags instead of lerobot PEFT. Requires `flash-attn`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### 4. Deployment

**Package:** [sobits_vla_deploy](./sobits_vla_deploy/)

Runs real-time VLA inference on the robot with async chunk execution and RTC-enabled chunk smoothing when supported by the policy.

#### Run

```bash
ros2 run sobits_vla_deploy sobits_vla_deploy.py --ros-args \
  --params-file $(ros2 pkg prefix sobits_vla_deploy)/share/sobits_vla_deploy/config/deploy_config.yaml
```

To use a robot-specific setup, pass a `deploy_config_<robot_name>.yaml` file (for example, `deploy_config_sobit_home.yaml`).

#### Launch

```bash
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py
```

With a robot-specific config:

```bash
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py \
  config_file:=$(ros2 pkg prefix sobits_vla_deploy)/share/sobits_vla_deploy/config/deploy_config_sobit_home.yaml
```

#### Config layout

- Generic template (legacy inline format): [deploy_config.yaml](./sobits_vla_deploy/config/deploy_config.yaml)
- Robot-specific preset: [deploy_config_sobit_home_left.yaml](./sobits_vla_deploy/config/deploy_config_sobit_home_left.yaml)

The deploy node reads morphology from the **robot descriptor** and selects a subset per task:

```yaml
robot:
  descriptor_id: sobit_home              # loads sobits_vla_common/robots/sobit_home.robot.yaml
  exclude:
    groups: [arm_right, hand_right]
    cameras: [hand_right_camera]
    mobile_base: false
```

Command topics, joints, `max_joint_delta`, mobile-base, and camera topics all come from the descriptor — no inline joint/topic lists. (If `descriptor_id` is empty, the node falls back to the legacy inline `robot.*` schema shown in `deploy_config.yaml`.) The `model` / `runtime` / `rtc` sections stay in the deploy config.

#### Gamepad play/stop

Play/stop is driven by the shared `GamepadClient` node (`sobits_vla_common`), which calls the deploy node's `VlaCommand` **service** (no direct `/joy` subscription). Button mappings and the service name live in `sobits_vla_common/config/gamepad_config.yaml`:

```yaml
gamepad:
  command_service: "vla/deploy_command"  # relative; resolves under the node's namespace
  controller: quest
  quest:
    button_mapping: { play: 4, stop: 4 }
  dualshock4:
    button_mapping: { play: 7, stop: 7 }
```

Bus names are relative (`vla/...`), so a namespaced launch resolves them to `/<robot_name>/vla/...`;
a bare `ros2 run` with no namespace keeps the old `/vla/...` form. `vla/play` (Bool) and `vla/task`
(String) topics remain available for programmatic control (e.g. `/sobit_home/vla/play` when namespaced).

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


<!-- TODO -->
## TODO

### Full support for `relative_exclude_joints`

The `sobits_vla_rosbag_conversion` and `sobits_vla_deploy` packages do not yet explicitly handle the `relative_exclude_joints` parameter.

**Current state and reasoning:**

- **Conversion package (`sobits_vla_rosbag_conversion`):** The relative action conversion (delta subtraction) is applied only to joint positions. Mobile base velocities (`base_x`, `base_y`, `base_theta`) are appended to the action vector after the delta step, so they are excluded implicitly by code structure. This is sufficient for the current SOBIT HOME configuration.
- **Deploy package (`sobits_vla_deploy`):** The postprocessor (`absolute_actions_processor`) uses the mask saved in `policy_postprocessor.json` loaded from the model repository, which correctly reflects the exclusions from training time. The manual delta fallback path hardcodes exclusion of base keys only and does not read `relative_exclude_joints` from the policy config.

**Future work:**

Different robot morphologies may require different exclusion sets — for example, velocity-controlled wheels, binary gripper joints, or passive joints that should never be delta-converted. The following should be implemented when supporting new morphologies:

- `sobits_vla_rosbag_conversion`: Add a `relative_exclude_joints` parameter to the YAML config so that joints to skip during delta conversion can be specified explicitly rather than relying on code structure.
- `sobits_vla_deploy`: Update the manual delta fallback path to read `relative_exclude_joints` from the loaded policy config, so that deployment without a postprocessor file still applies the correct exclusions.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

- [LeRobot](https://github.com/huggingface/lerobot) — Dataset format and training framework
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
