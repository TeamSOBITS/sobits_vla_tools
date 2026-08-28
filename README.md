<a name="readme-top"></a>

[JA](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS VLA Tools

SOBITS VLA Tools is a monorepo providing the full pipeline for controlling
SOBITS-developed robots with Vision-Language-Action (VLA) models — from data
collection through training to real-time deployment, all integrated via
ROS 2. See [CONTRIBUTING.md](CONTRIBUTING.md) for the conventions new code
must follow.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Packages

| Package | Purpose | README |
| ------- | ------- | ------ |
| `sobits_vla_common` | Shared hub: robot descriptors, param-schema loader, lerobot compat seam, gamepad client, world reset | [README](sobits_vla_common/README.md) |
| `sobits_vla_rosbag_collection` | Gamepad-triggered rosbag recording (C++) | [README](sobits_vla_rosbag_collection/README.md) |
| `sobits_vla_rosbag_conversion` | Rosbags → [LeRobot](https://github.com/huggingface/lerobot) dataset | [README](sobits_vla_rosbag_conversion/README.md) |
| `sobits_vla_training` | Trains/fine-tunes VLA policies via lerobot | [README](sobits_vla_training/README.md) |
| `sobits_vla_deploy` | Real-time VLA inference + offline eval | [README](sobits_vla_deploy/README.md) |
| `sobits_vla_visualization` | Reserved for future debug/viz nodes (empty skeleton) | [README](sobits_vla_visualization/README.md) |

All pipeline stages read robot morphology from one **robot descriptor**
(`sobits_vla_common/robots/<robot_id>.robot.yaml`) — the single source of
truth for joint groups, command topics, sensors, and mobile base. Scaffold a
new one with:

```bash
ros2 run sobits_vla_common new_robot \
  --robot_id sobit_mini --dof 7 --cameras head,hand_left --mobile_base diff \
  --gen_collection_config
ros2 run sobits_vla_common new_robot --robot_id sobit_mini --validate_only
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Pipeline overview

```
┌────────────┐   ┌────────────┐   ┌────────────┐   ┌────────────┐   ┌────────────┐
│ Collection │──▶│ Conversion │──▶│  Training  │──▶│   Deploy   │──▶│    Eval    │
│ (C++, gamepad│  │  rosbags → │   │ fine-tune  │   │  real-time │   │  offline   │
│  -driven    │  │ LeRobot    │   │  VLA policy│   │  inference │   │  analysis  │
│  recording) │  │  dataset   │   │            │   │  on robot  │   │  of logs   │
└────────────┘   └────────────┘   └────────────┘   └────────────┘   └────────────┘
```

1. **Collect** demonstrations by teleoperating the robot with a gamepad
   (`sobits_vla_rosbag_collection`).
2. **Convert** recorded rosbags into a LeRobot dataset
   (`sobits_vla_rosbag_conversion`).
3. **Train** a VLA policy on the dataset (`sobits_vla_training`).
4. **Deploy** the trained policy for autonomous control
   (`sobits_vla_deploy`).
5. **Evaluate** the resulting episode logs offline
   (`sobits_vla_deploy`'s `vla_eval`).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Namespace model

Every node advertises its own topics/services **privately**: `~/<channel>`,
which resolves to `/<node_name>/<channel>` bare or
`/<robot_name>/<node_name>/<channel>` under a namespaced launch. Other nodes
address it by the owner's relative name, `<owner_node>/<channel>` (e.g. the
gamepad client's `command_service` defaults to `sobits_vla_deploy/command`
or `vla_rosbag_collection/command`), which resolves alongside it under the
same namespace. Robot I/O topics (joint states, cameras, cmd_vel — from the
descriptor) stay absolute. A bare `ros2 run` with no namespace keeps
everything under `/`, so a single robot works with no namespace at all;
namespaced launches let multiple robots share one ROS domain.

## Output roots

Each package writes its generated artifacts (rosbags, datasets, checkpoints,
logs) under `<pkg_src>/<artifact-dir>/`, resolved by
`sobits_vla_common.output_root.output_root()` regardless of whether you're
running from a source tree, a colcon `--symlink-install`, or a regular
install. Every `<artifact-dir>/` carries a uniform `*\n!.gitignore\n`
ignore — the directory itself is tracked, its generated contents are not.
See each package's README for its specific output path.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Getting started

### Prerequisites

| System | Version |
| ------ | ------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Jazzy Jalisco           |
| Python | ≥3.10                  |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

### Installation

```sh
cd ~/colcon_ws/src/
git clone https://github.com/TeamSOBITS/sobits_vla_tools
cd sobits_vla_tools/
bash install.sh
cd ~/colcon_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

Non-ROS Python dependencies (numpy, pandas, torch, lerobot) live in
per-package `pixi` environments, not the system interpreter — see
`pixi.toml` at the repo root and each package README's "How to run" section
for the `enable_gpu:=`/`pixi_env:=` launch args that select them.

### Quickstart

```sh
# 1. Record a demonstration (gamepad-driven, real or sim robot)
ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py enable_world_reset:=false robot_name:=sobit_home

# 2. Convert the recorded rosbags into a LeRobot dataset
ros2 launch sobits_vla_rosbag_conversion rosbag_conversion.launch.py robot:=sobit_home

# 3. Train a policy on the dataset
ros2 launch sobits_vla_training sobits_vla_training.launch.py robot:=sobit_home_left_smolvla_fft steps:=30000

# 4. Deploy the trained policy
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py deploy_config:=deploy_config_sobit_home robot_name:=sobit_home
```

Every argument above is a real, verified launch arg — run any file with
`--show-args` for the full list.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

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
