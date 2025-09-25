<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS VLA Tools

<!-- TABLE OF CONTENTS -->
<details>
    <summary>Table of Contents</summary>
    <ol>
        <li><a href="#introduction">Introduction</a></li>
        <li>
            <a href="#getting-started">Getting Started</a>
            <ul>
                <li><a href="#prerequisites">Prerequisites</a></li>
                <li><a href="#installation">Installation</a></li>
            </ul>
        </li>
        <li><a href="#data-collection-setup">Data Collection Setup</a></li>
        <li><a href="#launch-and-usage">Launch and Usage</a></li>
        <li><a href="#acknowledgments">Acknowledgments</a></li>
    </ol>
</details>



> [!WARNING]
> This repository currently supports "data collection" only. In the future, we plan to provide a package that can convert Rosbags to the LeRobot dataset format.

<!-- INTRODUCTION -->
## Introduction

SOBITS VLA Tools is a repository designed to facilitate the usage of VLA models with SOBITS-developed robots and with the purpose to allow easier robot data collection to action inference through ROS.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

Here you will find out instructions on setting up this project locally.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System | Version |
| ------ | ------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill        |
| Python | ≥3.10                  |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to the `src` folder of ROS.
    ```sh
    $ cd ~/colcon_ws/src/
    ```
2. Clone this repository.
    ```sh
    $ git clone https://github.com/TeamSOBITS/sobits_vla_tools
    ```
3. Install the required dependencies.
    ```sh
    $ cd sobits_vla_tools
    $ bash install.sh
    ```
4. Compile the package.
    ```sh
    $ cd ~/colcon_ws
    $ rosdep update
    $ rosdep install --from-paths src -y --ignore-src
    $ colcon build
    $ source install/setup.bash
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Data Collection Setup

In order to start collecting data from our robot, we need to first setup the [record_settings.yaml](./sobits_vla_rosbag_collection/config/record_settings.yaml).

1. Make a copy of the [record_settings.yaml](./sobits_vla_rosbag_collection/config/record_settings.yaml) in the same folder and name it as `record_settings_{robot_name}.yaml`.

2. Once created the new yaml file, let's begin updating the metadata. Here, we collect **robot description information (required)** to understand which kind of robot you are using and **user data (optional)** to be able to contact with the user about the dataset.
```yaml
# Robot information
robot_info:
    # Name of the robot
    name: "robot_name"
    # Version of the robot
    version: "1.0.0"
    # Description of the robot
    morphology:
    # Type of the robot (e.g., "mobile_manipulator", "humanoid", "quadruped", etc.)
    type: "mobile_manipulator"
    parts:
        - head
        - manipulator
        - end_effector
        - mobile_base
        # - legs
    head:
        joint_names:
        - "head_pan"
        - "head_tilt"
    manipulator:
        joint_names:
        - "arm_joint_1"
        - "arm_joint_2"
        - "arm_joint_3"
        - "arm_joint_4"
        - "arm_joint_5"
        - "arm_joint_6"
    end_effector:
        joint_names:
        - "gripper"
    mobile_base:
        joint_names:
        - "base_wheel_left"
        - "base_wheel_right"
    # legs:
    #   joint_names:
    #     - ""
    # List of sensors on the robot
    sensors:
    types:
        - rgbd # color and depth camera
        - lidar # light detection and ranging sensor
        - imu # inertial measurement unit
        - camera # regular camera
        - depth_camera # depth camera without color
        - fish_eye # fisheye camera
    rgbd:
        names: [""]
        models: [""]
    lidar:
        names: [""]
        models: [""]
    imu:
        names: [""]
        models: [""]
    camera:
        names: [""]
        models: [""]
    depth_camera:
        names: [""]
        models: [""]
    fish_eye:
        names: [""]
        models: [""]

# User information for contact
user_info:
    # Name of the user
    name: ""
    # Location of the user
    location: ""
    # Contact email for the user
    email: ""
```

3. Next, let's decide which data to be collected as Rosbag and how to compress/convert the data.
```yaml
# ROS bag recording configuration
rosbag_config:
    # Path to the directory where the recorded bag files will be saved
    record_directory: "/path/to/recorded_bags"
    # List of topics to record (cameras, joint_states, tf, sensors, etc.)
    topics_to_record:
    - "/camera/image_raw"
    - "/camera/camera_info"
    # List of services to record (TODO: not available until ROS 2 Jazzy)
    services_to_record:
    - "/camera/get_image"
    # List of actions to record (TODO: not available until ROS 2 Kilted)
    actions_to_record:
    - "/robot/move_base"
    # Format for the recorded bag files (e.g., "mcap", "sqlite3")
    conversion_format: "mcap" 
    # Specify the compression mode ("none", "file", "message")
    compression_mode: "none"
    # If compression is enabled, specify the compression format ("zstd" is the only supported compression format)
    compression_format: "zstd"
```

4. (Not recommended) You can change the configuration for the controller. However, it is recommended not to be changed as it can overlap with the teleoperation configuration.
```yaml
# Control settings for the gamepad
gamepad_config:
    # Gamepad type (e.g., "dualshock4", "keyboard")
    name: "dualshock4"
    # Mapping of gamepad buttons to actions
    dualshock4:
        button_mapping:
            record: 7 # Top
            save  : 6 # Left
            delete: 6 # Right
    keyboard:
        button_mapping:
            record: "r"
            save  : "s"
            delete: "d"
```

> [!WARNING]
> Right now `dualshock4` is the only controller supported for data collection. More controllers will be available in the future.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Launch and Usage

Once already set up the yaml file based on the data that you want to collect, we are ready to launch the program.

1. Connect your controller (in this case we will use `DualShock4`) via USB or bluetooth.

2. Launch your robot.

3. Update the [rosbag_collection.launch.py](./sobits_vla_rosbag_collection/launch/rosbag_collection.launch.py) file with the name of your robot (the one you wrote for the yaml file).

```python
def generate_launch_description():
    robot_name = "sobit_light" # Change this to the desired robot name
```

4. Save the changes and launch it.
```bash
ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py
```

5. Lastly update the task name to start recording.
```bash
ros2 service call /{robot_name}/vla_task_update sobits_interfaces/srv/VlaUpdateTask "label: '{Name of the task}'" 
```

6. Once updated the task label, you can now start recording.
- **Up Button**: start recording
- **Left Button**: save the rosbag
- **Right Button**: delete the rosbag

7. Enjoy your data collection!

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [Rosbag2](https://github.com/ros2/rosbag2)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
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
