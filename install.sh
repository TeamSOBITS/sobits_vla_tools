#!/bin/bash

echo "╔══╣ Setup: SOBITS VLA Tools (STARTING) ╠══╗"


# Clone GitHub repository
if [ ! -d "$../sobits_interfaces" ]; then
    git clone https://github.com/TeamSOBITS/sobits_interfaces.git ../sobits_interfaces
    bash ../sobits_interfaces/install.sh
fi

# Download ROS packages
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-rosbag2-compression \
    ros-$ROS_DISTRO-rosbag2-cpp \
    ros-$ROS_DISTRO-rosbag2-interfaces \
    ros-$ROS_DISTRO-rosbag2-py \
    ros-$ROS_DISTRO-rosbag2-storage \
    ros-$ROS_DISTRO-rosbag2-storage-default-plugins \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-rosbag2-to-video \
    ros-$ROS_DISTRO-rosbag2-transport \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-linux \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-rcl-interfaces \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclcpp-action \
    ros-$ROS_DISTRO-rclcpp-components \
    ros-$ROS_DISTRO-yaml-cpp-vendor


echo "╚══╣ Setup: SOBITS VLA Tools (FINISHED) ╠══╝"
