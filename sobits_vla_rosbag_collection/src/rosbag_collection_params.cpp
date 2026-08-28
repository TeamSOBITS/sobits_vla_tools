// Copyright (c) 2026, Team SOBITS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include <string>
#include <vector>

#include "sobits_vla_rosbag_collection/robot_descriptor_loader.hpp"
#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

namespace sobits_vla
{

// Pure move of the constructor's 41 declare_parameter/get_parameter calls;
// same names, defaults, order, and dynamic per-part loop as before.
void RosbagCollection::declareAndReadParameters()
{
  // Declare and get parameters
  this->declare_parameter<std::string>("robot_descriptor_id", "");
  std::string robot_descriptor_id = this->get_parameter("robot_descriptor_id").as_string();

  if (!robot_descriptor_id.empty()) {
    RCLCPP_INFO(this->get_logger(), "Loading robot descriptor: %s", robot_descriptor_id.c_str());
    try {
      RobotDescriptorCpp desc = loadRobotDescriptor(robot_descriptor_id);
      robot_info_ = toRobotInfo(desc);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load robot descriptor: %s", e.what());
      throw;
    }
  } else {
    RCLCPP_INFO(this->get_logger(),
        "No robot_descriptor_id provided, loading morphology from legacy parameters...");
    // (1) Robot info parameters
    this->declare_parameter<std::string>("robot_info.name", "sobit_robot");
    this->declare_parameter<std::string>("robot_info.version", "1.0.0");
    this->declare_parameter<std::string>("robot_info.morphology.type", "mobile_manipulator");
    this->declare_parameter<std::string>("robot_info.morphology.joint_states_topic",
        "/joint_states");
    this->declare_parameter<std::vector<std::string>>("robot_info.morphology.parts",
        std::vector<std::string>{"base", "arm", "gripper"});

    robot_info_.name = this->get_parameter("robot_info.name").as_string();
    robot_info_.version = this->get_parameter("robot_info.version").as_string();
    robot_info_.morphology = this->get_parameter("robot_info.morphology.type").as_string();
    robot_info_.joint_states_topic =
      this->get_parameter("robot_info.morphology.joint_states_topic").as_string();

    robot_info_.parts = this->get_parameter("robot_info.morphology.parts").as_string_array();
    robot_info_.joint_names.clear();
    for (const auto & part : robot_info_.parts) {
      RCLCPP_INFO(this->get_logger(), "Robot part: %s", part.c_str());
      this->declare_parameter<bool>("robot_info.morphology." + part + ".is_actionable", false);
      this->declare_parameter<std::string>("robot_info.morphology." + part + ".command_topic", "");
      this->declare_parameter<std::string>("robot_info.morphology." + part + ".state_topic", "");
      this->declare_parameter<std::vector<std::string>>("robot_info.morphology." + part +
          ".actions",
          std::vector<std::string>{});
      this->declare_parameter<std::vector<std::string>>("robot_info.morphology." + part +
          ".joint_names", std::vector<std::string>{});
      robot_info_.is_actionable[part] = this->get_parameter("robot_info.morphology." + part +
          ".is_actionable").as_bool();
      robot_info_.part_command_topic[part] = this->get_parameter("robot_info.morphology." + part +
          ".command_topic").as_string();
      robot_info_.part_state_topic[part] = this->get_parameter("robot_info.morphology." + part +
          ".state_topic").as_string();
      robot_info_.part_actions[part] = this->get_parameter("robot_info.morphology." + part +
          ".actions").as_string_array();
      robot_info_.joint_names[part] = this->get_parameter("robot_info.morphology." + part +
          ".joint_names").as_string_array();

      // A part is a base if it declares a cmd_vel_topic, whatever it is named.
      this->declare_parameter<bool>("robot_info.morphology." + part + ".has_cmd_vel_y", false);
      this->declare_parameter<bool>("robot_info.morphology." + part + ".has_cmd_vel_z", false);
      this->declare_parameter<std::string>("robot_info.morphology." + part + ".cmd_vel_topic", "");
      this->declare_parameter<std::string>("robot_info.morphology." + part + ".odom_topic", "");

      const std::string cmd_vel_topic =
        this->get_parameter("robot_info.morphology." + part + ".cmd_vel_topic").as_string();
      if (!cmd_vel_topic.empty()) {
        robot_info_.part_cmd_vel_topic[part] = cmd_vel_topic;
        robot_info_.part_has_cmd_vel_y[part] = this->get_parameter("robot_info.morphology." + part +
            ".has_cmd_vel_y").as_bool();
        robot_info_.part_has_cmd_vel_z[part] = this->get_parameter("robot_info.morphology." + part +
            ".has_cmd_vel_z").as_bool();
        robot_info_.part_odom_topic[part] = this->get_parameter("robot_info.morphology." + part +
            ".odom_topic").as_string();
      }
    }
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors.types",
        std::vector<std::string>{"camera", "lidar", "imu"});
    robot_info_.sensor_types = this->get_parameter("robot_info.sensors.types").as_string_array();
    robot_info_.sensor_names.clear();
    robot_info_.sensor_models.clear();
    robot_info_.sensor_topics.clear();
    for (const auto & sensor_type : robot_info_.sensor_types) {
      RCLCPP_INFO(this->get_logger(), "Robot sensor: %s", sensor_type.c_str());
      this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type +
          ".names", std::vector<std::string>{});
      this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type +
          ".models", std::vector<std::string>{});
      this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type +
          ".topics", std::vector<std::string>{});
      this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type +
          ".info_topics", std::vector<std::string>{});
      this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type +
          ".compressed_topics", std::vector<std::string>{});
      robot_info_.sensor_names[sensor_type] = this->get_parameter("robot_info.sensors." +
          sensor_type + ".names").as_string_array();
      robot_info_.sensor_models[sensor_type] = this->get_parameter("robot_info.sensors." +
          sensor_type + ".models").as_string_array();
      robot_info_.sensor_topics[sensor_type] = this->get_parameter("robot_info.sensors." +
          sensor_type + ".topics").as_string_array();
      robot_info_.sensor_info_topics[sensor_type] = this->get_parameter("robot_info.sensors." +
          sensor_type + ".info_topics").as_string_array();
      robot_info_.sensor_compressed_topics[sensor_type] =
        this->get_parameter("robot_info.sensors." +
          sensor_type + ".compressed_topics").as_string_array();
    }
  }

  // (2) User info parameters
  this->declare_parameter<std::string>("user_info.name", "default_user");
  this->declare_parameter<std::string>("user_info.email", "default_user@example.com");
  this->declare_parameter<std::string>("user_info.location", "default_location");
  user_info_.name = this->get_parameter("user_info.name").as_string();
  user_info_.email = this->get_parameter("user_info.email").as_string();
  user_info_.location = this->get_parameter("user_info.location").as_string();

  // (3) Rosbag parameters
  this->declare_parameter<std::string>("rosbag_config.record_directory", "");
  this->declare_parameter<double>("rosbag_config.min_episode_duration", 1.0);
  this->declare_parameter<double>("rosbag_config.max_episode_duration", 0.0);
  this->declare_parameter<double>("rosbag_config.timestamp_jump_threshold", 1.0);
  this->declare_parameter<int>("rosbag_config.min_disk_space_mb", 2048);
  this->declare_parameter<int>("rosbag_config.expected_sensor_fps", 0);
  this->declare_parameter<std::vector<std::string>>("rosbag_config.additional_topics",
      std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.additional_services",
      std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.additional_actions",
      std::vector<std::string>{});
  this->declare_parameter<std::string>("rosbag_config.conversion_format", "mcap");
  this->declare_parameter<std::string>("rosbag_config.compression_format", "zstd");
  this->declare_parameter<std::string>("rosbag_config.compression_mode", "none");
  this->declare_parameter<std::string>("rosbag_config.rmw_serialization_format", "cdr");
  min_episode_duration_sec_ = this->get_parameter("rosbag_config.min_episode_duration").as_double();
  max_episode_duration_sec_ = this->get_parameter("rosbag_config.max_episode_duration").as_double();
  timestamp_jump_threshold_sec_ =
    this->get_parameter("rosbag_config.timestamp_jump_threshold").as_double();
  min_disk_space_mb_ =
    static_cast<uint64_t>(this->get_parameter("rosbag_config.min_disk_space_mb").as_int());
  expected_sensor_fps_ = this->get_parameter("rosbag_config.expected_sensor_fps").as_int();
  rosbag_info_.recording_dir = this->get_parameter("rosbag_config.record_directory").as_string();
  rosbag_info_.additional_topics =
    this->get_parameter("rosbag_config.additional_topics").as_string_array();
  rosbag_info_.additional_services =
    this->get_parameter("rosbag_config.additional_services").as_string_array();
  rosbag_info_.additional_actions =
    this->get_parameter("rosbag_config.additional_actions").as_string_array();
  rosbag_info_.conversion_format =
    this->get_parameter("rosbag_config.conversion_format").as_string();
  rosbag_info_.compression_format =
    this->get_parameter("rosbag_config.compression_format").as_string();
  rosbag_info_.compression_mode = this->get_parameter("rosbag_config.compression_mode").as_string();
  rosbag_info_.rmw_serialization_format =
    this->get_parameter("rosbag_config.rmw_serialization_format").as_string();

  // (4) Gamepad parameters
  this->declare_parameter<std::string>("gamepad.controller", "dualshock4");
  this->declare_parameter<std::string>("gamepad.command_service", "vla/collect_command");
  gamepad_name_ = this->get_parameter("gamepad.controller").as_string();
  command_service_name_ = this->get_parameter("gamepad.command_service").as_string();

  // (4b) World reset client -- RESET forwards to the shared world_reset_node.
  this->declare_parameter<std::string>(
    "rosbag_config.world_reset_service", "world_reset_node/reset_world");
  world_reset_service_ = this->get_parameter("rosbag_config.world_reset_service").as_string();
  // Empty defers to the reset node's world_reset.active_preset; set this only
  // to override which scene the RESET button restores.
  this->declare_parameter<std::string>("rosbag_config.world_reset_preset", "");
  world_reset_preset_ = this->get_parameter("rosbag_config.world_reset_preset").as_string();
  world_reset_client_ = this->create_client<sobits_interfaces::srv::VlaResetWorld>(
    world_reset_service_);
}

}  // namespace sobits_vla
