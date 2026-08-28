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

#ifndef SOBITS_VLA_ROSBAG_COLLECTION__ROSBAG_COLLECTION_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__ROSBAG_COLLECTION_HPP_

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sobits_interfaces/srv/vla_command.hpp>
#include <sobits_interfaces/srv/vla_reset_world.hpp>
#include <sobits_interfaces/srv/vla_update_task.hpp>

#include "rosbag2_storage/storage_options.hpp"
#include "sobits_vla_rosbag_collection/episode_lifecycle.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/recorder.hpp"

namespace sobits_vla
{
class RecordingMonitor;
class BagMetadataManager;
class EpisodeLifecycle;

class RobotInfo
{
public:
  std::string name;
  std::string version;
  std::string morphology;
  std::vector<std::string> parts;
  std::map<std::string, bool> is_actionable;
  std::map<std::string, std::string> part_command_topic;  // joint_trajectory topic per part
  std::map<std::string, std::string> part_state_topic;    // controller_state topic per part
  std::map<std::string, std::vector<std::string>> part_actions;
  std::map<std::string, std::vector<std::string>> joint_names;

  // Custom properties for specific parts like mobile_base/legs
  std::map<std::string, bool> part_has_cmd_vel_y;
  std::map<std::string, bool> part_has_cmd_vel_z;
  std::map<std::string, std::string> part_cmd_vel_topic;
  std::map<std::string, std::string> part_odom_topic;
  std::string joint_states_topic;

  std::vector<std::string> sensor_types;
  std::map<std::string, std::vector<std::string>> sensor_names;
  std::map<std::string, std::vector<std::string>> sensor_models;
  std::map<std::string, std::vector<std::string>> sensor_topics;
  // Explicit camera_info topics
  std::map<std::string, std::vector<std::string>> sensor_info_topics;
  // Explicit compressed image topics
  std::map<std::string, std::vector<std::string>> sensor_compressed_topics;
};

class UserInfo
{
public:
  std::string name;
  std::string email;
  std::string location;
};

class SubtaskInfo
{
public:
  std::string key;
  std::string label;
  double start_timestamp;
  double end_timestamp;
};

class RosbagInfo
{
public:
  std::string recording_dir;
  std::vector<std::string> topics_to_record;
  std::vector<std::string> additional_topics;
  std::vector<std::string> additional_services;
  std::vector<std::string> additional_actions;
  // uint8_t recording_duration;
  std::string conversion_format;
  bool compress_output;
  std::string compression_format;
  std::string compression_mode;
  std::string rmw_serialization_format;
  std::string storage_config_file;
  std::string rosbag_options;
};

class RosbagCollection : public rclcpp::Node
{
public:
  explicit RosbagCollection(const rclcpp::NodeOptions & options);
  ~RosbagCollection();

  void createRosbag();
  void removeRosbag();
  bool saveRosbag();  // returns false if bag was discarded (too short, integrity fail)

  void createRosbagYaml();
  void updateRosbagYaml();
  void updateEpisodeYaml();
  void removeEpisodeFromYaml();
  void buildTopicList();
  bool validateTopics();
  void startRecordingMonitor();
  void stopRecordingMonitor();
  bool verifyBagIntegrity(const std::string & bag_path);
  std::string getTimestampString();

private:
  // Implemented in rosbag_collection_params.cpp: pure move of the 41
  // declare_parameter/get_parameter calls out of the constructor.
  void declareAndReadParameters();

  void taskUpdateCallback(
    const std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Request> request,
    std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Response> response);

  void subtaskUpdateCallback(
    const std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Request> request,
    std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Response> response);

  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg,
    const std::string topic_name);

  rclcpp::Service<sobits_interfaces::srv::VlaUpdateTask>::SharedPtr task_update_service_;
  rclcpp::Service<sobits_interfaces::srv::VlaUpdateTask>::SharedPtr subtask_update_service_;
  rclcpp::Service<sobits_interfaces::srv::VlaCommand>::SharedPtr command_service_;

  void handleVlaCommand(
    const std::shared_ptr<sobits_interfaces::srv::VlaCommand::Request> request,
    std::shared_ptr<sobits_interfaces::srv::VlaCommand::Response> response);

  void requestWorldReset();

  rclcpp::Client<sobits_interfaces::srv::VlaResetWorld>::SharedPtr world_reset_client_;
  std::string world_reset_service_;
  // Empty -> reset node's world_reset.active_preset picks the scene.
  std::string world_reset_preset_;

  std::unique_ptr<RecordingMonitor> recording_monitor_;
  std::unique_ptr<BagMetadataManager> bag_metadata_manager_;
  std::unique_ptr<EpisodeLifecycle> episode_lifecycle_;

  std::shared_ptr<rosbag2_transport::Recorder> recorder_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> recorder_executor_;
  std::thread recorder_thread_;
  std::thread auto_save_thread_;  // joinable; joined before teardown, see destructor
  std::mutex recorder_mutex_;
  std::atomic<bool> is_recording_{false};
  std::chrono::steady_clock::time_point recording_start_time_;
  double min_episode_duration_sec_{0.0};  // 0 = disabled
  double max_episode_duration_sec_{0.0};  // 0 = disabled
  bool task_has_been_set_{false};

  std::map<std::string,
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
  std::map<std::string, std::pair<uint32_t, uint32_t>> camera_dimensions_;

  // Recording health monitoring. The per-topic counters, FPS timer and
  // timestamp-drift state live in RecordingMonitor, not here.
  int expected_sensor_fps_{0};
  uint64_t min_disk_space_mb_{2048};  // minimum free disk space in MB (default 2GB)
  std::atomic<bool> max_duration_triggered_{false};  // prevents repeated auto-save
  // Prevent use-after-free
  std::shared_ptr<std::atomic<bool>> node_alive_ = std::make_shared<std::atomic<bool>>(true);
  // Max drift between ROS clock and wall clock per check
  double timestamp_jump_threshold_sec_{1.0};

  // Parameters
  RobotInfo robot_info_;
  UserInfo user_info_;
  RosbagInfo rosbag_info_;
  std::string gamepad_name_;
  // Read alongside gamepad_name_ but used later in the constructor, after
  // declareAndReadParameters() returns -- kept as a member for that reason.
  std::string command_service_name_;

  // State management
  uint8_t current_state_;
  uint8_t previous_state_;

  std::string current_task_name_;
  std::string current_task_dir_name_;
  std::string previous_task_name_;
  std::string current_task_path_;

  std::string current_subtask_name_;
  std::vector<SubtaskInfo> current_episode_subtasks_;

  std::string current_bag_name_;
  std::string previous_bag_name_;
  std::string current_bag_path_;
  std::string previous_bag_path_;
};

}  // namespace sobits_vla

#endif  // SOBITS_VLA_ROSBAG_COLLECTION__ROSBAG_COLLECTION_HPP_
