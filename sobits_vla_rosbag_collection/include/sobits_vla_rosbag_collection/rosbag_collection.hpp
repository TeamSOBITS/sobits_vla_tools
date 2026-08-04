#ifndef SOBITS_VLA_ROSBAG_COLLECTION__ROSBAG_COLLECTION_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__ROSBAG_COLLECTION_HPP_

#include <rcl_interfaces/msg/parameter_type.hpp>
#include <sobits_interfaces/srv/vla_update_task.hpp>
#include <sobits_interfaces/srv/vla_command.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "rosbag2_transport/recorder.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/record_options.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

namespace sobits_vla
{
class RecordingMonitor;
class BagMetadataManager;

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
  std::map<std::string, std::vector<std::string>> sensor_info_topics;        // explicit camera_info topics
  std::map<std::string, std::vector<std::string>> sensor_compressed_topics;  // explicit compressed image topics
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

  std::unique_ptr<RecordingMonitor> recording_monitor_;
  std::unique_ptr<BagMetadataManager> bag_metadata_manager_;

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

  // Recording health monitoring
  int expected_sensor_fps_{0};
  uint64_t min_disk_space_mb_{2048};  // minimum free disk space in MB (default 2GB)
  std::vector<rclcpp::GenericSubscription::SharedPtr> monitor_subs_;
  std::map<std::string, std::atomic<uint64_t>> monitor_counts_;
  std::map<std::string, uint64_t> monitor_prev_counts_;
  rclcpp::TimerBase::SharedPtr fps_monitor_timer_;
  std::atomic<bool> max_duration_triggered_{false};  // prevents repeated auto-save
  std::shared_ptr<std::atomic<bool>> node_alive_ = std::make_shared<std::atomic<bool>>(true);  // prevent use-after-free
  bool fps_warmup_{true};  // skip first FPS check tick (topics warming up)
  // Timestamp jump detection
  double timestamp_jump_threshold_sec_{1.0};  // max drift between ROS clock and wall clock per check
  rclcpp::Time prev_ros_time_;
  std::chrono::steady_clock::time_point prev_wall_time_;
  bool timestamp_monitor_initialized_{false};

  // Parameters
  RobotInfo robot_info_;
  UserInfo user_info_;
  RosbagInfo rosbag_info_;
  std::string gamepad_name_;

  // State management
  uint8_t current_state_;
  uint8_t previous_state_;

  std::string current_task_name_;
  std::string current_task_dir_name_;
  std::string previous_task_name_;
  std::string current_task_path_;
  std::string previous_task_path_;

  std::string current_subtask_name_;
  std::vector<SubtaskInfo> current_episode_subtasks_;

  std::string current_bag_name_;
  std::string previous_bag_name_;
  std::string current_bag_path_;
  std::string previous_bag_path_;

};

} // namespace sobits_vla

#endif // SOBITS_VLA_ROSBAG_COLLECTION__ROSBAG_COLLECTION_HPP_
