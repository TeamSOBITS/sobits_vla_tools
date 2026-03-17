#include <rcl_interfaces/msg/parameter_type.hpp>
#include <sobits_interfaces/srv/vla_update_task.hpp>
#include <sobits_interfaces/action/vla_record_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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
// #include <sys/wait.h>   // For waitpid, WIFEXITED, WIFSIGNALED
// #include <unistd.h>     // For fork, execl, _exit
// #include <iostream>     // For std::cerr
// #include <filesystem>   // For std::filesystem operations
// #include <algorithm>    // For std::replace, std::transform
// #include <fstream>      // For std::ofstream
// #include <string>       // For std::string
// #include <vector>       // For std::vector
// #include <thread>       // For std::this_thread::sleep_for
// #include <chrono>       // For std::chrono::seconds

namespace sobits_vla
{

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
  void saveRosbag();

  void createRosbagYaml();
  void updateRosbagYaml();
  void updateEpisodeYaml();
  void removeEpisodeFromYaml();
  void buildTopicList();
  bool validateTopics();
  void startFpsMonitor();
  void stopFpsMonitor();
  std::string getTimestampString();

private:
  void taskUpdateCallback(
    const std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Request> request,
    std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Response> response);
  
  void subtaskUpdateCallback(
    const std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Request> request,
    std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Response> response);

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, const std::string topic_name);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const sobits_interfaces::action::VlaRecordState::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sobits_interfaces::action::VlaRecordState>> goal_handle);
  void handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sobits_interfaces::action::VlaRecordState>> goal_handle);
  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sobits_interfaces::action::VlaRecordState>> goal_handle);

  rclcpp_action::Server<sobits_interfaces::action::VlaRecordState>::SharedPtr record_action_server_;
  rclcpp::Service<sobits_interfaces::srv::VlaUpdateTask>::SharedPtr task_update_service_;
  rclcpp::Service<sobits_interfaces::srv::VlaUpdateTask>::SharedPtr subtask_update_service_;

  std::shared_ptr<rosbag2_transport::Recorder> recorder_node_;
  std::thread recorder_thread_;
  std::mutex recorder_mutex_;
  std::atomic<bool> is_recording_{false};
  bool task_has_been_set_{false};

  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
  std::map<std::string, std::pair<uint32_t, uint32_t>> camera_dimensions_;

  // FPS monitoring during recording
  int expected_sensor_fps_{0};
  std::vector<rclcpp::GenericSubscription::SharedPtr> monitor_subs_;
  std::map<std::string, std::atomic<uint64_t>> monitor_counts_;
  std::map<std::string, uint64_t> monitor_prev_counts_;
  rclcpp::TimerBase::SharedPtr fps_monitor_timer_;

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
