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
  std::map<std::string, std::vector<std::string>> joint_names;
  std::vector<std::string> sensor_types;
  std::map<std::string, std::vector<std::string>> sensor_names;
  std::map<std::string, std::vector<std::string>> sensor_models;
  
  // Locomotion specific variables
  bool has_mobile_base;
  bool has_cmd_vel_y;
  
  // Core Topics
  std::string joint_states_topic;
  std::string cmd_vel_topic;
};

class UserInfo
{
public:
  std::string name;
  std::string email;
  std::string location;
};

class RosbagInfo
{
public:
  std::string recording_dir;
  std::vector<std::string> topics_to_record;
  std::vector<std::string> services_to_record;
  std::vector<std::string> actions_to_record;
  int fps;
  double sync_threshold;
  // uint8_t recording_duration;
  std::string conversion_format;
  bool compress_output;
  std::string compression_format;
  std::string compression_mode;
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
  void updateSubtaskYaml(const std::string& subtask_name);

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

  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
  std::map<std::string, std::pair<uint32_t, uint32_t>> camera_dimensions_; // topic -> {width, height}

  // Parameters
  RobotInfo robot_info_;
  UserInfo user_info_;
  RosbagInfo rosbag_info_;
  std::string gamepad_name_;

  // State management
  uint8_t current_state_;
  uint8_t previous_state_;

  std::string current_task_name_;
  std::string previous_task_name_;
  std::string current_task_path_;
  std::string previous_task_path_;
  uint8_t current_task_id_;
  uint8_t previous_task_id_;
  
  std::string current_subtask_name_;

  std::string current_bag_name_;
  std::string previous_bag_name_;
  std::string current_bag_path_;
  std::string previous_bag_path_;
  uint8_t current_bag_id_;
  uint8_t previous_bag_id_;

};

} // namespace sobits_vla
