#include <rcl_interfaces/msg/parameter_type.hpp>
#include <sobits_interfaces/srv/vla_update_task.hpp>
#include <sobits_interfaces/action/vla_record_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <filesystem> 
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace sobits_vla
{

class RobotInfo
{
public:
  std::string name;
  std::string version;
  std::string morphology;
  std::vector<std::string> parts;
  std::map<std::string, std::vector<std::string>> joint_names;
  std::vector<std::string> sensor_types;
  std::map<std::string, std::vector<std::string>> sensor_names;
  std::map<std::string, std::vector<std::string>> sensor_models;
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
  uint8_t recording_duration;
  std::string conversion_format;
  bool compress_output;
  std::string compression_format;
  std::string compression_mode;
  std::string storage_config_file;
};

class RosbagCollection : public rclcpp::Node
{
public:
  explicit RosbagCollection(const rclcpp::NodeOptions & options);
  ~RosbagCollection();

  void createRosbag();
  void removeRosbag();
  void saveRosbag();
  void pauseRosbag();
  void resumeRosbag();

  void createRosbagYaml();
  bool updateRosbagYaml();

private:
  void taskUpdateCallback(
    const std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Request> request,
    std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Response> response);

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


  FILE* bag_process_;
  pid_t bag_pid_;

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
  std::string current_bag_name_;
  std::string previous_bag_name_;
  std::string current_bag_path_;
  std::string previous_bag_path_;
  uint8_t current_bag_id_;
  uint8_t previous_bag_id_;

  std::string rosbag_collection_dir_;
  std::string rosbag_options_;

};

} // namespace sobits_vla

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_vla::RosbagCollection)
