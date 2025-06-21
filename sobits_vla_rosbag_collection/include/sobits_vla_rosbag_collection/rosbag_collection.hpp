#include <rcl_interfaces/msg/parameter_type.hpp>
#include <sobits_interfaces/srv/vla_task_update.hpp>
#include <sobits_interfaces/action/vla_record_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sobits_vla
{

class RobotInfo
{
public:
  std::string name;
  std::string version;
  std::string morphology;
  std::vector<std::string> parts;
  std::map<std::string, std::string> joint_names;
  std::vector<std::string> sensor_types;
  std::map<std::string, std::string> sensor_names;
  std::map<std::string, std::string> sensor_models;
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
  std::string recording_dir_;
  std::string topics_to_record_;
  std::string services_to_record_;
  std::string actions_to_record_;
  uint8_t recording_duration_;
  std::string conversion_format_;
  bool compress_output_;
  std::string compression_format_;
  std::string compression_mode_;
  std::string storage_config_file_;
};

class RosbagCollection : public rclcpp::Node
{
public:
  explicit RosbagCollection(const rclcpp::NodeOptions & options);
  ~RosbagCollection();

  void createRosbag(const std::string & task_name);
  void removeRosbag(const std::string & task_name);
  void saveRosbag(const std::string & task_name);
  void pauseRosbag();
  void resumeRosbag();
  void updateTaskName(
    const std::string & old_task_name,
    const std::string & new_task_name);

  void createRosbagYaml();
  void updateRosbagYaml( 
    const std::string & task_name,
    const std::string & bag_dir);

private:
  // TODO: service server for update task name
  void taskUpdateCallback(
    const std::shared_ptr<sobits_interfaces::srv::VlaTaskUpdate::Request> request,
    std::shared_ptr<sobits_interfaces::srv::VlaTaskUpdate::Response> response);
  // TODO: action server for save/remove/pause the rosbag
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
  rclcpp::Service<sobits_interfaces::srv::VlaTaskUpdate>::SharedPtr task_update_service_;

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
  std::string current_bag_name_;
  std::string previous_bag_name_;
  std::string current_bag_path_;
  std::string previous_bag_path_;

  std::string rosbag_dir_;

};

} // namespace sobits_vla

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_vla::RosbagCollection)
