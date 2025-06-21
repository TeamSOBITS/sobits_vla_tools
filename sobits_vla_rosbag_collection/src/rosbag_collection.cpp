#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

namespace sobits_vla
{

RosbagCollection::RosbagCollection(const rclcpp::NodeOptions & options)
: Node("rosbag_collection", options)
{
  // TODO: Configure QoS settings
  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
  // qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  // qos.durability(rclcpp::DurabilityPolicy::Volatile);
  // qos.history(rclcpp::HistoryPolicy::KEEP_LAST);

  // Initialize Action Server
  record_action_server_ = rclcpp_action::create_server<sobits_interfaces::action::VlaRecordState>(
    this,
    "vla_record_state",
    std::bind(&RosbagCollection::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RosbagCollection::handleCancel, this, std::placeholders::_1),
    std::bind(&RosbagCollection::handleAccepted, this, std::placeholders::_1));
  // Initialize Service Server
  task_update_service_ = this->create_service<sobits_interfaces::srv::VlaTaskUpdate>(
    "vla_task_update",
    std::bind(&RosbagCollection::taskUpdateCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Declare and get parameters (TODO: make it into a new function)
  // (1) Robot info parameters
  this->declare_parameter<std::string>("robot_info.name", "sobit_robot");
  this->declare_parameter<std::string>("robot_info.version", "1.0.0");
  this->declare_parameter<std::string>("robot_info.morphology.type", "mobile_manipulator");
  this->declare_parameter<std::vector<std::string>>("robot_info.morphology.parts", std::vector<std::string>{"base", "arm", "gripper"});
  robot_info_.name = this->get_parameter("robot_info.name").as_string();
  robot_info_.version = this->get_parameter("robot_info.version").as_string();
  robot_info_.morphology = this->get_parameter("robot_info.morphology.type").as_string();
  robot_info_.parts = this->get_parameter("robot_info.morphology.parts").as_string_array();
  robot_info_.joint_names.clear();
  robot_info_.joint_names.resize(robot_info_.parts.size());
  for (const auto & part : robot_info_.parts) {
    RCLCPP_INFO(this->get_logger(), "Robot part: %s", part.c_str());
    this->declare_parameter<std::vector<std::string>>("robot_info.morphology." + part + ".joint_names", std::vector<std::string>{});
    robot_info_.joint_names[part] = this->get_parameter("robot_info.morphology." + part + ".joint_names").as_string_array();
  }
  this->declare_parameter<std::vector<std::string>>("robot_info.sensors.types", std::vector<std::string>{"camera", "lidar", "imu"});
  robot_info_.sensor_types = this->get_parameter("robot_info.sensors.types").as_string_array();
  robot_info_.sensor_names.clear();
  robot_info_.sensor_models.clear();
  robot_info_.sensor_names.resize(robot_info_.sensor_types.size());
  robot_info_.sensor_models.resize(robot_info_.sensor_types.size());
  for (const auto & sensor : robot_info_.sensor_types) {
    RCLCPP_INFO(this->get_logger(), "Robot sensor: %s", sensor.c_str());
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor + ".names", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor + ".models", std::vector<std::string>{});
    robot_info_.sensor_names[sensor] = this->get_parameter("robot_info.sensors." + sensor + ".names").as_string_array();
    robot_info_.sensor_models[sensor] = this->get_parameter("robot_info.sensors." + sensor + ".models").as_string_array();
  }

  // (2) User info parameters
  this->declare_parameter<std::string>("user_info.name", "default_user");
  this->declare_parameter<std::string>("user_info.email", "default_user@example.com");
  this->declare_parameter<std::string>("user_info.location", "default_location");
  user_info_.name = this->get_parameter("user_info.name").as_string();
  user_info_.email = this->get_parameter("user_info.email").as_string();
  user_info_.location = this->get_parameter("user_info.location").as_string();

  // (3) Rosbag parameters
  this->declare_parameter<std::string>("rosbag_config.recording_dir", "/path/to/recorded_bags");
  this->declare_parameter<std::vector<std::string>>("rosbag_config.topics_to_record", std::vector<std::string>{"/topic1", "/topic2"});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.services_to_record", std::vector<std::string>{"/topic1", "/topic2"});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.actions_to_record", std::vector<std::string>{"/topic1", "/topic2"});
  this->declare_parameter<uint8_t>("rosbag_config.recording_duration", 0);
  this->declare_parameter<std::string>("rosbag_config.conversion_format", "mcap");
  this->declare_parameter<bool>("rosbag_config.compress_output", false);
  this->declare_parameter<std::string>("rosbag_config.compression_format", "zstd");
  this->declare_parameter<std::string>("rosbag_config.compression_mode", "file");
  this->declare_parameter<std::string>("rosbag_config.storage_config_file", "/path/to/storage_config.yaml");
  rosbag_info_.recording_dir = this->get_parameter("rosbag_config.recording_dir").as_string();
  rosbag_info_.topics_to_record = this->get_parameter("rosbag_config.topics_to_record").as_string_array();
  rosbag_info_.services_to_record = this->get_parameter("rosbag_config.services_to_record").as_string_array();
  rosbag_info_.actions_to_record = this->get_parameter("rosbag_config.actions_to_record").as_string_array();
  rosbag_info_.recording_duration = this->get_parameter("rosbag_config.recording_duration").as_int();
  rosbag_info_.conversion_format = this->get_parameter("rosbag_config.conversion_format").as_string();
  rosbag_info_.compress_output = this->get_parameter("rosbag_config.compress_output").as_bool();
  rosbag_info_.compression_format = this->get_parameter("rosbag_config.compression_format").as_string();
  rosbag_info_.compression_mode = this->get_parameter("rosbag_config.compression_mode").as_string();
  rosbag_info_.storage_config_file = this->get_parameter("rosbag_config.storage_config_file").as_string();

  // (4) Gamepad parameters
  this->declare_parameter<std::string>("gamepad_config.name", "default_gamepad");
  gamepad_name_ = this->get_parameter("gamepad_config.name").as_string();

  // Init values
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED; // PAUSED, RECORDING, STOPPED
  previous_state_ = current_state_;
  current_task_name_ = "default_task";
  previous_task_name_ = current_task_name_;
  current_bag_name_ = "default_bag_episode_0";
  previous_bag_name_ = current_bag_name_;
  current_bag_path_ = rosbag_recording_dir_ + "/" + current_bag_name_;
  previous_bag_path_ = current_bag_path_;
  rosbag_dir_ = rosbag_info_.recording_dir;

  // Create the recording directory if it does not exist
  if (!std::filesystem::exists(rosbag_dir_)) {
    try {
      std::filesystem::create_directories(rosbag_dir_);
      RCLCPP_INFO(this->get_logger(), "Created recording directory: %s", rosbag_dir_.c_str());
    } catch (const std::filesystem::filesystem_error & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create recording directory: %s", e.what());
      throw std::runtime_error("Failed to create recording directory");
    }
  }

  // Create the rosbag YAML file
  createRosbagYaml();

  RCLCPP_INFO(this->get_logger(), "RosbagCollection initialized");
}

RosbagCollection::~RosbagCollection()
{
  RCLCPP_INFO(this->get_logger(), "RosbagCollection destructor called");
}

rclcpp_action::GoalResponse RosbagCollection::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const sobits_interfaces::action::VlaRecordState::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request: %d", goal->command);
  // Accept the goal
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosbagCollection::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<sobits_interfaces::action::VlaRecordState>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  // Accept the cancel request
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosbagCollection::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<sobits_interfaces::action::VlaRecordState>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted, executing...");
  // Execute the goal
  this->execute(goal_handle);
}

// TODO: Implement the execute function to handle the recording state changes
void RosbagCollection::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<sobits_interfaces::action::VlaRecordState>> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  switch (goal->command)
  {
    case sobits_interfaces::action::VlaRecordState_Goal::RECORD:
      RCLCPP_INFO(this->get_logger(), "Starting recording...");
    case sobits_interfaces::action::VlaRecordState_Goal::PAUSE:
      RCLCPP_INFO(this->get_logger(), "Pausing recording...");
      break;
    case sobits_interfaces::action::VlaRecordState_Goal::RESUME:
      RCLCPP_INFO(this->get_logger(), "Resuming recording...")
    case sobits_interfaces::action::VlaRecordState_Goal::SAVE:
      RCLCPP_INFO(this->get_logger(), "Saving recording...");
      break;
    case sobits_interfaces::action::VlaRecordState_Goal::DELETE:
      RCLCPP_INFO(this->get_logger(), "Deleting recording...");
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown command: %d", goal->command);
      auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();
      result->result = sobits_interfaces::action::VlaRecordState_Result::FAILURE;
      goal_handle->abort(result);
      return;
  }

  auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();
  result->result = sobits_interfaces::action::VlaRecordState_Result::SUCCESS;
  goal_handle->succeed(result);

}

void RosbagCollection::taskUpdateCallback(
  const std::shared_ptr<sobits_interfaces::srv::VlaTaskUpdate::Request> request,
  std::shared_ptr<sobits_interfaces::srv::VlaTaskUpdate::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received task update request: %s", request->label.c_str());
  
  // Update the task name
  if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::STOPPED) {
    RCLCPP_WARN(this->get_logger(), "Cannot update task name while recording is in progress");
    response->success = false;
    response->message = "Cannot update task name while recording is in progress";
    return;
  }
  if (request->label != current_task_name_) {
    previous_task_name_ = current_task_name_;
    current_task_name_ = request->new_task_name;
    RCLCPP_INFO(this->get_logger(), "Updated task name from '%s' to '%s'", previous_task_name_.c_str(), current_task_name_.c_str());

    // Update the rosbag YAML file
    bool is_updated = updateRosbagYaml();
    if (!is_updated) {
      RCLCPP_ERROR(this->get_logger(), "Failed to update rosbag YAML file");
      response->success = true;
      response->message = "Task name updated, but failed to update rosbag YAML file";
      return;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Task name '%s' is already the current task name", request->label.c_str());
    response->success = false;
    response->message = "Task name is already the current task name";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Task name updated successfully to '%s'", current_task_name_.c_str());
  response->success = true;
  response->message = "Task name updated successfully and rosbag YAML file updated";
}

} // namespace sobits_vla
