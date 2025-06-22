#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

namespace sobits_vla
{

RosbagCollection::RosbagCollection(const rclcpp::NodeOptions & options)
: Node("rosbag_collection", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing RosbagCollection Node...");
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
  task_update_service_ = this->create_service<sobits_interfaces::srv::VlaUpdateTask>(
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
  for (const auto & part : robot_info_.parts) {
    RCLCPP_INFO(this->get_logger(), "Robot part: %s", part.c_str());
    this->declare_parameter<std::vector<std::string>>("robot_info.morphology." + part + ".joint_names", std::vector<std::string>{});
    robot_info_.joint_names[part] = this->get_parameter("robot_info.morphology." + part + ".joint_names").as_string_array();
  }
  this->declare_parameter<std::vector<std::string>>("robot_info.sensors.types", std::vector<std::string>{"camera", "lidar", "imu"});
  robot_info_.sensor_types = this->get_parameter("robot_info.sensors.types").as_string_array();
  robot_info_.sensor_names.clear();
  robot_info_.sensor_models.clear();
  for (const auto & sensor_type : robot_info_.sensor_types) {
    RCLCPP_INFO(this->get_logger(), "Robot sensor: %s", sensor_type.c_str());
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type + ".names", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type + ".models", std::vector<std::string>{});
    robot_info_.sensor_names[sensor_type] = this->get_parameter("robot_info.sensors." + sensor_type + ".names").as_string_array();
    robot_info_.sensor_models[sensor_type] = this->get_parameter("robot_info.sensors." + sensor_type + ".models").as_string_array();
  }

  // (2) User info parameters
  this->declare_parameter<std::string>("user_info.name", "default_user");
  this->declare_parameter<std::string>("user_info.email", "default_user@example.com");
  this->declare_parameter<std::string>("user_info.location", "default_location");
  user_info_.name = this->get_parameter("user_info.name").as_string();
  user_info_.email = this->get_parameter("user_info.email").as_string();
  user_info_.location = this->get_parameter("user_info.location").as_string();

  // (3) Rosbag parameters
  this->declare_parameter<std::string>("rosbag_config.record_directory", "/path/to/recorded_bags");
  this->declare_parameter<std::vector<std::string>>("rosbag_config.topics_to_record", std::vector<std::string>{"/topic1", "/topic2"});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.services_to_record", std::vector<std::string>{"/topic1", "/topic2"});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.actions_to_record", std::vector<std::string>{"/topic1", "/topic2"});
  this->declare_parameter<uint8_t>("rosbag_config.recording_duration", 0);
  this->declare_parameter<std::string>("rosbag_config.conversion_format", "mcap");
  this->declare_parameter<std::string>("rosbag_config.compression_format", "zstd");
  this->declare_parameter<std::string>("rosbag_config.compression_mode", "none");
  rosbag_info_.recording_dir      = this->get_parameter("rosbag_config.record_directory").as_string();
  rosbag_info_.topics_to_record   = this->get_parameter("rosbag_config.topics_to_record").as_string_array();
  rosbag_info_.services_to_record = this->get_parameter("rosbag_config.services_to_record").as_string_array();
  rosbag_info_.actions_to_record  = this->get_parameter("rosbag_config.actions_to_record").as_string_array();
  rosbag_info_.recording_duration = this->get_parameter("rosbag_config.recording_duration").as_int();
  rosbag_info_.conversion_format  = this->get_parameter("rosbag_config.conversion_format").as_string();
  rosbag_info_.compression_format = this->get_parameter("rosbag_config.compression_format").as_string();
  rosbag_info_.compression_mode   = this->get_parameter("rosbag_config.compression_mode").as_string();

  // (4) Gamepad parameters
  this->declare_parameter<std::string>("gamepad_config.name", "default_gamepad");
  gamepad_name_ = this->get_parameter("gamepad_config.name").as_string();

  // Init values
  current_state_      = sobits_interfaces::action::VlaRecordState_Result::STOPPED; // PAUSED, RECORDING, STOPPED
  previous_state_     = current_state_;

  current_bag_id_     = -1;
  previous_bag_id_    = current_bag_id_;
  current_bag_name_   = "episode_" + std::to_string(current_bag_id_);
  previous_bag_name_  = current_bag_name_;
  current_task_name_  = "default task";
  previous_task_name_ = current_task_name_;
  current_task_path_  = rosbag_info_.recording_dir + "/" + current_task_name_;
  previous_task_path_ = current_task_path_;
  current_bag_path_   = current_task_path_ + "/" + current_bag_name_;
  std::replace(current_bag_path_.begin(), current_bag_path_.end(), ' ', '_');
  std::transform(current_bag_path_.begin(), current_bag_path_.end(), current_bag_path_.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  previous_bag_path_  = current_bag_path_;

  rosbag_collection_dir_ = rosbag_info_.recording_dir;
  rosbag_options_ = "";

  bag_process_ = nullptr;
  bag_pid_ = -1;
  
  // Prepare the rosbag configuration
  if (rosbag_info_.recording_duration <= 0) {
    RCLCPP_WARN(this->get_logger(), "Recording will not stop automatically");
  } else {
    RCLCPP_INFO(this->get_logger(), "Recording will stop automatically after %d seconds", rosbag_info_.recording_duration);
    rosbag_options_ += " --max-bag-duration " + std::to_string(rosbag_info_.recording_duration) + "s";
  }
  if (rosbag_info_.conversion_format.empty()) {
    RCLCPP_WARN(this->get_logger(), "No conversion format specified, using default 'sqlite3'");
  } else {
    RCLCPP_INFO(this->get_logger(), "Using conversion format: %s", rosbag_info_.conversion_format.c_str());
    rosbag_options_ += " --storage " + rosbag_info_.conversion_format;
  }
  if (rosbag_info_.compression_mode == "none") {
    RCLCPP_INFO(this->get_logger(), "Output compression is disabled");
  } else {
      RCLCPP_INFO(this->get_logger(), "Output compression is enabled with mode: %s", rosbag_info_.compression_mode.c_str());
      rosbag_options_ += " --compression-mode " + rosbag_info_.compression_mode;
      RCLCPP_INFO(this->get_logger(), "Using compression format: %s", rosbag_info_.compression_format.c_str());
      rosbag_options_ += " --compression-format " + rosbag_info_.compression_format;
    }
  if (rosbag_info_.topics_to_record.empty()) {
    RCLCPP_WARN(this->get_logger(), "No topics to record specified in the rosbag configuration. Using all topics.");
    rosbag_options_ = " --all"; // Record all topics
  } else {
    for (const auto & topic : rosbag_info_.topics_to_record) {
      rosbag_options_ += " " + topic;
    }
    RCLCPP_DEBUG(this->get_logger(), "Topics to record: %s", rosbag_options_.c_str());
  }
  if (rosbag_info_.services_to_record.empty()) {
    RCLCPP_WARN(this->get_logger(), "No services to record specified in the rosbag configuration");
  } // TODO: From Jazzy services can be recorded, but not in Humble
  if (rosbag_info_.actions_to_record.empty()) {
    RCLCPP_WARN(this->get_logger(), "No actions to record specified in the rosbag configuration");
  } // TODO: From Kilted services can be recorded, but not in Humble

  // Create the recording directory if it does not exist
  if (!std::filesystem::exists(rosbag_collection_dir_)) {
    try {
      std::filesystem::create_directories(rosbag_collection_dir_);
      RCLCPP_INFO(this->get_logger(), "Created recording directory: %s", rosbag_collection_dir_.c_str());
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

void RosbagCollection::createRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Starting recording...");
  previous_bag_id_ = current_bag_id_;
  current_bag_id_++;

  previous_state_ = current_state_;
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::RECORDING;

  previous_task_path_ = current_task_path_;
  current_task_path_ = rosbag_collection_dir_ + "/" + current_task_name_;
  std::replace(current_task_path_.begin(), current_task_path_.end(), ' ', '_');
  std::transform(current_task_path_.begin(), current_task_path_.end(), current_task_path_.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  previous_bag_name_ = current_bag_name_;
  current_bag_name_ = "episode_" + std::to_string(current_bag_id_);

  previous_bag_path_ = current_bag_path_;
  current_bag_path_ = current_task_path_ + "/" + current_bag_name_;

  // Create the directory for the current bag
  if (!std::filesystem::exists(current_task_path_)) {
    try {
      std::filesystem::create_directories(current_task_path_);
      RCLCPP_INFO(this->get_logger(), "Created bag directory: %s", current_bag_path_.c_str());
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_DEBUG(this->get_logger(), "The directory %s already exists, skipping creation", current_task_path_.c_str());
    }
  }

  // Call the rosbag record command
  std::string command = "ros2 bag record --output " + current_bag_path_ + " " + rosbag_options_;
  RCLCPP_INFO(this->get_logger(), "Executing command: %s", command.c_str());

  bag_process_ = popen(command.c_str(), "r");
  if (!bag_process_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute command: %s", command.c_str());
    throw std::runtime_error("Failed to start rosbag record process");
  
  // Read the PID from the command output
    char buffer[128];
    if (fgets(buffer, sizeof(buffer), bag_process_) != nullptr) {
      bag_pid_ = std::atoi(buffer);
      RCLCPP_INFO(this->get_logger(), "Started recording with PID %d", bag_pid_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to read PID from command output");
      pclose(bag_process_);
      throw std::runtime_error("Failed to read PID from rosbag record process");
    }
  }

  // Set the current state to RECORDING
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
  RCLCPP_INFO(this->get_logger(), "Rosbag recording started successfully");

}

// TODO
void RosbagCollection::removeRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Removing rosbag...");

  if (kill(bag_pid_, SIGTERM) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to terminate rosbag process with PID %d", bag_pid_);
    throw std::runtime_error("Failed to terminate rosbag process");
  }
  pclose(bag_process_);
  bag_process_ = nullptr;
  bag_pid_ = -1;
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;

  // Remove the current bag directory
  if (std::filesystem::exists(current_bag_path_)) {
    try {
      std::filesystem::remove_all(current_bag_path_);
      RCLCPP_INFO(this->get_logger(), "Removed bag directory: %s", current_bag_path_.c_str());
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to remove bag directory: %s", e.what());
      throw std::runtime_error("Failed to remove bag directory");
    }
  }

  previous_bag_id_ = current_bag_id_;
  current_bag_id_ = -1;
  previous_bag_name_ = current_bag_name_;
  current_bag_name_ = "episode_" + std::to_string(current_bag_id_);
  previous_bag_path_ = current_bag_path_;
  current_bag_path_ = current_task_path_ + "/" + current_bag_name_;

  RCLCPP_INFO(this->get_logger(), "Rosbag removed successfully");
}

void RosbagCollection::saveRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Saving rosbag...");

  if (kill(bag_pid_, SIGTERM) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to terminate rosbag process with PID %d", bag_pid_);
    throw std::runtime_error("Failed to terminate rosbag process");
  }
  pclose(bag_process_);
  bag_process_ = nullptr;
  bag_pid_ = -1;

  current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;
  RCLCPP_INFO(this->get_logger(), "Rosbag saved successfully");
}

void RosbagCollection::pauseRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Pausing rosbag...");

  if (kill(bag_pid_, SIGSTOP) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to pause rosbag process with PID %d", bag_pid_);
    throw std::runtime_error("Failed to pause rosbag process");
  }
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::PAUSED;
  RCLCPP_INFO(this->get_logger(), "Rosbag paused successfully");
}

void RosbagCollection::resumeRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Resuming rosbag...");

  if (kill(bag_pid_, SIGCONT) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to resume rosbag process with PID %d", bag_pid_);
    throw std::runtime_error("Failed to resume rosbag process");
  }
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
  RCLCPP_INFO(this->get_logger(), "Rosbag resumed successfully");
}

void RosbagCollection::createRosbagYaml()
{
  RCLCPP_INFO(this->get_logger(), "Creating rosbag YAML file...");
  // Create the YAML node structure
  YAML::Node yaml_node;

  // (1) Add robot info
  yaml_node["robot_info"]["name"] = robot_info_.name;
  yaml_node["robot_info"]["version"] = robot_info_.version;
  yaml_node["robot_info"]["morphology"]["type"] = robot_info_.morphology;
  yaml_node["robot_info"]["morphology"]["parts"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & part : robot_info_.parts) {
    yaml_node["robot_info"]["morphology"]["parts"].push_back(part);
    yaml_node["robot_info"]["morphology"][part]["joint_names"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto & joint_name : robot_info_.joint_names[part]) {
      yaml_node["robot_info"]["morphology"][part]["joint_names"].push_back(joint_name);
    }
  }
  yaml_node["robot_info"]["sensors"]["types"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & sensor_type : robot_info_.sensor_types) {
    yaml_node["robot_info"]["sensors"]["types"].push_back(sensor_type);
    yaml_node["robot_info"]["sensors"][sensor_type]["names"] = YAML::Node(YAML::NodeType::Sequence);
    yaml_node["robot_info"]["sensors"][sensor_type]["models"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto & sensor_name : robot_info_.sensor_names[sensor_type]) {
      yaml_node["robot_info"]["sensors"][sensor_type]["names"].push_back(sensor_name);
    }
    for (const auto & sensor_model : robot_info_.sensor_models[sensor_type]) {
      yaml_node["robot_info"]["sensors"][sensor_type]["models"].push_back(sensor_model);
    }
  }

  // (2) Add user info
  yaml_node["user_info"]["name"] = user_info_.name;
  yaml_node["user_info"]["email"] = user_info_.email;
  yaml_node["user_info"]["location"] = user_info_.location;

  // (3) Add gamepad info
  yaml_node["gamepad_config"]["name"] = gamepad_name_;

  // Save the YAML node to a file
  std::string yaml_file_path = rosbag_collection_dir_ + "/recorded_bags_meta.yaml";
  try {
    std::ofstream yaml_file(yaml_file_path);
    if (!yaml_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file for writing: %s", yaml_file_path.c_str());
      throw std::runtime_error("Failed to open YAML file for writing");
    }
    yaml_file << yaml_node;
    yaml_file.close();
    RCLCPP_INFO(this->get_logger(), "Created rosbag YAML file: %s", yaml_file_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create rosbag YAML file: %s", e.what());
    throw std::runtime_error("Failed to create rosbag YAML file");
  }

}

bool RosbagCollection::updateRosbagYaml()
{
  RCLCPP_INFO(this->get_logger(), "Updating rosbag YAML file...");

  // Load the existing YAML file
  std::string yaml_file_path = rosbag_collection_dir_ + "/recorded_bags_meta.yaml";
  YAML::Node yaml_node;
  try {
    yaml_node = YAML::LoadFile(yaml_file_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
    return false;
  }

  // Update the task name in the YAML file
  std::string current_task_label = current_task_name_;
  std::replace(current_task_label.begin(), current_task_label.end(), ' ', '_');
  std::transform(current_task_label.begin(), current_task_label.end(), current_task_label.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  // If the tasks node is defined, append the current task
  // If not, create the tasks node 
  if (yaml_node["recorded_bags"]["tasks"].IsDefined()) {
    yaml_node["recorded_bags"]["tasks"].push_back(current_task_label);
  } else {
    yaml_node["recorded_bags"]["tasks"] = YAML::Node(YAML::NodeType::Sequence);
    yaml_node["recorded_bags"]["tasks"].push_back(current_task_label);
  }
  // Add the current task name to the YAML file
  yaml_node["recorded_bags"][current_task_label]["label"] = current_task_label;
  yaml_node["recorded_bags"][current_task_label]["bag_dir"] = current_task_path_;

  // Save the updated YAML node to the file
  try {
    std::ofstream yaml_file(yaml_file_path);
    if (!yaml_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file for writing: %s", yaml_file_path.c_str());
      return false;
    }
    yaml_file << yaml_node;
    yaml_file.close();
    RCLCPP_DEBUG(this->get_logger(), "Updated rosbag YAML file: %s", yaml_file_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update rosbag YAML file: %s", e.what());
    return false;
  }

  return true;
}



// TODO: Implement the execute function to handle the recording state changes
void RosbagCollection::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<sobits_interfaces::action::VlaRecordState>> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::RECORD) {
    if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::STOPPED) {
      RCLCPP_WARN(this->get_logger(), "Cannot start recording while already in state: %d", current_state_);
      auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    createRosbag();
  } else if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::PAUSE) {
    if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::RECORDING) {
      RCLCPP_WARN(this->get_logger(), "Cannot pause recording while not in RECORDING state");
      auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    pauseRosbag();
  } else if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::RESUME) {
    if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::PAUSED) {
      RCLCPP_WARN(this->get_logger(), "Cannot resume recording while not in PAUSED state");
      auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    resumeRosbag();
  } else if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::SAVE) {
    if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::RECORDING
        && current_state_ != sobits_interfaces::action::VlaRecordState_Result::PAUSED) {
      RCLCPP_WARN(this->get_logger(), "Cannot save recording while not in RECORDING or PAUSED state");
      auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    saveRosbag();
  } else if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::DELETE) {
    if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::STOPPED) {
      RCLCPP_WARN(this->get_logger(), "Cannot delete recording while recording is in progress");
      auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    removeRosbag();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown command received: %d", goal->command);
    auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();
    result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
    goal_handle->abort(result);
    return;
  }

  auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();
  result->status = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
  goal_handle->succeed(result);

}

void RosbagCollection::taskUpdateCallback(
  const std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Request> request,
  std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Response> response)
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
    current_task_name_ = request->label;
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
