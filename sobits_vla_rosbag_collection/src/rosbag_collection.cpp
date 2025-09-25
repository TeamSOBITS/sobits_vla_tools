#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

#include <csignal>      // For kill
#include <sys/wait.h>   // For waitpid
#include <unistd.h>     // For fork, execl, _exit
#include <iostream>     // For std::cerr
#include <filesystem>   // For std::filesystem operations
#include <algorithm>    // For std::replace, std::transform
#include <fstream>      // For std::ofstream
#include <string>       // For std::string
#include <vector>       // For std::vector

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
  this->declare_parameter<std::string>("rosbag_config.conversion_format", "mcap");
  this->declare_parameter<std::string>("rosbag_config.compression_format", "zstd");
  this->declare_parameter<std::string>("rosbag_config.compression_mode", "none");
  rosbag_info_.recording_dir      = this->get_parameter("rosbag_config.record_directory").as_string();
  rosbag_info_.topics_to_record   = this->get_parameter("rosbag_config.topics_to_record").as_string_array();
  rosbag_info_.services_to_record = this->get_parameter("rosbag_config.services_to_record").as_string_array();
  rosbag_info_.actions_to_record  = this->get_parameter("rosbag_config.actions_to_record").as_string_array();
  rosbag_info_.conversion_format  = this->get_parameter("rosbag_config.conversion_format").as_string();
  rosbag_info_.compression_format = this->get_parameter("rosbag_config.compression_format").as_string();
  rosbag_info_.compression_mode   = this->get_parameter("rosbag_config.compression_mode").as_string();

  // (4) Gamepad parameters
  this->declare_parameter<std::string>("gamepad_config.name", "default_gamepad");
  gamepad_name_ = this->get_parameter("gamepad_config.name").as_string();

  // Init values
  current_state_      = sobits_interfaces::action::VlaRecordState_Result::STOPPED; // PAUSED, RECORDING, STOPPED, ERROR
  previous_state_     = current_state_;

  current_task_id_    = 0;
  previous_task_id_   = current_task_id_;
  current_task_name_  = "default task";
  previous_task_name_ = current_task_name_;
  current_task_path_  = rosbag_info_.recording_dir + "/" + current_task_name_;
  std::replace(current_task_path_.begin(), current_task_path_.end(), ' ', '_');
  std::transform(current_task_path_.begin(), current_task_path_.end(), current_task_path_.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  previous_task_path_ = current_task_path_;

  current_bag_id_     = 0;
  previous_bag_id_    = current_bag_id_;
  current_bag_name_   = "episode_" + std::to_string(current_bag_id_);
  previous_bag_name_  = current_bag_name_;
  current_bag_path_   = current_task_path_ + "/" + current_bag_name_;
  previous_bag_path_  = current_bag_path_;

  rosbag_info_.rosbag_options = "";

  bag_pid_ = -1;
  
  // Prepare the rosbag configuration
  if (rosbag_info_.conversion_format.empty()) {
    RCLCPP_WARN(this->get_logger(), "No conversion format specified, using default 'sqlite3'");
  } else {
    RCLCPP_INFO(this->get_logger(), "Using conversion format: %s", rosbag_info_.conversion_format.c_str());
    rosbag_info_.rosbag_options += " --storage " + rosbag_info_.conversion_format;
  }
  if (rosbag_info_.compression_mode == "none") {
    RCLCPP_INFO(this->get_logger(), "Output compression is disabled");
  } else {
      RCLCPP_INFO(this->get_logger(), "Output compression is enabled with mode: %s", rosbag_info_.compression_mode.c_str());
      rosbag_info_.rosbag_options += " --compression-mode " + rosbag_info_.compression_mode;
      RCLCPP_INFO(this->get_logger(), "Using compression format: %s", rosbag_info_.compression_format.c_str());
      rosbag_info_.rosbag_options += " --compression-format " + rosbag_info_.compression_format;
    }
  if (rosbag_info_.topics_to_record.empty()) {
    RCLCPP_WARN(this->get_logger(), "No topics to record specified in the rosbag configuration. Using all topics.");
    rosbag_info_.rosbag_options = " --all"; // Record all topics
  } else {
    for (const auto & topic : rosbag_info_.topics_to_record) {
      rosbag_info_.rosbag_options += " " + topic;
    }
    RCLCPP_DEBUG(this->get_logger(), "Topics to record: %s", rosbag_info_.rosbag_options.c_str());
  }
  if (rosbag_info_.services_to_record.empty()) {
    RCLCPP_WARN(this->get_logger(), "No services to record specified in the rosbag configuration");
  } // TODO: From Jazzy services can be recorded, but not in Humble
  if (rosbag_info_.actions_to_record.empty()) {
    RCLCPP_WARN(this->get_logger(), "No actions to record specified in the rosbag configuration");
  } // TODO: From Kilted services can be recorded, but not in Humble

  // Create the recording directory if it does not exist
  if (!std::filesystem::exists(rosbag_info_.recording_dir)) {
    try {
      std::filesystem::create_directories(rosbag_info_.recording_dir);
      RCLCPP_INFO(this->get_logger(), "Created recording directory: %s", rosbag_info_.recording_dir.c_str());
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
  // Ensure the rosbag process is stopped if it's still running
  if (bag_pid_ != -1) {
      RCLCPP_WARN(this->get_logger(), "Rosbag process (PID: %d) still active in destructor. Attempting to save.", bag_pid_);
      // Try to save the bag on destruction for robustness
      try {
          saveRosbag();
      } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Error during bag saving in destructor: %s", e.what());
      }
  }
}

void RosbagCollection::createRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Starting recording...");

  previous_bag_id_ = current_bag_id_;
  current_bag_id_++;

  previous_task_path_ = current_task_path_;
  current_task_path_ = rosbag_info_.recording_dir + "/" + current_task_name_;
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
      RCLCPP_DEBUG(this->get_logger(), "The directory %s already exists, skipping creation: %s", current_task_path_.c_str(), e.what());
    }
  }

  // Call the rosbag record command
  std::string command = "exec ros2 bag record --output " + current_bag_path_ + " " + rosbag_info_.rosbag_options;
  RCLCPP_INFO(this->get_logger(), "Executing command: %s", command.c_str());

  pid_t pid = fork();
  if (pid == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to fork for rosbag record: %s", strerror(errno));
    previous_state_ = current_state_;
    current_state_ = sobits_interfaces::action::VlaRecordState_Result::ERROR;
    throw std::runtime_error("Failed to fork for rosbag record");
  } else if (pid == 0) {
    // Execute the command string via /bin/sh -c
    execl("/bin/sh", "sh", "-c", command.c_str(), (char *)nullptr);
    
    // If execl fails, print an error and exit the child process
    perror("execl failed");
    _exit(EXIT_FAILURE); 
  } else {
    // Parent process
    bag_pid_ = pid;
    RCLCPP_INFO(this->get_logger(), "Started recording with PID %d", bag_pid_);
  }

  // Set the current state to RECORDING
  previous_state_ = current_state_;
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::RECORDING;

  RCLCPP_INFO(this->get_logger(), "Rosbag recording started successfully");
}


void RosbagCollection::removeRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Removing rosbag...");

  if (bag_pid_ != -1) {
    RCLCPP_INFO(this->get_logger(), "Sending SIGINT to rosbag process (PID: %d) for removal...", bag_pid_);
    // Send SIGINT for graceful termination
    if (kill(bag_pid_, SIGINT) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send SIGINT to rosbag process with PID %d: %s", bag_pid_, strerror(errno));
      previous_state_ = current_state_;
      current_state_ = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      throw std::runtime_error("Failed to terminate rosbag process for removal");
    }

    int status = 0;
    pid_t result = 0;
    // Increased timeout for rosbag to gracefully exit and flush its cache
    int timeout_seconds = 20; // Changed from 5 to 20 seconds
    int poll_interval_ms = 200; // Check every 200 ms

    RCLCPP_INFO(this->get_logger(), "Waiting for rosbag process (PID: %d) to terminate for removal (timeout: %d seconds)...", bag_pid_, timeout_seconds);

    for (int i = 0; i < (timeout_seconds * 1000) / poll_interval_ms; ++i) {
        result = waitpid(bag_pid_, &status, WNOHANG); // Use WNOHANG
        if (result == bag_pid_) { // Child has terminated
            if (WIFEXITED(status)) {
                RCLCPP_INFO(this->get_logger(), "Rosbag process (PID: %d) exited with status: %d", bag_pid_, WEXITSTATUS(status));
            } else if (WIFSIGNALED(status)) {
                RCLCPP_INFO(this->get_logger(), "Rosbag process (PID: %d) terminated by signal: %d", bag_pid_, WTERMSIG(status));
            }
            bag_pid_ = -1; // Reset PID since process is gone
            break; // Exit loop
        } else if (result == 0) { // Child is still running
            std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
        } else { // Error from waitpid
            RCLCPP_ERROR(this->get_logger(), "Error waiting for rosbag process (PID: %d) during removal: %s", bag_pid_, strerror(errno));
            break; // Exit loop on error
        }
    }

    if (bag_pid_ != -1) { // If it's still not -1, it means the process didn't terminate within the timeout
        RCLCPP_WARN(this->get_logger(), "Rosbag process (PID: %d) did not terminate gracefully within timeout (%d seconds) during removal. Sending SIGKILL.", bag_pid_, timeout_seconds);
        if (kill(bag_pid_, SIGKILL) == -1) { // Force terminate
            RCLCPP_ERROR(this->get_logger(), "Failed to send SIGKILL to rosbag process with PID %d: %s", bag_pid_, strerror(errno));
        }
        // Even after SIGKILL, wait for it to be reaped to avoid zombie processes
        waitpid(bag_pid_, &status, 0); // Blocking wait for SIGKILL to ensure cleanup
        RCLCPP_INFO(this->get_logger(), "Rosbag process (PID: %d) forcibly terminated during removal.", bag_pid_);
        bag_pid_ = -1;
    }
  }

  // Remove the current bag directory
  if (std::filesystem::exists(current_bag_path_)) {
    try {
      std::filesystem::remove_all(current_bag_path_);
      RCLCPP_INFO(this->get_logger(), "Removed bag directory: %s", current_bag_path_.c_str());
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to remove bag directory '%s': %s", current_bag_path_.c_str(), e.what());
      previous_state_ = current_state_;
      current_state_ = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      throw std::runtime_error("Failed to remove bag directory");
    }
  }

  previous_bag_id_ = current_bag_id_;
  if (current_bag_id_ > 0) {
    current_bag_id_ = current_bag_id_ - 1;
  } else {
    current_bag_id_ = 0;
  }
  previous_bag_name_ = current_bag_name_;
  current_bag_name_ = "episode_" + std::to_string(current_bag_id_); // Adjust as needed based on new ID logic
  previous_bag_path_ = current_bag_path_;
  current_bag_path_ = current_task_path_ + "/" + current_bag_name_;

  previous_state_ = current_state_;
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;

  RCLCPP_INFO(this->get_logger(), "Rosbag removed successfully");
}

void RosbagCollection::saveRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Saving rosbag...");

  if (bag_pid_ == -1) {
    RCLCPP_WARN(this->get_logger(), "No rosbag process to terminate (bag_pid_ == -1)");
    previous_state_ = current_state_;
    current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Sending SIGINT to rosbag process (PID: %d) for saving...", bag_pid_);
  // Send SIGINT for graceful termination
  if (kill(bag_pid_, SIGINT) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send SIGINT to rosbag process with PID %d: %s", bag_pid_, strerror(errno));
    previous_state_ = current_state_;
    current_state_ = sobits_interfaces::action::VlaRecordState_Result::ERROR;
    throw std::runtime_error("Failed to terminate rosbag process for saving");
  }

  int status = 0;
  pid_t result = 0;
  // Increased timeout for rosbag to gracefully exit and flush its cache
  int timeout_seconds = 20; // Changed from 5 to 20 seconds
  int poll_interval_ms = 200; // Check every 200 ms

  RCLCPP_INFO(this->get_logger(), "Waiting for rosbag process (PID: %d) to terminate for saving (timeout: %d seconds)...", bag_pid_, timeout_seconds);

  for (int i = 0; i < (timeout_seconds * 1000) / poll_interval_ms; ++i) {
      result = waitpid(bag_pid_, &status, WNOHANG); // Use WNOHANG
      if (result == bag_pid_) { // Child has terminated
          if (WIFEXITED(status)) {
              RCLCPP_INFO(this->get_logger(), "Rosbag process (PID: %d) exited with status: %d", bag_pid_, WEXITSTATUS(status));
          } else if (WIFSIGNALED(status)) {
              RCLCPP_INFO(this->get_logger(), "Rosbag process (PID: %d) terminated by signal: %d", bag_pid_, WTERMSIG(status));
          }
          bag_pid_ = -1; // Reset PID since process is gone
          break; // Exit loop
      } else if (result == 0) { // Child is still running
          std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
      } else { // Error from waitpid
          RCLCPP_ERROR(this->get_logger(), "Error waiting for rosbag process (PID: %d) during saving: %s", bag_pid_, strerror(errno));
          break; // Exit loop on error
      }
  }

  if (bag_pid_ != -1) { // If it's still not -1, it means the process didn't terminate within the timeout
      RCLCPP_WARN(this->get_logger(), "Rosbag process (PID: %d) did not terminate gracefully within timeout (%d seconds) during saving. Sending SIGKILL.", bag_pid_, timeout_seconds);
      if (kill(bag_pid_, SIGKILL) == -1) { // Force terminate
          RCLCPP_ERROR(this->get_logger(), "Failed to send SIGKILL to rosbag process with PID %d: %s", bag_pid_, strerror(errno));
      }
      // Even after SIGKILL, wait for it to be reaped to avoid zombie processes
      waitpid(bag_pid_, &status, 0); // Blocking wait for SIGKILL to ensure cleanup
      RCLCPP_INFO(this->get_logger(), "Rosbag process (PID: %d) forcibly terminated during saving.", bag_pid_);
      bag_pid_ = -1;
  }

  previous_state_ = current_state_;
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;
  RCLCPP_INFO(this->get_logger(), "Rosbag saved successfully");
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

  // Save the YAML node to a file
  std::string yaml_file_path = rosbag_info_.recording_dir + "/recorded_bags_meta.yaml";
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

void RosbagCollection::updateRosbagYaml()
{
  RCLCPP_INFO(this->get_logger(), "Updating rosbag YAML file...");

  // Load the existing YAML file
  std::string yaml_file_path = rosbag_info_.recording_dir + "/recorded_bags_meta.yaml";
  YAML::Node yaml_node;
  try {
    yaml_node = YAML::LoadFile(yaml_file_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
    throw std::runtime_error("Failed to load YAML file");
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
  yaml_node["recorded_bags"][current_task_label]["gamepad"] = gamepad_name_;

  // Save the updated YAML node to the file
  try {
    std::ofstream yaml_file(yaml_file_path);
    if (!yaml_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file for writing: %s", yaml_file_path.c_str());
      throw std::runtime_error("Failed to open YAML file for writing");
    }
    yaml_file << yaml_node;
    yaml_file.close();
    RCLCPP_DEBUG(this->get_logger(), "Updated rosbag YAML file: %s", yaml_file_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update rosbag YAML file: %s", e.what());
    throw std::runtime_error("Failed to update rosbag YAML file");
  }

  return;
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

void RosbagCollection::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<sobits_interfaces::action::VlaRecordState>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<sobits_interfaces::action::VlaRecordState::Result>();

  if (current_task_name_ == previous_task_name_){
    RCLCPP_WARN(this->get_logger(), "Please update the task name before starting a new recording");
    result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
    goal_handle->abort(result);
    return;
  }

  if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::RECORD) {
    if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::STOPPED) {
      RCLCPP_WARN(this->get_logger(), "Cannot start recording while already in state: %d", current_state_);
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    createRosbag();
    result->status = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Recording started successfully");
  } else if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::SAVE) {
    if (current_state_ == sobits_interfaces::action::VlaRecordState_Result::STOPPED) {
      RCLCPP_WARN(this->get_logger(), "Cannot save recording while not in RECORDING or PAUSED state");
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    saveRosbag();
    result->status = sobits_interfaces::action::VlaRecordState_Result::STOPPED;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Recording saved successfully");
  } else if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::DELETE) {
    removeRosbag();
    result->status = sobits_interfaces::action::VlaRecordState_Result::STOPPED;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Recording deleted successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown command received: %d", goal->command);
    result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
    goal_handle->abort(result);
    return;
  }
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
    previous_task_id_ = current_task_id_;
    current_task_id_ = 0;

    previous_task_name_ = current_task_name_;
    current_task_name_ = request->label;

    previous_task_path_ = current_task_path_;
    current_task_path_ = rosbag_info_.recording_dir + "/" + current_task_name_;
    std::replace(current_task_path_.begin(), current_task_path_.end(), ' ', '_');
    std::transform(current_task_path_.begin(), current_task_path_.end(), current_task_path_.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    RCLCPP_INFO(this->get_logger(), "Updated task name from '%s' to '%s'", previous_task_name_.c_str(), current_task_name_.c_str());

    previous_bag_id_ = current_bag_id_;
    current_bag_id_ = 0;

    previous_bag_name_ = current_bag_name_;
    current_bag_name_ = "episode_" + std::to_string(current_bag_id_) + "_" + std::to_string(this->now().nanoseconds());
    std::replace(current_bag_name_.begin(), current_bag_name_.end(), ' ', '_');
    std::transform(current_bag_name_.begin(), current_bag_name_.end(), current_bag_name_.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    previous_bag_name_ = current_bag_name_;

    previous_bag_path_ = current_bag_path_;
    current_bag_path_ = current_task_path_ + "/" + current_bag_name_;

    // Update the rosbag YAML file
    // TODO: Call when rosbag with the new task name is created
    updateRosbagYaml();
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
