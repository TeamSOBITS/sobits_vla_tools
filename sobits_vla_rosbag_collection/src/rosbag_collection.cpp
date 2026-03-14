#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

#include <iostream>     // For std::cerr
#include <filesystem>   // For std::filesystem operations
#include <algorithm>    // For std::replace, std::transform
#include <fstream>      // For std::ofstream
#include <string>       // For std::string
#include <vector>       // For std::vector

#include "rosbag2_cpp/writer.hpp"

// Assuming sobits_interfaces is available and properly defined
// and that YAML::Node and related are from the yaml-cpp library.

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

  // Initialize Service Server for Tasks
  task_update_service_ = this->create_service<sobits_interfaces::srv::VlaUpdateTask>(
    "vla_task_update",
    std::bind(&RosbagCollection::taskUpdateCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize Service Server for Subtasks (long-horizon)
  subtask_update_service_ = this->create_service<sobits_interfaces::srv::VlaUpdateTask>(
    "vla_subtask_update",
    std::bind(&RosbagCollection::subtaskUpdateCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Declare and get parameters (TODO: make it into a new function)
  // (1) Robot info parameters
  this->declare_parameter<std::string>("robot_info.name", "sobit_robot");
  this->declare_parameter<std::string>("robot_info.version", "1.0.0");
  this->declare_parameter<std::string>("robot_info.morphology.type", "mobile_manipulator");
  this->declare_parameter<bool>("robot_info.morphology.has_mobile_base", true);
  this->declare_parameter<bool>("robot_info.morphology.has_cmd_vel_y", false);
  this->declare_parameter<std::string>("robot_info.morphology.joint_states_topic", "/joint_states");
  this->declare_parameter<std::string>("robot_info.morphology.cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::vector<std::string>>("robot_info.morphology.parts", std::vector<std::string>{"base", "arm", "gripper"});
  
  robot_info_.name = this->get_parameter("robot_info.name").as_string();
  robot_info_.version = this->get_parameter("robot_info.version").as_string();
  robot_info_.morphology = this->get_parameter("robot_info.morphology.type").as_string();
  robot_info_.has_mobile_base = this->get_parameter("robot_info.morphology.has_mobile_base").as_bool();
  robot_info_.has_cmd_vel_y = this->get_parameter("robot_info.morphology.has_cmd_vel_y").as_bool();
  robot_info_.joint_states_topic = this->get_parameter("robot_info.morphology.joint_states_topic").as_string();
  robot_info_.cmd_vel_topic = this->get_parameter("robot_info.morphology.cmd_vel_topic").as_string();
  
  robot_info_.parts = this->get_parameter("robot_info.morphology.parts").as_string_array();
  robot_info_.joint_names.clear();
  for (const auto & part : robot_info_.parts) {
    RCLCPP_INFO(this->get_logger(), "Robot part: %s", part.c_str());
    this->declare_parameter<bool>("robot_info.morphology." + part + ".is_actionable", false);
    this->declare_parameter<std::vector<std::string>>("robot_info.morphology." + part + ".joint_names", std::vector<std::string>{});
    robot_info_.is_actionable[part] = this->get_parameter("robot_info.morphology." + part + ".is_actionable").as_bool();
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
  this->declare_parameter<std::string>("rosbag_config.record_directory", "");
  this->declare_parameter<int>("rosbag_config.fps", 10);
  this->declare_parameter<double>("rosbag_config.sync_threshold", 0.1);
  this->declare_parameter<std::vector<std::string>>("rosbag_config.topics_to_record", std::vector<std::string>{"/topic1", "/topic2"});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.services_to_record", std::vector<std::string>{"/topic1", "/topic2"});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.actions_to_record", std::vector<std::string>{"/topic1", "/topic2"});
  this->declare_parameter<std::string>("rosbag_config.conversion_format", "mcap");
  this->declare_parameter<std::string>("rosbag_config.compression_format", "zstd");
  this->declare_parameter<std::string>("rosbag_config.compression_mode", "none");
  rosbag_info_.recording_dir      = this->get_parameter("rosbag_config.record_directory").as_string();
  rosbag_info_.fps                = this->get_parameter("rosbag_config.fps").as_int();
  rosbag_info_.sync_threshold     = this->get_parameter("rosbag_config.sync_threshold").as_double();
  rosbag_info_.topics_to_record   = this->get_parameter("rosbag_config.topics_to_record").as_string_array();
  rosbag_info_.services_to_record = this->get_parameter("rosbag_config.services_to_record").as_string_array();
  rosbag_info_.actions_to_record  = this->get_parameter("rosbag_config.actions_to_record").as_string_array();
  rosbag_info_.conversion_format  = this->get_parameter("rosbag_config.conversion_format").as_string();
  rosbag_info_.compression_format = this->get_parameter("rosbag_config.compression_format").as_string();
  rosbag_info_.compression_mode   = this->get_parameter("rosbag_config.compression_mode").as_string();

  // (4) Gamepad parameters
  this->declare_parameter<std::string>("gamepad_config.name", "default_gamepad");
  gamepad_name_ = this->get_parameter("gamepad_config.name").as_string();

  // Sniff Camera Info if provided
  for (const std::string& topic : rosbag_info_.topics_to_record) {
    if (topic.find("camera_info") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "Subscribing to sniff dimensions: %s", topic.c_str());
      camera_info_subs_[topic] = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          topic,
          rclcpp::QoS(1).best_effort(),
          [this, topic](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
              this->cameraInfoCallback(msg, topic);
          }
      );
    }
  }

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

  // (5) Internal State
  current_subtask_name_ = "";
  
  rosbag_info_.rosbag_options = "";
  
  // Prepare the rosbag configuration (for record options)
  if (rosbag_info_.conversion_format.empty()) {
    RCLCPP_WARN(this->get_logger(), "No conversion format specified, using default 'sqlite3'");
    rosbag_info_.conversion_format = "sqlite3";
  } else {
    RCLCPP_INFO(this->get_logger(), "Using conversion format: %s", rosbag_info_.conversion_format.c_str());
  }
  if (rosbag_info_.compression_mode == "none" || rosbag_info_.compression_mode.empty()) {
    RCLCPP_INFO(this->get_logger(), "Output compression is disabled");
    rosbag_info_.compression_mode = "";
  } else {
    RCLCPP_INFO(this->get_logger(), "Output compression is enabled with mode: %s", rosbag_info_.compression_mode.c_str());
    RCLCPP_INFO(this->get_logger(), "Using compression format: %s", rosbag_info_.compression_format.c_str());
  }
  
  if (rosbag_info_.topics_to_record.empty()) {
    RCLCPP_WARN(this->get_logger(), "No topics to record specified in the rosbag configuration. Using all topics.");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Specific topics to record provided.");
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
  if (is_recording_) {
    try {
      saveRosbag();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error stopping recording in destructor: %s", e.what());
    }
  }
}

void RosbagCollection::createRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Starting recording...");

  std::lock_guard<std::mutex> lock(recorder_mutex_);

  if (is_recording_) {
    RCLCPP_WARN(this->get_logger(), "Already recording!");
    return;
  }

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

  // Clear subtasks
  current_subtask_name_ = "";

  // Set the current state to RECORDING
  previous_state_ = current_state_;
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::RECORDING;

  // Configure rosbag2 transport options
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = current_bag_path_;
  storage_options.storage_id = rosbag_info_.conversion_format;
  
  rosbag2_transport::RecordOptions record_options;
  if (rosbag_info_.topics_to_record.empty()) {
    record_options.all_topics = true;
  } else {
    record_options.topics = rosbag_info_.topics_to_record;
  }
  record_options.use_sim_time = this->get_parameter("use_sim_time").as_bool();
  
  if (!rosbag_info_.compression_mode.empty()) {
    record_options.compression_mode = rosbag_info_.compression_mode;
    record_options.compression_format = rosbag_info_.compression_format;
  }

  // Create a writer instance
  auto writer = std::make_shared<rosbag2_cpp::Writer>();

  // Create the recorder node and run it in a separate thread
  recorder_node_ = std::make_shared<rosbag2_transport::Recorder>(
    writer,
    storage_options,
    record_options,
    "rosbag2_recorder_node",
    rclcpp::NodeOptions()
  );

  is_recording_ = true;
  recorder_thread_ = std::thread([this]() {
    try {
      recorder_node_->record();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error during bag recording: %s", e.what());
      current_state_ = sobits_interfaces::action::VlaRecordState_Result::ERROR;
    }
  });

  RCLCPP_INFO(this->get_logger(), "Rosbag recording started successfully");
}


void RosbagCollection::removeRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Removing rosbag...");

  std::lock_guard<std::mutex> lock(recorder_mutex_);

  if (is_recording_) {
    RCLCPP_INFO(this->get_logger(), "Stopping recorder to remove bag...");
    is_recording_ = false;
    current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;

    if (recorder_thread_.joinable()) {
       recorder_node_.reset(); 
       if(recorder_thread_.joinable()) {
           recorder_thread_.join();
       }
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

  std::lock_guard<std::mutex> lock(recorder_mutex_);

  if (!is_recording_) {
    RCLCPP_WARN(this->get_logger(), "No rosbag process to terminate");
    previous_state_ = current_state_;
    current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Stopping recorder for saving...");
  is_recording_ = false;

  // The safest way is to destroy the recorder object, which guarantees the cache is flushed and bag is closed.
  recorder_node_.reset(); 

  if (recorder_thread_.joinable()) {
       recorder_thread_.join();
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
  yaml_node["robot_info"]["morphology"]["has_mobile_base"] = robot_info_.has_mobile_base;
  yaml_node["robot_info"]["morphology"]["has_cmd_vel_y"] = robot_info_.has_cmd_vel_y;
  yaml_node["robot_info"]["morphology"]["joint_states_topic"] = robot_info_.joint_states_topic;
  yaml_node["robot_info"]["morphology"]["cmd_vel_topic"] = robot_info_.cmd_vel_topic;
  yaml_node["robot_info"]["morphology"]["parts"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & part : robot_info_.parts) {
    yaml_node["robot_info"]["morphology"]["parts"].push_back(part);
    yaml_node["robot_info"]["morphology"][part]["is_actionable"] = robot_info_.is_actionable[part];
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
      
      // Inject inferred dimensions from the sniffed camera_info topics if present
      // Try to find the matching camera_info topic name. e.g "head_camera" -> "/head_camera/.../camera_info"
      for (const auto& pair : camera_dimensions_) {
        if (pair.first.find(sensor_name) != std::string::npos) {
          yaml_node["robot_info"]["sensors"][sensor_type]["properties"][sensor_name]["width"] = pair.second.first;
          yaml_node["robot_info"]["sensors"][sensor_type]["properties"][sensor_name]["height"] = pair.second.second;
          yaml_node["robot_info"]["sensors"][sensor_type]["properties"][sensor_name]["topic"] = pair.first;
          break; // Use the first match
        }
      }
    }
    for (const auto & sensor_model : robot_info_.sensor_models[sensor_type]) {
      yaml_node["robot_info"]["sensors"][sensor_type]["models"].push_back(sensor_model);
    }
  }

  yaml_node["convert_info"]["fps"] = rosbag_info_.fps;
  yaml_node["convert_info"]["sync_threshold"] = rosbag_info_.sync_threshold;

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

void RosbagCollection::updateSubtaskYaml(const std::string& subtask_name)
{
  RCLCPP_INFO(this->get_logger(), "Updating subtask in rosbag YAML file: %s", subtask_name.c_str());

  if (!is_recording_) {
    RCLCPP_WARN(this->get_logger(), "Cannot add subtask annotation while not recording.");
    return;
  }

  // Load the existing YAML file
  std::string yaml_file_path = rosbag_info_.recording_dir + "/recorded_bags_meta.yaml";
  YAML::Node yaml_node;
  try {
    if (std::filesystem::exists(yaml_file_path)) {
      yaml_node = YAML::LoadFile(yaml_file_path);
    } else {
      RCLCPP_WARN(this->get_logger(), "YAML file not found, creating a new node");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
    return;
  }

  std::string current_task_label = current_task_name_;
  std::replace(current_task_label.begin(), current_task_label.end(), ' ', '_');
  std::transform(current_task_label.begin(), current_task_label.end(), current_task_label.begin(),
                 [](unsigned char c) { return std::tolower(c); });
                 
  // Extract time from ROS time
  double current_time_sec = this->now().seconds();

  if (yaml_node["recorded_bags"][current_task_label].IsDefined()) {
    if (!yaml_node["recorded_bags"][current_task_label]["episodes"].IsDefined()) {
       yaml_node["recorded_bags"][current_task_label]["episodes"] = YAML::Node(YAML::NodeType::Map);
    }
    
    // Check if the current bag entry exists
    if (!yaml_node["recorded_bags"][current_task_label]["episodes"][current_bag_name_].IsDefined()) {
       yaml_node["recorded_bags"][current_task_label]["episodes"][current_bag_name_] = YAML::Node(YAML::NodeType::Map);
       yaml_node["recorded_bags"][current_task_label]["episodes"][current_bag_name_]["subtasks"] = YAML::Node(YAML::NodeType::Sequence);
    }
    
    YAML::Node subtask_node;
    subtask_node["name"] = subtask_name;
    subtask_node["timestamp"] = current_time_sec;
    yaml_node["recorded_bags"][current_task_label]["episodes"][current_bag_name_]["subtasks"].push_back(subtask_node);
  }

  // Save the updated YAML node to the file
  try {
    std::ofstream yaml_file(yaml_file_path);
    yaml_file << yaml_node;
    yaml_file.close();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update subtask YAML file: %s", e.what());
  }
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

  // This check seems problematic. If current_task_name_ is never different from previous_task_name_ initially,
  // this will always warn. Perhaps it should be checking if a task has been set at all.
  if (current_task_name_ == previous_task_name_){
    RCLCPP_WARN(this->get_logger(), "Please update the task name before starting a new recording");
    result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
    goal_handle->abort(result);
    return;
  }

  if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::RECORD) {
    if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::STOPPED && current_state_ != sobits_interfaces::action::VlaRecordState_Result::PAUSED) {
      RCLCPP_WARN(this->get_logger(), "Cannot start/resume recording while already in state: %d", current_state_);
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    
    if (current_state_ == sobits_interfaces::action::VlaRecordState_Result::STOPPED) {
      createRosbag();
      result->status = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Recording started successfully");
    } else if (current_state_ == sobits_interfaces::action::VlaRecordState_Result::PAUSED) {
      if (recorder_node_) {
        recorder_node_->resume();
        current_state_ = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
        result->status = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Recording resumed successfully");
      }
    }
  } else if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::PAUSE) {
    if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::RECORDING) {
      RCLCPP_WARN(this->get_logger(), "Cannot pause while not recording");
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    
    if (recorder_node_) {
      recorder_node_->pause();
      current_state_ = sobits_interfaces::action::VlaRecordState_Result::PAUSED;
      result->status = sobits_interfaces::action::VlaRecordState_Result::PAUSED;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Recording paused successfully");
    }
  } else if (goal->command == sobits_interfaces::action::VlaRecordState_Goal::RESUME) {
    if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::PAUSED) {
      RCLCPP_WARN(this->get_logger(), "Cannot resume while not paused");
      result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
      goal_handle->abort(result);
      return;
    }
    
    if (recorder_node_) {
      recorder_node_->resume();
      current_state_ = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
      result->status = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Recording resumed successfully");
    }
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
    previous_bag_name_ = current_bag_name_; // This line seems to re-assign current_bag_name_ to previous_bag_name_ after modification. Maybe intentional?

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

void RosbagCollection::subtaskUpdateCallback(
  const std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Request> request,
  std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received subtask update request: %s", request->label.c_str());
  
  if (current_state_ != sobits_interfaces::action::VlaRecordState_Result::RECORDING) {
    RCLCPP_WARN(this->get_logger(), "Cannot update subtask while not recording.");
    response->success = false;
    response->message = "Cannot update subtask while not recording.";
    return;
  }

  current_subtask_name_ = request->label;
  updateSubtaskYaml(current_subtask_name_);

  RCLCPP_INFO(this->get_logger(), "Subtask name updated successfully to '%s'", current_subtask_name_.c_str());
  response->success = true;
  response->message = "Subtask name updated successfully and recorded in subtask YAML file";
}

void RosbagCollection::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, const std::string topic_name)
{
  if (camera_dimensions_.find(topic_name) == camera_dimensions_.end()) {
    camera_dimensions_[topic_name] = {msg->width, msg->height};
    RCLCPP_INFO(this->get_logger(), "Captured dimensions for %s: %dx%d", topic_name.c_str(), msg->width, msg->height);
    // Unsubscribe after getting the info once
    camera_info_subs_.erase(topic_name);
  }
}

} // namespace sobits_vla

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sobits_vla::RosbagCollection)
