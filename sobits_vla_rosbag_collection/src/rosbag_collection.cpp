#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

#include <iostream>     // For std::cerr
#include <filesystem>   // For std::filesystem operations
#include <algorithm>    // For std::replace, std::transform
#include <fstream>      // For std::ofstream
#include <string>       // For std::string
#include <vector>       // For std::vector
#include <set>          // For topic deduplication in buildTopicList()

#include "rosbag2_cpp/writer.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>

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
    this->get_name() + std::string("/vla_record_state"),
    std::bind(&RosbagCollection::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RosbagCollection::handleCancel, this, std::placeholders::_1),
    std::bind(&RosbagCollection::handleAccepted, this, std::placeholders::_1));

  // Initialize Service Server for Tasks
  task_update_service_ = this->create_service<sobits_interfaces::srv::VlaUpdateTask>(
    this->get_name() + std::string("/vla_task_update"),
    std::bind(&RosbagCollection::taskUpdateCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize Service Server for Subtasks (long-horizon)
  subtask_update_service_ = this->create_service<sobits_interfaces::srv::VlaUpdateTask>(
    this->get_name() + std::string("/vla_subtask_update"),
    std::bind(&RosbagCollection::subtaskUpdateCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Declare and get parameters
  // (1) Robot info parameters
  this->declare_parameter<std::string>("robot_info.name", "sobit_robot");
  this->declare_parameter<std::string>("robot_info.version", "1.0.0");
  this->declare_parameter<std::string>("robot_info.morphology.type", "mobile_manipulator");
  this->declare_parameter<std::string>("robot_info.morphology.joint_states_topic", "/joint_states");
  this->declare_parameter<std::vector<std::string>>("robot_info.morphology.parts", std::vector<std::string>{"base", "arm", "gripper"});
  
  robot_info_.name = this->get_parameter("robot_info.name").as_string();
  robot_info_.version = this->get_parameter("robot_info.version").as_string();
  robot_info_.morphology = this->get_parameter("robot_info.morphology.type").as_string();
  robot_info_.joint_states_topic = this->get_parameter("robot_info.morphology.joint_states_topic").as_string();
  
  robot_info_.parts = this->get_parameter("robot_info.morphology.parts").as_string_array();
  robot_info_.joint_names.clear();
  for (const auto & part : robot_info_.parts) {
    RCLCPP_INFO(this->get_logger(), "Robot part: %s", part.c_str());
    this->declare_parameter<bool>("robot_info.morphology." + part + ".is_actionable", false);
    this->declare_parameter<std::string>("robot_info.morphology." + part + ".command_topic", "");
    this->declare_parameter<std::string>("robot_info.morphology." + part + ".state_topic", "");
    this->declare_parameter<std::vector<std::string>>("robot_info.morphology." + part + ".actions", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("robot_info.morphology." + part + ".joint_names", std::vector<std::string>{});
    robot_info_.is_actionable[part] = this->get_parameter("robot_info.morphology." + part + ".is_actionable").as_bool();
    robot_info_.part_command_topic[part] = this->get_parameter("robot_info.morphology." + part + ".command_topic").as_string();
    robot_info_.part_state_topic[part] = this->get_parameter("robot_info.morphology." + part + ".state_topic").as_string();
    robot_info_.part_actions[part] = this->get_parameter("robot_info.morphology." + part + ".actions").as_string_array();
    robot_info_.joint_names[part] = this->get_parameter("robot_info.morphology." + part + ".joint_names").as_string_array();
    
    // Only fetch mobile_base/legs specific properties if it is the target part
    if (part == "mobile_base" || part == "legs") {
        this->declare_parameter<bool>("robot_info.morphology." + part + ".has_cmd_vel_y", false);
        this->declare_parameter<bool>("robot_info.morphology." + part + ".has_cmd_vel_z", false);
        this->declare_parameter<std::string>("robot_info.morphology." + part + ".cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<std::string>("robot_info.morphology." + part + ".odom_topic", "");
        
        robot_info_.part_has_cmd_vel_y[part] = this->get_parameter("robot_info.morphology." + part + ".has_cmd_vel_y").as_bool();
        robot_info_.part_has_cmd_vel_z[part] = this->get_parameter("robot_info.morphology." + part + ".has_cmd_vel_z").as_bool();
        robot_info_.part_cmd_vel_topic[part] = this->get_parameter("robot_info.morphology." + part + ".cmd_vel_topic").as_string();
        robot_info_.part_odom_topic[part] = this->get_parameter("robot_info.morphology." + part + ".odom_topic").as_string();
    }
  }
  this->declare_parameter<std::vector<std::string>>("robot_info.sensors.types", std::vector<std::string>{"camera", "lidar", "imu"});
  robot_info_.sensor_types = this->get_parameter("robot_info.sensors.types").as_string_array();
    robot_info_.sensor_names.clear();
  robot_info_.sensor_models.clear();
  robot_info_.sensor_topics.clear();
  for (const auto & sensor_type : robot_info_.sensor_types) {
    RCLCPP_INFO(this->get_logger(), "Robot sensor: %s", sensor_type.c_str());
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type + ".names", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type + ".models", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type + ".topics", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type + ".info_topics", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("robot_info.sensors." + sensor_type + ".compressed_topics", std::vector<std::string>{});
    robot_info_.sensor_names[sensor_type]             = this->get_parameter("robot_info.sensors." + sensor_type + ".names").as_string_array();
    robot_info_.sensor_models[sensor_type]            = this->get_parameter("robot_info.sensors." + sensor_type + ".models").as_string_array();
    robot_info_.sensor_topics[sensor_type]            = this->get_parameter("robot_info.sensors." + sensor_type + ".topics").as_string_array();
    robot_info_.sensor_info_topics[sensor_type]       = this->get_parameter("robot_info.sensors." + sensor_type + ".info_topics").as_string_array();
    robot_info_.sensor_compressed_topics[sensor_type] = this->get_parameter("robot_info.sensors." + sensor_type + ".compressed_topics").as_string_array();
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
  this->declare_parameter<std::vector<std::string>>("rosbag_config.additional_topics", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.additional_services", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("rosbag_config.additional_actions", std::vector<std::string>{});
  this->declare_parameter<std::string>("rosbag_config.conversion_format", "mcap");
  this->declare_parameter<std::string>("rosbag_config.compression_format", "zstd");
  this->declare_parameter<std::string>("rosbag_config.compression_mode", "none");
  this->declare_parameter<std::string>("rosbag_config.rmw_serialization_format", "cdr");
  rosbag_info_.recording_dir        = this->get_parameter("rosbag_config.record_directory").as_string();
  rosbag_info_.fps                  = this->get_parameter("rosbag_config.fps").as_int();
  rosbag_info_.sync_threshold       = this->get_parameter("rosbag_config.sync_threshold").as_double();
  rosbag_info_.additional_topics    = this->get_parameter("rosbag_config.additional_topics").as_string_array();
  rosbag_info_.additional_services  = this->get_parameter("rosbag_config.additional_services").as_string_array();
  rosbag_info_.additional_actions   = this->get_parameter("rosbag_config.additional_actions").as_string_array();
  rosbag_info_.conversion_format    = this->get_parameter("rosbag_config.conversion_format").as_string();
  rosbag_info_.compression_format   = this->get_parameter("rosbag_config.compression_format").as_string();
  rosbag_info_.compression_mode     = this->get_parameter("rosbag_config.compression_mode").as_string();
  rosbag_info_.rmw_serialization_format = this->get_parameter("rosbag_config.rmw_serialization_format").as_string();

  // (4) Gamepad parameters
  this->declare_parameter<std::string>("gamepad_config.name", "default_gamepad");
  gamepad_name_ = this->get_parameter("gamepad_config.name").as_string();

  // Subscribe to info_topics per sensor type to obtain camera dimension
  for (const auto & sensor_type : robot_info_.sensor_types) {
    const auto & info_topics = robot_info_.sensor_info_topics[sensor_type];
    for (const auto & cam_info_topic : info_topics) {
      if (cam_info_topic.empty()) continue;
      RCLCPP_INFO(this->get_logger(), "Subscribing to sniff dimensions: %s", cam_info_topic.c_str());
      camera_info_subs_[cam_info_topic] = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          cam_info_topic,
          rclcpp::QoS(1).best_effort(),
          [this, cam_info_topic](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
              this->cameraInfoCallback(msg, cam_info_topic);
          }
      );
    }
  }

  // Init values
  current_state_      = sobits_interfaces::action::VlaRecordState_Result::STOPPED; // PAUSED, RECORDING, STOPPED, ERROR
  previous_state_     = current_state_;

  current_task_name_  = "default task";
  previous_task_name_ = current_task_name_;
  
  current_task_dir_name_ = current_task_name_ + "_" + getTimestampString();
  std::replace(current_task_dir_name_.begin(), current_task_dir_name_.end(), ' ', '_');
  std::transform(current_task_dir_name_.begin(), current_task_dir_name_.end(), current_task_dir_name_.begin(),
                 [](unsigned char c) { return std::tolower(c); });
                 
  current_task_path_  = rosbag_info_.recording_dir + "/" + current_task_dir_name_;
  previous_task_path_ = current_task_path_;

  current_bag_name_   = "episode_" + getTimestampString();
  previous_bag_name_  = current_bag_name_;
  current_bag_path_   = current_task_path_ + "/" + current_bag_name_;
  previous_bag_path_  = current_bag_path_;

  // (5) Internal State
  current_subtask_name_ = "";
  current_episode_subtasks_.clear();
  
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
  if (rosbag_info_.additional_services.empty()) {
    RCLCPP_WARN(this->get_logger(), "No services to record specified in the rosbag configuration");
  } // TODO: From Jazzy services can be recorded, but not in Humble
  if (rosbag_info_.additional_actions.empty()) {
    RCLCPP_INFO(this->get_logger(), "No additional actions to record specified.");
  }

  // Build the topic list once and cache it
  buildTopicList();

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

  // Create (or verify + append to) the rosbag YAML file
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

std::string RosbagCollection::getTimestampString()
{
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  return ss.str();
}

void RosbagCollection::buildTopicList()
{
  std::vector<std::string> all_topics;

  // Sensor topics: base + optional compressed + optional cam_info variants
  for (const auto & sensor_type : robot_info_.sensor_types) {
    for (const auto & topic : robot_info_.sensor_topics[sensor_type]) {
      all_topics.push_back(topic);
    }
    // Compressed topics from the explicit compressed_topics list
    for (const auto & compressed_topic : robot_info_.sensor_compressed_topics[sensor_type]) {
      if (!compressed_topic.empty()) {
        all_topics.push_back(compressed_topic);
      }
    }
    // Camera info topics from the explicit info_topics list
    for (const auto & info_topic : robot_info_.sensor_info_topics[sensor_type]) {
      if (!info_topic.empty()) {
        all_topics.push_back(info_topic);
      }
    }
  }

  // Morphology: joint_states, cmd_vel, odom, and per-part explicit topics
  if (!robot_info_.joint_states_topic.empty()) {
    all_topics.push_back(robot_info_.joint_states_topic);
  }
  for (const auto & part : robot_info_.parts) {
    if (robot_info_.part_cmd_vel_topic.count(part) && !robot_info_.part_cmd_vel_topic.at(part).empty()) {
      all_topics.push_back(robot_info_.part_cmd_vel_topic.at(part));
    }
    if (robot_info_.part_odom_topic.count(part) && !robot_info_.part_odom_topic.at(part).empty()) {
      all_topics.push_back(robot_info_.part_odom_topic.at(part));
    }
    if (!robot_info_.part_command_topic[part].empty()) {
      all_topics.push_back(robot_info_.part_command_topic[part]);
    }
    if (!robot_info_.part_state_topic[part].empty()) {
      all_topics.push_back(robot_info_.part_state_topic[part]);
    }
  }

  // Additional explicit topics from config
  for (const auto & topic : rosbag_info_.additional_topics) {
    if (!topic.empty()) {
      all_topics.push_back(topic);
    }
  }

  // Deduplicate while preserving order
  std::vector<std::string> deduped;
  std::set<std::string> seen;
  for (const auto & t : all_topics) {
    if (seen.insert(t).second) {
      deduped.push_back(t);
    }
  }

  rosbag_info_.topics_to_record = std::move(deduped);
  RCLCPP_INFO(this->get_logger(), "Cached %zu topics to record.", rosbag_info_.topics_to_record.size());
}

void RosbagCollection::createRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Starting recording...");

  std::lock_guard<std::mutex> lock(recorder_mutex_);

  if (is_recording_) {
    RCLCPP_WARN(this->get_logger(), "Already recording!");
    return;
  }

  previous_task_path_ = current_task_path_;
  current_task_path_ = rosbag_info_.recording_dir + "/" + current_task_dir_name_;

  previous_bag_name_ = current_bag_name_;
  current_bag_name_ = "episode_" + getTimestampString();

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
  current_episode_subtasks_.clear();

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
  record_options.rmw_serialization_format = rosbag_info_.rmw_serialization_format;
  
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
  
  // Wipe from metadata BEFORE rolling back names
  removeEpisodeFromYaml();

  current_bag_name_ = previous_bag_name_;
  current_bag_path_ = previous_bag_path_;

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

  recorder_node_.reset(); 

  if (recorder_thread_.joinable()) {
       recorder_thread_.join();
  }

  previous_state_ = current_state_;
  current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;
  
  updateEpisodeYaml();

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
  yaml_node["robot_info"]["morphology"]["joint_states_topic"] = robot_info_.joint_states_topic;
  yaml_node["robot_info"]["morphology"]["parts"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & part : robot_info_.parts) {
    yaml_node["robot_info"]["morphology"]["parts"].push_back(part);
    yaml_node["robot_info"]["morphology"][part]["is_actionable"] = robot_info_.is_actionable[part];
    
    // Serialize per-part command/state topics and actions (for non-locomotion parts)
    if (!robot_info_.part_command_topic[part].empty()) {
      yaml_node["robot_info"]["morphology"][part]["command_topic"] = robot_info_.part_command_topic[part];
    }
    if (!robot_info_.part_state_topic[part].empty()) {
      yaml_node["robot_info"]["morphology"][part]["state_topic"] = robot_info_.part_state_topic[part];
    }
    if (!robot_info_.part_actions[part].empty()) {
      yaml_node["robot_info"]["morphology"][part]["actions"] = YAML::Node(YAML::NodeType::Sequence);
      for (const auto & action : robot_info_.part_actions[part]) {
        yaml_node["robot_info"]["morphology"][part]["actions"].push_back(action);
      }
    }
    
    if (part == "mobile_base" || part == "legs") {
        yaml_node["robot_info"]["morphology"][part]["has_cmd_vel_y"] = robot_info_.part_has_cmd_vel_y[part];
        yaml_node["robot_info"]["morphology"][part]["has_cmd_vel_z"] = robot_info_.part_has_cmd_vel_z[part];
        yaml_node["robot_info"]["morphology"][part]["cmd_vel_topic"] = robot_info_.part_cmd_vel_topic[part];
        yaml_node["robot_info"]["morphology"][part]["odom_topic"] = robot_info_.part_odom_topic[part];
    }
    
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
    yaml_node["robot_info"]["sensors"][sensor_type]["topics"] = YAML::Node(YAML::NodeType::Sequence);
    
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
    for (const auto & sensor_topic : robot_info_.sensor_topics[sensor_type]) {
      yaml_node["robot_info"]["sensors"][sensor_type]["topics"].push_back(sensor_topic);
    }
    for (const auto & compressed_topic : robot_info_.sensor_compressed_topics[sensor_type]) {
      yaml_node["robot_info"]["sensors"][sensor_type]["compressed_topics"].push_back(compressed_topic);
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
  std::string current_task_label = current_task_dir_name_;

  // Add the current task name to the YAML file
  yaml_node["recorded_bags"]["tasks_list"].push_back(current_task_label);
  yaml_node["recorded_bags"]["tasks"][current_task_label]["label"] = current_task_name_;
  yaml_node["recorded_bags"]["tasks"][current_task_label]["bag_dir"] = current_task_path_;
  yaml_node["recorded_bags"]["tasks"][current_task_label]["gamepad"] = gamepad_name_;

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

void RosbagCollection::updateEpisodeYaml()
{
  RCLCPP_INFO(this->get_logger(), "Updating episode in rosbag YAML file...");

  // Close any running subtask
  if (!current_episode_subtasks_.empty() && current_episode_subtasks_.back().end_timestamp == 0.0) {
    current_episode_subtasks_.back().end_timestamp = this->now().seconds();
  }

  // Load the existing YAML file
  std::string yaml_file_path = rosbag_info_.recording_dir + "/recorded_bags_meta.yaml";
  YAML::Node yaml_node;
  try {
    if (std::filesystem::exists(yaml_file_path)) {
      yaml_node = YAML::LoadFile(yaml_file_path);
    } else {
      RCLCPP_WARN(this->get_logger(), "YAML file not found, creating a new one");
      createRosbagYaml();
      yaml_node = YAML::LoadFile(yaml_file_path);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
    return;
  }

  std::string current_task_label = current_task_dir_name_;

  // Ensure task exists
  if (yaml_node["recorded_bags"]["tasks"][current_task_label].IsDefined()) {
    
    // 1. episodes_list
    if (!yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].IsDefined()) {
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"] = YAML::Node(YAML::NodeType::Sequence);
    }
    // Push if not already in list (for safety, though episode names are unique)
    bool episode_in_list = false;
    for (YAML::const_iterator it = yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].begin(); it != yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].end(); ++it) {
      if (it->as<std::string>() == current_bag_name_) { episode_in_list = true; break; }
    }
    if (!episode_in_list) {
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].push_back(current_bag_name_);
    }

    // 2. episodes map
    if (!yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"].IsDefined()) {
       yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"] = YAML::Node(YAML::NodeType::Map);
    }
    
    // Create/Update the specific episode
    YAML::Node episode_node = YAML::Node(YAML::NodeType::Map);
    episode_node["bag_path"] = current_bag_path_;
    
    // 3. Subtasks logic
    if (!current_episode_subtasks_.empty()) {
      YAML::Node subtasks_list = YAML::Node(YAML::NodeType::Sequence);
      YAML::Node subtasks_map = YAML::Node(YAML::NodeType::Map);

      for (size_t i = 0; i < current_episode_subtasks_.size(); ++i) {
        std::string subtask_key = current_episode_subtasks_[i].key;
        subtasks_list.push_back(subtask_key);

        YAML::Node single_subtask;
        single_subtask["label"] = current_episode_subtasks_[i].label;
        single_subtask["start_timestamp"] = current_episode_subtasks_[i].start_timestamp;
        single_subtask["end_timestamp"] = current_episode_subtasks_[i].end_timestamp;
        
        subtasks_map[subtask_key] = single_subtask;
      }

      episode_node["subtasks_list"] = subtasks_list;
      episode_node["subtasks"] = subtasks_map;
    }

    yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"][current_bag_name_] = episode_node;
  }

  // Save the updated YAML node to the file
  try {
    std::ofstream yaml_file(yaml_file_path);
    yaml_file << yaml_node;
    yaml_file.close();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update episode YAML file: %s", e.what());
  }
}

void RosbagCollection::removeEpisodeFromYaml()
{
  RCLCPP_INFO(this->get_logger(), "Removing episode from rosbag YAML file: %s", current_bag_name_.c_str());

  std::string yaml_file_path = rosbag_info_.recording_dir + "/recorded_bags_meta.yaml";
  if (!std::filesystem::exists(yaml_file_path)) {
    return;
  }

  YAML::Node yaml_node;
  try {
    yaml_node = YAML::LoadFile(yaml_file_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file for removal: %s", e.what());
    return;
  }

  std::string current_task_label = current_task_dir_name_;

  if (yaml_node["recorded_bags"]["tasks"][current_task_label].IsDefined()) {
    
    // Remove from episodes_list
    if (yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].IsDefined()) {
      YAML::Node old_list = yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"];
      YAML::Node new_list = YAML::Node(YAML::NodeType::Sequence);
      
      for (YAML::const_iterator it = old_list.begin(); it != old_list.end(); ++it) {
        if (it->as<std::string>() != current_bag_name_) {
          new_list.push_back(it->as<std::string>());
        }
      }
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"] = new_list;
    }

    // Remove from episodes map
    if (yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"].IsDefined()) {
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"].remove(current_bag_name_);
    }

    // Save
    try {
      std::ofstream yaml_file(yaml_file_path);
      yaml_file << yaml_node;
      yaml_file.close();
      RCLCPP_INFO(this->get_logger(), "Successfully removed episode from YAML.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save YAML file after removal: %s", e.what());
    }
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

  if (!task_has_been_set_) {
    RCLCPP_WARN(this->get_logger(), "Please set a task name via the vla_task_update service before starting a recording.");
    result->status = sobits_interfaces::action::VlaRecordState_Result::ERROR;
    goal_handle->abort(result);
    return;
  }

  if (current_state_ == sobits_interfaces::action::VlaRecordState_Result::ERROR) {
    RCLCPP_WARN(this->get_logger(), "Cannot start/resume recording while in ERROR state");
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
    previous_task_name_ = current_task_name_;
    current_task_name_ = request->label;

    current_task_dir_name_ = current_task_name_ + "_" + getTimestampString();
    std::replace(current_task_dir_name_.begin(), current_task_dir_name_.end(), ' ', '_');
    std::transform(current_task_dir_name_.begin(), current_task_dir_name_.end(), current_task_dir_name_.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    previous_task_path_ = current_task_path_;
    current_task_path_ = rosbag_info_.recording_dir + "/" + current_task_dir_name_;
    RCLCPP_INFO(this->get_logger(), "Updated task name from '%s' to '%s'", previous_task_name_.c_str(), current_task_dir_name_.c_str());

    previous_bag_name_ = current_bag_name_;
    current_bag_name_ = "episode_" + getTimestampString();
    previous_bag_path_ = current_bag_path_;
    current_bag_path_ = current_task_path_ + "/" + current_bag_name_;

    // Update the rosbag YAML file
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
  task_has_been_set_ = true;
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

  double current_time_sec = this->now().seconds();

  // Close the previous subtask if one exists
  if (!current_episode_subtasks_.empty()) {
    current_episode_subtasks_.back().end_timestamp = current_time_sec;
  }

  current_subtask_name_ = request->label;
  
  SubtaskInfo new_subtask;
  new_subtask.key = "subtask_" + getTimestampString();
  new_subtask.label = current_subtask_name_;
  new_subtask.start_timestamp = current_time_sec;
  new_subtask.end_timestamp = 0.0; // Will be updated on the next subtask or bag save
  
  current_episode_subtasks_.push_back(new_subtask);

  RCLCPP_INFO(this->get_logger(), "Subtask name updated successfully to '%s'", current_subtask_name_.c_str());
  response->success = true;
  response->message = "Subtask name updated successfully";
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
