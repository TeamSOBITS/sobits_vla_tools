// Copyright (c) 2026, Team SOBITS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>          // For topic deduplication in buildTopicList()
#include <sstream>
#include <string>
#include <vector>

#include "rosbag2_cpp/writer.hpp"
#include "sobits_vla_rosbag_collection/bag_metadata_manager.hpp"
#include "sobits_vla_rosbag_collection/episode_lifecycle.hpp"
#include "sobits_vla_rosbag_collection/recording_monitor.hpp"
#include "sobits_vla_rosbag_collection/robot_descriptor_loader.hpp"
#include "sobits_vla_rosbag_collection/topic_builder.hpp"

namespace sobits_vla
{

RosbagCollection::RosbagCollection(const rclcpp::NodeOptions & options)
: Node("rosbag_collection", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing RosbagCollection Node...");

  // ~/ resolves under this node's own name, same result as get_name()+"/..."
  // but robust to a future rename.
  task_update_service_ = this->create_service<sobits_interfaces::srv::VlaUpdateTask>(
    "~/vla_task_update",
    std::bind(&RosbagCollection::taskUpdateCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  subtask_update_service_ = this->create_service<sobits_interfaces::srv::VlaUpdateTask>(
    "~/vla_subtask_update",
    std::bind(&RosbagCollection::subtaskUpdateCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  declareAndReadParameters();

  for (const auto & sensor_type : robot_info_.sensor_types) {
    const auto & info_topics = robot_info_.sensor_info_topics[sensor_type];
    for (const auto & cam_info_topic : info_topics) {
      if (cam_info_topic.empty()) {continue;}
      RCLCPP_INFO(this->get_logger(), "Subscribing to sniff dimensions: %s",
          cam_info_topic.c_str());
      camera_info_subs_[cam_info_topic] = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          cam_info_topic,
          rclcpp::QoS(1).best_effort(),
        [this, cam_info_topic](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          this->cameraInfoCallback(msg, cam_info_topic);
          }
      );
    }
  }

  current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;
  previous_state_ = current_state_;

  current_task_name_ = "default task";
  previous_task_name_ = current_task_name_;

  current_task_dir_name_ = current_task_name_ + "_" + getTimestampString();
  std::replace(current_task_dir_name_.begin(), current_task_dir_name_.end(), ' ', '_');
  std::transform(current_task_dir_name_.begin(), current_task_dir_name_.end(),
      current_task_dir_name_.begin(),
    [](unsigned char c) {return std::tolower(c);});

  current_task_path_ = rosbag_info_.recording_dir + "/" + current_task_dir_name_;

  current_bag_name_ = EpisodeLifecycle::makeBagName();
  previous_bag_name_ = current_bag_name_;
  current_bag_path_ = EpisodeLifecycle::makeBagPath(current_task_path_, current_bag_name_);
  previous_bag_path_ = current_bag_path_;

  current_subtask_name_ = "";
  current_episode_subtasks_.clear();

  rosbag_info_.rosbag_options = "";

  if (rosbag_info_.conversion_format.empty()) {
    RCLCPP_WARN(this->get_logger(), "No conversion format specified, using default 'sqlite3'");
    rosbag_info_.conversion_format = "sqlite3";
  } else {
    RCLCPP_INFO(this->get_logger(), "Using conversion format: %s",
        rosbag_info_.conversion_format.c_str());
  }
  if (rosbag_info_.compression_mode == "none" || rosbag_info_.compression_mode.empty()) {
    RCLCPP_INFO(this->get_logger(), "Output compression is disabled");
    rosbag_info_.compression_mode = "";
  } else {
    RCLCPP_INFO(this->get_logger(), "Output compression is enabled with mode: %s",
        rosbag_info_.compression_mode.c_str());
    RCLCPP_INFO(this->get_logger(), "Using compression format: %s",
        rosbag_info_.compression_format.c_str());
  }

  if (rosbag_info_.topics_to_record.empty()) {
    RCLCPP_WARN(this->get_logger(),
        "No topics to record specified in the rosbag configuration. Using all topics.");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Specific topics to record provided.");
  }
  if (rosbag_info_.additional_services.empty()) {
    RCLCPP_WARN(this->get_logger(), "No services to record specified in the rosbag configuration");
  }
  // TODO(MrKeith99): From Jazzy services can be recorded, but not in Humble
  if (rosbag_info_.additional_actions.empty()) {
    RCLCPP_INFO(this->get_logger(), "No additional actions to record specified.");
  }

  buildTopicList();

  validateTopics();

  if (!std::filesystem::exists(rosbag_info_.recording_dir)) {
    try {
      std::filesystem::create_directories(rosbag_info_.recording_dir);
      RCLCPP_INFO(this->get_logger(), "Created recording directory: %s",
          rosbag_info_.recording_dir.c_str());
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create recording directory: %s", e.what());
      throw std::runtime_error("Failed to create recording directory");
    }
  }

  if (min_disk_space_mb_ > 0) {
    try {
      auto space = std::filesystem::space(rosbag_info_.recording_dir);
      uint64_t free_mb = space.available / (1024 * 1024);
      RCLCPP_INFO(this->get_logger(), "Disk space: %lu MB free (minimum: %lu MB)", free_mb,
          min_disk_space_mb_);
      if (free_mb < min_disk_space_mb_) {
        RCLCPP_ERROR(this->get_logger(),
          "LOW DISK SPACE at startup! Free up space before recording.");
      }
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to check disk space: %s", e.what());
    }
  }

  bag_metadata_manager_ = std::make_unique<BagMetadataManager>(
    this,
    rosbag_info_.recording_dir,
    robot_info_,
    user_info_);

  episode_lifecycle_ = std::make_unique<EpisodeLifecycle>(
    rosbag_info_.conversion_format,
    [this](const std::string & msg) {RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());},
    [this](const std::string & msg) {RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());},
    [this](const std::string & msg) {RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());});

  recording_monitor_ = std::make_unique<RecordingMonitor>(
    this,
    robot_info_,
    rosbag_info_.recording_dir,
    expected_sensor_fps_,
    min_disk_space_mb_,
    max_episode_duration_sec_,
    timestamp_jump_threshold_sec_,
    [this]() {
      // join any previous auto-save before starting a new one (runs on timer thread)
      if (auto_save_thread_.joinable()) {auto_save_thread_.join();}
      auto alive = node_alive_;
      auto_save_thread_ = std::thread([this, alive]() {
        if (alive->load()) {this->saveRosbag();}
      });
    });

  command_service_ = this->create_service<sobits_interfaces::srv::VlaCommand>(
    command_service_name_,
    std::bind(&RosbagCollection::handleVlaCommand, this, std::placeholders::_1,
      std::placeholders::_2));

  createRosbagYaml();

  RCLCPP_INFO(this->get_logger(), "RosbagCollection initialized");
}

RosbagCollection::~RosbagCollection()
{
  RCLCPP_INFO(this->get_logger(), "RosbagCollection destructor called");
  node_alive_->store(false);  // prevent late auto-save from starting saveRosbag()
  stopRecordingMonitor();     // cancel timer before any further teardown
  // join any in-flight auto-save before members it uses (this) get torn down
  if (auto_save_thread_.joinable()) {auto_save_thread_.join();}
  if (is_recording_) {
    try {
      saveRosbag();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error stopping recording in destructor: %s", e.what());
    }
  }
}

std::string RosbagCollection::getTimestampString()
{
  return EpisodeLifecycle::timestampString();
}

void RosbagCollection::buildTopicList()
{
  rosbag_info_.topics_to_record = TopicBuilder::buildTopicList(robot_info_, rosbag_info_);
  RCLCPP_INFO(this->get_logger(), "Cached %zu topics to record.",
      rosbag_info_.topics_to_record.size());
}

bool RosbagCollection::validateTopics()
{
  return TopicBuilder::validateTopics(this, rosbag_info_.topics_to_record, robot_info_);
}

void RosbagCollection::startRecordingMonitor()
{
  if (recording_monitor_) {
    recording_monitor_->start();
  }
}

void RosbagCollection::stopRecordingMonitor()
{
  if (recording_monitor_) {
    recording_monitor_->stop();
  }
}

void RosbagCollection::createRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Starting recording...");

  if (!validateTopics()) {
    RCLCPP_WARN(this->get_logger(),
      "Some critical topics are missing. Recording will proceed, but the bag may not be "
      "convertible.");
  }

  std::lock_guard<std::mutex> lock(recorder_mutex_);

  if (is_recording_) {
    RCLCPP_WARN(this->get_logger(), "Already recording!");
    return;
  }

  current_task_path_ = rosbag_info_.recording_dir + "/" + current_task_dir_name_;

  previous_bag_name_ = current_bag_name_;
  current_bag_name_ = EpisodeLifecycle::makeBagName();

  previous_bag_path_ = current_bag_path_;
  current_bag_path_ = EpisodeLifecycle::makeBagPath(current_task_path_, current_bag_name_);

  if (!std::filesystem::exists(current_task_path_)) {
    try {
      std::filesystem::create_directories(current_task_path_);
      RCLCPP_INFO(this->get_logger(), "Created bag directory: %s", current_bag_path_.c_str());
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_DEBUG(this->get_logger(), "The directory %s already exists, skipping creation: %s",
          current_task_path_.c_str(), e.what());
    }
  }

  current_subtask_name_ = "";
  current_episode_subtasks_.clear();

  recording_start_time_ = std::chrono::steady_clock::now();
  max_duration_triggered_ = false;
  previous_state_ = current_state_;
  current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING;

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = current_bag_path_;
  storage_options.storage_id = rosbag_info_.conversion_format;

  rosbag2_transport::RecordOptions record_options;

  if (!rosbag_info_.topics_to_record.empty()) {
    record_options.topics = rosbag_info_.topics_to_record;
  } else {
    record_options.all_topics = true;
  }
  record_options.use_sim_time = this->get_parameter("use_sim_time").as_bool();
  record_options.rmw_serialization_format = rosbag_info_.rmw_serialization_format;

  if (!rosbag_info_.compression_mode.empty()) {
    record_options.compression_mode = rosbag_info_.compression_mode;
    record_options.compression_format = rosbag_info_.compression_format;
  }

  auto writer = std::make_shared<rosbag2_cpp::Writer>();

  recorder_node_ = std::make_shared<rosbag2_transport::Recorder>(
    writer,
    storage_options,
    record_options,
    "rosbag2_recorder_node",
    rclcpp::NodeOptions()
  );

  recorder_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  recorder_executor_->add_node(recorder_node_);

  is_recording_ = true;
  recorder_thread_ = std::thread([this]() {
        try {
          recorder_node_->record();    // opens writer + sets up subs, returns immediately
          recorder_executor_->spin();  // processes subscription callbacks until cancel()
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Error during bag recording: %s", e.what());
          current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_ERROR;
        }
  });

  RCLCPP_INFO(this->get_logger(), "Rosbag recording started successfully");
  startRecordingMonitor();
}


void RosbagCollection::removeRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Removing rosbag...");
  stopRecordingMonitor();

  std::lock_guard<std::mutex> lock(recorder_mutex_);

  if (is_recording_) {
    RCLCPP_INFO(this->get_logger(), "Stopping recorder to remove bag...");
    is_recording_ = false;
    current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;

    if (recorder_node_) {
      recorder_node_->stop();
    }
    if (recorder_executor_) {
      recorder_executor_->cancel();
    }
    if (recorder_thread_.joinable()) {
      recorder_thread_.join();
    }
    recorder_executor_.reset();
    recorder_node_.reset();
  }

  // Remove the current bag directory. removeBagDir() already logs the
  // failure reason; this call site adds the success log and the
  // state-machine transition on failure that only removeRosbag() needs.
  if (std::filesystem::exists(current_bag_path_)) {
    if (episode_lifecycle_->removeBagDir(current_bag_path_)) {
      RCLCPP_INFO(this->get_logger(), "Removed bag directory: %s", current_bag_path_.c_str());
    } else {
      previous_state_ = current_state_;
      current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_ERROR;
      throw std::runtime_error("Failed to remove bag directory");
    }
  }

  // Wipe from metadata BEFORE rolling back names
  removeEpisodeFromYaml();

  current_bag_name_ = previous_bag_name_;
  current_bag_path_ = previous_bag_path_;

  previous_state_ = current_state_;
  current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;

  RCLCPP_INFO(this->get_logger(), "Rosbag removed successfully");
}

bool RosbagCollection::saveRosbag()
{
  RCLCPP_INFO(this->get_logger(), "Saving rosbag...");
  stopRecordingMonitor();

  std::lock_guard<std::mutex> lock(recorder_mutex_);

  if (!is_recording_) {
    RCLCPP_WARN(this->get_logger(), "No rosbag process to terminate");
    previous_state_ = current_state_;
    current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Stopping recorder for saving...");
  is_recording_ = false;

  // Order: stop() flushes MCAP, cancel() unblocks spin(), join() waits for thread exit,
  // then reset() — executor must outlive the thread that calls spin().
  if (recorder_node_) {
    recorder_node_->stop();
  }
  if (recorder_executor_) {
    recorder_executor_->cancel();
  }
  if (recorder_thread_.joinable()) {
    recorder_thread_.join();
  }
  recorder_executor_.reset();
  recorder_node_.reset();

  previous_state_ = current_state_;
  current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;

  // Episode duration validation (steady_clock: monotonic, unaffected by sim_time or NTP)
  auto elapsed = std::chrono::steady_clock::now() - recording_start_time_;
  double duration_sec = std::chrono::duration<double>(elapsed).count();

  // Min-duration + integrity check; EpisodeLifecycle logs its own messages.
  auto decision = episode_lifecycle_->decideSave(
    current_bag_path_, duration_sec, min_episode_duration_sec_);
  if (!decision.keep) {
    episode_lifecycle_->removeBagDir(current_bag_path_);
    removeEpisodeFromYaml();
    return false;
  }

  updateEpisodeYaml();

  RCLCPP_INFO(this->get_logger(), "Rosbag saved successfully (duration: %.1fs)", duration_sec);
  return true;
}

bool RosbagCollection::verifyBagIntegrity(const std::string & bag_path)
{
  // Message texts live in EpisodeLifecycle; it logs them itself via the
  // callbacks wired in the constructor, so nothing to re-log here.
  return episode_lifecycle_->verifyIntegrity(bag_path).ok;
}

void RosbagCollection::createRosbagYaml()
{
  if (bag_metadata_manager_) {
    bag_metadata_manager_->createOrValidate(camera_dimensions_);
  }
}

void RosbagCollection::updateRosbagYaml()
{
  if (bag_metadata_manager_) {
    bag_metadata_manager_->updateRosbagYaml(current_task_dir_name_, current_task_name_,
        current_task_path_, gamepad_name_, camera_dimensions_);
  }
}

void RosbagCollection::updateEpisodeYaml()
{
  if (bag_metadata_manager_) {
    bag_metadata_manager_->updateEpisodeYaml(current_task_dir_name_, current_bag_name_,
        current_bag_path_, current_episode_subtasks_);
  }
}

void RosbagCollection::removeEpisodeFromYaml()
{
  if (bag_metadata_manager_) {
    bag_metadata_manager_->removeEpisodeFromYaml(current_task_dir_name_, current_bag_name_);
  }
}

void RosbagCollection::handleVlaCommand(
  const std::shared_ptr<sobits_interfaces::srv::VlaCommand::Request> request,
  std::shared_ptr<sobits_interfaces::srv::VlaCommand::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received VlaCommand service request: %d", request->command);

  if (!task_has_been_set_) {
    response->success = false;
    response->message =
      "Please set a task name via the vla_task_update service before sending commands.";
    response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_ERROR;
    return;
  }

  if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_ERROR) {
    response->success = false;
    response->message = "Cannot process command while in ERROR state.";
    response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_ERROR;
    return;
  }

  if (request->command == sobits_interfaces::srv::VlaCommand::Request::RECORD) {
    if (current_state_ != sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED &&
      current_state_ != sobits_interfaces::srv::VlaCommand::Response::STATE_PAUSED)
    {
      response->success = false;
      response->message = "Cannot start/resume recording while already in state: " +
        std::to_string(current_state_);
      response->status = current_state_;
      return;
    }

    if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED) {
      createRosbag();
      response->success = true;
      response->message = "Recording started successfully";
      response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING;
    } else if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_PAUSED) {
      if (recorder_node_) {
        recorder_node_->resume();
        current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING;
        response->success = true;
        response->message = "Recording resumed successfully";
        response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING;
      } else {
        response->success = false;
        response->message = "Recorder node not initialized";
        response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_ERROR;
      }
    }
  } else if (request->command == sobits_interfaces::srv::VlaCommand::Request::PAUSE) {
    if (current_state_ != sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING) {
      response->success = false;
      response->message = "Cannot pause while not recording";
      response->status = current_state_;
      return;
    }

    if (recorder_node_) {
      recorder_node_->pause();
      current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_PAUSED;
      response->success = true;
      response->message = "Recording paused successfully";
      response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_PAUSED;
    } else {
      response->success = false;
      response->message = "Recorder node not initialized";
      response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_ERROR;
    }
  } else if (request->command == sobits_interfaces::srv::VlaCommand::Request::RESUME) {
    if (current_state_ != sobits_interfaces::srv::VlaCommand::Response::STATE_PAUSED) {
      response->success = false;
      response->message = "Cannot resume while not paused";
      response->status = current_state_;
      return;
    }

    if (recorder_node_) {
      recorder_node_->resume();
      current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING;
      response->success = true;
      response->message = "Recording resumed successfully";
      response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING;
    } else {
      response->success = false;
      response->message = "Recorder node not initialized";
      response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_ERROR;
    }
  } else if (request->command == sobits_interfaces::srv::VlaCommand::Request::SAVE) {
    if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED) {
      response->success = false;
      response->message = "Cannot save recording while not in RECORDING or PAUSED state";
      response->status = current_state_;
      return;
    }
    if (saveRosbag()) {
      response->success = true;
      response->message = "Recording saved successfully";
      response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;
    } else {
      response->success = false;
      response->message = "Recording was discarded (too short or integrity failed)";
      response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;
    }
  } else if (request->command == sobits_interfaces::srv::VlaCommand::Request::DELETE) {
    removeRosbag();
    response->success = true;
    response->message = "Recording deleted successfully";
    response->status = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;
  } else if (request->command == sobits_interfaces::srv::VlaCommand::Request::RESET) {
    requestWorldReset();
    response->success = true;
    response->message = "World reset requested";
    response->status = current_state_;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown command received: %d", request->command);
    response->success = false;
    response->message = "Unknown command received: " + std::to_string(request->command);
    response->status = current_state_;
  }
}

void RosbagCollection::taskUpdateCallback(
  const std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Request> request,
  std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received task update request: %s", request->label.c_str());

  if (current_state_ != sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED) {
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
    std::transform(current_task_dir_name_.begin(), current_task_dir_name_.end(),
        current_task_dir_name_.begin(),
      [](unsigned char c) {return std::tolower(c);});

    current_task_path_ = rosbag_info_.recording_dir + "/" + current_task_dir_name_;
    RCLCPP_INFO(this->get_logger(), "Updated task name from '%s' to '%s'",
        previous_task_name_.c_str(), current_task_dir_name_.c_str());

    previous_bag_name_ = current_bag_name_;
    current_bag_name_ = EpisodeLifecycle::makeBagName();
    previous_bag_path_ = current_bag_path_;
    current_bag_path_ = EpisodeLifecycle::makeBagPath(current_task_path_, current_bag_name_);

    updateRosbagYaml();
  } else {
    RCLCPP_WARN(this->get_logger(), "Task name '%s' is already the current task name",
        request->label.c_str());
    response->success = false;
    response->message = "Task name is already the current task name";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Task name updated successfully to '%s'",
      current_task_name_.c_str());
  response->success = true;
  response->message = "Task name updated successfully and rosbag YAML file updated";
  task_has_been_set_ = true;
}

void RosbagCollection::subtaskUpdateCallback(
  const std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Request> request,
  std::shared_ptr<sobits_interfaces::srv::VlaUpdateTask::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received subtask update request: %s", request->label.c_str());

  if (current_state_ != sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING) {
    RCLCPP_WARN(this->get_logger(), "Cannot update subtask while not recording.");
    response->success = false;
    response->message = "Cannot update subtask while not recording.";
    return;
  }

  double current_time_sec = this->now().seconds();

  if (!current_episode_subtasks_.empty()) {
    current_episode_subtasks_.back().end_timestamp = current_time_sec;
  }

  current_subtask_name_ = request->label;

  SubtaskInfo new_subtask;
  new_subtask.key = "subtask_" + getTimestampString();
  new_subtask.label = current_subtask_name_;
  new_subtask.start_timestamp = current_time_sec;
  new_subtask.end_timestamp = 0.0;  // Will be updated on the next subtask or bag save

  current_episode_subtasks_.push_back(new_subtask);

  RCLCPP_INFO(this->get_logger(), "Subtask name updated successfully to '%s'",
      current_subtask_name_.c_str());
  response->success = true;
  response->message = "Subtask name updated successfully";
}

void RosbagCollection::requestWorldReset()
{
  if (!world_reset_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "World reset service '%s' not available, skipping reset",
        world_reset_service_.c_str());
    return;
  }

  auto request = std::make_shared<sobits_interfaces::srv::VlaResetWorld::Request>();
  request->preset = world_reset_preset_;
  // async_send_request + callback: spin_until_future_complete here would
  // deadlock, since this runs on the same executor that services the request.
  world_reset_client_->async_send_request(
    request,
    [this](rclcpp::Client<sobits_interfaces::srv::VlaResetWorld>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "World reset succeeded: %s", response->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "World reset failed: %s", response->message.c_str());
      }
    });
}

void RosbagCollection::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg,
  const std::string topic_name)
{
  if (camera_dimensions_.find(topic_name) == camera_dimensions_.end()) {
    camera_dimensions_[topic_name] = {msg->width, msg->height};
    RCLCPP_INFO(this->get_logger(), "Captured dimensions for %s: %dx%d", topic_name.c_str(),
        msg->width, msg->height);
    camera_info_subs_.erase(topic_name);
  }
}

}  // namespace sobits_vla

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sobits_vla::RosbagCollection)
