#include "sobits_vla_rosbag_collection/recording_monitor.hpp"
#include <filesystem>
#include <cmath>

namespace sobits_vla
{

RecordingMonitor::RecordingMonitor(
  rclcpp::Node * node,
  const RobotInfo & robot_info,
  const std::string & recording_dir,
  int expected_fps,
  uint64_t min_disk_space_mb,
  double max_duration_sec,
  double timestamp_threshold,
  std::function<void()> on_max_duration)
: node_(node),
  robot_info_(robot_info),
  recording_dir_(recording_dir),
  expected_sensor_fps_(expected_fps),
  min_disk_space_mb_(min_disk_space_mb),
  max_episode_duration_sec_(max_duration_sec),
  timestamp_jump_threshold_sec_(timestamp_threshold),
  on_max_duration_callback_(on_max_duration)
{
}

RecordingMonitor::~RecordingMonitor()
{
  stop();
}

void RecordingMonitor::start()
{
  bool need_fps = expected_sensor_fps_ > 0;
  bool need_disk = min_disk_space_mb_ > 0;
  bool need_timestamp = timestamp_jump_threshold_sec_ > 0.0;
  bool need_max_duration = max_episode_duration_sec_ > 0.0;
  if (!need_fps && !need_disk && !need_timestamp && !need_max_duration) {
    return;
  }

  is_recording_ = true;
  max_duration_triggered_ = false;
  fps_warmup_ = true;
  timestamp_monitor_initialized_ = false;
  recording_start_time_ = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(monitor_mutex_);  // guard population vs. a racing stop()/tick

  if (need_fps) {
    std::set<std::string> monitor_topics;
    monitor_topics.insert(robot_info_.joint_states_topic);
    for (const auto & stype : robot_info_.sensor_types) {
      auto it_topics = robot_info_.sensor_topics.find(stype);
      if (it_topics != robot_info_.sensor_topics.end()) {
        for (const auto & topic : it_topics->second) {
          if (!topic.empty()) {
            monitor_topics.insert(topic);
          }
        }
      }
      auto it_comp = robot_info_.sensor_compressed_topics.find(stype);
      if (it_comp != robot_info_.sensor_compressed_topics.end()) {
        for (const auto & topic : it_comp->second) {
          if (!topic.empty()) {
            monitor_topics.insert(topic);
          }
        }
      }
    }
    for (const auto & part : robot_info_.parts) {
      auto it_cmd = robot_info_.part_command_topic.find(part);
      if (it_cmd != robot_info_.part_command_topic.end() && !it_cmd->second.empty()) {
        monitor_topics.insert(it_cmd->second);
      }
    }

    auto graph_topics = node_->get_topic_names_and_types();

    for (const auto & topic : monitor_topics) {
      auto it = graph_topics.find(topic);
      if (it == graph_topics.end() || it->second.empty()) {
        continue;
      }

      const std::string & topic_type = it->second[0];

      auto count_atomic = std::make_shared<std::atomic<uint64_t>>(0);
      monitor_counts_[topic] = count_atomic;
      monitor_prev_counts_[topic] = 0;

      auto sub = node_->create_generic_subscription(
        topic, topic_type, rclcpp::SensorDataQoS(),
        [count_atomic](std::shared_ptr<rclcpp::SerializedMessage>) {
          (*count_atomic)++;
        });
      monitor_subs_.push_back(sub);
    }
  }

  fps_monitor_timer_ = node_->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&RecordingMonitor::runMonitorTick, this));

  RCLCPP_INFO(node_->get_logger(),
    "Recording monitor started (fps_topics=%zu, expected_hz=%d, min_disk_mb=%lu).",
    monitor_subs_.size(), expected_sensor_fps_, min_disk_space_mb_);
}

void RecordingMonitor::stop()
{
  is_recording_ = false;
  if (fps_monitor_timer_) {
    fps_monitor_timer_->cancel();
    fps_monitor_timer_.reset();
  }
  std::lock_guard<std::mutex> lock(monitor_mutex_);  // serialize against a racing tick
  monitor_subs_.clear();
  monitor_counts_.clear();
  monitor_prev_counts_.clear();
  timestamp_monitor_initialized_ = false;
  fps_warmup_ = true;
}

void RecordingMonitor::runMonitorTick()
{
  if (!is_recording_) {
    return;
  }

  {
    // Scoped: only the map-touching checks need monitor_mutex_. The
    // max-duration callback below must run unlocked — it joins the
    // previous auto-save thread, which calls back into stop() and
    // would deadlock against this lock if held here.
    std::lock_guard<std::mutex> lock(monitor_mutex_);  // guard maps vs. concurrent stop()/start()

    // FPS checks
    if (expected_sensor_fps_ > 0) {
      if (fps_warmup_) {
        for (auto & [topic, prev_count] : monitor_prev_counts_) {
          auto it = monitor_counts_.find(topic);
          if (it != monitor_counts_.end()) {
            prev_count = it->second->load();
          }
        }
        fps_warmup_ = false;
      } else {
        double interval = 2.0;
        double threshold = expected_sensor_fps_ * 0.8;
        for (auto & [topic, prev_count] : monitor_prev_counts_) {
          auto it = monitor_counts_.find(topic);
          if (it != monitor_counts_.end()) {
            uint64_t current = it->second->load();
            double rate = static_cast<double>(current - prev_count) / interval;
            prev_count = current;

            if (rate < threshold && rate > 0.0) {
              RCLCPP_WARN(node_->get_logger(),
                "FPS DROP: '%s' publishing at %.1f Hz (expected >= %.1f Hz)",
                topic.c_str(), rate, static_cast<double>(expected_sensor_fps_));
            } else if (rate == 0.0 && current > 0) {
              RCLCPP_ERROR(node_->get_logger(),
                "FPS STALL: '%s' stopped publishing!", topic.c_str());
            }
          }
        }
      }
    }
  }

  // Disk space check
  if (min_disk_space_mb_ > 0) {
    try {
      auto space = std::filesystem::space(recording_dir_);
      uint64_t free_mb = space.available / (1024 * 1024);
      if (free_mb < min_disk_space_mb_) {
        RCLCPP_ERROR(node_->get_logger(),
          "LOW DISK SPACE: %lu MB free (minimum: %lu MB). "
          "Recording may produce corrupted bags!",
          free_mb, min_disk_space_mb_);
      }
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_WARN(node_->get_logger(), "Failed to check disk space: %s", e.what());
    }
  }

  // Timestamp jump detection
  if (timestamp_jump_threshold_sec_ > 0.0) {
    auto now_wall = std::chrono::steady_clock::now();
    rclcpp::Time now_ros = node_->get_clock()->now();

    if (timestamp_monitor_initialized_) {
      double wall_delta = std::chrono::duration<double>(now_wall - prev_wall_time_).count();
      double ros_delta = (now_ros - prev_ros_time_).seconds();
      double drift = std::abs(ros_delta - wall_delta);

      if (drift > timestamp_jump_threshold_sec_) {
        RCLCPP_ERROR(node_->get_logger(),
          "TIMESTAMP JUMP: ROS clock drifted %.2fs from wall clock in %.1fs interval "
          "(ros_delta=%.2fs, wall_delta=%.2fs). Bag timestamps may be inconsistent!",
          drift, wall_delta, ros_delta, wall_delta);
      }
    } else {
      timestamp_monitor_initialized_ = true;
    }
    prev_wall_time_ = now_wall;
    prev_ros_time_ = now_ros;
  }

  // Max duration check
  if (max_episode_duration_sec_ > 0.0 && !max_duration_triggered_) {
    auto elapsed = std::chrono::steady_clock::now() - recording_start_time_;
    double duration_sec = std::chrono::duration<double>(elapsed).count();
    if (duration_sec >= max_episode_duration_sec_) {
      max_duration_triggered_ = true;
      RCLCPP_WARN(node_->get_logger(),
        "Max episode duration reached (%.1fs >= %.1fs). Auto-saving.",
        duration_sec, max_episode_duration_sec_);
      if (on_max_duration_callback_) {
        on_max_duration_callback_();
      }
    }
  }
}

} // namespace sobits_vla
