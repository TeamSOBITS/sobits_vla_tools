#ifndef SOBITS_VLA_ROSBAG_COLLECTION__RECORDING_MONITOR_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__RECORDING_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <map>
#include <vector>
#include <string>
#include <set>
#include <mutex>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

namespace sobits_vla
{

class RecordingMonitor
{
public:
  RecordingMonitor(
    rclcpp::Node * node,
    const RobotInfo & robot_info,
    const std::string & recording_dir,
    int expected_fps,
    uint64_t min_disk_space_mb,
    double max_duration_sec,
    double timestamp_threshold,
    std::function<void()> on_max_duration);

  ~RecordingMonitor();

  void start();
  void stop();
  bool isRecording() const {return is_recording_;}

private:
  void runMonitorTick();

  rclcpp::Node * node_;
  RobotInfo robot_info_;
  std::string recording_dir_;
  int expected_sensor_fps_;
  uint64_t min_disk_space_mb_;
  double max_episode_duration_sec_;
  double timestamp_jump_threshold_sec_;
  std::function<void()> on_max_duration_callback_;

  std::atomic<bool> is_recording_{false};
  std::mutex monitor_mutex_;  // guards monitor_subs_/counts_ vs. concurrent tick
  std::vector<rclcpp::GenericSubscription::SharedPtr> monitor_subs_;
  std::map<std::string, std::shared_ptr<std::atomic<uint64_t>>> monitor_counts_;
  std::map<std::string, uint64_t> monitor_prev_counts_;
  rclcpp::TimerBase::SharedPtr fps_monitor_timer_;
  std::atomic<bool> max_duration_triggered_{false};
  bool fps_warmup_{true};

  // Timestamp drift check
  rclcpp::Time prev_ros_time_;
  std::chrono::steady_clock::time_point prev_wall_time_;
  bool timestamp_monitor_initialized_{false};
  std::chrono::steady_clock::time_point recording_start_time_;
};

} // namespace sobits_vla

#endif // SOBITS_VLA_ROSBAG_COLLECTION__RECORDING_MONITOR_HPP_
