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

#ifndef SOBITS_VLA_ROSBAG_COLLECTION__RECORDING_MONITOR_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__RECORDING_MONITOR_HPP_

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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

}  // namespace sobits_vla

#endif  // SOBITS_VLA_ROSBAG_COLLECTION__RECORDING_MONITOR_HPP_
