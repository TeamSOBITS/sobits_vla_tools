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

#ifndef SOBITS_VLA_ROSBAG_COLLECTION__RECORD_STATUS_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__RECORD_STATUS_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace sobits_vla
{

// Recorder state + the last transition event, as plain values. No rclcpp:
// the node supplies timestamps and serialises snapshots into the ROS message.
// Not thread-safe; the node serialises access with its own mutex.
class RecordStatus
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  enum class State : uint8_t { Stopped = 0, Recording = 1, Paused = 2, Error = 4 };
  enum class Event : uint8_t
  {
    None = 0, Started = 1, Paused = 2, Resumed = 3, Saved = 4,
    Discarded = 5, Deleted = 6, Error = 7, TaskSet = 8, Rejected = 9
  };

  struct Snapshot
  {
    State state{State::Stopped};
    Event event{Event::None};
    uint32_t event_seq{0};
    bool task_set{false};
    std::string task_name;
    std::string episode_name;
    double elapsed_sec{0.0};
    std::string detail;
    std::string message;
  };

  void start(TimePoint now, const std::string & episode_name);
  void pause(TimePoint now);
  void resume(TimePoint now);
  // Freezes elapsed at the newest time this object has been told about.
  void stop(Event how, const std::string & detail = "");
  void error(const std::string & detail);
  void setTask(const std::string & name);
  void reject(const std::string & why);

  // Recorded time only; pauses are excluded.
  double elapsedSec(TimePoint now) const;
  // Returns the pending event and clears it, so the next call is a heartbeat.
  Snapshot snapshot(TimePoint now);

  static std::string formatElapsed(double sec);
  static std::string describe(const Snapshot & s);

private:
  void raise(Event ev, const std::string & detail);
  void seen(TimePoint now) const;

  State state_{State::Stopped};
  Event pending_{Event::None};
  uint32_t seq_{0};
  bool task_set_{false};
  std::string task_name_;
  std::string episode_name_;
  std::string detail_;
  std::string last_text_{"idle"};
  double accumulated_{0.0};
  TimePoint segment_start_{};
  mutable TimePoint last_now_{};
};

}  // namespace sobits_vla

#endif  // SOBITS_VLA_ROSBAG_COLLECTION__RECORD_STATUS_HPP_
