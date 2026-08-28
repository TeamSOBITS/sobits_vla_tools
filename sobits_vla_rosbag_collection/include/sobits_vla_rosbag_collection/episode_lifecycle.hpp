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


#ifndef SOBITS_VLA_ROSBAG_COLLECTION__EPISODE_LIFECYCLE_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__EPISODE_LIFECYCLE_HPP_

#include <cstdint>
#include <functional>
#include <string>

namespace sobits_vla
{

// Bag naming, min/max duration + integrity checks, and discard/remove
// decisions. No rclcpp dependency -- the node supplies a log sink and owns
// all ROS-side state (recorder, YAML metadata, timers).
class EpisodeLifecycle
{
public:
  using LogFn = std::function<void(const std::string &)>;

  struct SaveDecision
  {
    bool keep{false};
    // Populated when keep == false: "too_short" or "integrity_failed".
    std::string discard_reason;
    double duration_sec{0.0};
  };

  struct IntegrityResult
  {
    bool ok{false};
    std::string reason;
    size_t topic_count{0};
    size_t message_count{0};
    uintmax_t storage_bytes{0};
  };

  EpisodeLifecycle(
    std::string storage_id,
    LogFn log_info,
    LogFn log_warn,
    LogFn log_error);

  // "episode_" + getTimestampString(); the format itself lives here too so
  // task-dir names and bag names share exactly one timestamp implementation.
  static std::string timestampString();
  static std::string makeBagName();
  static std::string makeBagPath(const std::string & task_path, const std::string & bag_name);

  // storage-file presence/size + rosbag2 reader open + message-count checks.
  IntegrityResult verifyIntegrity(const std::string & bag_path) const;

  // Combines the min-duration check with verifyIntegrity(); does NOT touch
  // the filesystem -- the caller removes the bag dir on a discard decision.
  SaveDecision decideSave(
    const std::string & bag_path,
    double duration_sec,
    double min_episode_duration_sec) const;

  // Best-effort recursive remove; returns false (and logs) on failure.
  bool removeBagDir(const std::string & bag_path) const;

private:
  std::string storage_id_;
  LogFn log_info_;
  LogFn log_warn_;
  LogFn log_error_;
};

}  // namespace sobits_vla

#endif  // SOBITS_VLA_ROSBAG_COLLECTION__EPISODE_LIFECYCLE_HPP_
