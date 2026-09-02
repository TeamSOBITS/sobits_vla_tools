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


#include "sobits_vla_rosbag_collection/episode_lifecycle.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace sobits_vla
{

EpisodeLifecycle::EpisodeLifecycle(
  std::string storage_id,
  LogFn log_info,
  LogFn log_warn,
  LogFn log_error)
: storage_id_(std::move(storage_id)),
  log_info_(std::move(log_info)),
  log_warn_(std::move(log_warn)),
  log_error_(std::move(log_error))
{
}

std::string EpisodeLifecycle::timestampString()
{
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  return ss.str();
}

std::string EpisodeLifecycle::makeBagName()
{
  return "episode_" + timestampString();
}

std::string EpisodeLifecycle::makeBagPath(
  const std::string & task_path, const std::string & bag_name)
{
  return task_path + "/" + bag_name;
}

EpisodeLifecycle::IntegrityResult EpisodeLifecycle::verifyIntegrity(
  const std::string & bag_path) const
{
  IntegrityResult result;

  // 1. Check directory exists
  if (!std::filesystem::exists(bag_path)) {
    result.reason = "Bag directory does not exist: " + bag_path;
    if (log_error_) {log_error_(result.reason);}
    return result;
  }

  // 2. Check for storage files (.mcap or .db3)
  bool has_storage_file = false;
  uintmax_t storage_size = 0;
  for (const auto & entry : std::filesystem::directory_iterator(bag_path)) {
    auto ext = entry.path().extension().string();
    if (ext == ".mcap" || ext == ".db3") {
      has_storage_file = true;
      storage_size = entry.file_size();
      break;
    }
  }
  if (!has_storage_file) {
    result.reason = "No .mcap or .db3 file found in: " + bag_path;
    if (log_error_) {log_error_(result.reason);}
    return result;
  }
  if (storage_size == 0) {
    result.reason = "Storage file is empty (0 bytes) in: " + bag_path;
    if (log_error_) {log_error_(result.reason);}
    return result;
  }
  result.storage_bytes = storage_size;

  // 3. Try opening with rosbag2 reader and check topic/message counts
  try {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = bag_path;
    storage_opts.storage_id = storage_id_;
    reader.open(storage_opts);

    auto metadata = reader.get_metadata();
    size_t total_messages = 0;
    std::vector<std::string> empty_topics;
    for (const auto & topic_info : metadata.topics_with_message_count) {
      total_messages += topic_info.message_count;
      if (topic_info.message_count == 0) {
        empty_topics.push_back(topic_info.topic_metadata.name);
      }
    }
    result.topic_count = metadata.topics_with_message_count.size();
    result.message_count = total_messages;

    if (total_messages == 0) {
      result.reason = "Bag has 0 messages: " + bag_path;
      if (log_error_) {log_error_(result.reason);}
      return result;
    }

    if (!empty_topics.empty() && log_warn_) {
      log_warn_(
        "Bag has " + std::to_string(empty_topics.size()) + " topic(s) with 0 messages:");
      for (const auto & t : empty_topics) {
        log_warn_("  empty: " + t);
      }
    }

    if (log_info_) {
      std::ostringstream msg;
      msg << "Bag integrity OK: " << result.topic_count << " topics, " << total_messages
          << " messages, " << std::fixed << std::setprecision(1)
          << (static_cast<double>(storage_size) / (1024.0 * 1024.0)) << " MB";
      log_info_(msg.str());
    }
  } catch (const std::exception & e) {
    result.reason = "Failed to read bag: " + bag_path + " — " + e.what();
    if (log_error_) {log_error_(result.reason);}
    return result;
  }

  result.ok = true;
  return result;
}

EpisodeLifecycle::SaveDecision EpisodeLifecycle::decideSave(
  const std::string & bag_path,
  double duration_sec,
  double min_episode_duration_sec) const
{
  SaveDecision decision;
  decision.duration_sec = duration_sec;

  if (min_episode_duration_sec > 0.0 && duration_sec < min_episode_duration_sec) {
    decision.keep = false;
    decision.discard_reason = "too_short";
    if (log_warn_) {
      std::ostringstream msg;
      msg << "Episode too short (" << std::fixed << std::setprecision(1) << duration_sec
          << "s < " << min_episode_duration_sec << "s minimum). Discarding bag: " << bag_path;
      log_warn_(msg.str());
    }
    return decision;
  }

  auto integrity = verifyIntegrity(bag_path);
  if (!integrity.ok) {
    decision.keep = false;
    decision.discard_reason = "integrity_failed";
    if (log_error_) {
      log_error_("Bag integrity check FAILED for: " + bag_path + ". Discarding.");
    }
    return decision;
  }

  decision.keep = true;
  return decision;
}

bool EpisodeLifecycle::removeBagDir(const std::string & bag_path) const
{
  if (!std::filesystem::exists(bag_path)) {
    return true;
  }
  try {
    std::filesystem::remove_all(bag_path);
    return true;
  } catch (const std::filesystem::filesystem_error & e) {
    if (log_error_) {
      log_error_("Failed to remove bag directory '" + bag_path + "': " + e.what());
    }
    return false;
  }
}

}  // namespace sobits_vla
