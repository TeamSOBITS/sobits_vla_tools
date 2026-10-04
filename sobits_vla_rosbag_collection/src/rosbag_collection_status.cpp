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

// Recorder status publishing; split from rosbag_collection.cpp to keep that
// file from growing. The tracker itself lives in record_status.cpp.

#include <algorithm>
#include <string>

#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

namespace sobits_vla
{

using Msg = sobits_interfaces::msg::VlaRecordStatus;
using Cmd = sobits_interfaces::srv::VlaCommand;
using State = RecordStatus::State;
using Event = RecordStatus::Event;

// The tracker is ROS-free, so its numbers must match the wire constants.
static_assert(static_cast<uint8_t>(State::Stopped) == Msg::STATE_STOPPED);
static_assert(static_cast<uint8_t>(State::Recording) == Msg::STATE_RECORDING);
static_assert(static_cast<uint8_t>(State::Paused) == Msg::STATE_PAUSED);
static_assert(static_cast<uint8_t>(State::Error) == Msg::STATE_ERROR);
static_assert(Cmd::Response::STATE_STOPPED == Msg::STATE_STOPPED);
static_assert(Cmd::Response::STATE_RECORDING == Msg::STATE_RECORDING);
static_assert(Cmd::Response::STATE_PAUSED == Msg::STATE_PAUSED);
static_assert(Cmd::Response::STATE_ERROR == Msg::STATE_ERROR);
static_assert(static_cast<uint8_t>(Event::None) == Msg::EVENT_NONE);
static_assert(static_cast<uint8_t>(Event::Started) == Msg::EVENT_STARTED);
static_assert(static_cast<uint8_t>(Event::Paused) == Msg::EVENT_PAUSED);
static_assert(static_cast<uint8_t>(Event::Resumed) == Msg::EVENT_RESUMED);
static_assert(static_cast<uint8_t>(Event::Saved) == Msg::EVENT_SAVED);
static_assert(static_cast<uint8_t>(Event::Discarded) == Msg::EVENT_DISCARDED);
static_assert(static_cast<uint8_t>(Event::Deleted) == Msg::EVENT_DELETED);
static_assert(static_cast<uint8_t>(Event::Error) == Msg::EVENT_ERROR);
static_assert(static_cast<uint8_t>(Event::TaskSet) == Msg::EVENT_TASK_SET);
static_assert(static_cast<uint8_t>(Event::Rejected) == Msg::EVENT_REJECTED);

Msg RosbagCollection::toMsg(const RecordStatus::Snapshot & s)
{
  Msg m;
  m.state = static_cast<uint8_t>(s.state);
  m.event = static_cast<uint8_t>(s.event);
  m.event_seq = s.event_seq;
  m.task_set = s.task_set;
  m.task_name = s.task_name;
  m.episode_name = s.episode_name;
  m.elapsed_sec = static_cast<float>(s.elapsed_sec);
  m.detail = s.detail;
  m.message = s.message;
  return m;
}

void RosbagCollection::publishLocked()
{
  if (!record_status_pub_) {return;}
  Msg m = toMsg(record_status_.snapshot(RecordStatus::Clock::now()));
  m.stamp = this->now();
  record_status_pub_->publish(m);
}

void RosbagCollection::publishRecordStatus()
{
  std::lock_guard<std::mutex> lock(record_status_mutex_);
  publishLocked();
}

void RosbagCollection::transition(uint8_t state, Event ev, const std::string & detail)
{
  std::lock_guard<std::mutex> lock(record_status_mutex_);
  previous_state_ = current_state_;
  current_state_ = state;
  const auto now = RecordStatus::Clock::now();
  switch (ev) {
    case Event::Started: record_status_.start(now, current_bag_name_); break;
    case Event::Paused: record_status_.pause(now); break;
    case Event::Resumed: record_status_.resume(now); break;
    default:
      // Errors and stops freeze elapsed at "now", not at the last heartbeat.
      record_status_.elapsedSec(now);
      if (ev == Event::Error) {
        record_status_.error(detail);
      } else {
        record_status_.stop(ev, detail);
      }
      break;
  }
  publishLocked();
}

void RosbagCollection::rejectCommand(
  Cmd::Response & response, const std::string & msg,
  uint8_t status)
{
  response.success = false;
  response.message = msg;
  response.status = status;
  std::lock_guard<std::mutex> lock(record_status_mutex_);
  record_status_.reject(msg);
  publishLocked();
}

}  // namespace sobits_vla
