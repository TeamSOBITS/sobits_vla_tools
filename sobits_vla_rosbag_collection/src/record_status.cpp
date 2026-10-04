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

#include "sobits_vla_rosbag_collection/record_status.hpp"

#include <cstdio>

namespace sobits_vla
{

void RecordStatus::raise(Event ev, const std::string & detail)
{
  if (ev == Event::None) {return;}  // a silent state change is not an event
  pending_ = ev;
  detail_ = detail;
  ++seq_;
}

void RecordStatus::seen(TimePoint now) const
{
  if (now > last_now_) {last_now_ = now;}
}

void RecordStatus::start(TimePoint now, const std::string & episode_name)
{
  seen(now);
  state_ = State::Recording;
  episode_name_ = episode_name;
  accumulated_ = 0.0;
  segment_start_ = now;
  raise(Event::Started, "");
}

void RecordStatus::pause(TimePoint now)
{
  seen(now);
  if (state_ == State::Recording) {
    accumulated_ += std::chrono::duration<double>(now - segment_start_).count();
  }
  state_ = State::Paused;
  raise(Event::Paused, "");
}

void RecordStatus::resume(TimePoint now)
{
  seen(now);
  state_ = State::Recording;
  segment_start_ = now;
  raise(Event::Resumed, "");
}

void RecordStatus::stop(Event how, const std::string & detail)
{
  if (state_ == State::Recording) {
    accumulated_ += std::chrono::duration<double>(last_now_ - segment_start_).count();
  }
  state_ = State::Stopped;
  raise(how, detail);
}

void RecordStatus::error(const std::string & detail)
{
  state_ = State::Error;
  raise(Event::Error, detail);
}

void RecordStatus::setTask(const std::string & name)
{
  task_set_ = true;
  task_name_ = name;
  raise(Event::TaskSet, name);
}

void RecordStatus::reject(const std::string & why)
{
  raise(Event::Rejected, why);
}

double RecordStatus::elapsedSec(TimePoint now) const
{
  seen(now);
  if (state_ != State::Recording) {return accumulated_;}
  return accumulated_ + std::chrono::duration<double>(now - segment_start_).count();
}

RecordStatus::Snapshot RecordStatus::snapshot(TimePoint now)
{
  Snapshot s;
  s.state = state_;
  s.event = pending_;
  s.event_seq = seq_;
  s.task_set = task_set_;
  s.task_name = task_name_;
  s.episode_name = episode_name_;
  s.elapsed_sec = elapsedSec(now);
  s.detail = detail_;
  s.message = describe(s);
  // Idle/rejected while stopped keep showing the last meaningful text.
  const bool quiet = s.event == Event::None || s.event == Event::Rejected;
  if (state_ == State::Stopped && quiet) {
    s.message = last_text_;
  } else if (state_ == State::Stopped) {
    last_text_ = s.message;
  }
  pending_ = Event::None;
  detail_.clear();
  return s;
}

std::string RecordStatus::formatElapsed(double sec)
{
  long total = sec > 0.0 ? static_cast<long>(sec) : 0;  // NOLINT(runtime/int)
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02ld:%02ld:%02ld", total / 3600, (total % 3600) / 60,
    total % 60);
  return buf;
}

std::string RecordStatus::describe(const Snapshot & s)
{
  switch (s.state) {
    case State::Recording: return "recording " + formatElapsed(s.elapsed_sec);
    case State::Paused: return "paused " + formatElapsed(s.elapsed_sec);
    case State::Error: return "error";
    case State::Stopped: break;
  }
  switch (s.event) {
    case Event::Saved: return "saved";
    case Event::Discarded: return "discarded";
    case Event::Deleted: return "deleted";
    case Event::TaskSet: return "task_set: " + s.task_name;
    default: return "idle";
  }
}

}  // namespace sobits_vla
