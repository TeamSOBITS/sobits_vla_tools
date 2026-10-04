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

#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "sobits_vla_rosbag_collection/record_status.hpp"

namespace
{

using sobits_vla::RecordStatus;
using Event = RecordStatus::Event;
using State = RecordStatus::State;

const RecordStatus::TimePoint t0 = RecordStatus::Clock::time_point{} + std::chrono::hours(1);

RecordStatus::TimePoint at(double sec)
{
  return t0 + std::chrono::duration_cast<RecordStatus::Clock::duration>(
    std::chrono::duration<double>(sec));
}

}  // namespace

TEST(RecordStatus, InitialSnapshotIsIdle) {
  RecordStatus r;
  auto s = r.snapshot(at(0));
  EXPECT_EQ(s.state, State::Stopped);
  EXPECT_EQ(s.event, Event::None);
  EXPECT_EQ(s.message, "idle");
  EXPECT_FALSE(s.task_set);
  EXPECT_EQ(s.event_seq, 0u);
}

TEST(RecordStatus, SetTaskIsOneShotButTextSticks) {
  RecordStatus r;
  r.setTask("pick cup");
  auto s = r.snapshot(at(0));
  EXPECT_EQ(s.event, Event::TaskSet);
  EXPECT_EQ(s.message, "task_set: pick cup");
  EXPECT_TRUE(s.task_set);
  EXPECT_EQ(s.task_name, "pick cup");
  auto h = r.snapshot(at(1));
  EXPECT_EQ(h.event, Event::None);
  EXPECT_EQ(h.message, "task_set: pick cup");
}

TEST(RecordStatus, RecordPauseResumeElapsed) {
  RecordStatus r;
  r.start(at(0), "episode_1");
  auto s = r.snapshot(at(0));
  EXPECT_EQ(s.event, Event::Started);
  EXPECT_EQ(s.state, State::Recording);
  EXPECT_EQ(s.episode_name, "episode_1");
  EXPECT_EQ(s.message, "recording 00:00:00");

  s = r.snapshot(at(65));
  EXPECT_EQ(s.event, Event::None);
  EXPECT_DOUBLE_EQ(s.elapsed_sec, 65.0);
  EXPECT_EQ(s.message, "recording 00:01:05");

  r.pause(at(65));
  EXPECT_EQ(r.snapshot(at(65)).event, Event::Paused);
  s = r.snapshot(at(95));
  EXPECT_EQ(s.event, Event::None);
  EXPECT_EQ(s.state, State::Paused);
  EXPECT_DOUBLE_EQ(s.elapsed_sec, 65.0);
  EXPECT_EQ(s.message, "paused 00:01:05");

  r.resume(at(95));
  EXPECT_DOUBLE_EQ(r.elapsedSec(at(100)), 70.0);
}

TEST(RecordStatus, SaveFreezesElapsedAndTextSticks) {
  RecordStatus r;
  r.start(at(0), "e");
  r.pause(at(65));
  r.resume(at(95));
  EXPECT_DOUBLE_EQ(r.elapsedSec(at(100)), 70.0);
  r.stop(Event::Saved);
  auto s = r.snapshot(at(200));
  EXPECT_EQ(s.event, Event::Saved);
  EXPECT_EQ(s.state, State::Stopped);
  EXPECT_EQ(s.message, "saved");
  EXPECT_DOUBLE_EQ(s.elapsed_sec, 70.0);
  s = r.snapshot(at(201));
  EXPECT_EQ(s.event, Event::None);
  EXPECT_EQ(s.message, "saved");
  EXPECT_DOUBLE_EQ(s.elapsed_sec, 70.0);
}

TEST(RecordStatus, DiscardedCarriesDetail) {
  RecordStatus r;
  r.start(at(0), "e");
  r.stop(Event::Discarded, "too_short");
  auto s = r.snapshot(at(1));
  EXPECT_EQ(s.event, Event::Discarded);
  EXPECT_EQ(s.detail, "too_short");
  EXPECT_EQ(s.message, "discarded");
  EXPECT_EQ(r.snapshot(at(2)).detail, "");
}

TEST(RecordStatus, DeletedMessage) {
  RecordStatus r;
  r.start(at(0), "e");
  r.stop(Event::Deleted);
  auto s = r.snapshot(at(1));
  EXPECT_EQ(s.event, Event::Deleted);
  EXPECT_EQ(s.message, "deleted");
}

TEST(RecordStatus, ErrorState) {
  RecordStatus r;
  r.start(at(0), "e");
  r.error("boom");
  auto s = r.snapshot(at(1));
  EXPECT_EQ(s.state, State::Error);
  EXPECT_EQ(s.event, Event::Error);
  EXPECT_EQ(s.detail, "boom");
  EXPECT_EQ(s.message, "error");
  EXPECT_EQ(r.snapshot(at(2)).message, "error");
}

TEST(RecordStatus, RejectKeepsState) {
  RecordStatus r;
  r.start(at(0), "e");
  r.reject("nope");
  auto s = r.snapshot(at(3));
  EXPECT_EQ(s.event, Event::Rejected);
  EXPECT_EQ(s.state, State::Recording);
  EXPECT_EQ(s.detail, "nope");
  EXPECT_EQ(s.message, "recording 00:00:03");
}

TEST(RecordStatus, PausesAreExcludedFromElapsed) {
  RecordStatus r;
  r.start(at(0), "e");
  r.pause(at(0));
  r.resume(at(30));
  EXPECT_NEAR(r.elapsedSec(at(32)), 2.0, 1e-9);
}

TEST(RecordStatus, FormatElapsed) {
  EXPECT_EQ(RecordStatus::formatElapsed(3661), "01:01:01");
  EXPECT_EQ(RecordStatus::formatElapsed(-5), "00:00:00");
}

TEST(RecordStatus, EventSeqCountsEventsOnly) {
  RecordStatus r;
  EXPECT_EQ(r.snapshot(at(0)).event_seq, 0u);
  r.start(at(0), "e");
  EXPECT_EQ(r.snapshot(at(1)).event_seq, 1u);
  EXPECT_EQ(r.snapshot(at(2)).event_seq, 1u);
  r.pause(at(2));
  EXPECT_EQ(r.snapshot(at(3)).event_seq, 2u);
  EXPECT_EQ(r.snapshot(at(4)).event_seq, 2u);
}

TEST(RecordStatus, ErrorValueMatchesWireConstant) {
  EXPECT_EQ(static_cast<uint8_t>(State::Error), 4);
}
