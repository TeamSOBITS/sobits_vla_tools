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

#include "sobits_vla_common/status_topic.hpp"

using sobits_vla::recordStateName;
using sobits_vla::statusTopicFromService;

TEST(StatusTopic, ReplacesTrailingCommand)
{
  EXPECT_EQ(
    statusTopicFromService("vla_rosbag_collection/command"),
    "vla_rosbag_collection/record_status");
  EXPECT_EQ(
    statusTopicFromService("/sobit_home/vla_rosbag_collection/command"),
    "/sobit_home/vla_rosbag_collection/record_status");
}

TEST(StatusTopic, EmptyWhenNotACommandService)
{
  EXPECT_EQ(statusTopicFromService(""), "");
  EXPECT_EQ(statusTopicFromService("/command"), "");
  EXPECT_EQ(statusTopicFromService("command"), "");
  EXPECT_EQ(statusTopicFromService("a/command_x"), "");
}

TEST(StatusTopic, StateNames)
{
  EXPECT_STREQ(recordStateName(0), "STOPPED");
  EXPECT_STREQ(recordStateName(1), "RECORDING");
  EXPECT_STREQ(recordStateName(2), "PAUSED");
  EXPECT_STREQ(recordStateName(4), "ERROR");
  EXPECT_STREQ(recordStateName(3), "UNKNOWN");
}
