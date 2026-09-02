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

#include <algorithm>
#include <string>
#include <vector>

#include "sobits_vla_rosbag_collection/topic_builder.hpp"

namespace
{

using sobits_vla::RobotInfo;
using sobits_vla::RosbagInfo;

RobotInfo makeFabricatedRobotInfo()
{
  RobotInfo robot;
  robot.joint_states_topic = "/robot/joint_states";
  robot.parts = {"arm", "mobile_base"};

  robot.part_command_topic["arm"] = "/robot/arm/joint_trajectory";
  robot.part_state_topic["arm"] = "/robot/arm/controller_state";

  robot.part_cmd_vel_topic["mobile_base"] = "/robot/cmd_vel";
  robot.part_odom_topic["mobile_base"] = "/robot/odom";

  robot.sensor_types = {"camera"};
  robot.sensor_names["camera"] = {"head_camera"};
  robot.sensor_topics["camera"] = {"/robot/head_camera/color/image_raw"};
  robot.sensor_compressed_topics["camera"] = {"/robot/head_camera/color/image_raw/compressed"};
  robot.sensor_info_topics["camera"] = {"/robot/head_camera/camera_info"};

  return robot;
}

bool contains(const std::vector<std::string> & v, const std::string & item)
{
  return std::find(v.begin(), v.end(), item) != v.end();
}

}  // namespace

TEST(TopicBuilderTest, IncludesAllSensorAndMorphologyTopics)
{
  RobotInfo robot = makeFabricatedRobotInfo();
  RosbagInfo rosbag_info;

  auto topics = sobits_vla::TopicBuilder::buildTopicList(robot, rosbag_info);

  EXPECT_TRUE(contains(topics, "/robot/head_camera/color/image_raw"));
  EXPECT_TRUE(contains(topics, "/robot/head_camera/color/image_raw/compressed"));
  EXPECT_TRUE(contains(topics, "/robot/head_camera/camera_info"));
  EXPECT_TRUE(contains(topics, "/robot/joint_states"));
  EXPECT_TRUE(contains(topics, "/robot/cmd_vel"));
  EXPECT_TRUE(contains(topics, "/robot/odom"));
  EXPECT_TRUE(contains(topics, "/robot/arm/joint_trajectory"));
  EXPECT_TRUE(contains(topics, "/robot/arm/controller_state"));
}

TEST(TopicBuilderTest, IncludesAdditionalTopicsFromConfig)
{
  RobotInfo robot = makeFabricatedRobotInfo();
  RosbagInfo rosbag_info;
  rosbag_info.additional_topics = {"/tf", "/tf_static"};

  auto topics = sobits_vla::TopicBuilder::buildTopicList(robot, rosbag_info);

  EXPECT_TRUE(contains(topics, "/tf"));
  EXPECT_TRUE(contains(topics, "/tf_static"));
}

TEST(TopicBuilderTest, SkipsEmptyTopicStrings)
{
  RobotInfo robot = makeFabricatedRobotInfo();
  RosbagInfo rosbag_info;
  rosbag_info.additional_topics = {"", "/tf"};
  // A part with an empty command_topic must not contribute "".
  robot.part_command_topic["mobile_base"] = "";

  auto topics = sobits_vla::TopicBuilder::buildTopicList(robot, rosbag_info);

  EXPECT_FALSE(contains(topics, ""));
}

TEST(TopicBuilderTest, DeduplicatesWhilePreservingFirstOccurrenceOrder)
{
  RobotInfo robot = makeFabricatedRobotInfo();
  RosbagInfo rosbag_info;
  // Same topic reachable via sensor_topics AND additional_topics.
  rosbag_info.additional_topics = {"/robot/head_camera/color/image_raw"};

  auto topics = sobits_vla::TopicBuilder::buildTopicList(robot, rosbag_info);

  int count = 0;
  for (const auto & t : topics) {
    if (t == "/robot/head_camera/color/image_raw") {++count;}
  }
  EXPECT_EQ(count, 1);
  // First occurrence (from sensor_topics) comes before joint_states in the
  // original build order, so it should still be near the front.
  auto it = std::find(topics.begin(), topics.end(), "/robot/head_camera/color/image_raw");
  ASSERT_NE(it, topics.end());
  EXPECT_EQ(it, topics.begin());
}

TEST(TopicBuilderTest, EmptyRobotInfoProducesEmptyList)
{
  RobotInfo robot;
  RosbagInfo rosbag_info;
  auto topics = sobits_vla::TopicBuilder::buildTopicList(robot, rosbag_info);
  EXPECT_TRUE(topics.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
