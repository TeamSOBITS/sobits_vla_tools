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

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "sobits_vla_rosbag_collection/robot_descriptor_loader.hpp"

namespace
{

namespace srd = sobits_robot_descriptor;
using sobits_vla::RobotInfo;

const char * kDescriptor =
  R"(
schema_version: 2
robot_id: test_robot
namespace: test_ns
urdf: {xacro: robot.urdf.xacro}
base_frame: base_footprint
joint_states_topic: joint_states
groups:
  - name: arm
    controller: arm_controller
    joints: [shoulder, elbow]
  - name: gripper
    controller: gripper_controller
    interface: group
    joints: [grip]
mobile_base:
  command_topic: cmd_vel
  odom_topic: odom
  has_vel_x: true
  has_vel_y: true
sensors:
  cameras:
    - name: head_camera
      frame: head_camera_link
      color:
        frame: head_camera_color_optical_frame
        raw_topic: head_camera/color/image_raw
        compressed_topic: head_camera/color/image_raw/compressed
        info_topic: head_camera/color/camera_info
      depth:
        frame: head_camera_depth_optical_frame
        raw_topic: head_camera/depth/image_raw
        info_topic: head_camera/depth/camera_info
        encoding: 16UC1
)";

srd::RobotDescriptor descriptor()
{
  const auto path = std::filesystem::temp_directory_path() / "vla_test_robot.robot.yaml";
  std::ofstream(path) << kDescriptor;
  return srd::load_file(path.string());
}

TEST(RobotDescriptorLoader, MapsDescriptorWithAbsoluteTopics)
{
  const RobotInfo info = sobits_vla::toRobotInfo(descriptor(), YAML::Node());
  EXPECT_EQ(info.name, "test_robot");
  EXPECT_EQ(info.morphology, "mobile_manipulator");
  EXPECT_EQ(info.joint_states_topic, "/test_ns/joint_states");
  EXPECT_EQ(info.parts, (std::vector<std::string>{"arm", "gripper", "mobile_base"}));
  EXPECT_EQ(info.part_command_topic.at("arm"), "/test_ns/arm_controller/joint_trajectory");
  EXPECT_EQ(info.part_state_topic.at("arm"), "/test_ns/arm_controller/controller_state");
  EXPECT_EQ(info.part_actions.at("arm").front(),
    "/test_ns/arm_controller/follow_joint_trajectory");
  EXPECT_EQ(info.part_state_topic.at("gripper"), "/test_ns/gripper_controller/commands/state");
  EXPECT_EQ(info.part_actions.count("gripper"), 0u);
  EXPECT_EQ(info.joint_names.at("arm"), (std::vector<std::string>{"shoulder", "elbow"}));
  EXPECT_EQ(info.part_cmd_vel_topic.at("mobile_base"), "/test_ns/cmd_vel");
  EXPECT_TRUE(info.part_has_cmd_vel_y.at("mobile_base"));
  // Depth is opt-in; colour encoding falls back to rgb8.
  EXPECT_EQ(info.sensor_names.at("camera"), (std::vector<std::string>{"head_camera"}));
  EXPECT_EQ(info.sensor_models.at("camera"), (std::vector<std::string>{"rgb8"}));
  EXPECT_EQ(info.sensor_info_topics.at("camera").front(),
    "/test_ns/head_camera/color/camera_info");
}

TEST(RobotDescriptorLoader, OverridesDeactivateAndEnableDepth)
{
  const YAML::Node ov =
    YAML::Load(
      R"(
groups: {gripper: {active: false}}
mobile_base: {active: false}
cameras: {head_camera: {active: false, depth: {active: true}}}
)");
  const RobotInfo info = sobits_vla::toRobotInfo(descriptor(), ov);
  EXPECT_EQ(info.parts, (std::vector<std::string>{"arm"}));
  EXPECT_EQ(info.sensor_names.at("camera"), (std::vector<std::string>{"head_camera_depth"}));
  EXPECT_EQ(info.sensor_models.at("camera"), (std::vector<std::string>{"16UC1"}));
  EXPECT_EQ(info.sensor_topics.at("camera").front(), "/test_ns/head_camera/depth/image_raw");
  EXPECT_EQ(info.sensor_compressed_topics.at("camera").front(), "");
}

TEST(RobotDescriptorLoader, UnknownOverrideNameThrows)
{
  const YAML::Node ov = YAML::Load("groups: {wing: {active: false}}");
  EXPECT_THROW(sobits_vla::toRobotInfo(descriptor(), ov), std::runtime_error);
}

TEST(RobotDescriptorLoader, MergeOverridesIsDeepByName)
{
  const YAML::Node base = YAML::Load("groups: {a: {active: true, max_joint_delta: 0.1}}");
  const YAML::Node out = sobits_vla::mergeOverrides(
    base, YAML::Load("groups: {a: {active: false}, b: {active: true}}"));
  EXPECT_FALSE(out["groups"]["a"]["active"].as<bool>());
  EXPECT_DOUBLE_EQ(out["groups"]["a"]["max_joint_delta"].as<double>(), 0.1);
  EXPECT_TRUE(out["groups"]["b"]["active"].as<bool>());
  EXPECT_TRUE(base["groups"]["a"]["active"].as<bool>());
}

TEST(RobotDescriptorLoader, LoadsInstalledSobitHome)
{
  if (sobits_vla::resolveRobotOverridesPath("sobit_home").empty()) {
    GTEST_SKIP() << "sobits_vla_common overrides not installed";
  }
  const RobotInfo info = sobits_vla::loadRobotInfo("sobit_home");
  EXPECT_EQ(info.parts.size(), 7u);
  EXPECT_EQ(info.joint_names.at("arm_left").size(), 7u);
  EXPECT_EQ(info.part_state_topic.at("head"),
    "/sobit_home/head_position_controller/controller_state");
  EXPECT_EQ(info.sensor_names.at("camera"),
    (std::vector<std::string>{"head_camera", "hand_left_camera", "hand_right_camera"}));
  EXPECT_EQ(info.sensor_info_topics.at("camera").front(),
    "/sobit_home/head_camera/color/camera_info");
}

TEST(RobotDescriptorLoader, LoadsSobitHomeV11ViaDescriptorPackage)
{
  if (sobits_vla::resolveRobotOverridesPath("sobit_home_v1_1").empty()) {
    GTEST_SKIP() << "sobits_vla_common overrides not installed";
  }
  const RobotInfo info = sobits_vla::loadRobotInfo("sobit_home_v1_1");
  EXPECT_EQ(info.version, "1.1.0");
  EXPECT_EQ(info.joint_names.at("arm_left").size(), 6u);
}

}  // namespace
