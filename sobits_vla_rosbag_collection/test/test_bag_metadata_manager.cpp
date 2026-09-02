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


#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sobits_vla_rosbag_collection/bag_metadata_manager.hpp"

namespace
{

using sobits_vla::BagMetadataManager;
using sobits_vla::RobotInfo;
using sobits_vla::SubtaskInfo;
using sobits_vla::UserInfo;

// BagMetadataManager takes rclcpp::Node* (used only for logging + now()),
// so the minimal-seam choice is a real Node in the test fixture rather than
// changing the collaborator's constructor -- no production code touched.
class BagMetadataManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("bag_metadata_manager_test_node");
    tmp_dir_ = std::filesystem::temp_directory_path() /
      ("bag_meta_test_" + std::to_string(::testing::UnitTest::GetInstance()->
      current_test_info()->line()) + "_" + std::to_string(getpid()));
    std::filesystem::create_directories(tmp_dir_);

    robot_.name = "test_robot";
    robot_.version = "1.0.0";
    robot_.morphology = "mobile_manipulator";
    robot_.joint_states_topic = "/joint_states";
    robot_.parts = {"arm"};
    robot_.is_actionable["arm"] = true;
    robot_.joint_names["arm"] = {"shoulder", "elbow"};
    robot_.sensor_types = {"camera"};
    robot_.sensor_names["camera"] = {"head_camera"};
    robot_.sensor_topics["camera"] = {"/head_camera/image_raw"};

    user_.name = "tester";
    user_.email = "tester@example.com";
    user_.location = "test_lab";
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::filesystem::path tmp_dir_;
  RobotInfo robot_;
  UserInfo user_;
};

TEST_F(BagMetadataManagerTest, CreateOrValidateWritesExpectedYamlFields)
{
  BagMetadataManager mgr(node_.get(), tmp_dir_.string(), robot_, user_);
  std::map<std::string, std::pair<uint32_t, uint32_t>> dims;
  mgr.createOrValidate(dims);

  auto yaml_path = tmp_dir_ / "recorded_bags_meta.yaml";
  ASSERT_TRUE(std::filesystem::exists(yaml_path));

  YAML::Node doc = YAML::LoadFile(yaml_path.string());
  EXPECT_EQ(doc["robot_info"]["name"].as<std::string>(), "test_robot");
  EXPECT_EQ(doc["robot_info"]["morphology"]["joint_states_topic"].as<std::string>(),
    "/joint_states");
  EXPECT_EQ(doc["user_info"]["email"].as<std::string>(), "tester@example.com");
}

TEST_F(BagMetadataManagerTest, CreateOrValidateAcceptsMatchingExistingFile)
{
  BagMetadataManager mgr(node_.get(), tmp_dir_.string(), robot_, user_);
  std::map<std::string, std::pair<uint32_t, uint32_t>> dims;
  mgr.createOrValidate(dims);

  // Second manager over the same dir + same robot/user config -- must not throw.
  BagMetadataManager mgr2(node_.get(), tmp_dir_.string(), robot_, user_);
  EXPECT_NO_THROW(mgr2.createOrValidate(dims));
}

TEST_F(BagMetadataManagerTest, CreateOrValidateThrowsOnMismatchedConfig)
{
  BagMetadataManager mgr(node_.get(), tmp_dir_.string(), robot_, user_);
  std::map<std::string, std::pair<uint32_t, uint32_t>> dims;
  mgr.createOrValidate(dims);

  RobotInfo different = robot_;
  different.name = "a_different_robot";
  BagMetadataManager mgr2(node_.get(), tmp_dir_.string(), different, user_);
  EXPECT_THROW(mgr2.createOrValidate(dims), std::runtime_error);
}

TEST_F(BagMetadataManagerTest, UpdateAndReadBackEpisodeRoundTrip)
{
  BagMetadataManager mgr(node_.get(), tmp_dir_.string(), robot_, user_);
  std::map<std::string, std::pair<uint32_t, uint32_t>> dims;
  mgr.createOrValidate(dims);

  std::string task_dir = "pick_and_place_20260101_000000";
  std::string task_path = (tmp_dir_ / task_dir).string();
  std::filesystem::create_directories(task_path);
  mgr.updateRosbagYaml(task_dir, "pick and place", task_path, "quest", dims);

  std::string bag_name = "episode_20260101_000001";
  std::string bag_path = task_path + "/" + bag_name;
  std::vector<SubtaskInfo> subtasks;
  SubtaskInfo st;
  st.key = "subtask_20260101_000001";
  st.label = "reach";
  st.start_timestamp = 1.0;
  st.end_timestamp = 2.5;
  subtasks.push_back(st);

  mgr.updateEpisodeYaml(task_dir, bag_name, bag_path, subtasks);

  YAML::Node doc = YAML::LoadFile((tmp_dir_ / "recorded_bags_meta.yaml").string());
  auto episode = doc["recorded_bags"]["tasks"][task_dir]["episodes"][bag_name];
  ASSERT_TRUE(episode.IsDefined());
  EXPECT_EQ(episode["bag_path"].as<std::string>(), task_dir + "/" + bag_name);
  EXPECT_EQ(episode["subtasks"]["subtask_20260101_000001"]["label"].as<std::string>(), "reach");
  EXPECT_DOUBLE_EQ(
    episode["subtasks"]["subtask_20260101_000001"]["end_timestamp"].as<double>(), 2.5);

  auto episodes_list = doc["recorded_bags"]["tasks"][task_dir]["episodes_list"];
  ASSERT_TRUE(episodes_list.IsDefined());
  bool found = false;
  for (const auto & e : episodes_list) {
    if (e.as<std::string>() == bag_name) {found = true;}
  }
  EXPECT_TRUE(found);
}

TEST_F(BagMetadataManagerTest, RemoveEpisodeFromYamlDropsEntry)
{
  BagMetadataManager mgr(node_.get(), tmp_dir_.string(), robot_, user_);
  std::map<std::string, std::pair<uint32_t, uint32_t>> dims;
  mgr.createOrValidate(dims);

  std::string task_dir = "pick_and_place_20260101_000000";
  std::string task_path = (tmp_dir_ / task_dir).string();
  std::filesystem::create_directories(task_path);
  mgr.updateRosbagYaml(task_dir, "pick and place", task_path, "quest", dims);

  std::string bag_name = "episode_20260101_000001";
  std::string bag_path = task_path + "/" + bag_name;
  std::vector<SubtaskInfo> subtasks;
  mgr.updateEpisodeYaml(task_dir, bag_name, bag_path, subtasks);

  mgr.removeEpisodeFromYaml(task_dir, bag_name);

  YAML::Node doc = YAML::LoadFile((tmp_dir_ / "recorded_bags_meta.yaml").string());
  auto episode = doc["recorded_bags"]["tasks"][task_dir]["episodes"][bag_name];
  EXPECT_FALSE(episode.IsDefined());

  auto episodes_list = doc["recorded_bags"]["tasks"][task_dir]["episodes_list"];
  for (const auto & e : episodes_list) {
    EXPECT_NE(e.as<std::string>(), bag_name);
  }
}

TEST_F(BagMetadataManagerTest, RemoveEpisodeOnMissingYamlIsNoop)
{
  BagMetadataManager mgr(node_.get(), tmp_dir_.string(), robot_, user_);
  EXPECT_NO_THROW(mgr.removeEpisodeFromYaml("no_such_task", "no_such_bag"));
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
