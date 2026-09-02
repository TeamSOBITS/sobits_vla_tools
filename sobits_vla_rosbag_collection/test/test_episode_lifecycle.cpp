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
#include <memory>
#include <regex>
#include <string>
#include <vector>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/ros_helper.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include "sobits_vla_rosbag_collection/episode_lifecycle.hpp"

namespace
{

// Writes a genuine rosbag2 bag (real metadata.yaml + storage file) via the
// same Writer API the node uses, rather than hand-crafting YAML -- rosbag2's
// metadata schema has required fields (e.g. offered_qos_profiles) that are
// easy to get subtly wrong by hand.
void writeRealBag(
  const std::filesystem::path & bag_dir, const std::string & storage_id,
  size_t message_count)
{
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir.string();
  storage_options.storage_id = storage_id;
  writer.open(storage_options);

  rosbag2_storage::TopicMetadata topic;
  topic.name = "/test_topic";
  topic.type = "std_msgs/msg/String";
  topic.serialization_format = "cdr";
  writer.create_topic(topic);

  for (size_t i = 0; i < message_count; ++i) {
    auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    msg->topic_name = topic.name;
    msg->send_timestamp = static_cast<rcutils_time_point_value_t>(i);
    msg->recv_timestamp = static_cast<rcutils_time_point_value_t>(i);
    msg->serialized_data = rosbag2_storage::make_empty_serialized_message(0);
    writer.write(msg);
  }
  // Destructor also flushes, but close() explicitly finalizes metadata.yaml
  // before the test reads it back.
}

class EpisodeLifecycleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
      ("episode_lifecycle_test_" + std::to_string(::testing::UnitTest::GetInstance()->
      current_test_info()->line()) + "_" + std::to_string(getpid()));
    std::filesystem::create_directories(tmp_dir_);
    lifecycle_ = std::make_unique<sobits_vla::EpisodeLifecycle>(
      "mcap",
      [this](const std::string & m) {infos_.push_back(m);},
      [this](const std::string & m) {warns_.push_back(m);},
      [this](const std::string & m) {errors_.push_back(m);});
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
  std::unique_ptr<sobits_vla::EpisodeLifecycle> lifecycle_;
  std::vector<std::string> infos_, warns_, errors_;
};

TEST_F(EpisodeLifecycleTest, TimestampFormatIsFourteenDigitsWithUnderscore)
{
  std::string ts = sobits_vla::EpisodeLifecycle::timestampString();
  // %Y%m%d_%H%M%S -> 8 digits, underscore, 6 digits.
  EXPECT_TRUE(std::regex_match(ts, std::regex("^[0-9]{8}_[0-9]{6}$"))) << ts;
}

TEST_F(EpisodeLifecycleTest, MakeBagNameStartsWithEpisodePrefix)
{
  std::string name = sobits_vla::EpisodeLifecycle::makeBagName();
  EXPECT_EQ(name.rfind("episode_", 0), 0u);
  EXPECT_TRUE(std::regex_match(name, std::regex("^episode_[0-9]{8}_[0-9]{6}$"))) << name;
}

TEST_F(EpisodeLifecycleTest, MakeBagPathJoinsWithSlash)
{
  EXPECT_EQ(
    sobits_vla::EpisodeLifecycle::makeBagPath("/data/mytask", "episode_20260101_000000"),
    "/data/mytask/episode_20260101_000000");
}

TEST_F(EpisodeLifecycleTest, VerifyIntegrityFailsOnMissingDirectory)
{
  auto result = lifecycle_->verifyIntegrity((tmp_dir_ / "does_not_exist").string());
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(errors_.empty());
}

TEST_F(EpisodeLifecycleTest, VerifyIntegrityFailsOnNoStorageFile)
{
  auto bag_dir = tmp_dir_ / "empty_bag";
  std::filesystem::create_directories(bag_dir);
  auto result = lifecycle_->verifyIntegrity(bag_dir.string());
  EXPECT_FALSE(result.ok);
  ASSERT_FALSE(errors_.empty());
  EXPECT_NE(errors_.back().find("No .mcap or .db3"), std::string::npos);
}

TEST_F(EpisodeLifecycleTest, VerifyIntegrityFailsOnEmptyStorageFile)
{
  auto bag_dir = tmp_dir_ / "zero_byte_bag";
  std::filesystem::create_directories(bag_dir);
  std::ofstream(bag_dir / "file-000.mcap").close();
  auto result = lifecycle_->verifyIntegrity(bag_dir.string());
  EXPECT_FALSE(result.ok);
  ASSERT_FALSE(errors_.empty());
  EXPECT_NE(errors_.back().find("empty (0 bytes)"), std::string::npos);
}

TEST_F(EpisodeLifecycleTest, VerifyIntegrityFailsWhenReaderCannotOpen)
{
  // Non-empty but not a real mcap and no metadata.yaml -- reader.open() throws.
  auto bag_dir = tmp_dir_ / "garbage_bag";
  std::filesystem::create_directories(bag_dir);
  std::ofstream mcap(bag_dir / "file-000.mcap");
  mcap << "not a real mcap file, just needs nonzero size";
  mcap.close();
  auto result = lifecycle_->verifyIntegrity(bag_dir.string());
  EXPECT_FALSE(result.ok);
}

TEST_F(EpisodeLifecycleTest, VerifyIntegrityFailsOnZeroMessages)
{
  auto bag_dir = tmp_dir_ / "zero_msg_bag";
  writeRealBag(bag_dir, "mcap", 0);
  auto result = lifecycle_->verifyIntegrity(bag_dir.string());
  EXPECT_FALSE(result.ok);
}

TEST_F(EpisodeLifecycleTest, VerifyIntegritySucceedsWithMessages)
{
  auto bag_dir = tmp_dir_ / "valid_bag";
  writeRealBag(bag_dir, "mcap", 5);
  auto result = lifecycle_->verifyIntegrity(bag_dir.string());
  EXPECT_TRUE(result.ok) << (result.reason.empty() ? "no reason" : result.reason);
  EXPECT_EQ(result.message_count, 5u);
  EXPECT_EQ(result.topic_count, 1u);
}

TEST_F(EpisodeLifecycleTest, DecideSaveDiscardsWhenTooShort)
{
  auto decision = lifecycle_->decideSave("/irrelevant/path", 0.5, 1.0);
  EXPECT_FALSE(decision.keep);
  EXPECT_EQ(decision.discard_reason, "too_short");
}

TEST_F(EpisodeLifecycleTest, DecideSaveIgnoresMinDurationWhenDisabled)
{
  // min_episode_duration_sec == 0 disables the check; falls through to
  // integrity, which fails fast on a nonexistent path -- still a discard,
  // but for a different reason, proving the duration gate was skipped.
  auto decision = lifecycle_->decideSave("/irrelevant/path", 0.1, 0.0);
  EXPECT_FALSE(decision.keep);
  EXPECT_EQ(decision.discard_reason, "integrity_failed");
}

TEST_F(EpisodeLifecycleTest, DecideSaveKeepsWhenLongEnoughAndValid)
{
  auto bag_dir = tmp_dir_ / "keep_bag";
  writeRealBag(bag_dir, "mcap", 3);
  auto decision = lifecycle_->decideSave(bag_dir.string(), 5.0, 1.0);
  EXPECT_TRUE(decision.keep);
  EXPECT_TRUE(decision.discard_reason.empty());
}

TEST_F(EpisodeLifecycleTest, RemoveBagDirDeletesExistingDirectory)
{
  auto bag_dir = tmp_dir_ / "to_remove";
  std::filesystem::create_directories(bag_dir);
  std::ofstream(bag_dir / "file.mcap").close();
  ASSERT_TRUE(std::filesystem::exists(bag_dir));

  EXPECT_TRUE(lifecycle_->removeBagDir(bag_dir.string()));
  EXPECT_FALSE(std::filesystem::exists(bag_dir));
}

TEST_F(EpisodeLifecycleTest, RemoveBagDirOnMissingPathIsNoopSuccess)
{
  EXPECT_TRUE(lifecycle_->removeBagDir((tmp_dir_ / "never_existed").string()));
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
