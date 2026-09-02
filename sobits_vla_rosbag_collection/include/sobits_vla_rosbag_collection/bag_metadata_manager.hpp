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

#ifndef SOBITS_VLA_ROSBAG_COLLECTION__BAG_METADATA_MANAGER_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__BAG_METADATA_MANAGER_HPP_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

namespace sobits_vla
{

class BagMetadataManager
{
public:
  BagMetadataManager(
    rclcpp::Node * node,
    const std::string & recording_dir,
    const RobotInfo & robot_info,
    const UserInfo & user_info);

  void createOrValidate(
    const std::map<std::string,
    std::pair<uint32_t, uint32_t>> & camera_dimensions);

  void updateRosbagYaml(
    const std::string & current_task_dir_name,
    const std::string & current_task_name,
    const std::string & current_task_path,
    const std::string & gamepad_name,
    const std::map<std::string, std::pair<uint32_t, uint32_t>> & camera_dimensions);

  void updateEpisodeYaml(
    const std::string & current_task_dir_name,
    const std::string & current_bag_name,
    const std::string & current_bag_path,
    std::vector<SubtaskInfo> & current_episode_subtasks);

  void removeEpisodeFromYaml(
    const std::string & current_task_dir_name,
    const std::string & current_bag_name);

private:
  rclcpp::Node * node_;
  std::string recording_dir_;
  RobotInfo robot_info_;
  UserInfo user_info_;
};

}  // namespace sobits_vla

#endif  // SOBITS_VLA_ROSBAG_COLLECTION__BAG_METADATA_MANAGER_HPP_
