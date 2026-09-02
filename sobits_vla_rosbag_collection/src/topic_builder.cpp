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

#include "sobits_vla_rosbag_collection/topic_builder.hpp"
#include <set>

namespace sobits_vla
{

std::vector<std::string> TopicBuilder::buildTopicList(
  const RobotInfo & robot_info,
  const RosbagInfo & rosbag_info)
{
  std::vector<std::string> all_topics;

  for (const auto & sensor_type : robot_info.sensor_types) {
    auto it_topics = robot_info.sensor_topics.find(sensor_type);
    if (it_topics != robot_info.sensor_topics.end()) {
      for (const auto & topic : it_topics->second) {
        if (!topic.empty()) {
          all_topics.push_back(topic);
        }
      }
    }
    auto it_comp = robot_info.sensor_compressed_topics.find(sensor_type);
    if (it_comp != robot_info.sensor_compressed_topics.end()) {
      for (const auto & compressed_topic : it_comp->second) {
        if (!compressed_topic.empty()) {
          all_topics.push_back(compressed_topic);
        }
      }
    }
    auto it_info = robot_info.sensor_info_topics.find(sensor_type);
    if (it_info != robot_info.sensor_info_topics.end()) {
      for (const auto & info_topic : it_info->second) {
        if (!info_topic.empty()) {
          all_topics.push_back(info_topic);
        }
      }
    }
  }

  if (!robot_info.joint_states_topic.empty()) {
    all_topics.push_back(robot_info.joint_states_topic);
  }
  for (const auto & part : robot_info.parts) {
    auto it_cmd_vel = robot_info.part_cmd_vel_topic.find(part);
    if (it_cmd_vel != robot_info.part_cmd_vel_topic.end() && !it_cmd_vel->second.empty()) {
      all_topics.push_back(it_cmd_vel->second);
    }
    auto it_odom = robot_info.part_odom_topic.find(part);
    if (it_odom != robot_info.part_odom_topic.end() && !it_odom->second.empty()) {
      all_topics.push_back(it_odom->second);
    }
    auto it_cmd = robot_info.part_command_topic.find(part);
    if (it_cmd != robot_info.part_command_topic.end() && !it_cmd->second.empty()) {
      all_topics.push_back(it_cmd->second);
    }
    auto it_state = robot_info.part_state_topic.find(part);
    if (it_state != robot_info.part_state_topic.end() && !it_state->second.empty()) {
      all_topics.push_back(it_state->second);
    }
  }

  for (const auto & topic : rosbag_info.additional_topics) {
    if (!topic.empty()) {
      all_topics.push_back(topic);
    }
  }

  // Deduplicate while preserving order
  std::vector<std::string> deduped;
  std::set<std::string> seen;
  for (const auto & t : all_topics) {
    if (seen.insert(t).second) {
      deduped.push_back(t);
    }
  }

  return deduped;
}

bool TopicBuilder::validateTopics(
  rclcpp::Node * node,
  const std::vector<std::string> & topics_to_record,
  const RobotInfo & robot_info)
{
  auto graph_topics = node->get_topic_names_and_types();
  std::set<std::string> active_topics;
  for (const auto & [name, types] : graph_topics) {
    active_topics.insert(name);
  }

  // Build a set of critical topics (those needed by conversion)
  std::set<std::string> critical;
  critical.insert(robot_info.joint_states_topic);
  for (const auto & part : robot_info.parts) {
    auto it_cmd = robot_info.part_command_topic.find(part);
    if (it_cmd != robot_info.part_command_topic.end() && !it_cmd->second.empty()) {
      critical.insert(it_cmd->second);
    }
    auto it_cmd_vel = robot_info.part_cmd_vel_topic.find(part);
    if (it_cmd_vel != robot_info.part_cmd_vel_topic.end() && !it_cmd_vel->second.empty()) {
      critical.insert(it_cmd_vel->second);
    }
  }
  for (const auto & sensor_type : robot_info.sensor_types) {
    auto it_topics = robot_info.sensor_topics.find(sensor_type);
    if (it_topics != robot_info.sensor_topics.end()) {
      for (const auto & topic : it_topics->second) {
        if (!topic.empty()) {
          critical.insert(topic);
        }
      }
    }
    auto it_comp = robot_info.sensor_compressed_topics.find(sensor_type);
    if (it_comp != robot_info.sensor_compressed_topics.end()) {
      for (const auto & topic : it_comp->second) {
        if (!topic.empty()) {
          critical.insert(topic);
        }
      }
    }
  }

  bool all_ok = true;
  std::vector<std::string> missing_critical;
  std::vector<std::string> missing_other;

  for (const auto & topic : topics_to_record) {
    if (active_topics.find(topic) == active_topics.end()) {
      if (critical.count(topic)) {
        missing_critical.push_back(topic);
      } else {
        missing_other.push_back(topic);
      }
    }
  }

  if (!missing_critical.empty()) {
    all_ok = false;
    RCLCPP_ERROR(node->get_logger(),
      "CRITICAL: %zu topic(s) required for dataset conversion are NOT published!",
      missing_critical.size());
    for (const auto & t : missing_critical) {
      std::string role = "unknown";
      if (t == robot_info.joint_states_topic) {
        role = "joint state observation";
      } else {
        for (const auto & part : robot_info.parts) {
          auto it_cmd = robot_info.part_command_topic.find(part);
          if (it_cmd != robot_info.part_command_topic.end() && it_cmd->second == t) {
            role = "joint command (" + part + ")";
            break;
          }
          auto it_cmd_vel = robot_info.part_cmd_vel_topic.find(part);
          if (it_cmd_vel != robot_info.part_cmd_vel_topic.end() && it_cmd_vel->second == t) {
            role = "base velocity command";
            break;
          }
        }
        if (role == "unknown") {
          for (const auto & stype : robot_info.sensor_types) {
            auto it_topics = robot_info.sensor_topics.find(stype);
            if (it_topics != robot_info.sensor_topics.end()) {
              for (size_t i = 0; i < it_topics->second.size(); ++i) {
                if (it_topics->second[i] == t) {
                  auto it_names = robot_info.sensor_names.find(stype);
                  role = "camera (" + (it_names != robot_info.sensor_names.end() &&
                    i < it_names->second.size() ?
                    it_names->second[i] : "?") + ")";
                  break;
                }
              }
            }
            if (role == "unknown") {
              auto it_comp = robot_info.sensor_compressed_topics.find(stype);
              if (it_comp != robot_info.sensor_compressed_topics.end()) {
                for (size_t i = 0; i < it_comp->second.size(); ++i) {
                  if (it_comp->second[i] == t) {
                    auto it_names = robot_info.sensor_names.find(stype);
                    role = "camera compressed (" + (it_names != robot_info.sensor_names.end() &&
                      i < it_names->second.size() ?
                      it_names->second[i] : "?") + ")";
                    break;
                  }
                }
              }
            }
          }
        }
      }
      RCLCPP_ERROR(node->get_logger(), "  MISSING: %s  (%s)", t.c_str(), role.c_str());
    }
  }

  if (!missing_other.empty()) {
    RCLCPP_WARN(node->get_logger(),
      "%zu non-critical topic(s) are not currently published:", missing_other.size());
    for (const auto & t : missing_other) {
      RCLCPP_WARN(node->get_logger(), "  not found: %s", t.c_str());
    }
  }

  return all_ok;
}

}  // namespace sobits_vla
