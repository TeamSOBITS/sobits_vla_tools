#include "sobits_vla_rosbag_collection/topic_builder.hpp"
#include <set>

namespace sobits_vla
{

std::vector<std::string> TopicBuilder::buildTopicList(
  const RobotInfo & robot_info,
  const RosbagInfo & rosbag_info)
{
  std::vector<std::string> all_topics;

  // Sensor topics: base + optional compressed + optional cam_info variants
  for (const auto & sensor_type : robot_info.sensor_types) {
    auto it_topics = robot_info.sensor_topics.find(sensor_type);
    if (it_topics != robot_info.sensor_topics.end()) {
      for (const auto & topic : it_topics->second) {
        all_topics.push_back(topic);
      }
    }
    // Compressed topics from the explicit compressed_topics list
    auto it_comp = robot_info.sensor_compressed_topics.find(sensor_type);
    if (it_comp != robot_info.sensor_compressed_topics.end()) {
      for (const auto & compressed_topic : it_comp->second) {
        if (!compressed_topic.empty()) {
          all_topics.push_back(compressed_topic);
        }
      }
    }
    // Camera info topics from the explicit info_topics list
    auto it_info = robot_info.sensor_info_topics.find(sensor_type);
    if (it_info != robot_info.sensor_info_topics.end()) {
      for (const auto & info_topic : it_info->second) {
        if (!info_topic.empty()) {
          all_topics.push_back(info_topic);
        }
      }
    }
  }

  // Morphology: joint_states, cmd_vel, odom, and per-part explicit topics
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

  // Additional explicit topics from config
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
  // Query the live ROS graph for currently published topics
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
  // Primary camera topics
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

  // Check all topics to record against the graph
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

} // namespace sobits_vla
