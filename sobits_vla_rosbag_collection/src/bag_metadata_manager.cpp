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

#include "sobits_vla_rosbag_collection/bag_metadata_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>

namespace sobits_vla
{

BagMetadataManager::BagMetadataManager(
  rclcpp::Node * node,
  const std::string & recording_dir,
  const RobotInfo & robot_info,
  const UserInfo & user_info)
: node_(node),
  recording_dir_(recording_dir),
  robot_info_(robot_info),
  user_info_(user_info)
{
}

void BagMetadataManager::createOrValidate(
  const std::map<std::string,
  std::pair<uint32_t, uint32_t>> & camera_dimensions)
{
  std::string yaml_file_path = recording_dir_ + "/recorded_bags_meta.yaml";

  if (std::filesystem::exists(yaml_file_path)) {
    RCLCPP_INFO(node_->get_logger(),
        "Found existing metadata: %s — validating config consistency...", yaml_file_path.c_str());
    try {
      YAML::Node existing = YAML::LoadFile(yaml_file_path);
      auto existing_robot = existing["robot_info"];

      std::vector<std::string> mismatches;

      if (existing_robot["name"].as<std::string>("") != robot_info_.name) {
        mismatches.push_back("robot_info.name: '" + existing_robot["name"].as<std::string>("") +
            "' vs '" + robot_info_.name + "'");
      }
      if (existing_robot["version"].as<std::string>("") != robot_info_.version) {
        mismatches.push_back("robot_info.version: '" +
            existing_robot["version"].as<std::string>("") + "' vs '" + robot_info_.version + "'");
      }

      auto existing_morph = existing_robot["morphology"];
      if (existing_morph["type"].as<std::string>("") != robot_info_.morphology) {
        mismatches.push_back("morphology.type: '" + existing_morph["type"].as<std::string>("") +
            "' vs '" + robot_info_.morphology + "'");
      }
      if (existing_morph["joint_states_topic"].as<std::string>("") !=
        robot_info_.joint_states_topic)
      {
        mismatches.push_back("joint_states_topic: '" +
            existing_morph["joint_states_topic"].as<std::string>("") + "' vs '" +
            robot_info_.joint_states_topic + "'");
      }

      std::vector<std::string> existing_parts;
      if (existing_morph["parts"].IsDefined()) {
        for (const auto & p : existing_morph["parts"]) {
          existing_parts.push_back(p.as<std::string>());
        }
      }
      if (existing_parts != robot_info_.parts) {
        mismatches.push_back("morphology.parts differ");
      }

      for (const auto & part : robot_info_.parts) {
        if (!existing_morph[part].IsDefined()) {
          mismatches.push_back("part '" + part + "' missing from existing metadata");
          continue;
        }
        auto it_act = robot_info_.is_actionable.find(part);
        bool expected_act = (it_act != robot_info_.is_actionable.end()) ? it_act->second : false;
        if (existing_morph[part]["is_actionable"].as<bool>(false) != expected_act) {
          mismatches.push_back("part '" + part + "' is_actionable mismatch");
        }
        std::vector<std::string> existing_joints;
        if (existing_morph[part]["joint_names"].IsDefined()) {
          for (const auto & j : existing_morph[part]["joint_names"]) {
            existing_joints.push_back(j.as<std::string>());
          }
        }
        auto it_joints = robot_info_.joint_names.find(part);
        std::vector<std::string> expected_joints = (it_joints !=
          robot_info_.joint_names.end()) ? it_joints->second : std::vector<std::string>();
        if (existing_joints != expected_joints) {
          mismatches.push_back("part '" + part + "' joint_names differ");
        }
      }

      auto existing_sensors = existing_robot["sensors"];
      std::vector<std::string> existing_sensor_types;
      if (existing_sensors["types"].IsDefined()) {
        for (const auto & st : existing_sensors["types"]) {
          existing_sensor_types.push_back(st.as<std::string>());
        }
      }
      if (existing_sensor_types != robot_info_.sensor_types) {
        mismatches.push_back("sensors.types differ");
      }
      for (const auto & stype : robot_info_.sensor_types) {
        if (!existing_sensors[stype].IsDefined()) {
          mismatches.push_back("sensor type '" + stype + "' missing from existing metadata");
          continue;
        }
        std::vector<std::string> existing_names;
        if (existing_sensors[stype]["names"].IsDefined()) {
          for (const auto & n : existing_sensors[stype]["names"]) {
            existing_names.push_back(n.as<std::string>());
          }
        }
        auto it_names = robot_info_.sensor_names.find(stype);
        std::vector<std::string> expected_names = (it_names !=
          robot_info_.sensor_names.end()) ? it_names->second : std::vector<std::string>();
        if (existing_names != expected_names) {
          mismatches.push_back("sensor '" + stype + "' names differ");
        }
        std::vector<std::string> existing_topics;
        if (existing_sensors[stype]["topics"].IsDefined()) {
          for (const auto & t : existing_sensors[stype]["topics"]) {
            existing_topics.push_back(t.as<std::string>());
          }
        }
        auto it_topics = robot_info_.sensor_topics.find(stype);
        std::vector<std::string> expected_topics = (it_topics !=
          robot_info_.sensor_topics.end()) ? it_topics->second : std::vector<std::string>();
        if (existing_topics != expected_topics) {
          mismatches.push_back("sensor '" + stype + "' topics differ");
        }
      }

      auto existing_user = existing["user_info"];
      if (existing_user["name"].as<std::string>("") != user_info_.name) {
        mismatches.push_back("user_info.name: '" + existing_user["name"].as<std::string>("") +
            "' vs '" + user_info_.name + "'");
      }
      if (existing_user["email"].as<std::string>("") != user_info_.email) {
        mismatches.push_back("user_info.email: '" + existing_user["email"].as<std::string>("") +
            "' vs '" + user_info_.email + "'");
      }
      if (existing_user["location"].as<std::string>("") != user_info_.location) {
        mismatches.push_back("user_info.location: '" +
            existing_user["location"].as<std::string>("") + "' vs '" + user_info_.location + "'");
      }

      if (!mismatches.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
          "Config mismatch with existing metadata! %zu difference(s) found:", mismatches.size());
        for (const auto & m : mismatches) {
          RCLCPP_ERROR(node_->get_logger(), "  - %s", m.c_str());
        }
        RCLCPP_ERROR(node_->get_logger(),
          "Cannot resume recording with different robot config. "
          "Either use the same config or record to a different directory.");
        throw std::runtime_error("Config mismatch with existing recorded_bags_meta.yaml");
      }

      RCLCPP_INFO(node_->get_logger(),
          "Config validation passed — resuming with existing metadata.");
      return;
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(node_->get_logger(),
          "Failed to parse existing metadata: %s. Cannot resume safely.", e.what());
      throw std::runtime_error("Failed to parse existing recorded_bags_meta.yaml");
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Creating new rosbag YAML file...");
  YAML::Node yaml_node;

  yaml_node["robot_info"]["name"] = robot_info_.name;
  yaml_node["robot_info"]["version"] = robot_info_.version;
  yaml_node["robot_info"]["morphology"]["type"] = robot_info_.morphology;
  yaml_node["robot_info"]["morphology"]["joint_states_topic"] = robot_info_.joint_states_topic;
  yaml_node["robot_info"]["morphology"]["parts"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & part : robot_info_.parts) {
    yaml_node["robot_info"]["morphology"]["parts"].push_back(part);
    auto it_act = robot_info_.is_actionable.find(part);
    yaml_node["robot_info"]["morphology"][part]["is_actionable"] = (it_act !=
      robot_info_.is_actionable.end()) ? it_act->second : false;

    auto it_cmd = robot_info_.part_command_topic.find(part);
    if (it_cmd != robot_info_.part_command_topic.end() && !it_cmd->second.empty()) {
      yaml_node["robot_info"]["morphology"][part]["command_topic"] = it_cmd->second;
    }
    auto it_state = robot_info_.part_state_topic.find(part);
    if (it_state != robot_info_.part_state_topic.end() && !it_state->second.empty()) {
      yaml_node["robot_info"]["morphology"][part]["state_topic"] = it_state->second;
    }
    auto it_act_list = robot_info_.part_actions.find(part);
    if (it_act_list != robot_info_.part_actions.end() && !it_act_list->second.empty()) {
      yaml_node["robot_info"]["morphology"][part]["actions"] = YAML::Node(YAML::NodeType::Sequence);
      for (const auto & action : it_act_list->second) {
        yaml_node["robot_info"]["morphology"][part]["actions"].push_back(action);
      }
    }

    // A base part is one that carries a cmd_vel topic -- not one whose name
    // happens to match a known literal.
    if (robot_info_.part_cmd_vel_topic.count(part) > 0) {
      auto it_y = robot_info_.part_has_cmd_vel_y.find(part);
      yaml_node["robot_info"]["morphology"][part]["has_cmd_vel_y"] = (it_y !=
        robot_info_.part_has_cmd_vel_y.end()) ? it_y->second : false;
      auto it_z = robot_info_.part_has_cmd_vel_z.find(part);
      yaml_node["robot_info"]["morphology"][part]["has_cmd_vel_z"] = (it_z !=
        robot_info_.part_has_cmd_vel_z.end()) ? it_z->second : false;
      auto it_vel = robot_info_.part_cmd_vel_topic.find(part);
      if (it_vel != robot_info_.part_cmd_vel_topic.end()) {
        yaml_node["robot_info"]["morphology"][part]["cmd_vel_topic"] = it_vel->second;
      }
      auto it_odom = robot_info_.part_odom_topic.find(part);
      if (it_odom != robot_info_.part_odom_topic.end()) {
        yaml_node["robot_info"]["morphology"][part]["odom_topic"] = it_odom->second;
      }
    }

    yaml_node["robot_info"]["morphology"][part]["joint_names"] =
      YAML::Node(YAML::NodeType::Sequence);
    auto it_joints = robot_info_.joint_names.find(part);
    if (it_joints != robot_info_.joint_names.end()) {
      for (const auto & joint_name : it_joints->second) {
        yaml_node["robot_info"]["morphology"][part]["joint_names"].push_back(joint_name);
      }
    }
  }
  yaml_node["robot_info"]["sensors"]["types"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & sensor_type : robot_info_.sensor_types) {
    yaml_node["robot_info"]["sensors"]["types"].push_back(sensor_type);
    yaml_node["robot_info"]["sensors"][sensor_type]["names"] = YAML::Node(YAML::NodeType::Sequence);
    yaml_node["robot_info"]["sensors"][sensor_type]["models"] =
      YAML::Node(YAML::NodeType::Sequence);
    yaml_node["robot_info"]["sensors"][sensor_type]["topics"] =
      YAML::Node(YAML::NodeType::Sequence);

    auto it_names = robot_info_.sensor_names.find(sensor_type);
    std::vector<std::string> sensor_names = (it_names !=
      robot_info_.sensor_names.end()) ? it_names->second : std::vector<std::string>();
    auto it_info = robot_info_.sensor_info_topics.find(sensor_type);
    std::vector<std::string> info_topics = (it_info !=
      robot_info_.sensor_info_topics.end()) ? it_info->second : std::vector<std::string>();

    for (size_t i = 0; i < sensor_names.size(); ++i) {
      const auto & sensor_name = sensor_names[i];
      yaml_node["robot_info"]["sensors"][sensor_type]["names"].push_back(sensor_name);

      std::string matched_info_topic = (i < info_topics.size()) ? info_topics[i] : "";
      if (!matched_info_topic.empty()) {
        auto it = camera_dimensions.find(matched_info_topic);
        if (it != camera_dimensions.end()) {
          yaml_node["robot_info"]["sensors"][sensor_type]["properties"][sensor_name]["width"] =
            it->second.first;
          yaml_node["robot_info"]["sensors"][sensor_type]["properties"][sensor_name]["height"] =
            it->second.second;
          yaml_node["robot_info"]["sensors"][sensor_type]["properties"][sensor_name]["topic"] =
            matched_info_topic;
        }
      }
    }
    auto it_models = robot_info_.sensor_models.find(sensor_type);
    if (it_models != robot_info_.sensor_models.end()) {
      for (const auto & sensor_model : it_models->second) {
        yaml_node["robot_info"]["sensors"][sensor_type]["models"].push_back(sensor_model);
      }
    }
    auto it_topics = robot_info_.sensor_topics.find(sensor_type);
    if (it_topics != robot_info_.sensor_topics.end()) {
      for (const auto & sensor_topic : it_topics->second) {
        yaml_node["robot_info"]["sensors"][sensor_type]["topics"].push_back(sensor_topic);
      }
    }
    if (it_info != robot_info_.sensor_info_topics.end()) {
      for (const auto & info_topic : it_info->second) {
        yaml_node["robot_info"]["sensors"][sensor_type]["info_topics"].push_back(info_topic);
      }
    }
    auto it_comp = robot_info_.sensor_compressed_topics.find(sensor_type);
    if (it_comp != robot_info_.sensor_compressed_topics.end()) {
      for (const auto & compressed_topic : it_comp->second) {
        yaml_node["robot_info"]["sensors"][sensor_type]["compressed_topics"].push_back(
            compressed_topic);
      }
    }
  }

  yaml_node["user_info"]["name"] = user_info_.name;
  yaml_node["user_info"]["email"] = user_info_.email;
  yaml_node["user_info"]["location"] = user_info_.location;

  try {
    std::ofstream yaml_file(yaml_file_path);
    if (!yaml_file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open YAML file for writing: %s",
          yaml_file_path.c_str());
      throw std::runtime_error("Failed to open YAML file for writing");
    }
    YAML::Emitter emitter;
    emitter << YAML::Block << yaml_node;
    yaml_file << emitter.c_str();
    yaml_file.close();
    RCLCPP_INFO(node_->get_logger(), "Created rosbag YAML file: %s", yaml_file_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create rosbag YAML file: %s", e.what());
    throw std::runtime_error("Failed to create rosbag YAML file");
  }
}

void BagMetadataManager::updateRosbagYaml(
  const std::string & current_task_dir_name,
  const std::string & current_task_name,
  const std::string & current_task_path,
  const std::string & gamepad_name,
  const std::map<std::string, std::pair<uint32_t, uint32_t>> & camera_dimensions)
{
  RCLCPP_INFO(node_->get_logger(), "Updating rosbag YAML file...");

  std::string yaml_file_path = recording_dir_ + "/recorded_bags_meta.yaml";
  YAML::Node yaml_node;
  try {
    yaml_node = YAML::LoadFile(yaml_file_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file: %s", e.what());
    throw std::runtime_error("Failed to load YAML file");
  }

  std::string current_task_label = current_task_dir_name;

  yaml_node["recorded_bags"]["tasks_list"].push_back(current_task_label);
  yaml_node["recorded_bags"]["tasks"][current_task_label]["label"] = current_task_name;
  // Relative to recording_dir_, same convention as the per-episode bag_path
  // below, so metadata survives moving the rosbags folder.
  std::string stored_task_path = current_task_path;
  {
    const std::string recording_prefix = recording_dir_ + "/";
    if (stored_task_path.rfind(recording_prefix, 0) == 0) {
      stored_task_path = stored_task_path.substr(recording_prefix.size());
    }
  }
  yaml_node["recorded_bags"]["tasks"][current_task_label]["bag_dir"] = stored_task_path;
  yaml_node["recorded_bags"]["tasks"][current_task_label]["gamepad"] = gamepad_name;

  for (const auto & sensor_type : robot_info_.sensor_types) {
    auto it_names = robot_info_.sensor_names.find(sensor_type);
    std::vector<std::string> sensor_names = (it_names !=
      robot_info_.sensor_names.end()) ? it_names->second : std::vector<std::string>();
    auto it_info = robot_info_.sensor_info_topics.find(sensor_type);
    std::vector<std::string> info_topics = (it_info !=
      robot_info_.sensor_info_topics.end()) ? it_info->second : std::vector<std::string>();

    for (size_t i = 0; i < sensor_names.size(); ++i) {
      const auto & sensor_name = sensor_names[i];
      std::string matched_info_topic = (i < info_topics.size()) ? info_topics[i] : "";
      if (matched_info_topic.empty()) {
        continue;
      }
      auto it = camera_dimensions.find(matched_info_topic);
      if (it == camera_dimensions.end()) {
        continue;
      }
      yaml_node["robot_info"]["sensors"][sensor_type]["properties"][sensor_name]["width"] =
        it->second.first;
      yaml_node["robot_info"]["sensors"][sensor_type]["properties"][sensor_name]["height"] =
        it->second.second;
      yaml_node["robot_info"]["sensors"][sensor_type]["properties"][sensor_name]["topic"] =
        matched_info_topic;
    }
  }

  try {
    std::ofstream yaml_file(yaml_file_path);
    if (!yaml_file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open YAML file for writing: %s",
          yaml_file_path.c_str());
      throw std::runtime_error("Failed to open YAML file for writing");
    }
    YAML::Emitter emitter;
    emitter << YAML::Block << yaml_node;
    yaml_file << emitter.c_str();
    yaml_file.close();
    RCLCPP_DEBUG(node_->get_logger(), "Updated rosbag YAML file: %s", yaml_file_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to update rosbag YAML file: %s", e.what());
    throw std::runtime_error("Failed to update rosbag YAML file");
  }
}

void BagMetadataManager::updateEpisodeYaml(
  const std::string & current_task_dir_name,
  const std::string & current_bag_name,
  const std::string & current_bag_path,
  std::vector<SubtaskInfo> & current_episode_subtasks)
{
  RCLCPP_INFO(node_->get_logger(), "Updating episode in rosbag YAML file...");

  if (!current_episode_subtasks.empty() && current_episode_subtasks.back().end_timestamp == 0.0) {
    current_episode_subtasks.back().end_timestamp = node_->now().seconds();
  }

  std::string yaml_file_path = recording_dir_ + "/recorded_bags_meta.yaml";
  YAML::Node yaml_node;
  try {
    if (std::filesystem::exists(yaml_file_path)) {
      yaml_node = YAML::LoadFile(yaml_file_path);
    } else {
      RCLCPP_WARN(node_->get_logger(), "YAML file not found, creating a new one");
      std::map<std::string, std::pair<uint32_t, uint32_t>> dummy;
      createOrValidate(dummy);
      yaml_node = YAML::LoadFile(yaml_file_path);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file: %s", e.what());
    return;
  }

  std::string current_task_label = current_task_dir_name;

  if (yaml_node["recorded_bags"]["tasks"][current_task_label].IsDefined()) {
    if (!yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].IsDefined()) {
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"] =
        YAML::Node(YAML::NodeType::Sequence);
    }
    bool episode_in_list = false;
    for (YAML::const_iterator it =
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].begin();
      it != yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].end();
      ++it)
    {
      if (it->as<std::string>() == current_bag_name) {
        episode_in_list = true;
        break;
      }
    }
    if (!episode_in_list) {
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].push_back(
          current_bag_name);
    }

    if (!yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"].IsDefined()) {
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"] =
        YAML::Node(YAML::NodeType::Map);
    }

    YAML::Node episode_node = YAML::Node(YAML::NodeType::Map);
    // Relative to recording_dir_ so metadata survives moving the rosbags
    // folder; consumers join it against the yaml location.
    std::string stored_bag_path = current_bag_path;
    const std::string recording_prefix = recording_dir_ + "/";
    if (stored_bag_path.rfind(recording_prefix, 0) == 0) {
      stored_bag_path = stored_bag_path.substr(recording_prefix.size());
    }
    episode_node["bag_path"] = stored_bag_path;

    if (!current_episode_subtasks.empty()) {
      YAML::Node subtasks_list = YAML::Node(YAML::NodeType::Sequence);
      YAML::Node subtasks_map = YAML::Node(YAML::NodeType::Map);

      for (size_t i = 0; i < current_episode_subtasks.size(); ++i) {
        std::string subtask_key = current_episode_subtasks[i].key;
        subtasks_list.push_back(subtask_key);

        YAML::Node single_subtask;
        single_subtask["label"] = current_episode_subtasks[i].label;
        single_subtask["start_timestamp"] = current_episode_subtasks[i].start_timestamp;
        single_subtask["end_timestamp"] = current_episode_subtasks[i].end_timestamp;

        subtasks_map[subtask_key] = single_subtask;
      }

      episode_node["subtasks_list"] = subtasks_list;
      episode_node["subtasks"] = subtasks_map;
    }

    yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"][current_bag_name] =
      episode_node;
  } else {
    RCLCPP_ERROR(node_->get_logger(),
      "Task '%s' not found in YAML metadata. Episode '%s' will not be saved to metadata. "
      "Was updateRosbagYaml() called after setting the task?",
      current_task_label.c_str(), current_bag_name.c_str());
    return;
  }

  try {
    std::ofstream yaml_file(yaml_file_path);
    YAML::Emitter emitter;
    emitter << YAML::Block << yaml_node;
    yaml_file << emitter.c_str();
    yaml_file.close();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to update episode YAML file: %s", e.what());
  }
}

void BagMetadataManager::removeEpisodeFromYaml(
  const std::string & current_task_dir_name,
  const std::string & current_bag_name)
{
  RCLCPP_INFO(node_->get_logger(), "Removing episode from rosbag YAML file: %s",
      current_bag_name.c_str());

  std::string yaml_file_path = recording_dir_ + "/recorded_bags_meta.yaml";
  if (!std::filesystem::exists(yaml_file_path)) {
    return;
  }

  YAML::Node yaml_node;
  try {
    yaml_node = YAML::LoadFile(yaml_file_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file for removal: %s", e.what());
    return;
  }

  std::string current_task_label = current_task_dir_name;

  if (yaml_node["recorded_bags"]["tasks"][current_task_label].IsDefined()) {
    if (yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"].IsDefined()) {
      YAML::Node old_list =
        yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"];
      YAML::Node new_list = YAML::Node(YAML::NodeType::Sequence);

      for (YAML::const_iterator it = old_list.begin(); it != old_list.end(); ++it) {
        if (it->as<std::string>() != current_bag_name) {
          new_list.push_back(it->as<std::string>());
        }
      }
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes_list"] = new_list;
    }

    if (yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"].IsDefined()) {
      yaml_node["recorded_bags"]["tasks"][current_task_label]["episodes"].remove(current_bag_name);
    }

    try {
      std::ofstream yaml_file(yaml_file_path);
      YAML::Emitter emitter;
      emitter << YAML::Block << yaml_node;
      yaml_file << emitter.c_str();
      yaml_file.close();
      RCLCPP_INFO(node_->get_logger(), "Successfully removed episode from YAML.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to save YAML file after removal: %s", e.what());
    }
  }
}

}  // namespace sobits_vla
