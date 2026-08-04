#include "sobits_vla_rosbag_collection/robot_descriptor_loader.hpp"
#include <cstdlib>
#include <sstream>
#include <filesystem>
#include <iostream>

namespace sobits_vla
{

RobotDescriptorCpp loadRobotDescriptor(const std::string & robot_id)
{
  std::string file_path = "";
  const char * ament_prefix = std::getenv("AMENT_PREFIX_PATH");
  if (ament_prefix) {
    std::stringstream ss(ament_prefix);
    std::string path;
    while (std::getline(ss, path, ':')) {
      std::filesystem::path candidate = std::filesystem::path(path) / "share" /
        "sobits_vla_common" / "robots" / (robot_id + ".robot.yaml");
      if (std::filesystem::exists(candidate)) {
        file_path = candidate.string();
        break;
      }
    }
  }

  if (file_path.empty()) {
    std::filesystem::path fallback = std::filesystem::path("src") / "sobits_vla_tools" /
      "sobits_vla_common" / "robots" / (robot_id + ".robot.yaml");
    if (std::filesystem::exists(fallback)) {
      file_path = fallback.string();
    }
  }

  if (file_path.empty()) {
    throw std::runtime_error("Could not find robot descriptor for " + robot_id);
  }

  YAML::Node config = YAML::LoadFile(file_path);

  RobotDescriptorCpp desc;
  desc.robot_id = config["robot_id"].as<std::string>();
  desc.joint_states_topic = config["joint_states_topic"].as<std::string>();

  // Parse groups
  if (config["groups"]) {
    for (const auto & g_node : config["groups"]) {
      GroupSpecCpp g;
      g.name = g_node["name"].as<std::string>();
      g.command_topic = g_node["command_topic"].as<std::string>();
      g.command_action = g_node["command_action"] ? g_node["command_action"].as<std::string>() : "";
      g.max_joint_delta = g_node["max_joint_delta"] ? g_node["max_joint_delta"].as<double>() : 0.0;
      g.active = g_node["active"] ? g_node["active"].as<bool>() : true;

      if (g_node["joints"]) {
        for (const auto & j_node : g_node["joints"]) {
          JointSpecCpp j;
          j.ros_name = j_node["ros_name"].as<std::string>();
          j.feature = j_node["feature"].as<std::string>();
          g.joints.push_back(j);
        }
      }
      desc.groups.push_back(g);
    }
  }

  // Parse mobile_base
  if (config["mobile_base"]) {
    desc.has_mobile_base = true;
    auto mb_node = config["mobile_base"];
    desc.mobile_base.command_topic = mb_node["command_topic"].as<std::string>();
    desc.mobile_base.odom_topic = mb_node["odom_topic"].as<std::string>();
    desc.mobile_base.has_vel_x = mb_node["has_vel_x"] ? mb_node["has_vel_x"].as<bool>() : false;
    desc.mobile_base.has_vel_y = mb_node["has_vel_y"] ? mb_node["has_vel_y"].as<bool>() : false;
    desc.mobile_base.has_vel_z = mb_node["has_vel_z"] ? mb_node["has_vel_z"].as<bool>() : false;
    desc.mobile_base.has_vel_theta =
      mb_node["has_vel_theta"] ? mb_node["has_vel_theta"].as<bool>() : false;
    desc.mobile_base.max_vel_x = mb_node["max_vel_x"] ? mb_node["max_vel_x"].as<double>() : 0.0;
    desc.mobile_base.max_vel_y = mb_node["max_vel_y"] ? mb_node["max_vel_y"].as<double>() : 0.0;
    desc.mobile_base.max_vel_z = mb_node["max_vel_z"] ? mb_node["max_vel_z"].as<double>() : 0.0;
    desc.mobile_base.max_vel_theta =
      mb_node["max_vel_theta"] ? mb_node["max_vel_theta"].as<double>() : 0.0;

    if (mb_node["features"]) {
      for (const auto & f : mb_node["features"]) {
        desc.mobile_base.features.push_back(f.as<std::string>());
      }
    }
  }

  // Parse sensors
  if (config["sensors"] && config["sensors"]["cameras"]) {
    for (const auto & c_node : config["sensors"]["cameras"]) {
      CameraSpecCpp c;
      c.name = c_node["name"].as<std::string>();
      c.compressed_topic =
        c_node["compressed_topic"] ? c_node["compressed_topic"].as<std::string>() : "";
      c.raw_topic = c_node["raw_topic"] ? c_node["raw_topic"].as<std::string>() : "";
      c.info_topic = c_node["info_topic"] ? c_node["info_topic"].as<std::string>() : "";
      c.encoding = c_node["encoding"] ? c_node["encoding"].as<std::string>() : "";
      c.compressed = c_node["compressed"] ? c_node["compressed"].as<bool>() : false;
      c.active = c_node["active"] ? c_node["active"].as<bool>() : true;
      desc.cameras.push_back(c);
    }
  }

  // Parse excluded_joints
  if (config["excluded_joints"]) {
    for (const auto & ej : config["excluded_joints"]) {
      desc.excluded_joints.push_back(ej.as<std::string>());
    }
  }

  return desc;
}

RobotInfo toRobotInfo(const RobotDescriptorCpp & desc)
{
  RobotInfo info;
  info.name = desc.robot_id;
  info.version = "1.0.0";
  info.morphology = "mobile_manipulator";
  info.joint_states_topic = desc.joint_states_topic;

  for (const auto & group : desc.groups) {
    if (group.active) {
      info.parts.push_back(group.name);
      info.is_actionable[group.name] = true;
      info.part_command_topic[group.name] = group.command_topic;

      // derive state topic name (replace joint_trajectory with controller_state)
      std::string state_topic = group.command_topic;
      size_t pos = state_topic.find("joint_trajectory");
      if (pos != std::string::npos) {
        state_topic.replace(pos, std::string("joint_trajectory").length(), "controller_state");
      } else {
        state_topic += "/state";
      }
      info.part_state_topic[group.name] = state_topic;

      if (!group.command_action.empty()) {
        info.part_actions[group.name] = {group.command_action};
      }

      for (const auto & j : group.joints) {
        info.joint_names[group.name].push_back(j.ros_name);
      }
    }
  }

  if (desc.has_mobile_base) {
    info.parts.push_back("mobile_base");
    info.is_actionable["mobile_base"] = true;
    info.part_cmd_vel_topic["mobile_base"] = desc.mobile_base.command_topic;
    info.part_odom_topic["mobile_base"] = desc.mobile_base.odom_topic;
    info.part_has_cmd_vel_y["mobile_base"] = desc.mobile_base.has_vel_y;
    info.part_has_cmd_vel_z["mobile_base"] = desc.mobile_base.has_vel_z;
  }

  if (!desc.cameras.empty()) {
    info.sensor_types.push_back("camera");
    for (const auto & cam : desc.cameras) {
      if (cam.active) {
        info.sensor_names["camera"].push_back(cam.name);
        info.sensor_models["camera"].push_back(cam.encoding.empty() ? "rgb8" : cam.encoding);
        if (!cam.raw_topic.empty()) {
          info.sensor_topics["camera"].push_back(cam.raw_topic);
        }
        info.sensor_info_topics["camera"].push_back(cam.info_topic);
        info.sensor_compressed_topics["camera"].push_back(cam.compressed_topic);
      }
    }
  }

  return info;
}

} // namespace sobits_vla
