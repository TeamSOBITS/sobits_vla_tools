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

#include "sobits_vla_rosbag_collection/robot_descriptor_loader.hpp"

#include <cstdlib>
#include <filesystem>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sobits_vla
{

namespace srd = sobits_robot_descriptor;
namespace fs = std::filesystem;

namespace
{

std::vector<std::string> amentPrefixes()
{
  std::vector<std::string> out;
  const char * env = std::getenv("AMENT_PREFIX_PATH");
  if (!env) {return out;}
  std::stringstream ss(env);
  std::string item;
  while (std::getline(ss, item, ':')) {
    if (!item.empty()) {out.push_back(item);}
  }
  return out;
}

std::vector<std::string> descriptorSearchDirs(const std::string & package)
{
  std::vector<std::string> dirs;
  std::error_code ec;
  for (const auto & prefix : amentPrefixes()) {
    const fs::path d = fs::path(prefix) / "share" / package / "config";
    if (fs::is_directory(d, ec)) {dirs.push_back(d.string());}
  }
  // Source-tree fallback when run from the workspace root.
  if (fs::is_directory("src", ec)) {
    for (const auto & entry : fs::directory_iterator("src", ec)) {
      const fs::path d = entry.path() / package / "config";
      if (fs::is_directory(d, ec)) {dirs.push_back(d.string());}
    }
  }
  return dirs;
}

YAML::Node entry(const YAML::Node & section, const std::string & name)
{
  if (section && section.IsMap() && section[name] && section[name].IsMap()) {
    return section[name];
  }
  return YAML::Node(YAML::NodeType::Map);
}

template<typename T>
T get(const YAML::Node & node, const std::string & key, const T & dflt)
{
  return (node[key] && !node[key].IsNull()) ? node[key].as<T>() : dflt;
}

void checkNames(
  const YAML::Node & section, const std::string & kind, const std::set<std::string> & known)
{
  if (!section || !section.IsMap()) {return;}
  for (const auto & kv : section) {
    const auto name = kv.first.as<std::string>();
    if (!known.count(name)) {
      throw std::runtime_error("robot_overrides: unknown " + kind + " '" + name + "'");
    }
  }
}

void addCamera(
  RobotInfo & info, const srd::RobotDescriptor & desc, const std::string & name,
  const srd::StreamSpec & s, const YAML::Node & o, bool is_depth)
{
  // Depth is opt-in, as in the python loader.
  if (!get<bool>(o, "active", !is_depth)) {return;}
  const auto abs = [&desc](const std::string & rel) {return rel.empty() ? rel : desc.topic(rel);};
  const std::string encoding = get<std::string>(o, "encoding", s.encoding);
  info.sensor_names["camera"].push_back(name);
  info.sensor_models["camera"].push_back(encoding.empty() ? "rgb8" : encoding);
  if (!s.raw_topic.empty()) {info.sensor_topics["camera"].push_back(abs(s.raw_topic));}
  info.sensor_info_topics["camera"].push_back(abs(s.info_topic));
  info.sensor_compressed_topics["camera"].push_back(abs(s.compressed_topic));
}

}  // namespace

std::string resolveRobotOverridesPath(const std::string & robot_id)
{
  const std::string file = "robot_overrides_" + robot_id + ".yaml";
  std::error_code ec;
  for (const auto & prefix : amentPrefixes()) {
    const fs::path p = fs::path(prefix) / "share" / "sobits_vla_common" / "config" / file;
    if (fs::is_regular_file(p, ec)) {return p.string();}
  }
  const fs::path fallback =
    fs::path("src") / "sobits_vla_tools" / "sobits_vla_common" / "config" / file;
  return fs::is_regular_file(fallback, ec) ? fallback.string() : "";
}

YAML::Node mergeOverrides(const YAML::Node & base, const YAML::Node & extra)
{
  YAML::Node out = YAML::Clone(base);
  if (!out || !out.IsMap()) {out = YAML::Node(YAML::NodeType::Map);}
  if (!extra || !extra.IsMap()) {return out;}
  for (const auto & kv : extra) {
    const auto key = kv.first.as<std::string>();
    if (kv.second.IsMap() && out[key] && out[key].IsMap()) {
      out[key] = mergeOverrides(out[key], kv.second);
    } else {
      out[key] = YAML::Clone(kv.second);
    }
  }
  return out;
}

YAML::Node loadRobotOverrides(const std::string & robot_id, const YAML::Node & extra)
{
  const std::string path = resolveRobotOverridesPath(robot_id);
  const YAML::Node base = path.empty() ? YAML::Node(YAML::NodeType::Map) : YAML::LoadFile(path);
  return mergeOverrides(base, extra);
}

RobotInfo toRobotInfo(const srd::RobotDescriptor & desc, const YAML::Node & overrides)
{
  std::set<std::string> groups, cameras;
  for (const auto & g : desc.groups) {
    groups.insert(g.name);
  }
  for (const auto & c : desc.cameras) {
    cameras.insert(c.name);
  }
  checkNames(overrides["groups"], "group", groups);
  checkNames(overrides["cameras"], "camera", cameras);

  RobotInfo info;
  info.name = desc.robot_id;
  info.version = desc.version;
  info.morphology = get<std::string>(overrides, "morphology", "mobile_manipulator");
  info.joint_states_topic = desc.topic(desc.joint_states_topic);

  for (const auto & g : desc.groups) {
    if (!get<bool>(entry(overrides["groups"], g.name), "active", true)) {continue;}
    info.parts.push_back(g.name);
    info.is_actionable[g.name] = true;
    info.part_command_topic[g.name] = desc.topic(g.command_topic);
    // Group-interface controllers have no state topic; keep the v1 "/state" guess.
    info.part_state_topic[g.name] = g.state_topic.empty() ?
      desc.topic(g.command_topic) + "/state" : desc.topic(g.state_topic);
    if (!g.command_action.empty()) {info.part_actions[g.name] = {desc.topic(g.command_action)};}
    info.joint_names[g.name] = g.joints;
  }

  const YAML::Node base_ov = entry(overrides, "mobile_base");
  if (desc.mobile_base && get<bool>(base_ov, "active", true)) {
    const std::string base = "mobile_base";
    info.parts.push_back(base);
    info.is_actionable[base] = true;
    info.part_cmd_vel_topic[base] = desc.topic(desc.mobile_base->command_topic);
    info.part_odom_topic[base] = desc.topic(desc.mobile_base->odom_topic);
    info.part_has_cmd_vel_y[base] = desc.mobile_base->has_vel_y;
    info.part_has_cmd_vel_z[base] = desc.mobile_base->has_vel_z;
  }

  if (!desc.cameras.empty()) {
    info.sensor_types.push_back("camera");
    for (const auto & cam : desc.cameras) {
      const YAML::Node o = entry(overrides["cameras"], cam.name);
      if (cam.color) {addCamera(info, desc, cam.name, *cam.color, o, false);}
      if (cam.depth) {
        addCamera(info, desc, cam.name + "_depth", *cam.depth, entry(o, "depth"), true);
      }
    }
  }
  return info;
}

RobotInfo loadRobotInfo(const std::string & robot_id, const YAML::Node & stage_overrides)
{
  const YAML::Node overrides = loadRobotOverrides(robot_id, stage_overrides);
  const std::string package =
    get<std::string>(overrides, "descriptor_package", robot_id + "_description");
  const srd::RobotDescriptor desc =
    srd::load(robot_id, "", {}, descriptorSearchDirs(package));
  return toRobotInfo(desc, overrides);
}

}  // namespace sobits_vla
