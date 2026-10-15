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

#ifndef SOBITS_VLA_ROSBAG_COLLECTION__ROBOT_DESCRIPTOR_LOADER_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__ROBOT_DESCRIPTOR_LOADER_HPP_

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

#include <sobits_robot_descriptor/loader.hpp>

#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

namespace sobits_vla
{

// Same layering as sobits_vla_common.robot_descriptor: shared descriptor
// (sobits_robot_descriptor) + config/robot_overrides_<robot_id>.yaml.

// Installed share/sobits_vla_common/config file, else the source tree; "" if absent.
std::string resolveRobotOverridesPath(const std::string & robot_id);

// Deep merge by key: mappings merge, anything else in `extra` replaces.
YAML::Node mergeOverrides(const YAML::Node & base, const YAML::Node & extra);

YAML::Node loadRobotOverrides(const std::string & robot_id, const YAML::Node & extra = {});

RobotInfo toRobotInfo(
  const sobits_robot_descriptor::RobotDescriptor & desc, const YAML::Node & overrides);

RobotInfo loadRobotInfo(const std::string & robot_id, const YAML::Node & stage_overrides = {});

}  // namespace sobits_vla

#endif  // SOBITS_VLA_ROSBAG_COLLECTION__ROBOT_DESCRIPTOR_LOADER_HPP_
