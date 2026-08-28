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

#include <map>
#include <string>
#include <vector>

#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

namespace sobits_vla
{

struct JointSpecCpp
{
  std::string ros_name;
  std::string feature;
};

struct GroupSpecCpp
{
  std::string name;
  std::string command_topic;
  std::string command_action;
  // Empty means "derive from command_topic" (ros2_control naming fallback).
  std::string state_topic;
  double max_joint_delta;
  bool active;
  std::vector<JointSpecCpp> joints;
};

struct MobileBaseSpecCpp
{
  std::string command_topic;
  std::string odom_topic;
  bool has_vel_x;
  bool has_vel_y;
  bool has_vel_z;
  bool has_vel_theta;
  double max_vel_x;
  double max_vel_y;
  double max_vel_z;
  double max_vel_theta;
  std::vector<std::string> features;
};

struct CameraSpecCpp
{
  std::string name;
  std::string compressed_topic;
  std::string raw_topic;
  std::string info_topic;
  std::string encoding;
  bool compressed;
  bool active;
  bool is_depth{false};
};

struct RobotDescriptorCpp
{
  std::string robot_id;
  std::string joint_states_topic;
  // Optional descriptor keys; defaults preserve the pre-existing literals.
  std::string version{"1.0.0"};
  std::string morphology{"mobile_manipulator"};
  std::string mobile_base_name{"mobile_base"};
  std::vector<GroupSpecCpp> groups;
  bool has_mobile_base{false};
  MobileBaseSpecCpp mobile_base;
  std::vector<CameraSpecCpp> cameras;
  std::vector<std::string> excluded_joints;
};

RobotDescriptorCpp loadRobotDescriptor(const std::string & robot_id);
RobotInfo toRobotInfo(const RobotDescriptorCpp & desc);

}  // namespace sobits_vla

#endif  // SOBITS_VLA_ROSBAG_COLLECTION__ROBOT_DESCRIPTOR_LOADER_HPP_
