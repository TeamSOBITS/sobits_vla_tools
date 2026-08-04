#ifndef SOBITS_VLA_ROSBAG_COLLECTION__ROBOT_DESCRIPTOR_LOADER_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__ROBOT_DESCRIPTOR_LOADER_HPP_

#include <string>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
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
  std::vector<GroupSpecCpp> groups;
  bool has_mobile_base{false};
  MobileBaseSpecCpp mobile_base;
  std::vector<CameraSpecCpp> cameras;
  std::vector<std::string> excluded_joints;
};

RobotDescriptorCpp loadRobotDescriptor(const std::string & robot_id);
RobotInfo toRobotInfo(const RobotDescriptorCpp & desc);

} // namespace sobits_vla

#endif // SOBITS_VLA_ROSBAG_COLLECTION__ROBOT_DESCRIPTOR_LOADER_HPP_
