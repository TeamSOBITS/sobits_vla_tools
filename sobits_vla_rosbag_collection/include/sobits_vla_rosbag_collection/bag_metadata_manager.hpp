#ifndef SOBITS_VLA_ROSBAG_COLLECTION__BAG_METADATA_MANAGER_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__BAG_METADATA_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <map>
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

} // namespace sobits_vla

#endif // SOBITS_VLA_ROSBAG_COLLECTION__BAG_METADATA_MANAGER_HPP_
