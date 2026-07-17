#ifndef SOBITS_VLA_ROSBAG_COLLECTION__TOPIC_BUILDER_HPP_
#define SOBITS_VLA_ROSBAG_COLLECTION__TOPIC_BUILDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include "sobits_vla_rosbag_collection/rosbag_collection.hpp"

namespace sobits_vla
{

class TopicBuilder
{
public:
  static std::vector<std::string> buildTopicList(
    const RobotInfo & robot_info,
    const RosbagInfo & rosbag_info);

  static bool validateTopics(
    rclcpp::Node * node,
    const std::vector<std::string> & topics_to_record,
    const RobotInfo & robot_info);
};

} // namespace sobits_vla

#endif // SOBITS_VLA_ROSBAG_COLLECTION__TOPIC_BUILDER_HPP_
