#ifndef __MOCAP_NOKOV_MOCAP_CONFIG_H__
#define __MOCAP_NOKOV_MOCAP_CONFIG_H__

#include <vector>
#include <string>

#include <rclcpp/node.hpp>

namespace mocap_nokov
{

/// \brief Server communication info
struct ServerDescription
{
  struct Default
  {
    static const std::string IpAddress;
  };

  ServerDescription();
  std::string IpAddress;
  std::vector<int64_t> version;
};

/// \brief ROS publisher configuration
struct PublisherConfiguration
{
  int rigidBodyId;
  std::string poseTopicName;
  std::string pose2dTopicName;
  std::string childFrameId;
  std::string parentFrameId;

  bool publishPose;
  bool publishPose2d;
  bool publishTf;
};

typedef std::vector<PublisherConfiguration> PublisherConfigurations;

/// \brief Handles loading node configuration from different sources
struct NodeConfiguration
{
  static void fromRosParam(rclcpp::Node::SharedPtr& nh, 
    ServerDescription& serverDescription, 
    PublisherConfigurations& pubConfigs);
};

} // namespace

#endif  // __MOCAP_NOKOV_MOCAP_CONFIG_H__
