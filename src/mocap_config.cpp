#include <string>

#include "mocap_nokov/mocap_config.h"

namespace mocap_nokov
{
// Server description defaults
const std::string ServerDescription::Default::IpAddress = "10.1.1.198";

// Param keys
namespace rosparam
{
  namespace keys
  {
    const std::string ServerIpAddrss = "nokov_config.server_address";
    const std::string Version = "nokov_config.version";
    const std::string RigidBodies = "rigid_bodies";
    const std::string PoseTopicName = "pose";
    const std::string Pose2dTopicName = "pose2d";
    const std::string ChildFrameId = "child_frame_id";
    const std::string ParentFrameId = "parent_frame_id";
  }
}

ServerDescription::ServerDescription() :
  IpAddress(ServerDescription::Default::IpAddress)
{}

void NodeConfiguration::fromRosParam(
  rclcpp::Node::SharedPtr& node, 
  ServerDescription& serverDescription, 
  PublisherConfigurations& pubConfigs)
{
  // Get server cconfiguration from ROS parameter server
  
  if(!node->get_parameter_or(
    rosparam::keys::ServerIpAddrss, 
    serverDescription.IpAddress, 
    ServerDescription::Default::IpAddress)
  ) {
    RCLCPP_WARN(node->get_logger(), 
      "Could not get server address, using default: %s", serverDescription.IpAddress.c_str());
  }

  RCLCPP_INFO(node->get_logger(), "Use server address: %s", serverDescription.IpAddress.c_str());

  if(!node->get_parameter(
    rosparam::keys::Version, 
    serverDescription.version)
  ) {
    RCLCPP_WARN(node->get_logger(), 
      "Could not get server version, using auto");
  }

  const std::vector<std::string> prefix{rosparam::keys::RigidBodies};
  const std::string name_with_dot = std::string(node->get_name()) + ".";
  const auto result = node->get_node_parameters_interface()->list_parameters(
    prefix, 0
  );

  for(const auto &prefix : result.prefixes)
  {
    PublisherConfiguration publisherConfig;

    publisherConfig.rigidBodyId = std::atoi(prefix.substr(rosparam::keys::RigidBodies.length() + 1, 1).c_str());

    const bool readPoseTopicName = node->get_parameter(
      prefix + "." + rosparam::keys::PoseTopicName, publisherConfig.poseTopicName);

    if (!readPoseTopicName)
    {
      RCLCPP_WARN(node->get_logger(), 
        "Failed to parse %s for body %d. Pose publishing disabled.", 
        rosparam::keys::PoseTopicName.c_str(), 
        publisherConfig.rigidBodyId);

      publisherConfig.publishPose = false;
    }
    else
    {
      publisherConfig.publishPose = true;
    }

    const bool readPose2dTopicName = node->get_parameter(
      prefix + "." + rosparam::keys::Pose2dTopicName, publisherConfig.pose2dTopicName);

    if (!readPose2dTopicName)
    {
      RCLCPP_WARN(node->get_logger(),
        "Failed to parse %s for body %d. Pose publishing disabled.",
          rosparam::keys::Pose2dTopicName.c_str(),
          publisherConfig.rigidBodyId);

      publisherConfig.publishPose2d = false;
    }
    else
    {
      publisherConfig.publishPose2d = true;
    }
  
    const bool readChildFrameId = node->get_parameter(
      prefix + "." + rosparam::keys::ChildFrameId, publisherConfig.childFrameId);
      
    const bool readParentFrameId = node->get_parameter(
      prefix + "." + rosparam::keys::ParentFrameId, publisherConfig.parentFrameId);

    if (!readChildFrameId || !readParentFrameId)
    {
      if (!readChildFrameId)
        RCLCPP_WARN(node->get_logger(),
          "Failed to parse %s for body %d. TF publishing disabled.",
            rosparam::keys::ChildFrameId.c_str(),
            publisherConfig.rigidBodyId);

      if (!readParentFrameId)
        RCLCPP_WARN(node->get_logger(),
          "Failed to parse %s for body %d. TF publishing disabled.",
          rosparam::keys::ParentFrameId.c_str(),
          publisherConfig.rigidBodyId);

      publisherConfig.publishTf = false;
    }
    else
    {
      publisherConfig.publishTf = true;
    }

    pubConfigs.push_back(publisherConfig);
  }
}

}