/*
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * Copyright (c) 2012, Clearpath Robotics, Inc., Alex Bencz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mocap_nokov/mocap_config.h"
#include <string>
#include <XmlRpcValue.h>

namespace mocap_nokov
{
namespace impl
{

  template<typename T>
  bool check_and_get_param(
    XmlRpc::XmlRpcValue& config_node,
    std::string const& key,
    T& value)
  {
    return false;
  }

  template<>
  bool check_and_get_param<std::string>(
    XmlRpc::XmlRpcValue& config_node,
    std::string const& key,
    std::string& value)
  {
    if (config_node[key].getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      value = (std::string&)config_node[key];
      return true;
    }

    return false;
  }
}
// Server description defaults
const std::string ServerDescription::Default::IpAddress = "10.1.1.198";

// Param keys
namespace rosparam
{
  namespace keys
  {
    const std::string ServerIpAddrss = "nokov_config/server_address";
    const std::string Version = "nokov_config/version";
    const std::string RigidBodies = "rigid_bodies";
    const std::string PoseTopicName = "pose";
    const std::string Pose2dTopicName = "pose2d";
    const std::string OdomTopicName = "odom";
    const std::string EnableTfPublisher = "tf";
    const std::string ChildFrameId = "child_frame_id";
    const std::string ParentFrameId = "parent_frame_id";
  }
}

ServerDescription::ServerDescription() :
  IpAddress(ServerDescription::Default::IpAddress)
{}

void NodeConfiguration::fromRosParam(
  ros::NodeHandle& nh, 
  ServerDescription& serverDescription, 
  PublisherConfigurations& pubConfigs)
{
  // Get server cconfiguration from ROS parameter server
  if (nh.hasParam(rosparam::keys::ServerIpAddrss))
  {
    nh.getParam(rosparam::keys::ServerIpAddrss, serverDescription.IpAddress);
  }
  else
  {
    ROS_WARN_STREAM("Could not get server address, using default: " <<
                    serverDescription.IpAddress);
  }

  // if (nh.hasParam(rosparam::keys::Version))
  // {
  //   nh.getParam(rosparam::keys::Version, serverDescription.version);
  // }
  // else
  // {
  //   ROS_WARN_STREAM("Could not get server version, using auto");
  // }

if (nh.hasParam(rosparam::keys::RigidBodies))
  {
    XmlRpc::XmlRpcValue bodyList;
    nh.getParam(rosparam::keys::RigidBodies, bodyList);

    if (bodyList.getType() == XmlRpc::XmlRpcValue::TypeStruct && bodyList.size() > 0)
    {
      XmlRpc::XmlRpcValue::iterator iter;
      //  for (iter = bodyList.begin(); iter != bodyList.end(); ++iter) {
      for (auto const& iter : bodyList)
      {
        std::string strBodyId = iter.first;
        XmlRpc::XmlRpcValue bodyParameters = iter.second;

        if (bodyParameters.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          // Load configuration for this rigid body from ROS
          PublisherConfiguration publisherConfig;
          std::sscanf(strBodyId.c_str(), "%d", &publisherConfig.rigidBodyId);

          bool readPoseTopicName = impl::check_and_get_param(bodyParameters,
                                   rosparam::keys::PoseTopicName, publisherConfig.poseTopicName);

          if (!readPoseTopicName)
          {
            ROS_WARN_STREAM("Failed to parse " << rosparam::keys::PoseTopicName <<
                            " for body `" << publisherConfig.rigidBodyId << "`. Pose publishing disabled.");
            publisherConfig.publishPose = false;
          }
          else
          {
            publisherConfig.publishPose = true;
          }

          bool readPose2dTopicName = impl::check_and_get_param(bodyParameters,
                                     rosparam::keys::Pose2dTopicName, publisherConfig.pose2dTopicName);

          if (!readPose2dTopicName)
          {
            ROS_WARN_STREAM("Failed to parse " << rosparam::keys::Pose2dTopicName <<
                            " for body `" << publisherConfig.rigidBodyId << "`. Pose publishing disabled.");
            publisherConfig.publishPose2d = false;
          }
          else
          {
            publisherConfig.publishPose2d = true;
          }

          bool readOdomTopicName = impl::check_and_get_param(bodyParameters,
                                   rosparam::keys::OdomTopicName, publisherConfig.odomTopicName);

          if (!readOdomTopicName)
          {
            ROS_WARN_STREAM("Failed to parse " << rosparam::keys::OdomTopicName <<
                            " for body `" << publisherConfig.rigidBodyId << "`. Odom publishing disabled.");
            publisherConfig.publishOdom = false;
          }
          else
          {
            publisherConfig.publishOdom = true;
          }

          bool readEnableTfPublisher = impl::check_and_get_param(bodyParameters,
                               rosparam::keys::EnableTfPublisher, publisherConfig.enableTfPublisher);
          if (!readEnableTfPublisher)
          {
              ROS_WARN_STREAM("Failed to parse " << rosparam::keys::EnableTfPublisher <<
                              " for body `" << publisherConfig.enableTfPublisher << "`. Tf publishing disabled.");
            publisherConfig.publishTf = false;
          }
          else
          {
            publisherConfig.publishTf = true;
          }

          bool readChildFrameId = impl::check_and_get_param(bodyParameters,
                                  rosparam::keys::ChildFrameId, publisherConfig.childFrameId);

          bool readParentFrameId = impl::check_and_get_param(bodyParameters,
                                   rosparam::keys::ParentFrameId, publisherConfig.parentFrameId);

          if (!readChildFrameId || !readParentFrameId || !publisherConfig.publishTf)
          { if (!readEnableTfPublisher)
                  ROS_WARN_STREAM("tf is not found in the config" << rosparam::keys::EnableTfPublisher <<
                                  " for body `" << publisherConfig.rigidBodyId << "`. TF publishing disabled.");
            if (!readChildFrameId)
              ROS_WARN_STREAM("Failed to parse " << rosparam::keys::ChildFrameId <<
                              " for body `" << publisherConfig.rigidBodyId << "`. TF publishing disabled.");

            if (!readParentFrameId)
              ROS_WARN_STREAM("Failed to parse " << rosparam::keys::ParentFrameId <<
                              " for body `" << publisherConfig.rigidBodyId << "`. TF publishing disabled.");

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
  }
}

}
