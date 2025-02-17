#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mocap_nokov/rigid_body_publisher.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

namespace mocap_nokov
{

namespace utilities
{
  geometry_msgs::msg::PoseStamped getRosPose(RigidBody const& body, bool newCoordinates)
  {
    geometry_msgs::msg::PoseStamped poseStampedMsg;
    if (newCoordinates)
    {
      poseStampedMsg.pose.position.x = body.pose.position.x;
      poseStampedMsg.pose.position.y = body.pose.position.y;
      poseStampedMsg.pose.position.z = body.pose.position.z;
  
      poseStampedMsg.pose.orientation.x = body.pose.orientation.x;
      poseStampedMsg.pose.orientation.y = body.pose.orientation.y;
      poseStampedMsg.pose.orientation.z = body.pose.orientation.z;
      poseStampedMsg.pose.orientation.w = body.pose.orientation.w;
    }
    else
    {
      // y & z axes are swapped in the Nokov coordinate system
      poseStampedMsg.pose.position.x = body.pose.position.x;
      poseStampedMsg.pose.position.y = -body.pose.position.z;
      poseStampedMsg.pose.position.z = body.pose.position.y;
  
      poseStampedMsg.pose.orientation.x = body.pose.orientation.x;
      poseStampedMsg.pose.orientation.y = -body.pose.orientation.z;
      poseStampedMsg.pose.orientation.z = body.pose.orientation.y;
      poseStampedMsg.pose.orientation.w = body.pose.orientation.w;
    }
    return poseStampedMsg;
  }   
}

RigidBodyPublisher::RigidBodyPublisher(rclcpp::Node::SharedPtr &node, 
  Version const& sdkVersion,
  PublisherConfiguration const& config) :
    config(config), tfPublisher(node)
{
  if (config.publishPose)
    posePublisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(config.poseTopicName, 1000);

  if (config.publishPose2d)
    pose2dPublisher = node->create_publisher<geometry_msgs::msg::Pose2D>(config.pose2dTopicName, 1000);

  useNewCoordinates = (sdkVersion >= Version("3.0"));
}

RigidBodyPublisher::~RigidBodyPublisher()
{

}

void RigidBodyPublisher::publish(rclcpp::Time const& time, RigidBody const& body, rclcpp::Logger logger)
{
  // don't do anything if no new data was provided
  if (!body.hasValidData())
  {
    return;
  }

  // NaN?
  if (body.pose.position.x != body.pose.position.x)
  {
    return;
  }

  geometry_msgs::msg::PoseStamped pose = utilities::getRosPose(body, useNewCoordinates);

  double curTimeDifference = time.seconds() - body.trackTimestamp;

  // If timeDifference is 0 it has not yet been set
  if (timeDifference == 0){
    RCLCPP_INFO(logger, "Initial clock sync: %.0f seconds", curTimeDifference);
    timeDifference = curTimeDifference;
  }

  // Clock sync can be improved if the current timeDifference is the lowest seen
  if (curTimeDifference < timeDifference){
    RCLCPP_INFO(logger, "Improving clock sync by %.5f seconds", timeDifference - curTimeDifference);
    timeDifference = curTimeDifference;
  }

  // Calculate correct timestamp using time difference
  double corStamp = body.trackTimestamp + timeDifference;

  pose.header.stamp = rclcpp::Time((int)corStamp, (corStamp-floor(corStamp)) * 1000000000 );

  if (config.publishPose)
  {
    //pose.header.frame_id = config.parentFrameId;
    pose.header.frame_id = std::to_string(body.iFrame);
    posePublisher->publish(pose);
  }

  tf2::Quaternion q(pose.pose.orientation.x,
                   pose.pose.orientation.y,
                   pose.pose.orientation.z,
                   pose.pose.orientation.w);


  // publish 2D pose
  if (config.publishPose2d)
  {
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = pose.pose.position.x;
    pose2d.y = pose.pose.position.y;
    pose2d.theta = tf2::getYaw(q);
    pose2dPublisher->publish(pose2d);
  }

  if (config.publishTf)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = config.parentFrameId;
    transformStamped.header.stamp = time;
    transformStamped.child_frame_id = config.childFrameId;
    transformStamped.transform.rotation = pose.pose.orientation;
    transformStamped.transform.translation.x = pose.pose.position.x;
    transformStamped.transform.translation.y = pose.pose.position.y;
    transformStamped.transform.translation.z = pose.pose.position.z;

    tfPublisher.sendTransform(transformStamped);
  }
}


RigidBodyPublishDispatcher::RigidBodyPublishDispatcher(
  rclcpp::Node::SharedPtr &node, 
  Version const& sdkVersion,
  PublisherConfigurations const& configs)
{
  for (auto const& config : configs)
  {
    rigidBodyPublisherMap[config.rigidBodyId] = 
      RigidBodyPublisherPtr(new RigidBodyPublisher(node, sdkVersion, config));
  }
}

void RigidBodyPublishDispatcher::publish(
  rclcpp::Time const& time, 
  std::vector<RigidBody> const& rigidBodies,
  rclcpp::Logger logger
  )
{
  for (auto const& rigidBody : rigidBodies)
  {
    auto const& iter = rigidBodyPublisherMap.find(rigidBody.bodyId);

    if (iter != rigidBodyPublisherMap.end())
    {
      (*iter->second).publish(time, rigidBody, logger);
    }
  }
}


} // namespace
