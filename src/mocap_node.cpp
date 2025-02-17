// STL includes
#include <chrono>
#include <thread>

// Local includes
#include <mocap_nokov/mocap_config.h>
#include <mocap_nokov/data_model.h>
#include <mocap_nokov/rigid_body_publisher.h>

// STL includes
#include <mutex>   

// SDK includes
#include <NokovSDKClient.h>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

namespace mocap_nokov
{
  std::mutex mtx;
  DataModel frameObjData;

  void DataHandler(sFrameOfMocapData* pFrameOfData, void* pUserData)
  {
      if (nullptr == pFrameOfData)
          return;

      // Store the frame
      std::lock_guard<std::mutex> lck (mtx);

      int nmaker = (pFrameOfData->nOtherMarkers < MAX_MARKERS)?pFrameOfData->nOtherMarkers:MAX_MARKERS;

      frameObjData.frameNumber = pFrameOfData->iFrame;
      frameObjData.dataFrame.markerSets.resize(pFrameOfData->nMarkerSets);
      frameObjData.dataFrame.rigidBodies.resize(pFrameOfData->nRigidBodies);
      frameObjData.dataFrame.otherMarkers.resize(pFrameOfData->nOtherMarkers);
      frameObjData.dataFrame.latency = pFrameOfData->fLatency;
      
      for(int i = 0; i< nmaker; ++i)
      {   
        frameObjData.dataFrame.otherMarkers.push_back(
          Marker{pFrameOfData->OtherMarkers[i][0] * 0.001f,
                 pFrameOfData->OtherMarkers[i][1] * 0.001f,
                 pFrameOfData->OtherMarkers[i][2] * 0.001f}
        );        
      }

      for(int i = 0; i< pFrameOfData->nRigidBodies; ++i)
      {
          RigidBody body;
          body.bodyId = pFrameOfData->RigidBodies[i].ID;
          body.iFrame = pFrameOfData->iFrame;
          body.isTrackingValid = true;
          body.pose.position = {pFrameOfData->RigidBodies[i].x * 0.001f, 
                                pFrameOfData->RigidBodies[i].y * 0.001f,
                                pFrameOfData->RigidBodies[i].z * 0.001f};

          body.pose.orientation = {pFrameOfData->RigidBodies[i].qx, 
                                  pFrameOfData->RigidBodies[i].qy,
                                  pFrameOfData->RigidBodies[i].qz,
                                  pFrameOfData->RigidBodies[i].qw};

          frameObjData.dataFrame.rigidBodies.push_back(body);
      }
  }

  const DataModel& GetCurrentFrame()
  {
    static DataModel frame;
    {
        std::lock_guard<std::mutex> lck (mtx);
        frame = frameObjData;
    }
    return frame;
  }

  class NokovRosBridge
  {
  public:
    NokovRosBridge(
      rclcpp::Node::SharedPtr &node,
      ServerDescription const& serverDescr, 
      PublisherConfigurations const& pubConfigs) :
        node(node),
        clock(node->get_clock()),
        serverDescription(serverDescr),
        publisherConfigurations(pubConfigs)
    {

    }

    void initialize()
    {
      // Create client
      sdkClientPtr.reset(new NokovSDKClient());
      sdkClientPtr->SetDataCallback(DataHandler);

      unsigned char sdkVersion[4] = {0};
      sdkClientPtr->NokovSDKVersion(sdkVersion);    
      Version ver((int)sdkVersion[0], (int)sdkVersion[1], (int)sdkVersion[2], (int)sdkVersion[3]);
      RCLCPP_INFO(node->get_logger(), "Load SDK Ver:%s", (char*)ver.getVersionString().c_str());

      while(rclcpp::ok() && sdkClientPtr->Initialize((char*)serverDescription.IpAddress.c_str()))
      {
          RCLCPP_WARN(node->get_logger(), "Connecting to server again");
          std::this_thread::sleep_for(std::chrono::seconds(2));
      }

      // Once we have the server info, create publishers
      publishDispatcherPtr.reset(
        new RigidBodyPublishDispatcher(node, 
          ver, publisherConfigurations));
        
      RCLCPP_INFO(node->get_logger(), "Initialization complete");
    };

    void run()
    {
      while (rclcpp::ok())
      {
        static int preIFrame = 0;

        const auto frame = GetCurrentFrame();

        if (frame.frameNumber != preIFrame)
        {
          preIFrame = frame.frameNumber ;

          const rclcpp::Time time = clock->now();

          publishDispatcherPtr->publish(time, frame.dataFrame.rigidBodies, node->get_logger());
        }

        // If we processed some data, take a short break
        //usleep( 10 );
      }
    }

  private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Clock::SharedPtr clock;
    ServerDescription serverDescription;
    PublisherConfigurations publisherConfigurations;
    std::unique_ptr<RigidBodyPublishDispatcher> publishDispatcherPtr;
    std::unique_ptr<NokovSDKClient> sdkClientPtr;
  };

} // namespace


////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr rclcpp_node = std::make_shared<rclcpp::Node>("mocap_node", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
  
  // Grab node configuration from rosparam
  mocap_nokov::ServerDescription serverDescription;
  mocap_nokov::PublisherConfigurations publisherConfigurations;
  mocap_nokov::NodeConfiguration::fromRosParam(rclcpp_node, serverDescription, publisherConfigurations);

  // Create node object, initialize and run
  mocap_nokov::NokovRosBridge node(rclcpp_node, serverDescription, publisherConfigurations);
  node.initialize();
  node.run();

  return 0;
}
