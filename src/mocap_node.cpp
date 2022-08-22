/*
 * Copyright (c) 2022, Mocap Nokov Company., Lika.Ji
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

// Local includes
#include <mocap_nokov/mocap_config.h>
#include <mocap_nokov/MocapNokovConfig.h>
#include <mocap_nokov/data_model.h>
#include <mocap_nokov/rigid_body_publisher.h>
// STL includes
#include <mutex>   
// Ros includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
// SDK includes
#include <SeekerSDKClient.h>

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
      ros::NodeHandle &nh,
      ServerDescription const& serverDescr, 
      PublisherConfigurations const& pubConfigs) :
        nh(nh),
        server(ros::NodeHandle("~/nokov_config")),
        serverDescription(serverDescr),
        publisherConfigurations(pubConfigs)
    {
      server.setCallback(boost::bind(&NokovRosBridge::reconfigureCallback, this, _1, _2));
    }

    void reconfigureCallback(MocapNokovConfig& config, uint32_t)
    {
      serverDescription.IpAddress = config.server_address;

      initialize();
    }

    void initialize()
    {
      // Create client
      sdkClientPtr.reset(new SeekerSDKClient());
      sdkClientPtr->SetDataCallback(DataHandler);

      unsigned char sdkVersion[4] = {0};
      sdkClientPtr->SeekerSDKVersion(sdkVersion);    
      Version ver((int)sdkVersion[0], (int)sdkVersion[1], (int)sdkVersion[2], (int)sdkVersion[3]);
      ROS_INFO("Load SDK Ver:%s", (char*)ver.getVersionString().c_str());

      while(ros::ok() && sdkClientPtr->Initialize((char*)serverDescription.IpAddress.c_str()))
      {
          ROS_WARN("Connecting to server again");
          ros::Duration(2.).sleep();
      }

      // Once we have the server info, create publishers
      publishDispatcherPtr.reset(
        new RigidBodyPublishDispatcher(nh, 
          ver, publisherConfigurations));
        
      ROS_INFO("Initialization complete");

      initialized = true;
    };

    void run()
    {
      while (ros::ok())
      {
        if (initialized)
        {
          static int preIFrame = 0;

          const auto frame = GetCurrentFrame();

          if (frame.frameNumber != preIFrame)
          {
            preIFrame = frame.frameNumber ;

            const ros::Time time = ros::Time::now();

            publishDispatcherPtr->publish(time, frame.dataFrame.rigidBodies);
          }

          usleep(100);
        }
        else
        {
          ros::Duration(1.).sleep();
        }
        ros::spinOnce();
      }
    }

  private:
    ros::NodeHandle nh;
    bool initialized;
    ServerDescription serverDescription;
    PublisherConfigurations publisherConfigurations;
    std::unique_ptr<RigidBodyPublishDispatcher> publishDispatcherPtr;
    std::unique_ptr<SeekerSDKClient> sdkClientPtr;
    dynamic_reconfigure::Server<MocapNokovConfig> server;
  };

} // namespace


////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS nh
  ros::init(argc, argv, "mocap_nh");
  ros::NodeHandle nh("~");

  // Grab nh configuration from rosparam
  mocap_nokov::ServerDescription serverDescription;
  mocap_nokov::PublisherConfigurations publisherConfigurations;
  mocap_nokov::NodeConfiguration::fromRosParam(nh, serverDescription, publisherConfigurations);

  // Create nh object, initialize and run
  mocap_nokov::NokovRosBridge nb(nh, serverDescription, publisherConfigurations);
  nb.initialize();
  nb.run();

  return 0;
}
