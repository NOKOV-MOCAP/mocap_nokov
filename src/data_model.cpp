#include "mocap_nokov/data_model.h"

namespace mocap_nokov
{

RigidBody::RigidBody() : 
  isTrackingValid(false)
{
}

bool RigidBody::hasValidData() const
{
    return isTrackingValid;
}


void ModelDescription::clear()
{
  markerNames.clear();
}

void MarkerSet::clear()
{
  markers.clear();
}


ModelFrame::ModelFrame() : 
    latency(0.0)
{
}

void ModelFrame::clear()
{
  markerSets.clear();
  otherMarkers.clear();
  rigidBodies.clear();
}

DataModel::DataModel():frameNumber(0)
{

}

void DataModel::clear()
{
  dataFrame.clear();
}

}
