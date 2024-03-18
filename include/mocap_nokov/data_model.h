#ifndef __MOCAP_NOKOV_DATA_MODEL_H__
#define __MOCAP_NOKOV_DATA_MODEL_H__

#include <string>
#include <vector>

#include <mocap_nokov/version.h>

namespace mocap_nokov
{

/// \brief Data object holding the position of a single mocap marker in 3d space
struct Marker
{
  float x;
  float y;
  float z;
};

struct __attribute__ ((__packed__)) Pose
{
  struct __attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
  } position;
  struct __attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
    float w;
  } orientation;
};

/// \brief Data object holding information about a single rigid body within a mocap skeleton
struct RigidBody
{
    RigidBody();
    int iFrame;
    int bodyId;
    Pose pose;
    float meanMarkerError;
    bool isTrackingValid;
    double trackTimestamp;

    bool hasValidData() const;
};

/// \brief Data object describing a single tracked model
struct ModelDescription
{
    ModelDescription();
    void clear();

    std::string name;
    std::vector<std::string> markerNames;
};

struct MarkerSet
{
    void clear();

    char name[256];
    std::vector<Marker> markers;
};

/// \brief Data object holding poses of a tracked model's components
struct ModelFrame
{
    ModelFrame();
    void clear();

    std::vector<MarkerSet> markerSets;
    std::vector<Marker> otherMarkers;
    std::vector<RigidBody> rigidBodies;
    float latency;
};

/// \brief The data model for this node
class DataModel
{
public:
    DataModel();

    int frameNumber;
    ModelFrame dataFrame;

    void clear();
};

}

#endif
