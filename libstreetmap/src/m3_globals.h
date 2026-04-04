#pragma once

#include <vector>
#include <queue>
#include <limits>
#include "StreetsDatabaseAPI.h"

//Constants 
constexpr StreetSegmentIdx NO_EDGE = -1;
constexpr IntersectionIdx NO_INTERSECTION = -1;
constexpr StreetIdx NO_STREET = -1;

//Data Structures 
struct WaveElem {
  IntersectionIdx nodeID;
  StreetSegmentIdx edgeID;  
  double travelTime;        

  WaveElem(IntersectionIdx node, StreetSegmentIdx edge, double time)
      : nodeID(node), edgeID(edge), travelTime(time) {}
};

struct CompareWaveElem {
  bool operator()(const WaveElem& a, const WaveElem& b) const {
    return a.travelTime > b.travelTime;  
  }
};

struct Node {
  StreetSegmentIdx reachingEdge; 
  double bestTime;               
  bool visited;                  

  Node()
      : reachingEdge(NO_EDGE),
        bestTime(std::numeric_limits<double>::infinity()),
        visited(false) {}
};

//Shared Global Variables 
extern double maxSpeedLimit;
extern std::vector<double> segmentWalkingLengths;
extern std::vector<LatLon> intersectionPositions;
extern std::vector<StreetSegmentInfo> segmentInfos;
extern std::vector<std::vector<StreetSegmentIdx>> intersectionSegments;
extern std::vector<IntersectionIdx> touchedDriveNodes;
extern std::vector<IntersectionIdx> touchedWalkNodes;
extern std::vector<Node> walkNodes;
extern std::vector<Node> nodes;
extern std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> wavefront;
extern std::vector<double> segmentTravelTimes;

//Shared Functions 
void clearWavefront();
void initializePathfinding();
void cleanupPathfinding();