/*
 * Copyright 2026 University of Toronto
 *
 * ECE297 Milestone 3 - Pathfinding Implementation
 *
 * This file implements pathfinding algorithms including:
 * - Computing path travel times (driving and walking)
 * - Finding shortest paths between intersections
 * - Multi-modal pathfinding (walk + drive)
 */

#include "m3.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "StreetsDatabaseAPI.h"
#include "m1.h"

// Constants
constexpr StreetSegmentIdx NO_EDGE = -1;
constexpr IntersectionIdx NO_INTERSECTION = -1;
constexpr StreetIdx NO_STREET = -1;
static double maxSpeedLimit = 0.0;

// Helper functions (optional - can be kept internal to m3.cpp)
void resetNodes();
void clearWavefront();
void initializePathfinding();
void cleanupPathfinding();
double computeHeuristic(IntersectionIdx current, IntersectionIdx dest);
std::vector<StreetSegmentIdx> tracePath(IntersectionIdx src,
                                        IntersectionIdx dest);

static std::vector<double> segmentWalkingLengths;
static std::vector<LatLon> intersectionPositions;
static std::vector<StreetSegmentInfo> segmentInfos;
static std::vector<std::vector<StreetSegmentIdx>> intersectionSegments;

// Add this global alongside the others
static std::vector<IntersectionIdx> touchedDriveNodes;
static std::vector<IntersectionIdx> touchedWalkNodes;

// Data Structures for Pathfinding
struct WaveElem {
  IntersectionIdx nodeID; //The IntersectionIdx that it is on 
  StreetSegmentIdx edgeID;  //Street segment used to reach this node
  double travelTime;        //Time to reach this node from start

//Constructor 
  WaveElem(IntersectionIdx node, StreetSegmentIdx edge, double time)
      : nodeID(node), edgeID(edge), travelTime(time) {}
};

// Comparator for priority queue (min-heap based on travel time)
//Puts the lowest travel time to the top of wavefront 
struct CompareWaveElem {
  bool operator()(const WaveElem& a, const WaveElem& b) const {
    return a.travelTime > b.travelTime;  // Min-heap
  }
};

// Node information for pathfinding
struct Node {
  double bestTime;                // Best time to reach this node
  StreetSegmentIdx reachingEdge;  // Edge used to reach this node
  bool visited;                   // Whether node has been visited

  Node()
      : bestTime(std::numeric_limits<double>::infinity()),
        reachingEdge(NO_EDGE),
        visited(false) {}
};

// Separate node storage for walk and drive phases
static std::vector<Node> walkNodes;  // add this global alongside `nodes`
// Global data structures for pathfinding (reused across calls)
static std::vector<Node> nodes;
//Represents all the intersections just discovered, but havent explored yet 
static std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem>
    wavefront;

// Precomputed segment travel times (computed once in loadMap)
static std::vector<double> segmentTravelTimes;

// Helper functions

// Initialize pathfinding data structures
void initializePathfinding() {
  //Get how many intersections exist in total,then initialize the vectors to hold that many items
  int numIntersections = getNumIntersections();
  nodes.resize(numIntersections);
  walkNodes.resize(numIntersections);
  touchedDriveNodes.reserve(numIntersections);
  touchedWalkNodes.reserve(numIntersections);
  intersectionSegments.resize(numIntersections);

  //Cache the positions of each intersections 
  intersectionPositions.resize(numIntersections);
  for (int i = 0; i < numIntersections; i++) {
    intersectionPositions[i] = getIntersectionPosition(i);
    intersectionSegments[i] = findStreetSegmentsOfIntersection(i);
  }

//Get # of street segments and initlize vectors to hold that many items 
  int numSegments = getNumStreetSegments();
  segmentTravelTimes.resize(numSegments);
  segmentWalkingLengths.resize(numSegments);
  segmentInfos.resize(numSegments);

  for (int i = 0; i < numSegments; i++) {
    StreetSegmentInfo info = getStreetSegmentInfo(i);//Get each street segments info (speed limit, one way, to/from)
    segmentInfos[i] = info;
    double length = findStreetSegmentLength(i);
    //Store that information 
    segmentTravelTimes[i] = length / info.speedLimit;
    segmentWalkingLengths[i] = length;
    //Hold the maximum speed limit 
    if (info.speedLimit > maxSpeedLimit) maxSpeedLimit = info.speedLimit;
  }
}

// Clear the wavefront
void clearWavefront() {
  // Swap with empty — O(1) instead of O(n) while loop
  std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> empty;
  std::swap(wavefront, empty);
}

// Clean up pathfinding data structures
void cleanupPathfinding() {
  nodes.clear();
  nodes.shrink_to_fit();
  segmentTravelTimes.clear();
  segmentTravelTimes.shrink_to_fit();
  walkNodes.clear();
  walkNodes.shrink_to_fit();
  segmentWalkingLengths.clear();
  segmentWalkingLengths.shrink_to_fit();
  segmentInfos.clear();
  segmentInfos.shrink_to_fit();
  intersectionSegments.clear();
  intersectionSegments.shrink_to_fit();
  intersectionPositions.clear();
  intersectionPositions.shrink_to_fit();
  clearWavefront();
}

// Reset all nodes to initial state
void resetNodes() {
  for (IntersectionIdx idx : touchedDriveNodes) {
    nodes[idx].bestTime = std::numeric_limits<double>::infinity();
    nodes[idx].reachingEdge = NO_EDGE;
    nodes[idx].visited = false;
  }
  touchedDriveNodes.clear();
}

// Heuristic function for A* (straight-line distance / max speed)
double computeHeuristic(IntersectionIdx current, IntersectionIdx dest) {
  LatLon pos1 = intersectionPositions[current];
  LatLon pos2 = intersectionPositions[dest];
  double distance = findDistanceBetweenTwoPoints({pos1, pos2});
  return distance / maxSpeedLimit;
}

// Trace back the path from dest to src using reaching edges
std::vector<StreetSegmentIdx> tracePath(IntersectionIdx src,
                                        IntersectionIdx dest) {
  std::vector<StreetSegmentIdx> path;

  IntersectionIdx current = dest;

  // Trace backwards from destination to source
  while (current != src) {
    StreetSegmentIdx edge = nodes[current].reachingEdge;

    if (edge == NO_EDGE) {
      // No path exists
      return std::vector<StreetSegmentIdx>();
    }

    path.push_back(edge);

    // Move to the previous intersection
    const StreetSegmentInfo& info = segmentInfos[edge];
    if (info.to == current) {
      current = info.from;
    } else {
      current = info.to;
    }
  }

  // Reverse to get path from source to destination
  std::reverse(path.begin(), path.end());
  return path;
}

// Compute travel time for a given path (driving)
//Takes time penalty for making a turn & vectory containing ordered sequence of street segments that make up the route 
double computePathTravelTime(const double turn_penalty,
                             const std::vector<StreetSegmentIdx>& path) {
  if (path.empty()) return 0.0;

  double totalTime = 0.0;
  StreetIdx prevStreet = NO_STREET; //Stores ID of street you were just on 

  for (StreetSegmentIdx segID : path) {
    // Add segment travel time
    totalTime += segmentTravelTimes[segID]; //Uses precomputed segmentTravelTime in initializePathfinding() 

    // Check for turn
    StreetIdx currentStreet = segmentInfos[segID].streetID;

    if (prevStreet != NO_STREET && prevStreet != currentStreet) {
      // Turn detected
      totalTime += turn_penalty;
    }

    prevStreet = currentStreet;
  }

  return totalTime;
}

// Compute walking time for a given path
double computePathWalkingTime(const std::vector<StreetSegmentIdx>& path,
                              const double walking_speed,
                              const double turn_penalty) {
  if (path.empty()) return 0.0;

  double totalTime = 0.0;
  StreetIdx prevStreet = NO_STREET;

  for (StreetSegmentIdx segID : path) {
    // Add segment walking time
    totalTime += segmentWalkingLengths[segID] / walking_speed;

    // Check for turn
    StreetIdx currentStreet = segmentInfos[segID].streetID;

    if (prevStreet != NO_STREET && prevStreet != currentStreet) {
      // Turn detected
      totalTime += turn_penalty;
    }

    prevStreet = currentStreet;
  }

  return totalTime;
}

// Find shortest path between two intersections using A*
std::vector<StreetSegmentIdx> findPathBetweenIntersections(
    const double turn_penalty,
    const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids) {
  IntersectionIdx src = intersect_ids.first;
  IntersectionIdx dest = intersect_ids.second;

//If the same intersection is clicked 
  if (src == dest) return std::vector<StreetSegmentIdx>();

  // Dirty reset — only clear nodes touched last call
  //Only resets the time for the intersections that the last search touched 
  for (IntersectionIdx idx : touchedDriveNodes) {
    nodes[idx].bestTime = std::numeric_limits<double>::infinity();
    nodes[idx].reachingEdge = NO_EDGE;
    nodes[idx].visited = false;
  }
  touchedDriveNodes.clear();
  clearWavefront();

  // Initialize source
  nodes[src].bestTime = 0.0;
  touchedDriveNodes.push_back(src);
  wavefront.push(WaveElem(src, NO_EDGE, 0.0));

  while (!wavefront.empty()) {
    //Take the lowest estimated time from wavefront 
    WaveElem current = wavefront.top();
    wavefront.pop();

    //If the current node is already visited, skip it 
    IntersectionIdx currNode = current.nodeID;
    if (nodes[currNode].visited) continue;
    nodes[currNode].visited = true;

    //If the node just popped is the destination, trace from the src to destination using reachingEdge to get final route 
    if (currNode == dest) {
      // Trace path using nodes[]
      std::vector<StreetSegmentIdx> path;
      IntersectionIdx tracer = dest;
      while (tracer != src) {
        StreetSegmentIdx edge = nodes[tracer].reachingEdge;
        if (edge == NO_EDGE) return std::vector<StreetSegmentIdx>();
        path.push_back(edge);
        const StreetSegmentInfo& info = segmentInfos[edge];
        tracer = (info.to == tracer) ? info.from : info.to;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    //Is there a turn on the next step 
    StreetIdx currentStreet = NO_STREET;
    if (current.edgeID != NO_EDGE)
      currentStreet = segmentInfos[current.edgeID].streetID;

    //Is the street a one way 
    for (StreetSegmentIdx segID : intersectionSegments[currNode]) {
      const StreetSegmentInfo& info = segmentInfos[segID];

      IntersectionIdx neighbor = NO_INTERSECTION;
      if (info.from == currNode)
        neighbor = info.to;
      else if (!info.oneWay)
        neighbor = info.from;
      else
        continue;

      //Gets pre calculated driving time and adds if there is turn 
      double edgeTime = segmentTravelTimes[segID];
      double turnTime =
          (currentStreet != NO_STREET && currentStreet != info.streetID)
              ? turn_penalty
              : 0.0;
      double newTime = nodes[currNode].bestTime + edgeTime + turnTime;

      //Is the new route better than any route 
      if (newTime < nodes[neighbor].bestTime) {
        // Only track first time we discover this node
        if (nodes[neighbor].bestTime == std::numeric_limits<double>::infinity())
          touchedDriveNodes.push_back(neighbor);
        nodes[neighbor].bestTime = newTime;
        nodes[neighbor].reachingEdge = segID;
        //Calculate the priority of the neighbour intersection 
        //Past time + Future time (staight line)
        double priority = newTime + computeHeuristic(neighbor, dest);
        wavefront.push(WaveElem(neighbor, segID, priority)); 
      }
    }
  }

  return std::vector<StreetSegmentIdx>();  // No path found
}

// Multi-modal pathfinding: walk to pickup point, then drive
std::pair<std::vector<StreetSegmentIdx>, std::vector<StreetSegmentIdx>>
findPathWithWalkToPickUp(const IntersectionIdx start_intersection,
                         const IntersectionIdx end_intersection,
                         const double turn_penalty, const double walking_speed,
                         const double walking_time_limit) {
  std::pair<std::vector<StreetSegmentIdx>, std::vector<StreetSegmentIdx>>
      result;
  if (start_intersection == end_intersection) return result;

  //Walking Dijkstra — dirty-reset only touched walkNodes

  for (IntersectionIdx idx : touchedWalkNodes) {
    walkNodes[idx].bestTime = std::numeric_limits<double>::infinity();
    walkNodes[idx].reachingEdge = NO_EDGE;
    walkNodes[idx].visited = false;
  }
  touchedWalkNodes.clear();
  clearWavefront();

  walkNodes[start_intersection].bestTime = 0.0;
  touchedWalkNodes.push_back(start_intersection);
  wavefront.push(WaveElem(start_intersection, NO_EDGE, 0.0));

  std::vector<IntersectionIdx> reachablePickups;

  while (!wavefront.empty()) {
    WaveElem current = wavefront.top();
    wavefront.pop();

    IntersectionIdx currNode = current.nodeID;
    if (walkNodes[currNode].visited) continue;
    walkNodes[currNode].visited = true;

    double walkTime = walkNodes[currNode].bestTime;
    if (walkTime > walking_time_limit) continue;

    reachablePickups.push_back(currNode);

    StreetIdx currentStreet = NO_STREET;
    if (current.edgeID != NO_EDGE)
      currentStreet = segmentInfos[current.edgeID].streetID;

    for (StreetSegmentIdx segID : intersectionSegments[currNode]) {
      const StreetSegmentInfo& info = segmentInfos[segID];
      IntersectionIdx neighbor = (info.from == currNode) ? info.to : info.from;

      double edgeTime = segmentWalkingLengths[segID] / walking_speed;
      double turnTime =
          (currentStreet != NO_STREET && currentStreet != info.streetID)
              ? turn_penalty
              : 0.0;
      double newTime = walkTime + edgeTime + turnTime;

      if (newTime <= walking_time_limit &&
          newTime < walkNodes[neighbor].bestTime) {
        // Track for dirty reset
        if (walkNodes[neighbor].bestTime ==
            std::numeric_limits<double>::infinity())
          touchedWalkNodes.push_back(neighbor);
        walkNodes[neighbor].bestTime = newTime;
        walkNodes[neighbor].reachingEdge = segID;
        wavefront.push(WaveElem(neighbor, segID, newTime));
      }
    }
  }

  //Multi-Source A* driving — dirty-reset only touched driveNodes

  for (IntersectionIdx idx : touchedDriveNodes) {
    nodes[idx].bestTime = std::numeric_limits<double>::infinity();
    nodes[idx].reachingEdge = NO_EDGE;
    nodes[idx].visited = false;
  }
  touchedDriveNodes.clear();
  clearWavefront();

  for (IntersectionIdx pickup : reachablePickups) {
    if (nodes[pickup].bestTime == std::numeric_limits<double>::infinity())
      touchedDriveNodes.push_back(pickup);
    nodes[pickup].bestTime = 0.0;
    nodes[pickup].reachingEdge = NO_EDGE;
    double priority = computeHeuristic(pickup, end_intersection);
    wavefront.push(WaveElem(pickup, NO_EDGE, priority));
  }

  while (!wavefront.empty()) {
    WaveElem current = wavefront.top();
    wavefront.pop();

    IntersectionIdx currNode = current.nodeID;
    if (nodes[currNode].visited) continue;
    nodes[currNode].visited = true;

    if (currNode == end_intersection) break;

    StreetIdx currentStreet = NO_STREET;
    if (current.edgeID != NO_EDGE)
      currentStreet = segmentInfos[current.edgeID].streetID;

    for (StreetSegmentIdx segID : intersectionSegments[currNode]) {
      const StreetSegmentInfo& info = segmentInfos[segID];

      IntersectionIdx neighbor = NO_INTERSECTION;
      if (info.from == currNode)
        neighbor = info.to;
      else if (!info.oneWay)
        neighbor = info.from;
      else
        continue;

      double edgeTime = segmentTravelTimes[segID];
      double turnTime =
          (currentStreet != NO_STREET && currentStreet != info.streetID)
              ? turn_penalty
              : 0.0;
      double newTime = nodes[currNode].bestTime + edgeTime + turnTime;

      if (newTime < nodes[neighbor].bestTime) {
        // Track for dirty reset
        if (nodes[neighbor].bestTime == std::numeric_limits<double>::infinity())
          touchedDriveNodes.push_back(neighbor);
        nodes[neighbor].bestTime = newTime;
        nodes[neighbor].reachingEdge = segID;
        double priority =
            newTime + computeHeuristic(neighbor, end_intersection);
        wavefront.push(WaveElem(neighbor, segID, priority); 
      }
    }
  }

  //Reconstruct paths (unchanged from before)

  if (nodes[end_intersection].bestTime ==
      std::numeric_limits<double>::infinity())
    return result;

  std::vector<StreetSegmentIdx> drivingPath;
  IntersectionIdx tracer = end_intersection;
  while (nodes[tracer].reachingEdge != NO_EDGE) {
    StreetSegmentIdx edge = nodes[tracer].reachingEdge;
    drivingPath.push_back(edge);
    const StreetSegmentInfo& info = segmentInfos[edge];
    tracer = (info.to == tracer) ? info.from : info.to;
  }
  IntersectionIdx bestPickup = tracer;
  std::reverse(drivingPath.begin(), drivingPath.end());
  result.second = drivingPath;

  std::vector<StreetSegmentIdx> walkPath;
  IntersectionIdx walkTracer = bestPickup;
  while (walkTracer != start_intersection) {
    StreetSegmentIdx edge = walkNodes[walkTracer].reachingEdge;
    if (edge == NO_EDGE) break;
    walkPath.push_back(edge);
    const StreetSegmentInfo& info = segmentInfos[edge];
    walkTracer = (info.to == walkTracer) ? info.from : info.to;
  }
  std::reverse(walkPath.begin(), walkPath.end());
  result.first = walkPath;

  return result;
}
