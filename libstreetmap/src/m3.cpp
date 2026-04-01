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
#include "m3_globals.h" // <-- YOU MUST INCLUDE THIS!

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "StreetsDatabaseAPI.h"
#include "m1.h"

// Helper functions (Internal to m3.cpp)
void resetNodes();
double computeHeuristic(IntersectionIdx current, IntersectionIdx dest);
std::vector<StreetSegmentIdx> tracePath(IntersectionIdx src, IntersectionIdx dest);

// --- GLOBAL VARIABLES (Notice there is NO 'static' keyword!) ---
double maxSpeedLimit = 0.0;
std::vector<double> segmentWalkingLengths;
std::vector<LatLon> intersectionPositions;
std::vector<StreetSegmentInfo> segmentInfos;
std::vector<std::vector<StreetSegmentIdx>> intersectionSegments;
std::vector<IntersectionIdx> touchedDriveNodes;
std::vector<IntersectionIdx> touchedWalkNodes;
std::vector<Node> walkNodes;
std::vector<Node> nodes;
std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> wavefront;
std::vector<double> segmentTravelTimes;

// --- YOUR FUNCTIONS START HERE ---


// Initialize pathfinding data structures
void initializePathfinding() {
  int numIntersections = getNumIntersections();
  nodes.resize(numIntersections);
  walkNodes.resize(numIntersections);
  touchedDriveNodes.reserve(numIntersections);
  touchedWalkNodes.reserve(numIntersections);
  intersectionSegments.resize(numIntersections);
  intersectionPositions.resize(numIntersections);

  // Precompute intersection positions and segments
  for (int i = 0; i < numIntersections; i++) {
    intersectionPositions[i] = getIntersectionPosition(i);
    intersectionSegments[i] = findStreetSegmentsOfIntersection(i);
  }

  // Precompute segment travel times and walking lengths
  int numSegments = getNumStreetSegments();
  segmentTravelTimes.resize(numSegments);
  segmentWalkingLengths.resize(numSegments);
  segmentInfos.resize(numSegments);

  // Also find max speed limit for heuristic
  for (int i = 0; i < numSegments; i++) {
    StreetSegmentInfo info = getStreetSegmentInfo(i);
    segmentInfos[i] = info;
    double length = findStreetSegmentLength(i);
    segmentTravelTimes[i] = length / info.speedLimit;
    segmentWalkingLengths[i] = length;
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
double computePathTravelTime(const double turn_penalty,
                             const std::vector<StreetSegmentIdx>& path) {
  if (path.empty()) return 0.0;

  double totalTime = 0.0;
  StreetIdx prevStreet = NO_STREET;

  for (StreetSegmentIdx segID : path) {
    // Add segment travel time
    totalTime += segmentTravelTimes[segID];

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

  if (src == dest) return std::vector<StreetSegmentIdx>();

  // Dirty reset — only clear nodes touched last call
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

  // A* Search
  while (!wavefront.empty()) {
    WaveElem current = wavefront.top();
    wavefront.pop();

    IntersectionIdx currNode = current.nodeID;
    if (nodes[currNode].visited) continue;
    nodes[currNode].visited = true;

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

    // Determine current street for turn penalty calculation
    StreetIdx currentStreet = NO_STREET;
    if (current.edgeID != NO_EDGE)
      currentStreet = segmentInfos[current.edgeID].streetID;

    // Explore neighbors
    for (StreetSegmentIdx segID : intersectionSegments[currNode]) {
      const StreetSegmentInfo& info = segmentInfos[segID];

      // Determine neighbor intersection based on directions
      IntersectionIdx neighbor = NO_INTERSECTION;
      if (info.from == currNode)
        neighbor = info.to;
      else if (!info.oneWay)
        neighbor = info.from;
      else
        continue;

      // Calculate new time to neighbor
      double edgeTime = segmentTravelTimes[segID];
      double turnTime =
          (currentStreet != NO_STREET && currentStreet != info.streetID)
              ? turn_penalty
              : 0.0;
      double newTime = nodes[currNode].bestTime + edgeTime + turnTime;

      if (newTime < nodes[neighbor].bestTime) {
        // Only track first time we discover this node
        if (nodes[neighbor].bestTime == std::numeric_limits<double>::infinity())
          touchedDriveNodes.push_back(neighbor);
        nodes[neighbor].bestTime = newTime;
        nodes[neighbor].reachingEdge = segID;
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

  // Walking Dijkstra - dirty reset only touched the walkNodes
  for (IntersectionIdx idx : touchedWalkNodes) {
    walkNodes[idx].bestTime = std::numeric_limits<double>::infinity();
    walkNodes[idx].reachingEdge = NO_EDGE;
    walkNodes[idx].visited = false;
  }
  touchedWalkNodes.clear();
  clearWavefront();

  // Initialize walking source
  walkNodes[start_intersection].bestTime = 0.0;
  touchedWalkNodes.push_back(start_intersection);
  wavefront.push(WaveElem(start_intersection, NO_EDGE, 0.0));

  std::vector<IntersectionIdx>
      reachablePickups;  // Intersections reachable by walking within time limit

  // Dijkstra for walking to find reachable pickup points within time limit
  while (!wavefront.empty()) {
    WaveElem current = wavefront.top();
    wavefront.pop();

    // Skip if already visited or exceeds walking time limit
    IntersectionIdx currNode = current.nodeID;
    if (walkNodes[currNode].visited) continue;
    walkNodes[currNode].visited = true;

    double walkTime = walkNodes[currNode].bestTime;
    if (walkTime > walking_time_limit) continue;  // Exceeds walking time limit

    reachablePickups.push_back(currNode);

    StreetIdx currentStreet = NO_STREET;
    if (current.edgeID != NO_EDGE)
      currentStreet = segmentInfos[current.edgeID].streetID;

    // Explore neighbors for walking
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

  // Multi-Source A* driving - dirty-reset only touched the driveNodes
  for (IntersectionIdx idx : touchedDriveNodes) {
    nodes[idx].bestTime = std::numeric_limits<double>::infinity();
    nodes[idx].reachingEdge = NO_EDGE;
    nodes[idx].visited = false;
  }
  touchedDriveNodes.clear();
  clearWavefront();

  // Initialize wavefront with all reachable pickup points
  for (IntersectionIdx pickup : reachablePickups) {
    if (nodes[pickup].bestTime == std::numeric_limits<double>::infinity())
      touchedDriveNodes.push_back(pickup);
    nodes[pickup].bestTime = 0.0;
    nodes[pickup].reachingEdge = NO_EDGE;
    double priority = computeHeuristic(pickup, end_intersection);
    wavefront.push(WaveElem(pickup, NO_EDGE, priority));
  }

  // A* Search from all reachable pickup points to destination
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

    // Explore neighbors for driving
    for (StreetSegmentIdx segID : intersectionSegments[currNode]) {
      const StreetSegmentInfo& info = segmentInfos[segID];

      // Determine neighbor intersection based on directions
      IntersectionIdx neighbor = NO_INTERSECTION;
      if (info.from == currNode)
        neighbor = info.to;
      else if (!info.oneWay)
        neighbor = info.from;
      else
        continue;

      // Calculate new time to neighbor
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
        wavefront.push(WaveElem(neighbor, segID, priority));
      }
    }
  }

  // Reconstruct paths (unchanged from before)
  if (nodes[end_intersection].bestTime ==
      std::numeric_limits<double>::infinity())
    return result;

  // Trace driving path from destination back to best pickup point
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

  // Trace walking path from best pickup point back to start
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
