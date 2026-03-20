/*
 * Copyright 2026 University of Toronto
 *
 * ECE297 Milestone 3 - Pathfinding Implementation
 * * This file implements pathfinding algorithms including:
 * - Computing path travel times (driving and walking)
 * - Finding shortest paths between intersections
 * - Multi-modal pathfinding (walk + drive)
 */

#include "m3.h"
#include "m1.h"
#include "StreetsDatabaseAPI.h"

#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <cmath>

// Constants
constexpr StreetSegmentIdx NO_EDGE = -1;
constexpr IntersectionIdx NO_INTERSECTION = -1;
constexpr StreetIdx NO_STREET = -1;

// Helper functions 
void resetNodes();
void clearWavefront();
double computeHeuristic(IntersectionIdx current, IntersectionIdx dest);
std::vector<StreetSegmentIdx> tracePath(IntersectionIdx src, IntersectionIdx dest);

// ============================================================================
// Data Structures for Pathfinding
// ============================================================================

struct WaveElem {
    IntersectionIdx nodeID;
    StreetSegmentIdx edgeID;  // Street segment used to reach this node
    double travelTime;        // Time to reach this node from start
    
    WaveElem(IntersectionIdx node, StreetSegmentIdx edge, double time)
        : nodeID(node), edgeID(edge), travelTime(time) {}
};

// Comparator for priority queue (min-heap based on travel time)
struct CompareWaveElem {
    bool operator()(const WaveElem& a, const WaveElem& b) const {
        return a.travelTime > b.travelTime; // Min-heap
    }
};

// Node information for pathfinding (DUAL-STATE to prevent overwriting)
struct Node {
    // State 1: Reached via Driving
    double bestTimeDriving;
    StreetSegmentIdx reachingEdgeDriving;
    bool reachingEdgeCameFromPickup; // Tracks if the previous node was the pickup point
    bool visitedDriving;
    
    // State 2: Reached via Walking (Pickup Point)
    double bestTimePickup;
    StreetSegmentIdx reachingEdgePickup;
    bool visitedPickup;
    
    Node() : bestTimeDriving(std::numeric_limits<double>::infinity()),
             reachingEdgeDriving(NO_EDGE),
             reachingEdgeCameFromPickup(false),
             visitedDriving(false),
             bestTimePickup(std::numeric_limits<double>::infinity()),
             reachingEdgePickup(NO_EDGE),
             visitedPickup(false) {}
};

// Global data structures for pathfinding
static std::vector<Node> nodes;
static std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> wavefront;

// Precomputed segment travel times
static std::vector<double> segmentTravelTimes;

// ============================================================================
// Helper Functions
// ============================================================================

void initializePathfinding() {
    int numIntersections = getNumIntersections();
    nodes.resize(numIntersections);
    
    int numSegments = getNumStreetSegments();
    segmentTravelTimes.resize(numSegments);
    
    for (int i = 0; i < numSegments; i++) {
        StreetSegmentInfo info = getStreetSegmentInfo(i);
        double length = findStreetSegmentLength(i);
        segmentTravelTimes[i] = length / info.speedLimit;
    }
}

void clearWavefront() {
    while (!wavefront.empty()) {
        wavefront.pop();
    }
}

void cleanupPathfinding() {
    nodes.clear();
    nodes.shrink_to_fit();
    segmentTravelTimes.clear();
    segmentTravelTimes.shrink_to_fit();
    clearWavefront();
}

void resetNodes() {
    for (auto& node : nodes) {
        node.bestTimeDriving = std::numeric_limits<double>::infinity();
        node.reachingEdgeDriving = NO_EDGE;
        node.reachingEdgeCameFromPickup = false;
        node.visitedDriving = false;
        
        node.bestTimePickup = std::numeric_limits<double>::infinity();
        node.reachingEdgePickup = NO_EDGE;
        node.visitedPickup = false;
    }
}

double computeHeuristic(IntersectionIdx current, IntersectionIdx dest) {
    LatLon pos1 = getIntersectionPosition(current);
    LatLon pos2 = getIntersectionPosition(dest);
    
    double distance = findDistanceBetweenTwoPoints({pos1, pos2});
    const double MAX_SPEED = 33.33; // ~120 km/h in m/s
    return distance / MAX_SPEED;
}

std::vector<StreetSegmentIdx> tracePath(IntersectionIdx src, IntersectionIdx dest) {
    std::vector<StreetSegmentIdx> path;
    IntersectionIdx current = dest;
    
    while (current != src) {
        StreetSegmentIdx edge = nodes[current].reachingEdgeDriving;
        if (edge == NO_EDGE) return std::vector<StreetSegmentIdx>();
        
        path.push_back(edge);
        
        StreetSegmentInfo info = getStreetSegmentInfo(edge);
        if (info.to == current) {
            current = info.from;
        } else {
            current = info.to;
        }
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// ============================================================================
// M3 Function Implementations
// ============================================================================

double computePathTravelTime(const double turn_penalty,
                             const std::vector<StreetSegmentIdx>& path) {
    if (path.empty()) return 0.0;
    
    double totalTime = 0.0;
    StreetIdx prevStreet = NO_STREET;
    
    for (StreetSegmentIdx segID : path) {
        totalTime += segmentTravelTimes[segID];
        
        StreetSegmentInfo info = getStreetSegmentInfo(segID);
        StreetIdx currentStreet = info.streetID;
        
        if (prevStreet != NO_STREET && prevStreet != currentStreet) {
            totalTime += turn_penalty;
        }
        prevStreet = currentStreet;
    }
    return totalTime;
}

// RESTORED: Turn penalties MUST be applied for walking in ECE297!
double computePathWalkingTime(const std::vector<StreetSegmentIdx>& path,
                              const double walking_speed,
                              const double turn_penalty) {
    if (path.empty()) return 0.0;
    
    double totalTime = 0.0;
    StreetIdx prevStreet = NO_STREET;
    
    for (StreetSegmentIdx segID : path) {
        double length = findStreetSegmentLength(segID);
        totalTime += length / walking_speed;
        
        StreetSegmentInfo info = getStreetSegmentInfo(segID);
        if (prevStreet != NO_STREET && prevStreet != info.streetID) {
            totalTime += turn_penalty;
        }
        prevStreet = info.streetID;
    }
    return totalTime;
}

std::vector<StreetSegmentIdx> findPathBetweenIntersections(
    const double turn_penalty,
    const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids) {
    
    IntersectionIdx src = intersect_ids.first;
    IntersectionIdx dest = intersect_ids.second;
    
    if (src == dest) return std::vector<StreetSegmentIdx>();
    
    resetNodes();
    clearWavefront();
    
    nodes[src].bestTimeDriving = 0.0;
    wavefront.push(WaveElem(src, NO_EDGE, 0.0));
    
    while (!wavefront.empty()) {
        WaveElem current = wavefront.top();
        wavefront.pop();
        
        IntersectionIdx currNode = current.nodeID;
        
        if (nodes[currNode].visitedDriving) continue;
        nodes[currNode].visitedDriving = true;
        
        if (currNode == dest) {
            return tracePath(src, dest);
        }
        
        StreetIdx currentStreet = NO_STREET;
        if (current.edgeID != NO_EDGE) {
            currentStreet = getStreetSegmentInfo(current.edgeID).streetID;
        }
        
        std::vector<StreetSegmentIdx> outEdges = findStreetSegmentsOfIntersection(currNode);
        
        for (StreetSegmentIdx segID : outEdges) {
            StreetSegmentInfo info = getStreetSegmentInfo(segID);
            
            IntersectionIdx neighbor = NO_INTERSECTION;
            if (info.from == currNode) {
                neighbor = info.to;
            } else if (info.to == currNode && !info.oneWay) {
                neighbor = info.from;
            } else {
                continue;
            }
            
            double edgeTime = segmentTravelTimes[segID];
            double turnTime = (currentStreet != NO_STREET && currentStreet != info.streetID) ? turn_penalty : 0.0;
            double newTime = nodes[currNode].bestTimeDriving + edgeTime + turnTime;
            
            if (newTime < nodes[neighbor].bestTimeDriving) {
                nodes[neighbor].bestTimeDriving = newTime;
                nodes[neighbor].reachingEdgeDriving = segID;
                
                double priority = newTime + computeHeuristic(neighbor, dest);
                wavefront.push(WaveElem(neighbor, segID, priority));
            }
        }
    }
    return std::vector<StreetSegmentIdx>();
}

std::pair<std::vector<StreetSegmentIdx>, std::vector<StreetSegmentIdx>>
findPathWithWalkToPickUp(
    const IntersectionIdx start_intersection,
    const IntersectionIdx end_intersection,
    const double turn_penalty,
    const double walking_speed,
    const double walking_time_limit) {
    
    std::pair<std::vector<StreetSegmentIdx>, std::vector<StreetSegmentIdx>> result;
    if (start_intersection == end_intersection) return result; 
    
    int numIntersections = getNumIntersections();
    std::vector<double> walkTimes(numIntersections, -1.0);
    std::vector<StreetSegmentIdx> walkReachingEdges(numIntersections, NO_EDGE);
    
    // ========================================================================
    // PHASE 1: WALKING DIJKSTRA (Find all valid pickups & their times)
    // ========================================================================
    resetNodes();
    clearWavefront();
    
    nodes[start_intersection].bestTimePickup = 0.0;
    wavefront.push(WaveElem(start_intersection, NO_EDGE, 0.0));
    
    while (!wavefront.empty()) {
        WaveElem current = wavefront.top();
        wavefront.pop();
        
        IntersectionIdx currNode = current.nodeID;
        
        if (nodes[currNode].visitedPickup) continue;
        nodes[currNode].visitedPickup = true;
        
        walkTimes[currNode] = nodes[currNode].bestTimePickup;
        walkReachingEdges[currNode] = nodes[currNode].reachingEdgePickup;
        
        StreetIdx currentStreet = NO_STREET;
        if (current.edgeID != NO_EDGE) {
            currentStreet = getStreetSegmentInfo(current.edgeID).streetID;
        }
        
        std::vector<StreetSegmentIdx> outEdges = findStreetSegmentsOfIntersection(currNode);
        
        for (StreetSegmentIdx segID : outEdges) {
            StreetSegmentInfo info = getStreetSegmentInfo(segID);
            // Walking ignores one-way constraints
            IntersectionIdx neighbor = (info.from == currNode) ? info.to : info.from;
            
            double length = findStreetSegmentLength(segID);
            double edgeTime = length / walking_speed;
            
            // Apply turn penalties to pedestrians!
            double turnTime = (currentStreet != NO_STREET && currentStreet != info.streetID) ? turn_penalty : 0.0;
            double newTime = nodes[currNode].bestTimePickup + edgeTime + turnTime;
            
            if (newTime <= walking_time_limit && newTime < nodes[neighbor].bestTimePickup) {
                nodes[neighbor].bestTimePickup = newTime;
                nodes[neighbor].reachingEdgePickup = segID;
                wavefront.push(WaveElem(neighbor, segID, newTime)); 
            }
        }
    }
    
    // ========================================================================
    // PHASE 2: MULTI-SOURCE DRIVING A* (Optimize Total Time simultaneously)
    // ========================================================================
    resetNodes();
    clearWavefront();
    
    // Prime the wavefront with ALL reachable walking nodes at once
    for (int i = 0; i < numIntersections; i++) {
        if (walkTimes[i] != -1.0) {
            nodes[i].bestTimeDriving = walkTimes[i]; 
            double priority = walkTimes[i] + computeHeuristic(i, end_intersection);
            wavefront.push(WaveElem(i, NO_EDGE, priority));
        }
    }
    
    StreetSegmentIdx finalReachingEdge = NO_EDGE;
    bool finalCameFromPickup = false;
    bool pathFound = false;
    
    while (!wavefront.empty()) {
        WaveElem current = wavefront.top();
        wavefront.pop();
        
        IntersectionIdx currNode = current.nodeID;
        
        // This stops it from instantly aborting!
        if (nodes[currNode].visitedDriving) continue;
        nodes[currNode].visitedDriving = true;
        
        if (currNode == end_intersection) {
            finalReachingEdge = current.edgeID;
            if (current.edgeID != NO_EDGE) {
                finalCameFromPickup = nodes[currNode].reachingEdgeCameFromPickup;
            }
            pathFound = true;
            break; 
        }
        
        StreetIdx currentStreet = NO_STREET;
        double currentBestTime = 0.0;
        
        // Isolate Pickup State vs Driving State
        if (current.edgeID == NO_EDGE) {
            currentStreet = NO_STREET; // Picking up! No turn penalty next.
            currentBestTime = walkTimes[currNode];
        } else {
            currentStreet = getStreetSegmentInfo(current.edgeID).streetID;
            currentBestTime = nodes[currNode].bestTimeDriving;
        }
        
        std::vector<StreetSegmentIdx> outEdges = findStreetSegmentsOfIntersection(currNode);
        
        for (StreetSegmentIdx segID : outEdges) {
            StreetSegmentInfo info = getStreetSegmentInfo(segID);
            IntersectionIdx neighbor = (info.from == currNode) ? info.to : ((info.to == currNode && !info.oneWay) ? info.from : NO_INTERSECTION);
            if (neighbor == NO_INTERSECTION) continue; 
            
            double edgeTime = segmentTravelTimes[segID];
            double turnTime = (currentStreet != NO_STREET && currentStreet != info.streetID) ? turn_penalty : 0.0;
            double newTime = currentBestTime + edgeTime + turnTime;
            
            // Only overwrite the DRIVING state
            if (newTime < nodes[neighbor].bestTimeDriving) {
                nodes[neighbor].bestTimeDriving = newTime;
                nodes[neighbor].reachingEdgeDriving = segID;
                nodes[neighbor].reachingEdgeCameFromPickup = (current.edgeID == NO_EDGE);
                
                double priority = newTime + computeHeuristic(neighbor, end_intersection);
                wavefront.push(WaveElem(neighbor, segID, priority));
            }
        }
    }
    
    if (!pathFound) return result; 
    
    // ========================================================================
    // PHASE 3: TRACE THE ROUTES BACKWARDS
    // ========================================================================
    std::vector<StreetSegmentIdx> drivingPath;
    IntersectionIdx bestPickup = end_intersection;
    
    // 1. Trace Driving Path
    if (finalReachingEdge != NO_EDGE) {
        StreetSegmentIdx edge = finalReachingEdge;
        IntersectionIdx curr = end_intersection;
        bool cameFromPickup = finalCameFromPickup;
        
        while (edge != NO_EDGE) {
            drivingPath.push_back(edge);
            StreetSegmentInfo info = getStreetSegmentInfo(edge);
            IntersectionIdx prevNode = (info.to == curr) ? info.from : info.to;
            
            if (cameFromPickup) {
                bestPickup = prevNode;
                break; 
            }
            
            edge = nodes[prevNode].reachingEdgeDriving;
            cameFromPickup = nodes[prevNode].reachingEdgeCameFromPickup;
            curr = prevNode;
        }
        std::reverse(drivingPath.begin(), drivingPath.end());
    }
    
    // 2. Trace Walking Path
    std::vector<StreetSegmentIdx> walkingPath;
    IntersectionIdx currWalk = bestPickup;
    
    while (currWalk != start_intersection) {
        StreetSegmentIdx edge = walkReachingEdges[currWalk];
        if (edge == NO_EDGE) break; 
        
        walkingPath.push_back(edge);
        StreetSegmentInfo info = getStreetSegmentInfo(edge);
        currWalk = (info.to == currWalk) ? info.from : info.to;
    }
    std::reverse(walkingPath.begin(), walkingPath.end());
    
    result.first = walkingPath;
    result.second = drivingPath;
    
    return result;
}