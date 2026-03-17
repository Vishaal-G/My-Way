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
#include "m1.h"
#include "StreetsDatabaseAPI.h"

#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <cmath>

//Constants
constexpr StreetSegmentIdx NO_EDGE = -1;
constexpr IntersectionIdx NO_INTERSECTION = -1;
constexpr StreetIdx NO_STREET = -1;

// Helper functions (optional - can be kept internal to m3.cpp)
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
    double travelTime;         // Time to reach this node from start
    
    WaveElem(IntersectionIdx node, StreetSegmentIdx edge, double time)
        : nodeID(node), edgeID(edge), travelTime(time) {}
};

// Comparator for priority queue (min-heap based on travel time)
struct CompareWaveElem {
    bool operator()(const WaveElem& a, const WaveElem& b) const {
        return a.travelTime > b.travelTime; // Min-heap
    }
};

// Node information for pathfinding
struct Node {
    double bestTime;              // Best time to reach this node
    StreetSegmentIdx reachingEdge; // Edge used to reach this node
    bool visited;                  // Whether node has been visited
    
    Node() : bestTime(std::numeric_limits<double>::infinity()),
             reachingEdge(NO_EDGE),
             visited(false) {}
};

// Global data structures for pathfinding (reused across calls)
static std::vector<Node> nodes;
static std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> wavefront;

// Precomputed segment travel times (computed once in loadMap)
static std::vector<double> segmentTravelTimes;

// ============================================================================
// Helper Functions
// ============================================================================

// Initialize pathfinding data structures
void initializePathfinding() {
    int numIntersections = getNumIntersections();
    nodes.resize(numIntersections);
    
    // Precompute travel times for all segments
    int numSegments = getNumStreetSegments();
    segmentTravelTimes.resize(numSegments);
    
    for (int i = 0; i < numSegments; i++) {
        StreetSegmentInfo info = getStreetSegmentInfo(i);
        double length = findStreetSegmentLength(i);
        segmentTravelTimes[i] = length / info.speedLimit;
    }
}

// Clear the wavefront
void clearWavefront() {
    while (!wavefront.empty()) {
        wavefront.pop();
    }
}

// Clean up pathfinding data structures
void cleanupPathfinding() {
    nodes.clear();
    nodes.shrink_to_fit();
    segmentTravelTimes.clear();
    segmentTravelTimes.shrink_to_fit();
    clearWavefront();
}

// Reset all nodes to initial state
void resetNodes() {
    for (auto& node : nodes) {
        node.bestTime = std::numeric_limits<double>::infinity();
        node.reachingEdge = NO_EDGE;
        node.visited = false;
    }
}



// Heuristic function for A* (straight-line distance / max speed)
double computeHeuristic(IntersectionIdx current, IntersectionIdx dest) {
    LatLon pos1 = getIntersectionPosition(current);
    LatLon pos2 = getIntersectionPosition(dest);
    
    double distance = findDistanceBetweenTwoPoints({pos1, pos2});
    
    // Assume max speed of 120 km/h = 33.33 m/s for heuristic
    const double MAX_SPEED = 33.33;
    return distance / MAX_SPEED;
}

// Trace back the path from dest to src using reaching edges
std::vector<StreetSegmentIdx> tracePath(IntersectionIdx src, IntersectionIdx dest) {
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
        StreetSegmentInfo info = getStreetSegmentInfo(edge);
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

// ============================================================================
// M3 Function Implementations
// ============================================================================

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
        StreetSegmentInfo info = getStreetSegmentInfo(segID);
        StreetIdx currentStreet = info.streetID;
        
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
        double length = findStreetSegmentLength(segID);
        totalTime += length / walking_speed;
        
        // Check for turn
        StreetSegmentInfo info = getStreetSegmentInfo(segID);
        StreetIdx currentStreet = info.streetID;
        
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
    
    // Handle trivial case
    if (src == dest) {
        return std::vector<StreetSegmentIdx>();
    }
    
    // Reset state
    resetNodes();
    clearWavefront();
    
    // Initialize source node
    nodes[src].bestTime = 0.0;
    wavefront.push(WaveElem(src, NO_EDGE, 0.0));
    
    // A* search
    while (!wavefront.empty()) {
        WaveElem current = wavefront.top();
        wavefront.pop();
        
        IntersectionIdx currNode = current.nodeID;
        
        // Skip if already visited
        if (nodes[currNode].visited) continue;
        
        nodes[currNode].visited = true;
        
        // Found destination
        if (currNode == dest) {
            return tracePath(src, dest);
        }
        
        // Get current street for turn detection
        StreetIdx currentStreet = NO_STREET;
        if (current.edgeID != NO_EDGE) {
            currentStreet = getStreetSegmentInfo(current.edgeID).streetID;
        }
        
        // Explore neighbors
        std::vector<StreetSegmentIdx> outEdges = findStreetSegmentsOfIntersection(currNode);
        
        for (StreetSegmentIdx segID : outEdges) {
            StreetSegmentInfo info = getStreetSegmentInfo(segID);
            
            // Determine neighbor and check one-way constraint
            IntersectionIdx neighbor = NO_INTERSECTION;
            
            if (info.from == currNode) {
                neighbor = info.to;
            } else if (info.to == currNode && !info.oneWay) {
                // Can traverse in reverse if not one-way
                neighbor = info.from;
            } else {
                // One-way street, wrong direction
                continue;
            }
            
            // Calculate new travel time
            double edgeTime = segmentTravelTimes[segID];
            
            // Add turn penalty if changing streets
            double turnTime = 0.0;
            if (currentStreet != NO_STREET && currentStreet != info.streetID) {
                turnTime = turn_penalty;
            }
            
            double newTime = nodes[currNode].bestTime + edgeTime + turnTime;
            
            // Update if better path found
            if (newTime < nodes[neighbor].bestTime) {
                nodes[neighbor].bestTime = newTime;
                nodes[neighbor].reachingEdge = segID;
                
                // Add to wavefront with heuristic
                double priority = newTime + computeHeuristic(neighbor, dest);
                wavefront.push(WaveElem(neighbor, segID, priority));
            }
        }
    }
    
    // No path found
    return std::vector<StreetSegmentIdx>();
}

// Multi-modal pathfinding: walk to pickup point, then drive
std::pair<std::vector<StreetSegmentIdx>, std::vector<StreetSegmentIdx>>
findPathWithWalkToPickUp(
    const IntersectionIdx start_intersection,
    const IntersectionIdx end_intersection,
    const double turn_penalty,
    const double walking_speed,
    const double walking_time_limit) {
    
    // Result pair: {walking_path, driving_path}
    std::pair<std::vector<StreetSegmentIdx>, std::vector<StreetSegmentIdx>> result;
    
    // Special case: start == end
    if (start_intersection == end_intersection) {
        return result; // Empty paths
    }
    
    // Step 1: Find all intersections reachable by walking within time limit
    resetNodes();
    clearWavefront();
    
    nodes[start_intersection].bestTime = 0.0;
    wavefront.push(WaveElem(start_intersection, NO_EDGE, 0.0));
    
    std::vector<IntersectionIdx> reachableByWalking;
    
    // Dijkstra's algorithm for walking (no one-way constraints)
    while (!wavefront.empty()) {
        WaveElem current = wavefront.top();
        wavefront.pop();
        
        IntersectionIdx currNode = current.nodeID;
        
        if (nodes[currNode].visited) continue;
        nodes[currNode].visited = true;
        
        // Check if within walking time limit
        if (nodes[currNode].bestTime <= walking_time_limit) {
            reachableByWalking.push_back(currNode);
        }
        
        // Get current street for turn detection
        StreetIdx currentStreet = NO_STREET;
        if (current.edgeID != NO_EDGE) {
            currentStreet = getStreetSegmentInfo(current.edgeID).streetID;
        }
        
        // Explore neighbors
        std::vector<StreetSegmentIdx> outEdges = findStreetSegmentsOfIntersection(currNode);
        
        for (StreetSegmentIdx segID : outEdges) {
            StreetSegmentInfo info = getStreetSegmentInfo(segID);
            
            // Walking ignores one-way streets
            IntersectionIdx neighbor = (info.from == currNode) ? info.to : info.from;
            
            // Calculate walking time
            double length = findStreetSegmentLength(segID);
            double edgeTime = length / walking_speed;
            
            // Add turn penalty if changing streets
            double turnTime = 0.0;
            if (currentStreet != NO_STREET && currentStreet != info.streetID) {
                turnTime = turn_penalty;
            }
            
            double newTime = nodes[currNode].bestTime + edgeTime + turnTime;
            
            // Only explore if within time limit
            if (newTime <= walking_time_limit && newTime < nodes[neighbor].bestTime) {
                nodes[neighbor].bestTime = newTime;
                nodes[neighbor].reachingEdge = segID;
                wavefront.push(WaveElem(neighbor, segID, newTime));
            }
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // CRITICAL FIX: Save walking path data BEFORE it gets overwritten
    // ═══════════════════════════════════════════════════════════════════
    // Store the walking path information for each reachable intersection
    std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>> walkingPaths;
    
    for (IntersectionIdx pickup : reachableByWalking) {
        if (pickup != start_intersection) {
            walkingPaths[pickup] = tracePath(start_intersection, pickup);
        } else {
            walkingPaths[pickup] = std::vector<StreetSegmentIdx>(); // Empty path
        }
    }
    // ═══════════════════════════════════════════════════════════════════
    
    // Step 2: Find the best pickup point (minimizes driving time)
    IntersectionIdx bestPickup = NO_INTERSECTION;
    double bestDrivingTime = std::numeric_limits<double>::infinity();
    std::vector<StreetSegmentIdx> bestDrivingPath;
    
    for (IntersectionIdx pickup : reachableByWalking) {
        // Find driving path from pickup to destination
        // NOTE: This resets nodes[], so we saved walking paths above!
        std::vector<StreetSegmentIdx> drivingPath = 
            findPathBetweenIntersections(turn_penalty, {pickup, end_intersection});
        
        if (!drivingPath.empty() || pickup == end_intersection) {
            double drivingTime = computePathTravelTime(turn_penalty, drivingPath);
            
            if (drivingTime < bestDrivingTime) {
                bestDrivingTime = drivingTime;
                bestPickup = pickup;
                bestDrivingPath = drivingPath;
            }
        }
    }
    
    // Step 3: Construct result
    if (bestPickup == NO_INTERSECTION) {
        // No valid path found
        return result;
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // Use the SAVED walking path
    // ═══════════════════════════════════════════════════════════════════
    result.first = walkingPaths[bestPickup];
    result.second = bestDrivingPath;
    
    return result;
}