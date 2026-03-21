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



static std::vector<double> segmentWalkingLengths;
static std::vector<LatLon> intersectionPositions;

// Add this global alongside the others
static std::vector<IntersectionIdx> touchedDriveNodes;
static std::vector<IntersectionIdx> touchedWalkNodes;

// Data Structures for Pathfinding

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

// Separate node storage for walk and drive phases
static std::vector<Node> walkNodes;  // add this global alongside `nodes`
// Global data structures for pathfinding (reused across calls)
static std::vector<Node> nodes;
static std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> wavefront;

// Precomputed segment travel times (computed once in loadMap)
static std::vector<double> segmentTravelTimes;

// Helper functions 

// Initialize pathfinding data structures
void initializePathfinding() {
    int numIntersections = getNumIntersections();
    nodes.resize(numIntersections);
    walkNodes.resize(numIntersections);
    touchedDriveNodes.reserve(numIntersections);
    touchedWalkNodes.reserve(numIntersections);

    // Precompute intersection positions
    intersectionPositions.resize(numIntersections);
    for (int i = 0; i < numIntersections; i++) {
        intersectionPositions[i] = getIntersectionPosition(i);
    }
    
    // Precompute travel times for all segments
    int numSegments = getNumStreetSegments();
    segmentTravelTimes.resize(numSegments);
    segmentWalkingLengths.resize(numSegments);
    
    for (int i = 0; i < numSegments; i++) {
        StreetSegmentInfo info = getStreetSegmentInfo(i);
        double length = findStreetSegmentLength(i);
        segmentTravelTimes[i] = length / info.speedLimit;
        segmentWalkingLengths[i] = length;
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
    intersectionPositions.clear();
    intersectionPositions.shrink_to_fit();
    clearWavefront();
}
