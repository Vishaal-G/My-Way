#include <algorithm>
#include <chrono>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "m3.h"
#include "m3_globals.h"
#include "m4.h"

extern std::vector<IntersectionIdx> touchedDriveNodes;
extern std::vector<Node> nodes;
extern std::vector<StreetSegmentInfo> segmentInfos;
extern std::vector<double> segmentTravelTimes;
extern std::vector<std::vector<StreetSegmentIdx>> intersectionSegments;

// Data Structures
struct POI_Data {
  double travel_time;
  std::vector<StreetSegmentIdx> path;
};

// Global Cache: cache[source_intersection][dest_intersection]
std::unordered_map<IntersectionIdx,
                   std::unordered_map<IntersectionIdx, POI_Data>>
    travel_cache;

static std::vector<bool> is_poi;

// Helper to track cargo
struct DeliveryState {
  std::vector<bool> picked_up;
  std::vector<bool> dropped_off;
  int total_deliveries;
  int deliveries_completed = 0;
};

std::vector<IntersectionIdx> getUniquePOIs(
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots) {
  std::vector<IntersectionIdx> all_pois;

  // Add all pickups and dropoffs
  for (const auto& delivery : deliveries) {
    all_pois.push_back(delivery.pickUp);
    all_pois.push_back(delivery.dropOff);
  }

  // Add all depots
  for (IntersectionIdx depot : depots) {
    all_pois.push_back(depot);
  }

  // Remove duplicates (e.g., if two packages are picked up at the same place)
  std::sort(all_pois.begin(), all_pois.end());
  all_pois.erase(std::unique(all_pois.begin(), all_pois.end()), all_pois.end());

  return all_pois;
}

// Multi-Target Dijkstra's Algorithm
std::unordered_map<IntersectionIdx, POI_Data> multiTargetDijkstra(
    IntersectionIdx start_node, const std::vector<IntersectionIdx>& all_pois,
    const float turn_penalty) {

  std::unordered_map<IntersectionIdx, POI_Data> found_paths;
  int targets_found = 0;
  int total_targets = all_pois.size();

  // Dirty Reset (from M3)
  for (IntersectionIdx idx : touchedDriveNodes) {
    nodes[idx].bestTime = std::numeric_limits<double>::infinity();
    nodes[idx].reachingEdge = NO_EDGE;
    nodes[idx].visited = false;
  }
  touchedDriveNodes.clear();
  clearWavefront();

  // Initialize Start Node
  nodes[start_node].bestTime = 0.0;
  touchedDriveNodes.push_back(start_node);
  wavefront.push(WaveElem(start_node, NO_EDGE, 0.0));

  // The Dijkstra Expansion Loop
  while (!wavefront.empty() && targets_found < total_targets) {
    WaveElem current = wavefront.top();
    wavefront.pop();

    IntersectionIdx currNode = current.nodeID;
    if (nodes[currNode].visited) continue;
    nodes[currNode].visited = true;

    // If this is one of our target POIs, reconstruct the path and save it
    if (is_poi[currNode]) {
      targets_found++;

      // Reconstruct the path immediately
      std::vector<StreetSegmentIdx> path;
      IntersectionIdx tracer = currNode;
      while (tracer != start_node) {
        StreetSegmentIdx edge = nodes[tracer].reachingEdge;
        path.push_back(edge);
        tracer = segmentInfos[edge].from == tracer
                     ? segmentInfos[edge].to
                     : segmentInfos[edge].from;  // Careful with 1-way tracing
      }
      std::reverse(path.begin(), path.end());

      // Save to our results map
      found_paths[currNode] = {nodes[currNode].bestTime, path};
    }

    // Check neighbors
    StreetIdx currentStreet = (current.edgeID != NO_EDGE)
                                  ? segmentInfos[current.edgeID].streetID
                                  : NO_STREET;

    for (StreetSegmentIdx segID : intersectionSegments[currNode]) {
      const StreetSegmentInfo& info = segmentInfos[segID];

      IntersectionIdx neighbor = NO_INTERSECTION;
      if (info.from == currNode)
        neighbor = info.to;
      else if (!info.oneWay)
        neighbor = info.from;
      else
        continue;  // Obey one-way streets

      double edgeTime = segmentTravelTimes[segID];
      double turnTime =
          (currentStreet != NO_STREET && currentStreet != info.streetID)
              ? turn_penalty
              : 0.0;
      double newTime = nodes[currNode].bestTime + edgeTime + turnTime;

      if (newTime < nodes[neighbor].bestTime) {
        if (nodes[neighbor].bestTime ==
            std::numeric_limits<double>::infinity()) {
          touchedDriveNodes.push_back(neighbor);
        }
        nodes[neighbor].bestTime = newTime;
        nodes[neighbor].reachingEdge = segID;

        // Use plain Dijkstra time
        wavefront.push(WaveElem(neighbor, segID, newTime));
      }
    }
  }

  return found_paths;
}

void buildTravelCache(const std::vector<DeliveryInf>& deliveries,
                      const std::vector<IntersectionIdx>& depots,
                      const float turn_penalty) {
  travel_cache.clear();

  // Get unique POIs
  std::vector<IntersectionIdx> all_pois = getUniquePOIs(deliveries, depots);

  // Setup the fast O(1) POI checker
  int num_intersections = getNumIntersections();
  is_poi.assign(num_intersections, false);
  for (IntersectionIdx poi : all_pois) {
    is_poi[poi] = true;
  }

  // Flood the map from every POI
  for (int i = 0; i < all_pois.size(); i++) {
    IntersectionIdx start_poi = all_pois[i];
    travel_cache[start_poi] =
        multiTargetDijkstra(start_poi, all_pois, turn_penalty);
  }
}
