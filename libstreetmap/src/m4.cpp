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

std::vector<IntersectionIdx> getUniquePOIs(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);

std::unordered_map<IntersectionIdx, POI_Data> multiTargetDijkstra(IntersectionIdx start_node, const std::vector<IntersectionIdx>& all_pois, float turn_penalty);

void buildTravelCache(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, float turn_penalty);

double calculateRouteTime(const std::vector<IntersectionIdx>& route_sequence);

std::vector<IntersectionIdx> generateGreedyRoute(IntersectionIdx start_depot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);

bool isLegalRoute(const std::vector<IntersectionIdx>& test_route, const std::vector<DeliveryInf>& deliveries);


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

// Helper function to calculate total travel time for a given route sequence
double calculateRouteTime(const std::vector<IntersectionIdx>& route_sequence) {
  double total_time = 0.0;
  for (size_t i = 0; i < route_sequence.size() - 1; ++i) {
    IntersectionIdx from = route_sequence[i];
    IntersectionIdx to = route_sequence[i + 1];
    total_time += travel_cache[from][to].travel_time;
  }
  return total_time;
}

// Helper function to calculate a route starting from a specific depot
std::vector<IntersectionIdx> generateGreedyRoute(
    IntersectionIdx start_depot, const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots) {
  std::vector<IntersectionIdx> route;
  route.push_back(start_depot);

  int N = deliveries.size();
  std::vector<bool> picked_up(N, false);
  std::vector<bool> dropped_off(N, false);
  int deliveries_completed = 0;

  IntersectionIdx current_node = start_depot;

  // Greedy Loop: at each step, pick the closest next pickup or dropoff
  while (deliveries_completed < N) {
    double best_time = std::numeric_limits<double>::infinity();
    IntersectionIdx best_next_node = NO_INTERSECTION;
    int best_delivery_index = -1;
    bool is_pickup_move = false;

    // Check all possible next moves (pickups and dropoffs)
    for (int i = 0; i < N; ++i) {
      if (!picked_up[i]) {
        IntersectionIdx candidate = deliveries[i].pickUp;

        // Safely check if a path actually exists in the cache
        auto it = travel_cache[current_node].find(candidate);
        if (it != travel_cache[current_node].end()) {
          double time = it->second.travel_time;
          if (time < best_time) {
            best_time = time;
            best_next_node = candidate;
            best_delivery_index = i;
            is_pickup_move = true;
          }
        }
      } else if (picked_up[i] && !dropped_off[i]) {
        IntersectionIdx candidate = deliveries[i].dropOff;

        // Safely check if a path actually exists in the cache
        auto it = travel_cache[current_node].find(candidate);
        if (it != travel_cache[current_node].end()) {
          double time = it->second.travel_time;
          if (time < best_time) {
            best_time = time;
            best_next_node = candidate;
            best_delivery_index = i;
            is_pickup_move = false;
          }
        }
      }
    }

    // If we can't find any valid next move we must fail gracefully
    if (best_delivery_index == -1) {
      return std::vector<IntersectionIdx>();  // Return an empty, failed route
    }

    if (is_pickup_move)
      picked_up[best_delivery_index] = true;
    else {
      dropped_off[best_delivery_index] = true;
      deliveries_completed++;
    }

    if (best_next_node != current_node) {
      route.push_back(best_next_node);
      current_node = best_next_node;
    }
  }

  // Find closest end depot
  double best_depot_time = std::numeric_limits<double>::infinity();
  IntersectionIdx best_end_depot = NO_INTERSECTION;

  for (IntersectionIdx depot : depots) {
    auto it = travel_cache[current_node].find(depot);
    if (it != travel_cache[current_node].end()) {
      double time = it->second.travel_time;
      if (time < best_depot_time) {
        best_depot_time = time;
        best_end_depot = depot;
      }
    }
  }

  // Ensure we found a valid end depot before adding it to the route
  if (best_end_depot == NO_INTERSECTION) return std::vector<IntersectionIdx>();

  if (best_end_depot != current_node) {
    route.push_back(best_end_depot);
  }

  return route;
}

bool isLegalRoute(const std::vector<IntersectionIdx>& test_route,
                  const std::vector<DeliveryInf>& deliveries) {
  // Check that for every delivery, the pickup happens before the dropoff in the
  // route.
  std::unordered_map<IntersectionIdx, int> positions;

  // First, record the position of each intersection in the route
  for (int i = 0; i < test_route.size(); ++i) {
    IntersectionIdx current_node = test_route[i];
    if (positions.find(current_node) == positions.end()) {
      positions[current_node] = i;
    }
  }

  // Now check each delivery against the positions
  for (const auto& delivery : deliveries) {
    int pickup_pos = positions[delivery.pickUp];
    int dropoff_pos = positions[delivery.dropOff];

    // If either the pickup or dropoff is missing from the route, it's illegal
    if (dropoff_pos <= pickup_pos) {
      if (delivery.pickUp != delivery.dropOff) {
        return false;
      }
    }
  }

  return true;
}

std::vector<CourierSubPath> travelingCourier(
    const float turn_penalty, const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots) {
  // Start the master clock
  auto start_time = std::chrono::high_resolution_clock::now();

  // Build the travel cache
  buildTravelCache(deliveries, depots, turn_penalty);

  // Try a greedy route from each depot
  std::vector<IntersectionIdx> best_route_sequence;
  double best_route_time = std::numeric_limits<double>::infinity();

  // Generate a route starting from each depot, and keep the best one
  for (IntersectionIdx start_depot : depots) {
    std::vector<IntersectionIdx> current_route =
        generateGreedyRoute(start_depot, deliveries, depots);

    // Check if we got a valid route back before comparing times
    if (!current_route.empty()) {
      double current_time = calculateRouteTime(current_route);
      if (current_time < best_route_time) {
        best_route_time = current_time;
        best_route_sequence = current_route;
      }
    }
  }

  std::srand(
      8675309);  // Seed the random number generator so bugs are reproducible

  double time_limit = 48.0;
  if (deliveries.size() <= 5) {
    time_limit = 0.5;
  } else if (deliveries.size() <= 30) {
    time_limit = 5.0;
  }

  while (true) {
    // Check the clock
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                               current_time - start_time)
                               .count();

    // If we've hit our time limit, stop iterating
    if (elapsed_seconds >= time_limit) {
      break;
    }

    // Clone the route so we can mess with it
    std::vector<IntersectionIdx> test_route = best_route_sequence;

    // Pick two random stops to swap.
    int max_index = test_route.size() - 2;
    if (max_index <= 1) break;  // Route is too small to swap anything

    int swap_idx1 = 1 + std::rand() % max_index;
    int swap_idx2 = 1 + std::rand() % max_index;

    // Perform the swap
    std::swap(test_route[swap_idx1], test_route[swap_idx2]);

    // Test the mutated route
    if (isLegalRoute(test_route, deliveries)) {
      double test_time = calculateRouteTime(test_route);  // O(N) fast lookup

      // If it's better than our best, keep it
      if (test_time < best_route_time) {
        best_route_sequence = test_route;
        best_route_time = test_time;
      }
    }
  }

  std::vector<CourierSubPath> final_result;

  // Return empty if no valid route was found
  if (best_route_sequence.empty()) {
    return final_result;
  }

  // Iterate through our optimized sequence, one leg at a time
  for (size_t i = 0; i < best_route_sequence.size() - 1; ++i) {
    IntersectionIdx start_poi = best_route_sequence[i];
    IntersectionIdx end_poi = best_route_sequence[i + 1];

    CourierSubPath current_subpath;

    // Log the start and end of this leg
    current_subpath.intersections = std::make_pair(start_poi, end_poi);

    // Grab the actual street driving directions from our Phase 1 Matrix
    current_subpath.subpath = travel_cache[start_poi][end_poi].path;

    // Add this leg to the final route
    final_result.push_back(current_subpath);
  }

  // Return the final route
  return final_result;
}


