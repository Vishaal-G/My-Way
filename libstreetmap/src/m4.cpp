#include "m4.h"
#include "m3_globals.h"
#include "m3.h"
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <queue>
#include <chrono> 

extern std::vector<IntersectionIdx> touchedDriveNodes;
extern std::vector<Node> nodes; // Or whatever you named your Node struct/vector
extern std::vector<StreetSegmentInfo> segmentInfos;
extern std::vector<double> segmentTravelTimes;
extern std::vector<std::vector<StreetSegmentIdx>> intersectionSegments;

// --- Data Structures ---
struct POI_Data {
    double travel_time;
    std::vector<StreetSegmentIdx> path;
};

// Global Cache: cache[source_intersection][dest_intersection]
std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, POI_Data>> travel_cache;

static std::vector<bool> is_poi;

// Helper to track cargo
struct DeliveryState {
    std::vector<bool> picked_up;
    std::vector<bool> dropped_off;
    int total_deliveries;
    int deliveries_completed = 0;
};

std::vector<IntersectionIdx> getUniquePOIs(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    std::vector<IntersectionIdx> all_pois;
    
    // Add all pickups and dropoffs [cite: 204]
    for (const auto& delivery : deliveries) {
        all_pois.push_back(delivery.pickUp);
        all_pois.push_back(delivery.dropOff);
    }
    
    // Add all depots [cite: 206]
    for (IntersectionIdx depot : depots) {
        all_pois.push_back(depot);
    }
    
    // Remove duplicates (e.g., if two packages are picked up at the same place)
    std::sort(all_pois.begin(), all_pois.end());
    all_pois.erase(std::unique(all_pois.begin(), all_pois.end()), all_pois.end());
    
    return all_pois;
}

// You will need to bring over your WaveElem, CompareWaveElem, nodes, and touchedNodes from M3!

std::unordered_map<IntersectionIdx, POI_Data> multiTargetDijkstra(
    IntersectionIdx start_node, 
    const std::vector<IntersectionIdx>& all_pois, 
    const float turn_penalty) { // [cite: 203]

    std::unordered_map<IntersectionIdx, POI_Data> found_paths;
    int targets_found = 0;
    int total_targets = all_pois.size();

    // 1. Dirty Reset (from M3)
    for (IntersectionIdx idx : touchedDriveNodes) {
        nodes[idx].bestTime = std::numeric_limits<double>::infinity();
        nodes[idx].reachingEdge = NO_EDGE;
        nodes[idx].visited = false;
    }
    touchedDriveNodes.clear();
    clearWavefront();

    // 2. Initialize Start Node
    nodes[start_node].bestTime = 0.0;
    touchedDriveNodes.push_back(start_node);
    wavefront.push(WaveElem(start_node, NO_EDGE, 0.0));

    // 3. The Dijkstra Expansion Loop
    while (!wavefront.empty() && targets_found < total_targets) {
        WaveElem current = wavefront.top();
        wavefront.pop();

        IntersectionIdx currNode = current.nodeID;
        if (nodes[currNode].visited) continue;
        nodes[currNode].visited = true;

        // --- THE M4 MAGIC ---
        // If the node we just popped is a POI, we found a target!
        if (is_poi[currNode]) {
            targets_found++;
            
            // Reconstruct the path immediately
            std::vector<StreetSegmentIdx> path;
            IntersectionIdx tracer = currNode;
            while (tracer != start_node) {
                StreetSegmentIdx edge = nodes[tracer].reachingEdge;
                path.push_back(edge);
                tracer = segmentInfos[edge].from == tracer ? segmentInfos[edge].to : segmentInfos[edge].from; // Careful with 1-way tracing
            }
            std::reverse(path.begin(), path.end());
            
            // Save to our results map
            found_paths[currNode] = {nodes[currNode].bestTime, path};
            
            // NOTE: Do not 'break' or 'continue' here! We must keep expanding past this POI to find the others.
        }
        // --------------------

        // 4. Check Neighbors (Standard M3 Logic)
        StreetIdx currentStreet = (current.edgeID != NO_EDGE) ? segmentInfos[current.edgeID].streetID : NO_STREET;

        for (StreetSegmentIdx segID : intersectionSegments[currNode]) {
            const StreetSegmentInfo& info = segmentInfos[segID];
            
            IntersectionIdx neighbor = NO_INTERSECTION;
            if (info.from == currNode) neighbor = info.to;
            else if (!info.oneWay) neighbor = info.from;
            else continue; // Obey one-way streets

            double edgeTime = segmentTravelTimes[segID];
            double turnTime = (currentStreet != NO_STREET && currentStreet != info.streetID) ? turn_penalty : 0.0;
            double newTime = nodes[currNode].bestTime + edgeTime + turnTime;

            if (newTime < nodes[neighbor].bestTime) {
                if (nodes[neighbor].bestTime == std::numeric_limits<double>::infinity()) {
                    touchedDriveNodes.push_back(neighbor);
                }
                nodes[neighbor].bestTime = newTime;
                nodes[neighbor].reachingEdge = segID;
                
                // NO HEURISTIC. Pure Dijkstra time.
                wavefront.push(WaveElem(neighbor, segID, newTime));
            }
        }
    }

    return found_paths;
}

void buildTravelCache(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, const float turn_penalty) {
    travel_cache.clear();
    
    // 1. Get unique POIs
    std::vector<IntersectionIdx> all_pois = getUniquePOIs(deliveries, depots);
    
    // 2. Setup the fast O(1) POI checker
    int num_intersections = getNumIntersections();
    is_poi.assign(num_intersections, false);
    for (IntersectionIdx poi : all_pois) {
        is_poi[poi] = true;
    }

    // 3. Flood the map from every POI
    // (Captain's Tip: You can put `#pragma omp parallel for` right above this loop later to make it 8x faster!)
    for (int i = 0; i < all_pois.size(); i++) {
        IntersectionIdx start_poi = all_pois[i];
        travel_cache[start_poi] = multiTargetDijkstra(start_poi, all_pois, turn_penalty);
    }
}

double calculateRouteTime(const std::vector<IntersectionIdx>& route_sequence) {
    double total_time = 0.0;
    for (size_t i = 0; i < route_sequence.size() - 1; ++i) {
        IntersectionIdx from = route_sequence[i];
        IntersectionIdx to = route_sequence[i+1];
        total_time += travel_cache[from][to].travel_time;
    }
    return total_time;
}

// Helper function to calculate a route starting from a specific depot
// --- PATCHED Phase 2: Greedy Route Builder ---
std::vector<IntersectionIdx> generateGreedyRoute(
    IntersectionIdx start_depot, 
    const std::vector<DeliveryInf>& deliveries, 
    const std::vector<IntersectionIdx>& depots) {
    
    std::vector<IntersectionIdx> route;
    route.push_back(start_depot);
    
    int N = deliveries.size();
    std::vector<bool> picked_up(N, false);
    std::vector<bool> dropped_off(N, false);
    int deliveries_completed = 0;
    
    IntersectionIdx current_node = start_depot;
    
    while (deliveries_completed < N) {
        double best_time = std::numeric_limits<double>::infinity();
        IntersectionIdx best_next_node = NO_INTERSECTION;
        int best_delivery_index = -1;
        bool is_pickup_move = false;
        
        for (int i = 0; i < N; ++i) {
            if (!picked_up[i]) {
                IntersectionIdx candidate = deliveries[i].pickUp;
                // BUG FIX 1: Safely check if a path actually exists in the cache
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
            }
            else if (picked_up[i] && !dropped_off[i]) {
                IntersectionIdx candidate = deliveries[i].dropOff;
                // BUG FIX 1: Safely check cache
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
        
        // BUG FIX 2: If we couldn't find ANY legal moves, the map is impossible!
        if (best_delivery_index == -1) {
            return std::vector<IntersectionIdx>(); // Return an empty, failed route
        }
        
        if (is_pickup_move) picked_up[best_delivery_index] = true;
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
    
    // BUG FIX 2.5: Ensure we actually found a depot to park at
    if (best_end_depot == NO_INTERSECTION) return std::vector<IntersectionIdx>();
    
    if (best_end_depot != current_node) {
        route.push_back(best_end_depot);
    }
    
    return route;
}

bool isLegalRoute(const std::vector<IntersectionIdx>& test_route, const std::vector<DeliveryInf>& deliveries) {
    // To check legality fast, we find the index (position) of every node in the route.
    std::unordered_map<IntersectionIdx, int> positions;
    
    for (int i = 0; i < test_route.size(); ++i) {
        IntersectionIdx current_node = test_route[i];
        // If an intersection is visited multiple times, we only care about the FIRST time 
        // we arrive, because that is when the pickup/dropoff automatically triggers.
        if (positions.find(current_node) == positions.end()) {
            positions[current_node] = i;
        }
    }
    
    // Now verify every single package constraint
    for (const auto& delivery : deliveries) {
        int pickup_pos = positions[delivery.pickUp];
        int dropoff_pos = positions[delivery.dropOff];
        
        // If the drop-off happens before (or at the exact same time as) the pickup, it's illegal!
        if (dropoff_pos <= pickup_pos) {
            // Note: If pickUp and dropOff are the exact same intersection, 
            // the assignment allows it, but our greedy logic handles that. 
            // For distinct intersections, dropoff_pos MUST be > pickup_pos.
            if (delivery.pickUp != delivery.dropOff) {
                return false; 
            }
        }
    }
    
    return true;
}

std::vector<CourierSubPath> travelingCourier(
    const float turn_penalty,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots) {
    
    // 0. Start the master clock
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 1. Build the Matrix (Phase 1)
    buildTravelCache(deliveries, depots, turn_penalty);
    
    // 2. Multi-Start Greedy Construction (Phase 2)
 
    std::vector<IntersectionIdx> best_route_sequence;
    double best_route_time = std::numeric_limits<double>::infinity();
    
    for (IntersectionIdx start_depot : depots) {
        std::vector<IntersectionIdx> current_route = generateGreedyRoute(start_depot, deliveries, depots);
        
        // BUG FIX 3: Only calculate time if the route actually succeeded!
        if (!current_route.empty()) {
            double current_time = calculateRouteTime(current_route);
            if (current_time < best_route_time) {
                best_route_time = current_time;
                best_route_sequence = current_route;
            }
        }
    }
    
    // --- PHASE 3 WILL GO HERE ---
    // ... [End of Phase 2 logic. You now have best_route_sequence and best_route_time] ...

    // --- PHASE 3: Iterative Local Search ---
    
    std::srand(8675309); // Seed the random number generator so bugs are reproducible
    
    double time_limit = 48.0; 
    if (deliveries.size() <= 5) {
        time_limit = 0.5;  // Half a second is plenty for tiny tests
    } else if (deliveries.size() <= 30) {
        time_limit = 5.0;  // 5 seconds for medium tests
    }

    while (true) {
        // 1. Check the Wall Clock
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        // If we hit 48 seconds, completely abort the optimization so we don't fail the autotester
        if (elapsed_seconds >= time_limit) {
            break; 
        }

        // 2. Clone the route so we can mess with it
        std::vector<IntersectionIdx> test_route = best_route_sequence;
        
        // 3. Pick two random stops to swap. 
        // WARNING: Do NOT swap index 0 (Start Depot) or the last index (End Depot)!
        int max_index = test_route.size() - 2; 
        if (max_index <= 1) break; // Route is too small to swap anything

        int swap_idx1 = 1 + std::rand() % max_index;
        int swap_idx2 = 1 + std::rand() % max_index;
        
        // Perform the swap
        std::swap(test_route[swap_idx1], test_route[swap_idx2]);
        
        // 4. Test the mutated route
        if (isLegalRoute(test_route, deliveries)) {
            double test_time = calculateRouteTime(test_route); // O(N) fast lookup
            
            // 5. If it's faster, keep it!
            if (test_time < best_route_time) {
                best_route_sequence = test_route;
                best_route_time = test_time;
            }
        }
    }
    // --- PHASE 4 WILL GO HERE ---
    // ... [End of Phase 3 loop] ...

    // --- PHASE 4: Path Assembly ---
    std::vector<CourierSubPath> final_result;

    // Safety Check: If no route was found (e.g., impossible map), the spec requires returning an empty vector
    if (best_route_sequence.empty()) {
        return final_result; 
    }

    // Iterate through our optimized sequence, one leg at a time
    for (size_t i = 0; i < best_route_sequence.size() - 1; ++i) {
        IntersectionIdx start_poi = best_route_sequence[i];
        IntersectionIdx end_poi = best_route_sequence[i + 1];

        CourierSubPath current_subpath;
        
        // 1. Log the start and end of this leg
        current_subpath.intersections = std::make_pair(start_poi, end_poi);
        
        // 2. Grab the actual street driving directions from our Phase 1 Matrix
        current_subpath.subpath = travel_cache[start_poi][end_poi].path;

        // 3. Add this leg to the final route
        final_result.push_back(current_subpath);
    }

    // Ship it to the autotester!
    return final_result;
}