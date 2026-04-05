#include "m4.h"
#include "m3.h"           // Needed for findPathBetweenIntersections
#include "m3_globals.h"  
#include <limits>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <future>
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>

//Data structures and globals 

// Replace your old Route struct with this:
struct Route {
    std::vector<int> sequence;      // Stores Action IDs (0 to 2D+1)
    std::vector<int> action_to_poi; // Maps Action ID -> Dense POI index
    double total_travel_time = 0.0;
};

// Fast mappings between physical intersections and our dense 0 to K-1 matrix indices
std::unordered_map<IntersectionIdx, int> intersection_to_poi;
std::vector<IntersectionIdx> poi_to_intersection;

//Lookup matrix: dense_cost_matrix[source_poi][dest_poi] O(1) 
std::vector<std::vector<double>> dense_cost_matrix;


//Matrix and dependency builders 

void buildCostMatrix(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, float turn_penalty) {
    // Collect all unique POIs
    std::vector<IntersectionIdx> unique_pois;
    for (const auto& dep : depots) unique_pois.push_back(dep);
    for (const auto& del : deliveries) {
        unique_pois.push_back(del.pickUp);
        unique_pois.push_back(del.dropOff);
    }
    
    // Sort and remove duplicates to create dense indexing
    std::sort(unique_pois.begin(), unique_pois.end());
    unique_pois.erase(std::unique(unique_pois.begin(), unique_pois.end()), unique_pois.end());
    
    int num_pois = unique_pois.size();
    poi_to_intersection = unique_pois;
    
    intersection_to_poi.clear();
    for (int i = 0; i < num_pois; ++i) {
        intersection_to_poi[unique_pois[i]] = i;
    }
    
    // Allocate matrix
    dense_cost_matrix.assign(num_pois, std::vector<double>(num_pois, std::numeric_limits<double>::infinity()));
    int total_intersections = getNumIntersections(); 

    // Multi-Target Dijkstra
    for (int i = 0; i < num_pois; ++i) {
        IntersectionIdx start_node = poi_to_intersection[i];
        
        std::vector<double> best_times(total_intersections, std::numeric_limits<double>::infinity());
        std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> local_wavefront;
        
        best_times[start_node] = 0.0;
        local_wavefront.push(WaveElem(start_node, NO_EDGE, 0.0));
        
        int pois_found = 0;
        
        while (!local_wavefront.empty() && pois_found < num_pois) {
            WaveElem current = local_wavefront.top();
            local_wavefront.pop();
            
            IntersectionIdx currNode = current.nodeID;
            if (current.travelTime > best_times[currNode]) continue;
            
            auto it = intersection_to_poi.find(currNode);
            if (it != intersection_to_poi.end()) {
                int target_poi_idx = it->second;
                if (dense_cost_matrix[i][target_poi_idx] == std::numeric_limits<double>::infinity()) {
                    dense_cost_matrix[i][target_poi_idx] = best_times[currNode];
                    pois_found++;
                }
            }
            
            StreetIdx currentStreet = (current.edgeID != NO_EDGE) ? segmentInfos[current.edgeID].streetID : NO_STREET;
            
            for (StreetSegmentIdx segID : intersectionSegments[currNode]) {
                const StreetSegmentInfo& info = segmentInfos[segID];
                IntersectionIdx neighbor = (info.from == currNode) ? info.to : (!info.oneWay ? info.from : NO_INTERSECTION);
                if (neighbor == NO_INTERSECTION) continue;
                
                double edgeTime = segmentTravelTimes[segID];
                double turnTime = (currentStreet != NO_STREET && currentStreet != info.streetID) ? turn_penalty : 0.0;
                double newTime = best_times[currNode] + edgeTime + turnTime;
                
                if (newTime < best_times[neighbor]) {
                    best_times[neighbor] = newTime;
                    local_wavefront.push(WaveElem(neighbor, segID, newTime));
                }
            }
        }
    }
}



//Greedy baseline generator 

Route generateGreedyRoute(IntersectionIdx start_depot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    Route route;
    int D = deliveries.size();
    
    // Setup the thread-specific mapping
    route.action_to_poi.resize(2 * D + 2);
    for (int i = 0; i < D; ++i) {
        route.action_to_poi[i] = intersection_to_poi[deliveries[i].pickUp];
        route.action_to_poi[i + D] = intersection_to_poi[deliveries[i].dropOff];
    }
    route.action_to_poi[2 * D] = intersection_to_poi[start_depot];
    // End depot will be set at the end

    route.sequence.push_back(2 * D); // Start depot action
    
    std::vector<bool> picked_up(D, false);
    std::vector<bool> dropped_off(D, false);
    int deliveries_completed = 0;
    
    int current_poi = route.action_to_poi[2 * D];
    
    while (deliveries_completed < D) {
        int best_next_action = -1;
        double shortest_time = std::numeric_limits<double>::infinity();
        
        for (int i = 0; i < D; ++i) {
            // Check valid pickups
            if (!picked_up[i]) {
                int p_poi = route.action_to_poi[i];
                if (dense_cost_matrix[current_poi][p_poi] < shortest_time) {
                    shortest_time = dense_cost_matrix[current_poi][p_poi];
                    best_next_action = i;
                }
            } 
            // Check valid dropoffs
            else if (!dropped_off[i]) {
                int d_poi = route.action_to_poi[i + D];
                if (dense_cost_matrix[current_poi][d_poi] < shortest_time) {
                    shortest_time = dense_cost_matrix[current_poi][d_poi];
                    best_next_action = i + D;
                }
            }
        }
        if (best_next_action == -1) {
            route.sequence.clear();
            route.total_travel_time = std::numeric_limits<double>::infinity();
            return route;
        }
        
        route.sequence.push_back(best_next_action);
        route.total_travel_time += shortest_time;
        current_poi = route.action_to_poi[best_next_action];
        
        // Update statuses based on the ACTION, not the physical POI
        if (best_next_action < D) picked_up[best_next_action] = true;
        else {
            dropped_off[best_next_action - D] = true;
            deliveries_completed++;
        }
    }
    
    // Find closest end depot
    int best_end_depot_poi = -1;
    double shortest_depot_time = std::numeric_limits<double>::infinity();
    for (IntersectionIdx depot : depots) {
        int depot_poi = intersection_to_poi[depot];
        if (dense_cost_matrix[current_poi][depot_poi] < shortest_depot_time) {
            shortest_depot_time = dense_cost_matrix[current_poi][depot_poi];
            best_end_depot_poi = depot_poi;
        }
    }

    if (best_end_depot_poi == -1) {
        route.sequence.clear();
        route.total_travel_time = std::numeric_limits<double>::infinity();
        return route;
    }
    
    route.action_to_poi[2 * D + 1] = best_end_depot_poi;
    route.sequence.push_back(2 * D + 1); // End depot action
    route.total_travel_time += shortest_depot_time;
    
    return route;
}

//Simulated annealing 
bool isLegalFast(const std::vector<int>& seq, int D) {
    std::vector<bool> picked_up(D, false);
    for (int action : seq) {
        if (action < D) { // It's a pickup
            picked_up[action] = true;
        } else if (action < 2 * D) { // It's a dropoff
            if (!picked_up[action - D]) return false; // Dropped off before pickup!
        }
    }
    return true;
}

double calculateDeltaCost(const Route& route, int idxA, int idxB) {
    if (idxA > idxB) std::swap(idxA, idxB);
    
    const auto& seq = route.sequence;
    const auto& a2p = route.action_to_poi;
    
    int poi_A = a2p[seq[idxA]];
    int poi_B = a2p[seq[idxB]];
    int poi_prevA = a2p[seq[idxA - 1]];
    int poi_nextA = a2p[seq[idxA + 1]];
    int poi_prevB = a2p[seq[idxB - 1]];
    int poi_nextB = a2p[seq[idxB + 1]];

    if (idxA + 1 == idxB) {
        double old_cost = dense_cost_matrix[poi_prevA][poi_A] + dense_cost_matrix[poi_A][poi_B] + dense_cost_matrix[poi_B][poi_nextB];
        double new_cost = dense_cost_matrix[poi_prevA][poi_B] + dense_cost_matrix[poi_B][poi_A] + dense_cost_matrix[poi_A][poi_nextB];
        return new_cost - old_cost;
    } else {
        double old_cost = dense_cost_matrix[poi_prevA][poi_A] + dense_cost_matrix[poi_A][poi_nextA] + dense_cost_matrix[poi_prevB][poi_B] + dense_cost_matrix[poi_B][poi_nextB];
        double new_cost = dense_cost_matrix[poi_prevA][poi_B] + dense_cost_matrix[poi_B][poi_nextA] + dense_cost_matrix[poi_prevB][poi_A] + dense_cost_matrix[poi_A][poi_nextB];
        return new_cost - old_cost;
    }
}

Route simulatedAnnealing(Route initial_route, double time_limit_seconds) {
    auto sa_start = std::chrono::high_resolution_clock::now();
    Route current = initial_route;
    Route best = initial_route;
    
    uint32_t seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 rng(seed);
    double temp = 5000.0;
    double cooling_rate = 0.99999; 
    
    int iterations = 0;
    int seq_size = current.sequence.size();
    
    // If route is just depot -> depot, no optimization needed
    if (seq_size <= 3) return best; 
    
    // Calculate D (number of deliveries) from the sequence size
    int D = (seq_size - 2) / 2;
    
    std::uniform_int_distribution<int> dist(1, seq_size - 2);
    std::uniform_real_distribution<double> prob(0.0, 1.0);

    while (true) {
        if (++iterations % 1000 == 0) {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - sa_start;
            if (elapsed.count() > time_limit_seconds) break;
        }
        
        int idxA = dist(rng);
        int idxB = dist(rng);
        if (idxA == idxB) continue;
        
        std::swap(current.sequence[idxA], current.sequence[idxB]);
        
        //Pass D to isLegalFast
        if (!isLegalFast(current.sequence, D)) {
            std::swap(current.sequence[idxA], current.sequence[idxB]); // Revert
            continue;
        }
        
        std::swap(current.sequence[idxA], current.sequence[idxB]); // Unswap to calc delta safely
        
        //Pass the 'current' Route object instead of current.sequence
        double delta = calculateDeltaCost(current, idxA, idxB);
        
        if (delta < 0 || exp(-delta / temp) > prob(rng)) {
            std::swap(current.sequence[idxA], current.sequence[idxB]); // Accept
            current.total_travel_time += delta;
            if (current.total_travel_time < best.total_travel_time) best = current;
        }
        
        temp *= cooling_rate;
    }
    return best;
}

//Multithreading 
std::vector<CourierSubPath> travelingCourier(
    const float turn_penalty,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots) {
    
    auto global_start = std::chrono::high_resolution_clock::now();

    //Pre-computations
    buildCostMatrix(deliveries, depots, turn_penalty); 

    //ultithreading Setup
    int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 4;

    double max_time = 48.0;
    if (deliveries.size() <= 5) {
        max_time = 0.5;
    } else if (deliveries.size() <= 30) {
        max_time = 5.0;
    }
    
    std::vector<std::future<Route>> futures;
    
    for (int i = 0; i < num_threads; ++i) {
      
        futures.push_back(std::async(std::launch::async, [i, &deliveries, &depots, global_start, max_time]() {
            IntersectionIdx start_depot = depots[i % depots.size()];
            Route base = generateGreedyRoute(start_depot, deliveries, depots);
            
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - global_start;
            double time_left_for_sa = max_time - elapsed.count(); 
            
            if (time_left_for_sa <= 0) return base; // Fallback if matrix took too long
            return simulatedAnnealing(base, time_left_for_sa);
        }));
    }
    
    //Find Absolute Best Route
    Route absolute_best;
    absolute_best.total_travel_time = std::numeric_limits<double>::infinity();
    for (auto& f : futures) {
        Route r = f.get();
        if (r.total_travel_time < absolute_best.total_travel_time) {
            absolute_best = r;
        }
    }
    
    //Reconstruct Street Paths using M3 logic
    std::vector<CourierSubPath> final_route;
    if (absolute_best.sequence.empty() || absolute_best.total_travel_time == std::numeric_limits<double>::infinity()) {
        return final_route; // Return empty if no valid route found
    }
    
    for (size_t i = 0; i < absolute_best.sequence.size() - 1; ++i) {
        int action_start = absolute_best.sequence[i];
        int action_end = absolute_best.sequence[i+1];
        
        IntersectionIdx start_intersection = poi_to_intersection[absolute_best.action_to_poi[action_start]];
        IntersectionIdx end_intersection = poi_to_intersection[absolute_best.action_to_poi[action_end]];
        
        CourierSubPath sub;
        sub.intersections = {start_intersection, end_intersection};
        
        if (start_intersection == end_intersection) {
            // Do not 'continue'! Push an empty path so the autotester knows we stopped here.
            sub.subpath = std::vector<StreetSegmentIdx>(); 
        } else {
            sub.subpath = findPathBetweenIntersections(turn_penalty, {start_intersection, end_intersection});
        }
        
        final_route.push_back(sub);
    }
    
    return final_route;
}