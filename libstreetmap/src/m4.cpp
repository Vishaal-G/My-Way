#include "m4.h"
#include "m3.h"
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

// =============================================================================
// DATA STRUCTURES
// =============================================================================
struct Route {
    std::vector<int> sequence;      // Action IDs
    std::vector<int> action_to_poi; // Action ID -> Dense Matrix Index
    double total_travel_time = 0.0;
};

std::unordered_map<IntersectionIdx, int> intersection_to_poi;
std::vector<IntersectionIdx> poi_to_intersection;
std::vector<std::vector<double>> dense_cost_matrix;

// =============================================================================
// PHASE 1: PARALLEL MATRIX BUILDER (From Tutorial Slide 29/40)
// =============================================================================
void buildCostMatrix(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, float turn_penalty) {
    std::vector<IntersectionIdx> unique_pois;
    for (auto d : deliveries) { unique_pois.push_back(d.pickUp); unique_pois.push_back(d.dropOff); }
    for (auto d : depots) unique_pois.push_back(d);
    
    std::sort(unique_pois.begin(), unique_pois.end());
    unique_pois.erase(std::unique(unique_pois.begin(), unique_pois.end()), unique_pois.end());
    
    poi_to_intersection = unique_pois;
    intersection_to_poi.clear();
    for(size_t i=0; i<poi_to_intersection.size(); ++i) intersection_to_poi[poi_to_intersection[i]] = i;

    int num_pois = poi_to_intersection.size();
    dense_cost_matrix.assign(num_pois, std::vector<double>(num_pois, std::numeric_limits<double>::infinity()));
    int total_nodes = getNumIntersections();

    // Unlock massive speedup with OpenMP [cite: 269, 463]
    #pragma omp parallel for
    for (int i = 0; i < num_pois; ++i) {
        std::vector<double> best_times(total_nodes, std::numeric_limits<double>::infinity());
        std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> local_wavefront;
        
        IntersectionIdx start_node = poi_to_intersection[i];
        best_times[start_node] = 0.0;
        local_wavefront.push(WaveElem(start_node, NO_EDGE, 0.0));

        int found_count = 0;
        while(!local_wavefront.empty() && found_count < num_pois) {
            WaveElem current = local_wavefront.top();
            local_wavefront.pop();
            if (current.travelTime > best_times[current.nodeID]) continue;

            auto it = intersection_to_poi.find(current.nodeID);
            if (it != intersection_to_poi.end()) {
                if (dense_cost_matrix[i][it->second] == std::numeric_limits<double>::infinity()) {
                    dense_cost_matrix[i][it->second] = current.travelTime;
                    found_count++;
                }
            }

            StreetIdx currStreet = (current.edgeID != NO_EDGE) ? segmentInfos[current.edgeID].streetID : NO_STREET;
            for (auto segID : intersectionSegments[current.nodeID]) {
                const auto& info = segmentInfos[segID];
                IntersectionIdx neighbor = (info.from == current.nodeID) ? info.to : (!info.oneWay ? info.from : NO_INTERSECTION);
                if (neighbor == NO_INTERSECTION) continue;

                double newTime = current.travelTime + segmentTravelTimes[segID] + 
                                ((currStreet != NO_STREET && currStreet != info.streetID) ? turn_penalty : 0.0);
                
                if (newTime < best_times[neighbor]) {
                    best_times[neighbor] = newTime;
                    local_wavefront.push(WaveElem(neighbor, segID, newTime));
                }
            }
        }
    }
}

// =============================================================================
// PHASE 2: MULTI-START GREEDY BASELINE (From Lecture 20)
// =============================================================================
Route generateGreedyRoute(IntersectionIdx start_depot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    Route route;
    int D = deliveries.size();
    route.action_to_poi.resize(2*D + 2);
    for(int i=0; i<D; ++i) { 
        route.action_to_poi[i] = intersection_to_poi[deliveries[i].pickUp]; 
        route.action_to_poi[i+D] = intersection_to_poi[deliveries[i].dropOff]; 
    }
    route.action_to_poi[2*D] = intersection_to_poi[start_depot];
    route.sequence.push_back(2*D);

    std::vector<bool> picked(D, false), dropped(D, false);
    int curr_poi = route.action_to_poi[2*D], done = 0;

    while (done < D) {
        int next_act = -1; double best_t = std::numeric_limits<double>::infinity();
        for (int i=0; i<D; ++i) {
            if (!picked[i]) {
                if (dense_cost_matrix[curr_poi][route.action_to_poi[i]] < best_t) {
                    best_t = dense_cost_matrix[curr_poi][route.action_to_poi[i]]; next_act = i;
                }
            } else if (!dropped[i]) {
                if (dense_cost_matrix[curr_poi][route.action_to_poi[i+D]] < best_t) {
                    best_t = dense_cost_matrix[curr_poi][route.action_to_poi[i+D]]; next_act = i+D;
                }
            }
        }
        if (next_act == -1) { route.sequence.clear(); return route; }
        route.sequence.push_back(next_act);
        route.total_travel_time += best_t;
        curr_poi = route.action_to_poi[next_act];
        if (next_act < D) picked[next_act] = true; else { dropped[next_act-D] = true; done++; }
    }

    int end_poi = -1; double best_depot_t = std::numeric_limits<double>::infinity();
    for (auto d : depots) {
        int d_poi = intersection_to_poi[d];
        if (dense_cost_matrix[curr_poi][d_poi] < best_depot_t) {
            best_depot_t = dense_cost_matrix[curr_poi][d_poi]; end_poi = d_poi;
        }
    }
    if (end_poi == -1) { route.sequence.clear(); return route; }
    route.action_to_poi[2*D+1] = end_poi;
    route.sequence.push_back(2*D+1);
    route.total_travel_time += best_depot_t;
    return route;
}

// =============================================================================
// PHASE 3: CALIBRATED SIMULATED ANNEALING (From Lecture 22)
// =============================================================================
bool isLegalFast(const std::vector<int>& seq, int D) {
    std::vector<bool> p(D, false);
    for (int a : seq) {
        if (a < D) p[a] = true;
        else if (a < 2*D && !p[a-D]) return false;
    }
    return true;
}

double calculateDeltaCost(const Route& r, int idxA, int idxB) {
    if (idxA > idxB) std::swap(idxA, idxB);
    int A = r.sequence[idxA], B = r.sequence[idxB];
    int pA = r.action_to_poi[r.sequence[idxA-1]], nA = r.action_to_poi[r.sequence[idxA+1]];
    int pB = r.action_to_poi[r.sequence[idxB-1]], nB = r.action_to_poi[r.sequence[idxB+1]];
    int aPOI = r.action_to_poi[A], bPOI = r.action_to_poi[B];

    if (idxA + 1 == idxB) {
        return (dense_cost_matrix[pA][bPOI] + dense_cost_matrix[bPOI][aPOI] + dense_cost_matrix[aPOI][nB]) -
               (dense_cost_matrix[pA][aPOI] + dense_cost_matrix[aPOI][bPOI] + dense_cost_matrix[bPOI][nB]);
    }
    return (dense_cost_matrix[pA][bPOI] + dense_cost_matrix[bPOI][nA] + dense_cost_matrix[pB][aPOI] + dense_cost_matrix[aPOI][nB]) -
           (dense_cost_matrix[pA][aPOI] + dense_cost_matrix[aPOI][nA] + dense_cost_matrix[pB][bPOI] + dense_cost_matrix[bPOI][nB]);
}

Route simulatedAnnealing(Route init, double time_limit, int tid) {
    auto start = std::chrono::high_resolution_clock::now();
    Route curr = init, best = init;
    std::mt19937 rng(std::chrono::system_clock::now().time_since_epoch().count() + tid);
    // OPTIMIZED TEMP: Respect the baseline 
    double temp = 20.0, cooling = 0.99999;
    int D = (curr.sequence.size()-2)/2, iters = 0;

    while (true) {
        if (++iters % 1000 == 0) {
            if (std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-start).count() > time_limit) break;
        }
        std::uniform_int_distribution<int> dist(1, curr.sequence.size()-2);
        int iA = dist(rng), iB = dist(rng);
        if (iA == iB) continue;

        std::swap(curr.sequence[iA], curr.sequence[iB]);
        if (!isLegalFast(curr.sequence, D)) { std::swap(curr.sequence[iA], curr.sequence[iB]); continue; }
        
        std::swap(curr.sequence[iA], curr.sequence[iB]); // Unswap for delta
        double delta = calculateDeltaCost(curr, iA, iB);
        if (delta < 0 || std::exp(-delta/temp) > std::uniform_real_distribution<double>(0,1)(rng)) {
            std::swap(curr.sequence[iA], curr.sequence[iB]); // Accept
            curr.total_travel_time += delta;
            if (curr.total_travel_time < best.total_travel_time) best = curr;
        }
        temp *= cooling;
    }
    return best;
}

// =============================================================================
// MAIN ORCHESTRATOR
// =============================================================================
std::vector<CourierSubPath> travelingCourier(const float turn_penalty, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    auto g_start = std::chrono::high_resolution_clock::now();
    buildCostMatrix(deliveries, depots, turn_penalty);

    std::vector<Route> greedy_options;
    for (auto d : depots) {
        Route r = generateGreedyRoute(d, deliveries, depots);
        if (!r.sequence.empty()) greedy_options.push_back(r);
    }
    std::sort(greedy_options.begin(), greedy_options.end(), [](const Route& a, const Route& b){ return a.total_travel_time < b.total_travel_time; });

    int threads = std::thread::hardware_concurrency();
    double max_t = (deliveries.size() <= 5) ? 0.5 : ((deliveries.size() <= 30) ? 5.0 : 48.0);
    std::vector<std::future<Route>> futures;

    for (int i=0; i<threads; ++i) {
        Route base = greedy_options.empty() ? Route() : greedy_options[i % greedy_options.size()];
        futures.push_back(std::async(std::launch::async, [i, base, g_start, max_t](){
            if (base.sequence.empty()) return base;
            double remain = max_t - std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-g_start).count();
            return (remain <= 0) ? base : simulatedAnnealing(base, remain, i);
        }));
    }

    Route abs_best; abs_best.total_travel_time = std::numeric_limits<double>::infinity();
    for (auto& f : futures) { Route r = f.get(); if (r.total_travel_time < abs_best.total_travel_time) abs_best = r; }

    std::vector<CourierSubPath> result;
    if (abs_best.sequence.empty()) return result;

    for (size_t i=0; i<abs_best.sequence.size()-1; ++i) {
        IntersectionIdx s = poi_to_intersection[abs_best.action_to_poi[abs_best.sequence[i]]];
        IntersectionIdx e = poi_to_intersection[abs_best.action_to_poi[abs_best.sequence[i+1]]];
        CourierSubPath sub; sub.intersections = {s, e};
        sub.subpath = (s == e) ? std::vector<StreetSegmentIdx>() : findPathBetweenIntersections(turn_penalty, {s, e});
        result.push_back(sub);
    }
    return result;
}