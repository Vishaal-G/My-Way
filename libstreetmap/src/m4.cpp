#include "m4.h"
#include "m3.h"
#include "m3_globals.h"
#include <limits>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <future>
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>

// ============================================================
// DATA STRUCTURES
// ============================================================

struct CacheEntry {
    double travel_time = std::numeric_limits<double>::infinity();
    std::vector<StreetSegmentIdx> path;
};

struct Route {
    std::vector<int> sequence;       // sequence of action IDs
    std::vector<int> action_to_poi;  // action ID -> dense matrix index
    double total_travel_time = std::numeric_limits<double>::infinity();
};

// Global matrix and POI mappings
static std::unordered_map<IntersectionIdx, int> intersection_to_poi;
static std::vector<IntersectionIdx>              poi_to_intersection;
static std::vector<std::vector<CacheEntry>>      cost_matrix;

// ============================================================
// HELPERS
// ============================================================

static inline double edgeCost(int from_poi, int to_poi) {
    return cost_matrix[from_poi][to_poi].travel_time;
}

static double recalcRouteCost(const Route& r) {
    double total = 0.0;
    for (size_t i = 0; i + 1 < r.sequence.size(); ++i) {
        int a = r.action_to_poi[r.sequence[i]];
        int b = r.action_to_poi[r.sequence[i + 1]];
        total += edgeCost(a, b);
    }
    return total;
}

// Fast legality check: every dropoff must come after its pickup
static bool isLegalFast(const std::vector<int>& seq, int D) {
    std::vector<bool> picked(D, false);
    for (int a : seq) {
        if (a < D) {
            picked[a] = true;
        } else if (a < 2 * D) {
            if (!picked[a - D]) return false;
        }
    }
    return true;
}

// PHASE 1: PARALLEL MATRIX BUILDER (fropm tutorial) 

void buildCostMatrix(const std::vector<DeliveryInf>& deliveries,
                     const std::vector<IntersectionIdx>& depots,
                     float turn_penalty) {

    // Collect unique POIs
    std::vector<IntersectionIdx> unique_pois;
    unique_pois.reserve(2 * deliveries.size() + depots.size());
    for (auto& d : deliveries) {
        unique_pois.push_back(d.pickUp);
        unique_pois.push_back(d.dropOff);
    }
    for (auto dep : depots) unique_pois.push_back(dep);

    std::sort(unique_pois.begin(), unique_pois.end());
    unique_pois.erase(std::unique(unique_pois.begin(), unique_pois.end()), unique_pois.end());

    poi_to_intersection = unique_pois;
    intersection_to_poi.clear();
    for (size_t i = 0; i < poi_to_intersection.size(); ++i)
        intersection_to_poi[poi_to_intersection[i]] = (int)i;

    int num_pois    = (int)poi_to_intersection.size();
    int total_nodes = getNumIntersections();
    const double INF = std::numeric_limits<double>::infinity();

    cost_matrix.assign(num_pois, std::vector<CacheEntry>(num_pois));

    // Run one Dijkstra per POI in parallel, storing paths
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < num_pois; ++i) {
        IntersectionIdx start_node = poi_to_intersection[i];

        std::vector<double>          best_times(total_nodes, INF);
        std::vector<StreetSegmentIdx> prev_edge(total_nodes, NO_EDGE);
        std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> pq;

        best_times[start_node] = 0.0;
        pq.push(WaveElem(start_node, NO_EDGE, 0.0));

        int found_count = 0;

        while (!pq.empty() && found_count < num_pois) {
            WaveElem cur = pq.top(); pq.pop();
            if (cur.travelTime > best_times[cur.nodeID]) continue;

            // Check if this node is a POI target
            auto it = intersection_to_poi.find(cur.nodeID);
            if (it != intersection_to_poi.end()) {
                int j = it->second;
                if (cost_matrix[i][j].travel_time == INF) {
                    cost_matrix[i][j].travel_time = cur.travelTime;

                    // Reconstruct path from start_node -> cur.nodeID
                    std::vector<StreetSegmentIdx> path;
                    IntersectionIdx tracer = cur.nodeID;
                    while (tracer != start_node) {
                        StreetSegmentIdx e = prev_edge[tracer];
                        if (e == NO_EDGE) break;
                        path.push_back(e);
                        const auto& info = segmentInfos[e];
                        tracer = (info.from == tracer) ? info.to : info.from;
                    }
                    std::reverse(path.begin(), path.end());
                    cost_matrix[i][j].path = std::move(path);
                    ++found_count;
                }
            }

            StreetIdx currStreet = (cur.edgeID != NO_EDGE)
                                       ? segmentInfos[cur.edgeID].streetID
                                       : NO_STREET;

            for (StreetSegmentIdx segID : intersectionSegments[cur.nodeID]) {
                const auto& info = segmentInfos[segID];
                IntersectionIdx neighbor;
                if (info.from == cur.nodeID)        neighbor = info.to;
                else if (!info.oneWay)              neighbor = info.from;
                else                                continue;

                double turn   = (currStreet != NO_STREET && currStreet != info.streetID)
                                    ? turn_penalty : 0.0;
                double newTime = cur.travelTime + segmentTravelTimes[segID] + turn;

                if (newTime < best_times[neighbor]) {
                    best_times[neighbor] = newTime;
                    prev_edge[neighbor]  = segID;
                    pq.push(WaveElem(neighbor, segID, newTime));
                }
            }
        }
    }
}

//MULTI-START GREEDY BASELINE (From Lecture 20)

Route generateGreedyRoute(IntersectionIdx start_depot,
                          const std::vector<DeliveryInf>& deliveries,
                          const std::vector<IntersectionIdx>& depots) {
    Route route;
    int D = (int)deliveries.size();
    // Actions: 0..D-1 = pickups, D..2D-1 = dropoffs, 2D = start depot, 2D+1 = end depot
    route.action_to_poi.resize(2 * D + 2);
    for (int i = 0; i < D; ++i) {
        route.action_to_poi[i]     = intersection_to_poi[deliveries[i].pickUp];
        route.action_to_poi[i + D] = intersection_to_poi[deliveries[i].dropOff];
    }
    route.action_to_poi[2 * D] = intersection_to_poi[start_depot];
    route.sequence.reserve(2 * D + 2);
    route.sequence.push_back(2 * D);

    std::vector<bool> picked(D, false), dropped(D, false);
    int curr_poi = route.action_to_poi[2 * D];
    int done = 0;
    double total_time = 0.0;

    while (done < D) {
        int    best_act = -1;
        double best_t   = std::numeric_limits<double>::infinity();

        for (int i = 0; i < D; ++i) {
            if (!picked[i]) {
                double t = edgeCost(curr_poi, route.action_to_poi[i]);
                if (t < best_t) { best_t = t; best_act = i; }
            } else if (!dropped[i]) {
                double t = edgeCost(curr_poi, route.action_to_poi[i + D]);
                if (t < best_t) { best_t = t; best_act = i + D; }
            }
        }

        if (best_act == -1) { route.sequence.clear(); return route; }

        route.sequence.push_back(best_act);
        total_time += best_t;
        curr_poi = route.action_to_poi[best_act];

        if (best_act < D) picked[best_act] = true;
        else { dropped[best_act - D] = true; ++done; }
    }

    // Pick closest end depot
    int    end_poi    = -1;
    double best_dep_t = std::numeric_limits<double>::infinity();
    for (auto dep : depots) {
        int d_poi = intersection_to_poi[dep];
        double t  = edgeCost(curr_poi, d_poi);
        if (t < best_dep_t) { best_dep_t = t; end_poi = d_poi; }
    }
    if (end_poi == -1) { route.sequence.clear(); return route; }

    route.action_to_poi[2 * D + 1] = end_poi;
    route.sequence.push_back(2 * D + 1);
    total_time += best_dep_t;
    route.total_travel_time = total_time;
    return route;
}

// ============================================================
// PERTURBATION: DOUBLE-BRIDGE (large neighborhood escape)
// ============================================================

Route doubleBridge(const Route& r, std::mt19937& rng, int D) {
    int n = (int)r.sequence.size();
    if (n < 8) return r;

    // Pick 4 cut positions strictly inside (not the depot endpoints)
    std::uniform_int_distribution<int> d(1, n - 2);
    int c1 = d(rng), c2 = d(rng), c3 = d(rng), c4 = d(rng);
    std::array<int,4> cuts = {c1, c2, c3, c4};
    std::sort(cuts.begin(), cuts.end());
    // Ensure all distinct
    if (cuts[0]==cuts[1] || cuts[1]==cuts[2] || cuts[2]==cuts[3]) return r;

    // Reconnect as seg0 + seg2 + seg1 + seg3
    Route nr = r;
    nr.sequence.clear();
    for (int i = 0;          i < cuts[0]; ++i) nr.sequence.push_back(r.sequence[i]);
    for (int i = cuts[1];    i < cuts[2]; ++i) nr.sequence.push_back(r.sequence[i]);
    for (int i = cuts[0];    i < cuts[1]; ++i) nr.sequence.push_back(r.sequence[i]);
    for (int i = cuts[2];    i < cuts[3]; ++i) nr.sequence.push_back(r.sequence[i]);
    for (int i = cuts[3];    i < n;       ++i) nr.sequence.push_back(r.sequence[i]);

    if (!isLegalFast(nr.sequence, D)) return r;
    nr.total_travel_time = recalcRouteCost(nr);
    return nr;
}

// ============================================================
// OR-OPT: RELOCATE A SINGLE STOP
// ============================================================

// Returns the change in cost if we remove sequence[remove_idx] and
// insert it after sequence[insert_after] (indices in current sequence).
// Returns INF if move is out of bounds.
static double orOptDelta(const Route& r, int remove_idx, int insert_after) {
    int n = (int)r.sequence.size();
    if (remove_idx <= 0 || remove_idx >= n - 1) return std::numeric_limits<double>::infinity();
    if (insert_after <= 0 || insert_after >= n - 1) return std::numeric_limits<double>::infinity();
    if (insert_after == remove_idx || insert_after == remove_idx - 1) return 0.0;

    // Nodes involved (as POI indices)
    auto poi = [&](int idx) { return r.action_to_poi[r.sequence[idx]]; };

    int prev_r = poi(remove_idx - 1);
    int node_r = poi(remove_idx);
    int next_r = poi(remove_idx + 1);

    // After removal, insert_after index shifts if insert_after > remove_idx
    int real_insert = insert_after;
    if (insert_after > remove_idx) real_insert = insert_after - 1;

    // Indices in modified sequence (after removing remove_idx)
    // We simulate: build a temporary index mapping
    // Cost removed by removing node_r from its current position:
    double remove_cost = edgeCost(prev_r, node_r) + edgeCost(node_r, next_r);
    double bridge_cost = edgeCost(prev_r, next_r);

    // For insertion: find the two nodes between which we insert
    // We need the nodes at real_insert and real_insert+1 in the NEW sequence
    // Build the new sequence conceptually
    std::vector<int> tmp_seq;
    tmp_seq.reserve(n - 1);
    for (int i = 0; i < n; ++i)
        if (i != remove_idx) tmp_seq.push_back(r.sequence[i]);

    if (real_insert >= (int)tmp_seq.size() - 1) return std::numeric_limits<double>::infinity();

    int ins_prev = r.action_to_poi[tmp_seq[real_insert]];
    int ins_next = r.action_to_poi[tmp_seq[real_insert + 1]];

    double insert_cost  = edgeCost(ins_prev, node_r) + edgeCost(node_r, ins_next);
    double removed_edge = edgeCost(ins_prev, ins_next);

    return (bridge_cost + insert_cost) - (remove_cost + removed_edge);
}

// Apply the or-opt move and return the new route
static Route applyOrOpt(const Route& r, int remove_idx, int insert_after, double delta) {
    Route nr = r;
    int action = nr.sequence[remove_idx];
    nr.sequence.erase(nr.sequence.begin() + remove_idx);

    int real_insert = insert_after;
    if (insert_after > remove_idx) real_insert = insert_after - 1;

    nr.sequence.insert(nr.sequence.begin() + real_insert + 1, action);
    nr.total_travel_time = r.total_travel_time + delta;
    return nr;
}

// ============================================================
// SWAP DELTA COST (O(1))
// ============================================================

static double swapDelta(const Route& r, int iA, int iB) {
    if (iA > iB) std::swap(iA, iB);
    int n = (int)r.sequence.size();
    if (iA <= 0 || iB >= n - 1) return std::numeric_limits<double>::infinity();

    auto poi = [&](int idx) { return r.action_to_poi[r.sequence[idx]]; };

    int pA = poi(iA - 1), A = poi(iA), nA = poi(iA + 1);
    int pB = poi(iB - 1), B = poi(iB), nB = poi(iB + 1);

    if (iA + 1 == iB) {
        // Adjacent swap
        double old_c = edgeCost(pA, A) + edgeCost(A, B) + edgeCost(B, nB);
        double new_c = edgeCost(pA, B) + edgeCost(B, A) + edgeCost(A, nB);
        return new_c - old_c;
    } else {
        double old_c = edgeCost(pA, A) + edgeCost(A, nA) + edgeCost(pB, B) + edgeCost(B, nB);
        double new_c = edgeCost(pA, B) + edgeCost(B, nA) + edgeCost(pB, A) + edgeCost(A, nB);
        return new_c - old_c;
    }
}

//CALIBRATED SIMULATED ANNEALING (From Lecture 22)

Route simulatedAnnealing(Route init, double time_limit_sec, int tid,
                         const std::chrono::high_resolution_clock::time_point& g_start) {
    Route curr = init, best = init;
    int D = ((int)curr.sequence.size() - 2) / 2;
    if (D < 1) return best;

    std::mt19937 rng(std::chrono::system_clock::now().time_since_epoch().count()
                     + (uint64_t)tid * 6364136223846793005ULL);
    std::uniform_real_distribution<double> dist01(0.0, 1.0);

    // Calibrate starting temperature: accept ~80% of bad moves initially
    double temp     = 300.0 + tid * 50.0;  // Diversify starting temps across threads
    double cooling  = 0.999993;
    int    iters    = 0;
    int    no_improv= 0;

    auto timeLeft = [&]() -> double {
        return time_limit_sec - std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - g_start).count();
    };

    while (true) {
        ++iters;

        // Time check every 2000 iterations to reduce overhead
        if (iters % 2000 == 0) {
            if (timeLeft() <= 0.15) break;

            // Reheat if stuck
            if (no_improv > 300000) {
                curr = best;
                temp = 150.0;
                no_improv = 0;
                // Apply double-bridge to escape local optimum
                Route perturbed = doubleBridge(curr, rng, D);
                if (!perturbed.sequence.empty() &&
                    perturbed.total_travel_time < std::numeric_limits<double>::infinity()) {
                    curr = perturbed;
                }
            }
        }

        int n = (int)curr.sequence.size();
        if (n < 4) break;

        std::uniform_int_distribution<int> posD(1, n - 2);

        double delta = std::numeric_limits<double>::infinity();
        bool   use_or_opt = dist01(rng) < 0.45;  // 45% or-opt, 55% swap

        int iA = posD(rng);
        int iB = posD(rng);
        if (iA == iB) { temp *= cooling; continue; }

        if (use_or_opt) {
            // Or-opt: relocate iA after iB
            delta = orOptDelta(curr, iA, iB);
            if (delta >= std::numeric_limits<double>::infinity()) { temp *= cooling; continue; }

            // Check legality
            Route candidate = applyOrOpt(curr, iA, iB, delta);
            if (!isLegalFast(candidate.sequence, D)) { temp *= cooling; continue; }

            if (delta < 0 || dist01(rng) < std::exp(-delta / temp)) {
                curr = std::move(candidate);
                if (curr.total_travel_time < best.total_travel_time) {
                    best = curr;
                    no_improv = 0;
                } else {
                    ++no_improv;
                }
            } else {
                ++no_improv;
            }
        } else {
            // Swap move
            delta = swapDelta(curr, iA, iB);
            if (delta >= std::numeric_limits<double>::infinity()) { temp *= cooling; continue; }

            std::swap(curr.sequence[iA], curr.sequence[iB]);
            if (!isLegalFast(curr.sequence, D)) {
                std::swap(curr.sequence[iA], curr.sequence[iB]);
                temp *= cooling;
                continue;
            }
            curr.total_travel_time += delta;

            if (delta < 0) {
                if (curr.total_travel_time < best.total_travel_time) {
                    best = curr;
                    no_improv = 0;
                }
            } else if (dist01(rng) < std::exp(-delta / temp)) {
                // Accepted a worse move — already swapped
                ++no_improv;
            } else {
                // Reject: undo
                std::swap(curr.sequence[iA], curr.sequence[iB]);
                curr.total_travel_time -= delta;
                ++no_improv;
            }
        }

        temp *= cooling;
    }
    return best;
}

// ============================================================
// MAIN ENTRY POINT
// ============================================================

std::vector<CourierSubPath> travelingCourier(
    const float turn_penalty,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots) {

    auto g_start = std::chrono::high_resolution_clock::now();

    // Phase 1: Build cost matrix with stored paths
    buildCostMatrix(deliveries, depots, turn_penalty);

    // Phase 2: Multi-start greedy from every depot
    std::vector<Route> greedy_options;
    for (auto dep : depots) {
        Route r = generateGreedyRoute(dep, deliveries, depots);
        if (!r.sequence.empty() &&
            r.total_travel_time < std::numeric_limits<double>::infinity()) {
            greedy_options.push_back(r);
        }
    }

    if (greedy_options.empty()) return {};

    std::sort(greedy_options.begin(), greedy_options.end(),
              [](const Route& a, const Route& b) {
                  return a.total_travel_time < b.total_travel_time;
              });

    // Time budget
    double max_t;
    int    sz = (int)deliveries.size();
    if      (sz <= 5)  max_t = 0.45;
    else if (sz <= 30) max_t = 4.8;
    else               max_t = 47.5;

    // Phase 3: Parallel SA — each thread gets a (possibly perturbed) starting route
    int num_threads = std::max(1, (int)std::thread::hardware_concurrency());

    std::vector<std::future<Route>> futures;
    futures.reserve(num_threads);

    for (int tid = 0; tid < num_threads; ++tid) {
        // Vary starting routes across threads; top routes for first threads
        Route base = greedy_options[tid % greedy_options.size()];

        // Perturb routes for threads beyond the first to encourage diversity
        if (tid > 0 && base.sequence.size() >= 8) {
            std::mt19937 init_rng(tid * 12345 + 9999);
            int D = ((int)base.sequence.size() - 2) / 2;
            base = doubleBridge(base, init_rng, D);
            if (base.sequence.empty()) base = greedy_options[tid % greedy_options.size()];
        }

        futures.push_back(std::async(std::launch::async,
            [tid, base, g_start, max_t]() -> Route {
                double remain = max_t - std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now() - g_start).count();
                if (remain <= 0.1 || base.sequence.empty()) return base;
                return simulatedAnnealing(base, max_t, tid, g_start);
            }));
    }

    // Collect best result across all threads
    Route abs_best;
    abs_best.total_travel_time = std::numeric_limits<double>::infinity();
    for (auto& f : futures) {
        Route r = f.get();
        if (!r.sequence.empty() && r.total_travel_time < abs_best.total_travel_time) {
            abs_best = r;
        }
    }

    // Fallback to best greedy if SA somehow failed
    if (abs_best.sequence.empty()) abs_best = greedy_options[0];

    // Phase 4: Build result using stored paths (no findPathBetweenIntersections calls)
    std::vector<CourierSubPath> result;
    result.reserve(abs_best.sequence.size() - 1);

    for (size_t i = 0; i + 1 < abs_best.sequence.size(); ++i) {
        int s_poi = abs_best.action_to_poi[abs_best.sequence[i]];
        int e_poi = abs_best.action_to_poi[abs_best.sequence[i + 1]];

        IntersectionIdx s_inter = poi_to_intersection[s_poi];
        IntersectionIdx e_inter = poi_to_intersection[e_poi];

        CourierSubPath sub;
        sub.intersections = {s_inter, e_inter};

        if (s_poi == e_poi) {
            sub.subpath = {};
        } else {
            sub.subpath = cost_matrix[s_poi][e_poi].path;
        }
        result.push_back(sub);
    }

    return result;
}