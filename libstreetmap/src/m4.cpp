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
#include <thread>

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

static inline double edgeCost(int a, int b) {
    return cost_matrix[a][b].travel_time;
}

static double recalcRouteCost(const Route& r) {
    double total = 0.0;
    const int sz = (int)r.sequence.size();
    for (int i = 0; i + 1 < sz; ++i)
        total += edgeCost(r.action_to_poi[r.sequence[i]], r.action_to_poi[r.sequence[i+1]]);
    return total;
}

// Fast legality: track pickup positions, check dropoff comes after
static bool isLegalFast(const std::vector<int>& seq, int D) {
    std::vector<bool> picked(D, false);
    for (int a : seq) {
        if (a < D)                         picked[a] = true;
        else if (a < 2*D && !picked[a-D])  return false;
    }
    return true;
}

// Ultra-fast legality check specifically for swap moves
// Only validates the two positions that changed
static bool isSwapLegal(const std::vector<int>& seq, int iA, int iB, int D) {
    int aA = seq[iA], aB = seq[iB];
    bool aA_is_drop = (aA >= D && aA < 2*D);
    bool aB_is_drop = (aB >= D && aB < 2*D);
    bool aA_is_pick = (aA >= 0 && aA < D);
    bool aB_is_pick = (aB >= 0 && aB < D);

    // Dropoff at iA: its pickup must appear before iA
    if (aA_is_drop) {
        int pick = aA - D;
        bool found = false;
        for (int i = 0; i < iA; ++i) if (seq[i] == pick) { found = true; break; }
        if (!found) return false;
    }
    // Dropoff at iB: its pickup must appear before iB
    if (aB_is_drop) {
        int pick = aB - D;
        bool found = false;
        for (int i = 0; i < iB; ++i) if (seq[i] == pick) { found = true; break; }
        if (!found) return false;
    }
    // Pickup moved to iA: its dropoff must NOT appear before iA
    if (aA_is_pick) {
        int drop = aA + D;
        for (int i = 1; i < iA; ++i) if (seq[i] == drop) return false;
    }
    // Pickup moved to iB: its dropoff must NOT appear before iB
    if (aB_is_pick) {
        int drop = aB + D;
        for (int i = 1; i < iB; ++i) if (seq[i] == drop) return false;
    }
    return true;
}

// PHASE 1: PARALLEL MATRIX BUILDER (fropm tutorial) 

void buildCostMatrix(const std::vector<DeliveryInf>& deliveries,
                     const std::vector<IntersectionIdx>& depots,
                     float turn_penalty) {

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
    intersection_to_poi.reserve(unique_pois.size() * 2);
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

        std::vector<double>           best_times(total_nodes, INF);
        std::vector<StreetSegmentIdx> prev_edge(total_nodes, NO_EDGE);
        std::priority_queue<WaveElem, std::vector<WaveElem>, CompareWaveElem> pq;

        best_times[start_node] = 0.0;
        pq.push(WaveElem(start_node, NO_EDGE, 0.0));
        int found_count = 0;

        while (!pq.empty() && found_count < num_pois) {
            WaveElem cur = pq.top(); pq.pop();
            if (cur.travelTime > best_times[cur.nodeID]) continue;

            auto it = intersection_to_poi.find(cur.nodeID);
            if (it != intersection_to_poi.end()) {
                int j = it->second;
                if (cost_matrix[i][j].travel_time == INF) {
                    cost_matrix[i][j].travel_time = cur.travelTime;

                    // Reconstruct and store path from start_node -> cur.nodeID
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
                                       ? segmentInfos[cur.edgeID].streetID : NO_STREET;

            for (StreetSegmentIdx segID : intersectionSegments[cur.nodeID]) {
                const auto& info = segmentInfos[segID];
                IntersectionIdx neighbor;
                if      (info.from == cur.nodeID) neighbor = info.to;
                else if (!info.oneWay)            neighbor = info.from;
                else                              continue;

                double turn    = (currStreet != NO_STREET && currStreet != info.streetID)
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
    route.action_to_poi.resize(2 * D + 2);
    for (int i = 0; i < D; ++i) {
        route.action_to_poi[i]     = intersection_to_poi[deliveries[i].pickUp];
        route.action_to_poi[i + D] = intersection_to_poi[deliveries[i].dropOff];
    }
    route.action_to_poi[2 * D] = intersection_to_poi[start_depot];
    route.sequence.reserve(2 * D + 2);
    route.sequence.push_back(2 * D);

    std::vector<bool> picked(D, false), dropped(D, false);
    int    curr_poi   = route.action_to_poi[2 * D];
    int    done       = 0;
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
        int    d_poi = intersection_to_poi[dep];
        double t     = edgeCost(curr_poi, d_poi);
        if (t < best_dep_t) { best_dep_t = t; end_poi = d_poi; }
    }
    if (end_poi == -1) { route.sequence.clear(); return route; }

    route.action_to_poi[2 * D + 1] = end_poi;
    route.sequence.push_back(2 * D + 1);
    route.total_travel_time = total_time + best_dep_t;
    return route;
}

// ============================================================
// PERTURBATION: DOUBLE-BRIDGE (large neighborhood escape)
// ============================================================

Route doubleBridge(const Route& r, std::mt19937& rng, int D) {
    int n = (int)r.sequence.size();
    if (n < 8) return r;

    std::uniform_int_distribution<int> d(1, n - 2);
    int c[4] = {d(rng), d(rng), d(rng), d(rng)};
    std::sort(c, c + 4);
    if (c[0]==c[1] || c[1]==c[2] || c[2]==c[3]) return r;

    Route nr = r;
    nr.sequence.clear();
    nr.sequence.reserve(n);
    for (int i=0;    i<c[0]; ++i) nr.sequence.push_back(r.sequence[i]);
    for (int i=c[2]; i<c[3]; ++i) nr.sequence.push_back(r.sequence[i]);
    for (int i=c[1]; i<c[2]; ++i) nr.sequence.push_back(r.sequence[i]);
    for (int i=c[0]; i<c[1]; ++i) nr.sequence.push_back(r.sequence[i]);
    for (int i=c[3]; i<n;    ++i) nr.sequence.push_back(r.sequence[i]);

    if (!isLegalFast(nr.sequence, D)) return r;
    nr.total_travel_time = recalcRouteCost(nr);
    return nr;
}

// ============================================================
// O(1) DELTA COSTS
// ============================================================

static inline double swapDelta(const Route& r, int iA, int iB) {
    // iA < iB guaranteed by caller
    auto poi = [&](int idx) { return r.action_to_poi[r.sequence[idx]]; };
    int pA = poi(iA-1), A = poi(iA), nA = poi(iA+1);
    int pB = poi(iB-1), B = poi(iB), nB = poi(iB+1);

    if (iA + 1 == iB) {
        return (edgeCost(pA,B) + edgeCost(B,A) + edgeCost(A,nB))
             - (edgeCost(pA,A) + edgeCost(A,B) + edgeCost(B,nB));
    }
    return (edgeCost(pA,B) + edgeCost(B,nA) + edgeCost(pB,A) + edgeCost(A,nB))
         - (edgeCost(pA,A) + edgeCost(A,nA) + edgeCost(pB,B) + edgeCost(B,nB));
}

// Or-opt delta: cost change of removing seq[ri] and inserting after seq[ii]
static double orOptDelta(const Route& r, int ri, int ii) {
    int n = (int)r.sequence.size();
    if (ri <= 0 || ri >= n-1) return std::numeric_limits<double>::infinity();
    if (ii <= 0 || ii >= n-1) return std::numeric_limits<double>::infinity();
    if (ii == ri || ii == ri-1) return 0.0;

    auto p = [&](int idx) { return r.action_to_poi[r.sequence[idx]]; };

    int prev_r = p(ri-1), node_r = p(ri), next_r = p(ri+1);
    double rem  = edgeCost(prev_r, node_r) + edgeCost(node_r, next_r);
    double brdg = edgeCost(prev_r, next_r);

    // Insertion point in original sequence (before removal)
    int ins_next_orig = (ii > ri) ? ii + 1 : ii + 1;
    if (ins_next_orig >= n) return std::numeric_limits<double>::infinity();

    int ins_prev = p(ii);
    int ins_next = p(ins_next_orig);

    double add  = edgeCost(ins_prev, node_r) + edgeCost(node_r, ins_next);
    double repl = edgeCost(ins_prev, ins_next);

    return (brdg + add) - (rem + repl);
}

// Apply or-opt move: remove seq[ri], insert after seq[ii]
static Route applyOrOpt(Route r, int ri, int ii) {
    int action = r.sequence[ri];
    r.sequence.erase(r.sequence.begin() + ri);
    int ins = (ii > ri) ? ii - 1 : ii;
    r.sequence.insert(r.sequence.begin() + ins + 1, action);
    return r;
}

//CALIBRATED SIMULATED ANNEALING (From Lecture 22)

Route simulatedAnnealing(Route init, double time_limit_sec, int tid,
                         const std::chrono::high_resolution_clock::time_point& g_start) {

    Route curr = init, best = init;
    int D = ((int)curr.sequence.size() - 2) / 2;
    if (D < 1) return best;

    // Each thread gets unique seed and starting temperature for diversity
    std::mt19937 rng(std::chrono::system_clock::now().time_since_epoch().count()
                     + (uint64_t)tid * 6364136223846793005ULL);
    std::uniform_real_distribution<double> dist01(0.0, 1.0);

    // Calibrate starting temperature: accept ~80% of bad moves initially
    double temp    = 200.0 + tid * 40.0;  // Diversify starting temps across threads
    double cooling = 0.999992;
    int    iters   = 0, no_improv = 0;
    int    n       = (int)curr.sequence.size();

    auto timeLeft = [&]() {
        return time_limit_sec - std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - g_start).count();
    };

    while (true) {
        ++iters;

        // Time check every 2000 iterations to reduce overhead
        if (iters % 2000 == 0) {
            if (timeLeft() <= 0.1) break;
            // Reheat + double-bridge if stuck
            if (no_improv > 200000) {
                curr = best; temp = 120.0; no_improv = 0;
                Route perturbed = doubleBridge(curr, rng, D);
                if (!perturbed.sequence.empty()) { curr = perturbed; n = (int)curr.sequence.size(); }
            }
        }

        if (n < 4) break;

        std::uniform_int_distribution<int> posD(1, n - 2);
        int iA = posD(rng), iB = posD(rng);
        if (iA == iB) continue;

        // 40% or-opt (relocate), 60% swap — swap is cheaper so we do more of it
        if (dist01(rng) < 0.40) {
            // Or-opt move
            double delta = orOptDelta(curr, iA, iB);
            if (delta >= std::numeric_limits<double>::infinity()) { temp *= cooling; continue; }

            if (delta < 0 || dist01(rng) < std::exp(-delta / temp)) {
                Route candidate = applyOrOpt(curr, iA, iB);
                if (!isLegalFast(candidate.sequence, D)) { temp *= cooling; continue; }
                candidate.total_travel_time = curr.total_travel_time + delta;
                curr = std::move(candidate);
                n    = (int)curr.sequence.size();
                if (curr.total_travel_time < best.total_travel_time) { best = curr; no_improv = 0; }
                else ++no_improv;
            } else ++no_improv;

        } else {
            // Swap move
            if (iA > iB) std::swap(iA, iB);
            double delta = swapDelta(curr, iA, iB);
            if (delta >= std::numeric_limits<double>::infinity()) { temp *= cooling; continue; }

            std::swap(curr.sequence[iA], curr.sequence[iB]);

            if (!isSwapLegal(curr.sequence, iA, iB, D)) {
                std::swap(curr.sequence[iA], curr.sequence[iB]);
                temp *= cooling; continue;
            }

            if (delta < 0 || dist01(rng) < std::exp(-delta / temp)) {
                curr.total_travel_time += delta;
                if (curr.total_travel_time < best.total_travel_time) { best = curr; no_improv = 0; }
                else ++no_improv;
            } else {
                std::swap(curr.sequence[iA], curr.sequence[iB]); // reject: undo
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
        if (!r.sequence.empty() && r.total_travel_time < std::numeric_limits<double>::infinity())
            greedy_options.push_back(r);
    }
    if (greedy_options.empty()) return {};

    std::sort(greedy_options.begin(), greedy_options.end(),
              [](const Route& a, const Route& b){ return a.total_travel_time < b.total_travel_time; });

    // Time budget
    int    sz    = (int)deliveries.size();
    double max_t = (sz <= 5) ? 0.45 : (sz <= 30) ? 4.8 : 47.5;

    // Phase 3: Parallel SA — each thread gets a (possibly perturbed) starting route
    int num_threads = std::max(1, (int)std::thread::hardware_concurrency());

    std::vector<std::future<Route>> futures;
    futures.reserve(num_threads);

    for (int tid = 0; tid < num_threads; ++tid) {
        Route base = greedy_options[tid % greedy_options.size()];

        // Perturb routes for threads beyond the first to encourage diversity
        if (tid > 0 && (int)base.sequence.size() >= 8) {
            std::mt19937 init_rng((uint64_t)tid * 12345 + 9999);
            int D = ((int)base.sequence.size() - 2) / 2;
            Route pb = doubleBridge(base, init_rng, D);
            if (!pb.sequence.empty()) base = pb;
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
    Route abs_best; abs_best.total_travel_time = std::numeric_limits<double>::infinity();
    for (auto& f : futures) {
        Route r = f.get();
        if (!r.sequence.empty() && r.total_travel_time < abs_best.total_travel_time)
            abs_best = r;
    }

    // Fallback to best greedy if SA somehow failed
    if (abs_best.sequence.empty()) abs_best = greedy_options[0];

    // Phase 4: Build result using stored paths (no findPathBetweenIntersections calls)
    std::vector<CourierSubPath> result;
    result.reserve(abs_best.sequence.size() - 1);

    for (size_t i = 0; i + 1 < abs_best.sequence.size(); ++i) {
        int s_poi = abs_best.action_to_poi[abs_best.sequence[i]];
        int e_poi = abs_best.action_to_poi[abs_best.sequence[i + 1]];

        CourierSubPath sub;
        sub.intersections = { poi_to_intersection[s_poi], poi_to_intersection[e_poi] };
        sub.subpath = (s_poi == e_poi) ? std::vector<StreetSegmentIdx>()
                                       : cost_matrix[s_poi][e_poi].path;
        result.push_back(sub);
    }

    return result;
}