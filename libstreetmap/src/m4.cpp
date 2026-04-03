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
