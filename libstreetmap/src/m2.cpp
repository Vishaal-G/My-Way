/* * Copyright 2026 University of Toronto ... (Header omitted for brevity) */

#include "m2.h"
#include "m1.hpp"
#include <gtk/gtk.h>
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <chrono>
#include <sstream>
#include <unordered_set>
#include <filesystem>
#include <set>
#include "tts.h"   // ← TTS ADDED

double cos_lat_avg;
void draw_main_canvas(ezgl::renderer *g);

struct Intersection {
  LatLon position;
  std::string name;
  float x;
  float y;
};


struct MyPOI {
  LatLon position;
  std::string name;
  float x;
  float y;
};

struct streetSegments {
  std::vector<ezgl::point2d> points;
  float speedLimit;
  std::string name;
};

struct MyFeature {
  FeatureType type;
  std::vector<ezgl::point2d> points;
  bool is_closed;
};

struct PoiCategory {
  ezgl::color color;
  std::string label;
};


// --- Subway Structures ---
struct SubwayLine {
    std::string name;
    ezgl::color color;
    std::vector<std::vector<ezgl::point2d>> tracks; 
};

struct SubwayStation {
    std::string name;
    ezgl::point2d position;
};

// Global vector for drawing
std::vector<SubwayLine> subway_lines;
std::vector<SubwayStation> subway_stations;


// Hash maps to quickly look up OSM pointers by their ID
std::unordered_map<OSMID, const OSMWay*> osm_ways_map;
std::unordered_map<OSMID, const OSMNode*> osm_nodes_map;

// Global Data Structures
std::vector<MyFeature> features;
std::vector<streetSegments> streets;
std::vector<MyPOI> Mypois;
std::vector<Intersection> intersections;

static int selected_intersection = -1;
static std::vector<int> highlighted_intersections;

// --- Map Loading Globals ---
std::vector<std::string> discovered_map_paths;
double global_maxLat, global_minLat, global_maxLon, global_minLon;
static ezgl::rectangle g_map_world;

// --- Autocomplete Globals ---
static GtkListStore *autocomplete_store = nullptr;
enum { COL_NAME = 0, COL_TYPE, COL_IDX, N_COLS };

static int search_result_intersection = -1;
static float search_result_x = -1;
static float search_result_y = -1;
static bool search_result_is_poi = false;
static bool search_just_selected = false;

// Coordinate conversion declarations
float xFromLon(float lon);
float yFromLat(float lat);
float lonFromX(float x);
float latFromY(float y);
void act_on_mouse_click(ezgl::application *app, GdkEventButton *event, double x, double y);

// Global for popup in finding interesctions
static GtkWidget* active_find_dialog = nullptr;

// ----------------------------------------------------------------------------
// Coordinate Conversion Functions
// ----------------------------------------------------------------------------
float xFromLon(float lon) {
  return kEarthRadiusInMeters * (lon * kDegreeToRadian) * cos_lat_avg;
}
float yFromLat(float lat) {
  return kEarthRadiusInMeters * (lat * kDegreeToRadian);
}
float lonFromX(float x) {
  return x / (kEarthRadiusInMeters * kDegreeToRadian * cos_lat_avg);
}
float latFromY(float y) {
  return y / (kEarthRadiusInMeters * kDegreeToRadian);
}

// ----------------------------------------------------------------------------
// Data Loading & Map Discovery Functions
// ----------------------------------------------------------------------------
bool ends_with(const std::string &text, const std::string &suffix) {
  if (suffix.size() > text.size()) return false;
  return text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

static bool is_night_mode = false;

ezgl::color parse_hex_color(std::string hex_str) {
    if (hex_str.empty()) return ezgl::color(0, 100, 200);
    if (hex_str[0] == '#') hex_str = hex_str.substr(1);
    
    if (hex_str.length() == 6) {
        try {
            unsigned long val = std::stoul(hex_str, nullptr, 16);
            uint8_t r = (val >> 16) & 0xFF;
            uint8_t g = (val >> 8) & 0xFF;
            uint8_t b = val & 0xFF;
            return ezgl::color(r, g, b, 200); 
        } catch (...) {}
    }
    return ezgl::color(0, 100, 200);
}

void discover_map_paths() {
  namespace fs = std::filesystem;
  std::set<std::string> unique_paths;
  discovered_map_paths.clear();

  auto add_maps_under = [&](const fs::path &root, int max_depth) {
    std::error_code ec;
    if (!fs::exists(root, ec)) return;
    fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec);
    fs::recursive_directory_iterator end;
    while (it != end) {
      if (ec) { ec.clear(); it.increment(ec); continue; }
      if (it.depth() > max_depth) { it.disable_recursion_pending(); it.increment(ec); continue; }
      if (it->is_regular_file(ec)) {
        std::string file_name = it->path().filename().string();
        if (ends_with(file_name, ".streets.bin")) {
          std::string normalized = it->path().lexically_normal().string();
          if (unique_paths.insert(normalized).second) {
            discovered_map_paths.push_back(normalized);
          }
        }
      }
      it.increment(ec);
    }
  };

  add_maps_under(fs::path("."), 4);
  add_maps_under(fs::path("/cad2/ece297s/public/maps"), 2);

  std::sort(discovered_map_paths.begin(), discovered_map_paths.end(),
            [](const std::string &a, const std::string &b) {
              return std::filesystem::path(a).filename().string() <
                     std::filesystem::path(b).filename().string();
            });
}

void load_map_data() {
  selected_intersection = -1;
  highlighted_intersections.clear();

  search_result_intersection = -1;
  search_result_x = -1;
  search_result_y = -1;
  search_result_is_poi = false;
  search_just_selected = false;

  if (getNumIntersections() == 0) return;

  global_maxLat = getIntersectionPosition(0).latitude();
  global_minLat = global_maxLat;
  global_maxLon = getIntersectionPosition(0).longitude();
  global_minLon = global_maxLon;

  intersections.clear();
  intersections.resize(getNumIntersections());
  for (int i = 0; i < getNumIntersections(); ++i) {
    intersections[i].position = getIntersectionPosition(i);
    intersections[i].name = getIntersectionName(i);

    global_maxLat = std::max(global_maxLat, intersections[i].position.latitude());
    global_minLat = std::min(global_minLat, intersections[i].position.latitude());
    global_maxLon = std::max(global_maxLon, intersections[i].position.longitude());
    global_minLon = std::min(global_minLon, intersections[i].position.longitude());
  }

  double avgLat = ((global_maxLat + global_minLat) / 2.0) * kDegreeToRadian;
  cos_lat_avg = cos(avgLat);

  for (int i = 0; i < getNumIntersections(); ++i) {
    intersections[i].x = xFromLon(intersections[i].position.longitude());
    intersections[i].y = yFromLat(intersections[i].position.latitude());
  }

  streets.clear();
  int numSegments = getNumStreetSegments();
  streets.resize(numSegments);
  for (int i = 0; i < numSegments; i++) {
    StreetSegmentInfo info = getStreetSegmentInfo(i);
    streets[i].speedLimit = info.speedLimit;
    streets[i].name = getStreetName(info.streetID);

    LatLon fromPos = getIntersectionPosition(info.from);
    streets[i].points.push_back({xFromLon(fromPos.longitude()), yFromLat(fromPos.latitude())});

    for (int j = 0; j < info.numCurvePoints; j++) {
      LatLon curvePos = getStreetSegmentCurvePoint(i, j);
      streets[i].points.push_back({xFromLon(curvePos.longitude()), yFromLat(curvePos.latitude())});
    }

    LatLon toPos = getIntersectionPosition(info.to);
    streets[i].points.push_back({xFromLon(toPos.longitude()), yFromLat(toPos.latitude())});
  }

  Mypois.clear();
  int numPOIs = getNumPointsOfInterest();
  Mypois.resize(numPOIs);
  for (int i = 0; i < numPOIs; i++) {
    Mypois[i].position = getPOIPosition(i);
    Mypois[i].name = getPOIName(i);
    Mypois[i].x = xFromLon(Mypois[i].position.longitude());
    Mypois[i].y = yFromLat(Mypois[i].position.latitude());
  }

  features.clear();
  int numFeatures = getNumFeatures();
  for (int i = 0; i < numFeatures; i++) {
    MyFeature feat;
    FeatureType type = getFeatureType(i);

    feat.type = type;

    int numPoints = getNumFeaturePoints(i);
    for (int j = 0; j < numPoints; j++) {
      LatLon pos = getFeaturePoint(i, j);
      feat.points.push_back({xFromLon(pos.longitude()), yFromLat(pos.latitude())});
    }

    feat.is_closed = (feat.points.size() > 2 &&
                      feat.points.front().x == feat.points.back().x &&
                      feat.points.front().y == feat.points.back().y);

    features.push_back(feat);
  }

  // ---------------------------------------------------
  // Load Subway Lines from OSM
  // ---------------------------------------------------
  subway_lines.clear();
  osm_nodes_map.clear();
  osm_ways_map.clear();

  for (int i = 0; i < getNumberOfNodes(); ++i) {
      const OSMNode* node = getNodeByIndex(i);
      osm_nodes_map[node->id()] = node;
  }
  for (int i = 0; i < getNumberOfWays(); ++i) {
      const OSMWay* way = getWayByIndex(i);
      osm_ways_map[way->id()] = way;
  }

  for (int i = 0; i < getNumberOfRelations(); ++i) {
      const OSMRelation* rel = getRelationByIndex(i);
      
      bool is_subway = false;
      std::string name = "Unknown Line";
      std::string hex_color = "";
      
      int tagCount = getTagCount(rel);
      for (int j = 0; j < tagCount; ++j) {
          std::pair<std::string, std::string> tag = getTagPair(rel, j);
          
          if (tag.first == "route" && tag.second == "subway") is_subway = true;
          if (tag.first == "name") name = tag.second;
          if (tag.first == "colour") hex_color = tag.second;
      }

      if (is_subway) {
          SubwayLine line;
          line.name = name;
          line.color = parse_hex_color(hex_color);
          
          std::vector<TypedOSMID> members = getRelationMembers(rel); 
          std::vector<std::string> roles = getRelationMemberRoles(rel);
          
          for (size_t k = 0; k < members.size(); ++k) {
              const auto& member = members[k];
              const std::string& role = roles[k];
              
              if (member.type() == TypedOSMID::Way) {
                  auto way_it = osm_ways_map.find(member);
                  if (way_it != osm_ways_map.end()) {
                      const OSMWay* way = way_it->second;
                      std::vector<ezgl::point2d> track_points;
                      
                      const std::vector<OSMID>& nds = getWayMembers(way); 
                      
                      for (OSMID nd_id : nds) {
                          auto node_it = osm_nodes_map.find(nd_id);
                          if (node_it != osm_nodes_map.end()) {
                              LatLon ll = getNodeCoords(node_it->second); 
                              track_points.push_back({xFromLon(ll.longitude()), yFromLat(ll.latitude())});
                          }
                      }
                      
                      if (!track_points.empty()) {
                          line.tracks.push_back(track_points);
                      }
                  }
              }
              else if (member.type() == TypedOSMID::Node) {
                  if (role == "stop" || role == "station" || role == "platform") {
                      auto node_it = osm_nodes_map.find(member);
                      if (node_it != osm_nodes_map.end()) {
                          const OSMNode* node = node_it->second;
                          LatLon ll = getNodeCoords(node);
                          
                          std::string station_name = "";
                          for (int t = 0; t < getTagCount(node); ++t) {
                              auto tag = getTagPair(node, t);
                              if (tag.first == "name") station_name = tag.second;
                          }
                          
                          SubwayStation st;
                          st.name = station_name;
                          st.position = {xFromLon(ll.longitude()), yFromLat(ll.latitude())};
                          subway_stations.push_back(st);
                      }
                  }
              }
          }
          subway_lines.push_back(line);
      }
  }

    g_map_world = ezgl::rectangle(
        {xFromLon(global_minLon), yFromLat(global_minLat)},
        {xFromLon(global_maxLon), yFromLat(global_maxLat)}
  );
}

// ----------------------------------------------------------------------------
// Autocomplete Functions
// ----------------------------------------------------------------------------
void build_autocomplete_store() {
  if (autocomplete_store) {
    g_object_unref(autocomplete_store);
  }

  autocomplete_store = gtk_list_store_new(
      N_COLS,
      G_TYPE_STRING,
      G_TYPE_INT,
      G_TYPE_INT
  );

  GtkTreeIter iter;

  std::unordered_set<std::string> seen_streets;
  for (int i = 0; i < (int)streets.size(); i++) {
    const std::string &name = streets[i].name;
    if (name == "<unknown>" || seen_streets.count(name)) continue;
    seen_streets.insert(name);
    gtk_list_store_append(autocomplete_store, &iter);
    gtk_list_store_set(autocomplete_store, &iter,
                       COL_NAME, name.c_str(),
                       COL_TYPE, 0,
                       COL_IDX, i,
                       -1);
  }

  for (int i = 0; i < (int)Mypois.size(); i++) {
    std::string display_name = Mypois[i].name;
    int nearest = findClosestIntersection(Mypois[i].position);
    if (nearest >= 0 && nearest < (int)intersections.size()) {
      display_name += " (" + intersections[nearest].name + ")";
    }
    gtk_list_store_append(autocomplete_store, &iter);
    gtk_list_store_set(autocomplete_store, &iter,
                       COL_NAME, display_name.c_str(),
                       COL_TYPE, 1,
                       COL_IDX, i,
                       -1);
  }

  std::unordered_set<std::string> seen_ints;
  for (int i = 0; i < (int)intersections.size(); i++) {
    const std::string &name = intersections[i].name;
    if (name == "<unknown>" || seen_ints.count(name)) continue;
    seen_ints.insert(name);
    gtk_list_store_append(autocomplete_store, &iter);
    gtk_list_store_set(autocomplete_store, &iter,
                       COL_NAME, name.c_str(),
                       COL_TYPE, 2,
                       COL_IDX, i,
                       -1);
  }
}

static gboolean on_autocomplete_match_selected(GtkEntryCompletion*,
                                               GtkTreeModel *model,
                                               GtkTreeIter *iter,
                                               gpointer data) {
  auto *app = static_cast<ezgl::application *>(data);

  gchar *name = nullptr;
  gint type = 0;
  gint idx = 0;
  gtk_tree_model_get(model, iter,
                     COL_NAME, &name,
                     COL_TYPE, &type,
                     COL_IDX, &idx,
                     -1);

  search_result_intersection = -1;
  search_result_x = -1;
  search_result_y = -1;
  search_result_is_poi = false;

  if (type == 0) {
    // --- Street selected ---
    auto street_ids = findStreetIdsFromPartialStreetName(name ? name : "");
    highlighted_intersections.clear();
    selected_intersection = -1;
    if (!street_ids.empty()) {
      auto ints = findIntersectionsOfStreet(street_ids[0]);
      highlighted_intersections = ints;
      if (!ints.empty()) {
        selected_intersection = ints[0];
        ezgl::canvas *c = app->get_canvas("MainCanvas");
        if (c) {
          float min_x = intersections[ints[0]].x;
          float max_x = min_x;
          float min_y = intersections[ints[0]].y;
          float max_y = min_y;
          for (int id : ints) {
            min_x = std::min(min_x, intersections[id].x);
            max_x = std::max(max_x, intersections[id].x);
            min_y = std::min(min_y, intersections[id].y);
            max_y = std::max(max_y, intersections[id].y);
          }
          float pad = std::max((max_x - min_x) * 0.2f, 300.0f);
          ezgl::rectangle zoom_to({min_x - pad, min_y - pad},
                                  {max_x + pad, max_y + pad});
          c->get_camera().set_world(zoom_to);
        }
        search_result_is_poi = false;
        search_result_intersection = ints[0];
      }
    }

    // ── TTS: announce street ──────────────────────────────────────────────
    if (name) {
      speak(std::string("Street: ") + name);
    }
    // ─────────────────────────────────────────────────────────────────────

  } else if (type == 1) {
    // --- POI selected ---
    selected_intersection = -1;
    highlighted_intersections.clear();
    ezgl::canvas *c = app->get_canvas("MainCanvas");
    if (c && idx < (int)Mypois.size()) {
      float cx = Mypois[idx].x;
      float cy = Mypois[idx].y;
      float half = 500.0f;
      ezgl::rectangle zoom_to({cx - half, cy - half}, {cx + half, cy + half});
      c->get_camera().set_world(zoom_to);
      search_result_is_poi = true;
      search_result_x = cx;
      search_result_y = cy;
    }

    // ── TTS: announce POI (strip the "(near ...)" part for cleaner speech) 
    if (name) {
      std::string clean = Mypois[idx].name; // use raw POI name, no parenthetical
      speak(std::string("Point of interest: ") + clean);
    }
    // ─────────────────────────────────────────────────────────────────────

  } else if (type == 2) {
    // --- Intersection selected ---
    selected_intersection = idx;
    highlighted_intersections.clear();
    ezgl::canvas *c = app->get_canvas("MainCanvas");
    if (c && idx < (int)intersections.size()) {
      float cx = intersections[idx].x;
      float cy = intersections[idx].y;
      float half = 300.0f;
      ezgl::rectangle zoom_to({cx - half, cy - half}, {cx + half, cy + half});
      c->get_camera().set_world(zoom_to);
    }
    std::stringstream ss;
    ss << "Intersection: " << intersections[idx].name;
    app->update_message(ss.str());
    search_result_is_poi = false;
    search_result_intersection = idx;

    // ── TTS: announce intersection ────────────────────────────────────────
    speak(std::string("Intersection: ") + intersections[idx].name);
    // ─────────────────────────────────────────────────────────────────────
  }

std::stringstream ss;

if (type == 0) {
    ss << "STREET: " << (name ? name : "");
}
else if (type == 1) {
    ss << "POI: " << (name ? name : "");
}
else if (type == 2) {
    ss << "INTERSECTION: " << (name ? name : "");
}

app->update_message(ss.str());

if (name) g_free(name);

search_just_selected = true;
app->refresh_drawing();
return TRUE;
}

static void attach_autocomplete(GtkEntry *entry, ezgl::application *app) {
  GtkEntryCompletion *completion = gtk_entry_completion_new();

  gtk_entry_completion_set_model(completion, GTK_TREE_MODEL(autocomplete_store));
  gtk_entry_completion_set_text_column(completion, COL_NAME);
  gtk_entry_completion_set_minimum_key_length(completion, 2);
  gtk_entry_completion_set_popup_completion(completion, TRUE);
  gtk_entry_completion_set_inline_completion(completion, FALSE);

  g_signal_connect(completion, "match-selected",
                   G_CALLBACK(on_autocomplete_match_selected), app);

  gtk_entry_set_completion(entry, completion);
  g_object_unref(completion);
}

// ----------------------------------------------------------------------------
// Event Handlers & Callbacks
// ----------------------------------------------------------------------------
void act_on_mouse_click(ezgl::application *app, GdkEventButton *, double x, double y) {
  if (search_just_selected) {
    search_just_selected = false;
    app->refresh_drawing();
    return;
  }

  search_result_intersection = -1;
  search_result_x = -1;
  search_result_y = -1;
  search_result_is_poi = false;
  highlighted_intersections.clear();

  LatLon clicked_pos(latFromY(y), lonFromX(x));
  selected_intersection = findClosestIntersection(clicked_pos);
  if (selected_intersection != -1) {
    std::stringstream ss;
    ss << "Intersection Clicked: " << intersections[selected_intersection].name;
    app->update_message(ss.str());

    // ── TTS: speak the clicked intersection name ──────────────────────────
    speak(std::string("Intersection: ") + intersections[selected_intersection].name);
    // ─────────────────────────────────────────────────────────────────────
  }
  app->refresh_drawing();
}

static void find_and_highlight(const std::string &street1, const std::string &street2) {
  highlighted_intersections.clear();

  auto s1 = findStreetIdsFromPartialStreetName(street1);
  auto s2 = findStreetIdsFromPartialStreetName(street2);
  if (s1.empty() || s2.empty()) return;

  std::unordered_set<int> uniq;

  for (int id1 : s1) {
    for (int id2 : s2) {
      auto ints = findIntersectionsOfTwoStreets(id1, id2);
      for (int inter : ints) uniq.insert(inter);
    }
  }

  highlighted_intersections.assign(uniq.begin(), uniq.end());
}

static void find_button(GtkWidget *, gpointer data) {
  auto *app = static_cast<ezgl::application *>(data);

  // Close any previous find popup (always)
if (active_find_dialog != nullptr) {
  gtk_widget_destroy(active_find_dialog);
  active_find_dialog = nullptr;
}

  GtkEntry *e1 = GTK_ENTRY(app->get_object("Street1Entry"));
  GtkEntry *e2 = GTK_ENTRY(app->get_object("Street2Entry"));

  std::string s1 = gtk_entry_get_text(e1);
  std::string s2 = gtk_entry_get_text(e2);

  find_and_highlight(s1, s2);
const int MATCH_CAP = 50;

int n_matches = (int)highlighted_intersections.size();
if (n_matches > MATCH_CAP) {
  // clear highlights created by find_and_highlight
  highlighted_intersections.clear();

  // if you also have other highlight vectors/sets, clear them too:
  // highlighted_street_segments.clear();
  // highlighted_pois.clear();
  // highlighted_features.clear();

  selected_intersection = -1;

  std::string msg = "Too many matches (" + std::to_string(n_matches) +
                    "). Please type 1–2 more letters.";
  app->update_message(msg);
  speak(msg); // optional

  app->refresh_drawing();
  return;
}
std::stringstream ss;

if (highlighted_intersections.empty()) {
  ss << "No intersections found between " << s1 << " and " << s2 << ".";
  selected_intersection = -1;
} else if (highlighted_intersections.size() == 1) {
  int id = highlighted_intersections[0];
  ss << "Intersection Found: " << intersections[id].name;
  selected_intersection = id;  // optional: also mark it as selected
} else {
  ss << "Found " << highlighted_intersections.size() << " intersections.";
  selected_intersection = highlighted_intersections[0];
}

app->update_message(ss.str());
  // ── TTS: announce find result ─────────────────────────────────────────
  if (!highlighted_intersections.empty()) {
    std::ostringstream msg;
    msg << "Found " << highlighted_intersections.size()
        << (highlighted_intersections.size() == 1 ? " intersection" : " intersections")
        << " between " << s1 << " and " << s2;
    speak(msg.str());
  } else {
    speak(std::string("No intersections found between ") + s1 + " and " + s2);
  }
  // ─────────────────────────────────────────────────────────────────────

// ── Popup: show full list if multiple results (non-blocking) ───────────
if (highlighted_intersections.size() > 1) {

  // Make sure highlights are visible first
  app->refresh_drawing();

  active_find_dialog = gtk_dialog_new_with_buttons(
      (std::to_string(highlighted_intersections.size()) +
       " Possible Intersections: " + s1 + " & " + s2).c_str(),
      GTK_WINDOW(app->get_object("MainWindow")),
      GTK_DIALOG_DESTROY_WITH_PARENT,
      "Close", GTK_RESPONSE_CLOSE,
      nullptr
  );

  gtk_window_set_default_size(GTK_WINDOW(active_find_dialog), 420, 360);

  GtkWidget *scroll = gtk_scrolled_window_new(nullptr, nullptr);
  gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll),
                                 GTK_POLICY_NEVER,
                                 GTK_POLICY_AUTOMATIC);

  GtkWidget *text_view = gtk_text_view_new();
  gtk_text_view_set_editable(GTK_TEXT_VIEW(text_view), FALSE);
  gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(text_view), FALSE);
  gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(text_view), GTK_WRAP_WORD);

  GtkTextBuffer *buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(text_view));
  std::ostringstream list_text;

  for (int i = 0; i < (int)highlighted_intersections.size(); i++) {
    list_text << (i + 1) << ".  "
              << intersections[highlighted_intersections[i]].name
              << "\n";
  }

  gtk_text_buffer_set_text(buf, list_text.str().c_str(), -1);

  gtk_container_add(GTK_CONTAINER(scroll), text_view);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(active_find_dialog))),
                     scroll, TRUE, TRUE, 0);

  // Non-blocking close behavior + reset pointer
  g_signal_connect(active_find_dialog, "response",
                   G_CALLBACK(+[](GtkDialog *d, gint, gpointer) {
                     gtk_widget_destroy(GTK_WIDGET(d));
                     active_find_dialog = nullptr;
                   }),
                   nullptr);

  gtk_widget_show_all(active_find_dialog);
}

  app->refresh_drawing();
}

static void zoom_fit_button(GtkWidget*, gpointer data) {
  auto* app = static_cast<ezgl::application*>(data);
  ezgl::canvas* c = app->get_canvas("MainCanvas");
  if (!c) return;

  c->get_camera().set_world(g_map_world);
  app->refresh_drawing();
}

static void zoom_out_button(GtkWidget*, gpointer data) {
  auto* app = static_cast<ezgl::application*>(data);
  ezgl::canvas* c = app->get_canvas("MainCanvas");
  if (!c) return;

  ezgl::rectangle cur = c->get_camera().get_world();

  double factor = 1.25;
  ezgl::point2d center = cur.center();
  double new_w = cur.width() * factor;
  double new_h = cur.height() * factor;

  ezgl::rectangle next(
      {center.x - new_w / 2.0, center.y - new_h / 2.0},
      {center.x + new_w / 2.0, center.y + new_h / 2.0}
  );

  if (next.width() >= g_map_world.width() || next.height() >= g_map_world.height()) {
    c->get_camera().set_world(g_map_world);
  } else {
    c->get_camera().set_world(next);
  }

  app->refresh_drawing();
}


void load_selected_map(GtkWidget*, gpointer data) {
  auto *app = static_cast<ezgl::application *>(data);

  GtkComboBoxText *combo = GTK_COMBO_BOX_TEXT(app->get_object("MapCombo"));
  if (!combo) return;

  int active_idx = gtk_combo_box_get_active(GTK_COMBO_BOX(combo));
  if (active_idx < 0 || active_idx >= static_cast<int>(discovered_map_paths.size())) return;

  std::string new_map_path = discovered_map_paths[active_idx];

  closeMap();
  closeOSMDatabase(); 

  bool load_success = loadMap(new_map_path);
  
  std::string osm_path = new_map_path;
  size_t pos = osm_path.find(".streets.bin");
  if (pos != std::string::npos) {
      osm_path.replace(pos, 12, ".osm.bin");
  }
  bool osm_success = loadOSMDatabaseBIN(osm_path);

  if (!load_success || !osm_success) {
    std::cerr << "Failed to load map or OSM data." << std::endl;
    return;
  }

  load_map_data();
  build_autocomplete_store();

  GtkEntry *top_search = GTK_ENTRY(app->get_object("TopSearch"));
  GtkEntry *e1 = GTK_ENTRY(app->get_object("Street1Entry"));
  GtkEntry *e2 = GTK_ENTRY(app->get_object("Street2Entry"));

  if (top_search) {
    GtkEntryCompletion *c = gtk_entry_get_completion(top_search);
    if (c) gtk_entry_completion_set_model(c, GTK_TREE_MODEL(autocomplete_store));
  }
  if (e1) {
    GtkEntryCompletion *c = gtk_entry_get_completion(e1);
    if (c) gtk_entry_completion_set_model(c, GTK_TREE_MODEL(autocomplete_store));
  }
  if (e2) {
    GtkEntryCompletion *c = gtk_entry_get_completion(e2);
    if (c) gtk_entry_completion_set_model(c, GTK_TREE_MODEL(autocomplete_store));
  }

  ezgl::rectangle new_world({xFromLon(global_minLon), yFromLat(global_minLat)},
                            {xFromLon(global_maxLon), yFromLat(global_maxLat)});
  g_map_world = new_world;

  ezgl::canvas *main_canvas = app->get_canvas("MainCanvas");
  if (main_canvas) {
    main_canvas->get_camera().set_world(new_world);
  }

  app->refresh_drawing();
}

static void on_night_mode_toggled(GObject *object, GParamSpec*, gpointer data) {
    is_night_mode = gtk_switch_get_active(GTK_SWITCH(object));
    auto *app = static_cast<ezgl::application *>(data);
    app->refresh_drawing();
}

void initial_setup(ezgl::application *app, bool) {
  GtkWidget *find_btn = GTK_WIDGET(app->get_object("FindButton"));
  if (find_btn) g_signal_connect(find_btn, "clicked", G_CALLBACK(find_button), app);

  GtkWidget* zoom_fit_btn = GTK_WIDGET(app->get_object("ZoomFitButton"));
  if (zoom_fit_btn) g_signal_connect(zoom_fit_btn, "clicked", G_CALLBACK(zoom_fit_button), app);

  GtkWidget* zoom_out_btn = GTK_WIDGET(app->get_object("ZoomOutButton"));
  if (zoom_out_btn) g_signal_connect(zoom_out_btn, "clicked", G_CALLBACK(zoom_out_button), app);

  GtkWidget *night_switch = GTK_WIDGET(app->get_object("NightModeSwitch"));
  if (night_switch) {
    gtk_switch_set_active(GTK_SWITCH(night_switch), is_night_mode);
    g_signal_connect(night_switch, "notify::active", G_CALLBACK(on_night_mode_toggled), app);
  }

  build_autocomplete_store();

  GtkEntry *top_search = GTK_ENTRY(app->get_object("TopSearch"));
  GtkEntry *e1 = GTK_ENTRY(app->get_object("Street1Entry"));
  GtkEntry *e2 = GTK_ENTRY(app->get_object("Street2Entry"));

  if (top_search) attach_autocomplete(top_search, app);
  //if (e1) attach_autocomplete(e1, app);
  //if (e2) attach_autocomplete(e2, app);

  discover_map_paths();
  GtkComboBoxText *map_combo = GTK_COMBO_BOX_TEXT(app->get_object("MapCombo"));
  GtkWidget *load_map_btn = GTK_WIDGET(app->get_object("LoadMapButton"));

  if (map_combo) {
    for (const auto &path : discovered_map_paths) {
      std::string filename = std::filesystem::path(path).filename().string();
      gtk_combo_box_text_append_text(map_combo, filename.c_str());
    }
  }

  if (load_map_btn) {
    g_signal_connect(load_map_btn, "clicked", G_CALLBACK(load_selected_map), app);
  }

   auto draw_legend_swatch = [](GtkWidget* widget, cairo_t* cr, gpointer color_ptr) -> gboolean {
    ezgl::color* col = static_cast<ezgl::color*>(color_ptr);
    cairo_set_source_rgb(cr, col->red/255.0, col->green/255.0, col->blue/255.0);
    cairo_arc(cr, 8, 8, 6, 0, 2 * M_PI);
    cairo_fill(cr);
    return FALSE;
  };

  static ezgl::color food_col(230, 50, 30);
  static ezgl::color health_col(220, 20, 120);
  static ezgl::color edu_col(20, 100, 220);
  static ezgl::color finance_col(10, 180, 60);
  static ezgl::color shop_col(200, 30, 200);
  static ezgl::color emergency_col(200, 0, 0);
  static ezgl::color transport_col(100, 100, 100);
  static ezgl::color accommodation_col(230, 140, 0);
  static ezgl::color services_col(50, 150, 200);
  static ezgl::color religion_col(140, 70, 180);

  GtkWidget* legend_food = GTK_WIDGET(app->get_object("LegendFood"));
  GtkWidget* legend_health = GTK_WIDGET(app->get_object("LegendHealth"));
  GtkWidget* legend_edu = GTK_WIDGET(app->get_object("LegendEducation"));
  GtkWidget* legend_finance = GTK_WIDGET(app->get_object("LegendFinance"));
  GtkWidget* legend_shop = GTK_WIDGET(app->get_object("LegendShopping"));
  GtkWidget* legend_emergency = GTK_WIDGET(app->get_object("LegendEmergency"));
  GtkWidget* legend_transport      = GTK_WIDGET(app->get_object("LegendTransport"));
  GtkWidget* legend_accommodation  = GTK_WIDGET(app->get_object("LegendAccommodation"));
  GtkWidget* legend_services       = GTK_WIDGET(app->get_object("LegendServices"));
  GtkWidget* legend_religion       = GTK_WIDGET(app->get_object("LegendReligion"));

  if (legend_food) g_signal_connect(legend_food, "draw", G_CALLBACK(+draw_legend_swatch), &food_col);
  if (legend_health) g_signal_connect(legend_health, "draw", G_CALLBACK(+draw_legend_swatch), &health_col);
  if (legend_edu) g_signal_connect(legend_edu, "draw", G_CALLBACK(+draw_legend_swatch), &edu_col);
  if (legend_finance) g_signal_connect(legend_finance, "draw", G_CALLBACK(+draw_legend_swatch), &finance_col);
  if (legend_shop) g_signal_connect(legend_shop, "draw", G_CALLBACK(+draw_legend_swatch), &shop_col);
  if (legend_emergency) g_signal_connect(legend_emergency, "draw", G_CALLBACK(+draw_legend_swatch), &emergency_col);
  if (legend_transport)     g_signal_connect(legend_transport,     "draw", G_CALLBACK(+draw_legend_swatch), &transport_col);
  if (legend_accommodation) g_signal_connect(legend_accommodation, "draw", G_CALLBACK(+draw_legend_swatch), &accommodation_col);
  if (legend_services)      g_signal_connect(legend_services,      "draw", G_CALLBACK(+draw_legend_swatch), &services_col);
  if (legend_religion)      g_signal_connect(legend_religion,      "draw", G_CALLBACK(+draw_legend_swatch), &religion_col);
}

ezgl::color get_feature_color(FeatureType type, bool night) {
    if (night) {
        switch (type) {
            case PARK: case GREENSPACE: case GOLFCOURSE: return ezgl::color(35, 75, 45);
            case LAKE: case RIVER: case STREAM: return ezgl::color(25, 50, 90);
            case BEACH: return ezgl::color(90, 80, 50);
            case ISLAND: return ezgl::color(40, 40, 40);
            case BUILDING: return ezgl::color(60, 60, 60);
            case GLACIER: return ezgl::color(150, 150, 180);
            default: return ezgl::color(50, 50, 50);
        }
    } else {
        switch (type) {
            case PARK: case GREENSPACE: case GOLFCOURSE: return ezgl::color(200, 238, 200);
            case LAKE: case RIVER: case STREAM: return ezgl::color(170, 218, 255);
            case BEACH: return ezgl::color(255, 240, 180);
            case ISLAND: return ezgl::color(240, 240, 240);
            case BUILDING: return ezgl::color(220, 220, 220);
            case GLACIER: return ezgl::color(255, 255, 255);
            default: return ezgl::color(230, 230, 230);
        }
    }
}

static PoiCategory classify_poi(const std::string& type) {
  static const std::unordered_set<std::string> food = {
    "restaurant","cafe","fast_food","bar","pub","food_court","ice_cream",
    "bakery","butcher"
  };
  static const std::unordered_set<std::string> health = {
    "hospital","clinic","doctors","dentist","pharmacy","veterinary"
  };
  static const std::unordered_set<std::string> education = {
    "school","university","college","kindergarten","library"
  };
  static const std::unordered_set<std::string> transport = {
    "bus_station","taxi","car_rental","fuel","parking","charging_station"
  };
  static const std::unordered_set<std::string> finance = {
    "bank","atm","bureau_de_change"
  };
  static const std::unordered_set<std::string> shopping = {
    "marketplace","convenience","supermarket","mall"
  };
  static const std::unordered_set<std::string> accommodation = {
    "hotel","motel","hostel"
  };
  static const std::unordered_set<std::string> services = {
    "post_office","laundry","beauty","hairdresser"
  };
  static const std::unordered_set<std::string> religion = {
    "place_of_worship","church","mosque","synagogue","temple"
  };
  static const std::unordered_set<std::string> emergency = {
    "police","fire_station"
  };

  // Distinct, vibrant colors that stand out from each other
  if (food.count(type))          return { ezgl::color(230, 50,  30),   "Food & Drink"    };
  if (health.count(type))        return { ezgl::color(220, 20,  120),  "Healthcare"      };
  if (education.count(type))     return { ezgl::color(20,  100, 220),  "Education"       };
  if (transport.count(type))     return { ezgl::color(100, 100, 100),  "Transport"       };
  if (finance.count(type))       return { ezgl::color(10,  180, 60),   "Finance"         };
  if (shopping.count(type))      return { ezgl::color(200, 30,  200),  "Shopping"        };
  if (accommodation.count(type)) return { ezgl::color(230, 140, 0),    "Accommodation"   };
  if (services.count(type))      return { ezgl::color(50,  150, 200),  "Services"        };
  if (religion.count(type))      return { ezgl::color(140, 70,  180),  "Religion"        };
  if (emergency.count(type))     return { ezgl::color(200, 0,   0),    "Emergency"       };
  
  return { ezgl::color(120, 120, 120), "Other" };
}

// ----------------------------------------------------------------------------
// Core Drawing Function
// ----------------------------------------------------------------------------

void draw_main_canvas(ezgl::renderer *g) {
  ezgl::rectangle visible_world = g->get_visible_world();

  if (is_night_mode) g->set_color(30, 30, 30);
  else g->set_color(240, 240, 240);
  
  g->fill_rectangle(visible_world);

  for (const auto &feat : features) {
    g->set_color(get_feature_color(feat.type, is_night_mode));
    if (feat.is_closed && feat.points.size() > 2) {
      g->fill_poly(feat.points);
    } else {
      g->set_line_width(1);
      for (size_t i = 0; i < feat.points.size() - 1; i++) {
        g->draw_line(feat.points[i], feat.points[i + 1]);
      }
    }
  }

  double current_zoom_width = visible_world.width();
  
  std::vector<std::pair<StreetSegmentIdx, const streetSegments*>> oneway_segments;
  
  for (int idx = 0; idx < (int)streets.size(); ++idx) {
    const auto &seg = streets[idx];
    float speed_kmh = seg.speedLimit * 3.6f;
    if (current_zoom_width > 15000 && speed_kmh <= 50) continue;
    if (current_zoom_width > 5000 && speed_kmh <= 30) continue;

    if (is_night_mode) {
      if (speed_kmh >= 80) { g->set_color(ezgl::color(180, 100, 20)); g->set_line_width(3); }
      else if (speed_kmh >= 60) { g->set_color(ezgl::color(50, 100, 180)); g->set_line_width(2); }
      else { g->set_color(ezgl::color(100, 100, 100)); g->set_line_width(3); }
    } else {
      if (speed_kmh >= 80) { g->set_color(ezgl::ORANGE); g->set_line_width(3); }
      else if (speed_kmh >= 60) { g->set_color(ezgl::YELLOW); g->set_line_width(2); }
      else { g->set_color(250, 250, 250); g->set_line_width(3); }
    }

    for (size_t i = 0; i < seg.points.size() - 1; i++) {
      g->draw_line(seg.points[i], seg.points[i + 1]);
    }
    
    StreetSegmentInfo info = getStreetSegmentInfo(idx);
    if (info.oneWay && speed_kmh < 80) {
      oneway_segments.push_back({idx, &seg});
    }
  }

  if (current_zoom_width < 8000) {
    std::vector<ezgl::point2d> arrow_positions;
    const double MIN_ARROW_DISTANCE = 120.0;
    
    int skip_factor = 1;
    if (current_zoom_width > 4000) skip_factor = 3;
    else if (current_zoom_width > 2000) skip_factor = 2;
    else skip_factor = 1;
    
    int arrow_counter = 0;
    
    for (const auto &pair : oneway_segments) {
      if (arrow_counter % skip_factor != 0) {
        arrow_counter++;
        continue;
      }
      arrow_counter++;
      
      int seg_idx = pair.first;
      const streetSegments* seg = pair.second;
      
      StreetSegmentInfo info = getStreetSegmentInfo(seg_idx);
      
      if (seg->points.size() < 2) continue;
      
      size_t mid_idx = seg->points.size() / 2;
      
      ezgl::point2d start_pt, end_pt;
      if (seg->points.size() == 2) {
        start_pt = seg->points[0];
        end_pt = seg->points[1];
      } else {
        start_pt = seg->points[mid_idx - 1];
        end_pt = seg->points[mid_idx];
      }
      
      if (!visible_world.contains(start_pt) && !visible_world.contains(end_pt)) {
        continue;
      }
      
      double dx = end_pt.x - start_pt.x;
      double dy = end_pt.y - start_pt.y;
      double len = std::sqrt(dx * dx + dy * dy);
      
      if (len < 1.0) continue;
      
      dx /= len;
      dy /= len;
      
      ezgl::point2d arrow_pos = {
        (start_pt.x + end_pt.x) / 2.0,
        (start_pt.y + end_pt.y) / 2.0
      };
      
      bool overlap_found = true;
      int max_attempts = 5;
      int attempt = 0;
      
      while (overlap_found && attempt < max_attempts) {
        overlap_found = false;
        
        for (const auto &existing_pos : arrow_positions) {
          double dist = std::sqrt(
            (arrow_pos.x - existing_pos.x) * (arrow_pos.x - existing_pos.x) +
            (arrow_pos.y - existing_pos.y) * (arrow_pos.y - existing_pos.y)
          );
          
          if (dist < MIN_ARROW_DISTANCE) {
            overlap_found = true;
            break;
          }
        }
        
        if (overlap_found) {
          attempt++;
          double offset = (attempt % 2 == 0 ? 1 : -1) * attempt * 50.0;
          
          ezgl::point2d test_pos = {
            (start_pt.x + end_pt.x) / 2.0 + dx * offset,
            (start_pt.y + end_pt.y) / 2.0 + dy * offset
          };
          
          double t = 0.5 + offset / len;
          
          if (t >= 0.1 && t <= 0.9) {
            arrow_pos = test_pos;
          } else {
            break;
          }
        }
      }
      
      if (overlap_found) continue;
      
      arrow_positions.push_back(arrow_pos);
      
      double arrow_length = std::min(60.0, current_zoom_width / 100.0);
      double arrow_width = arrow_length * 0.6;
      
      ezgl::point2d tip = {
        arrow_pos.x + dx * arrow_length,
        arrow_pos.y + dy * arrow_length
      };
      
      double perp_x = -dy;
      double perp_y = dx;
      
      ezgl::point2d base1 = {
        arrow_pos.x + perp_x * arrow_width / 2.0,
        arrow_pos.y + perp_y * arrow_width / 2.0
      };
      
      ezgl::point2d base2 = {
        arrow_pos.x - perp_x * arrow_width / 2.0,
        arrow_pos.y - perp_y * arrow_width / 2.0
      };
      
      std::vector<ezgl::point2d> arrow_points = {tip, base1, base2};
      
      if (is_night_mode) g->set_color(ezgl::WHITE);
      else g->set_color(ezgl::BLACK);
      
      g->fill_poly(arrow_points);
      
      if (is_night_mode) g->set_color(ezgl::BLACK);
      else g->set_color(ezgl::WHITE);
      g->set_line_width(1);
      g->draw_line(tip, base1);
      g->draw_line(base1, base2);
      g->draw_line(base2, tip);
    }
  }

  if (current_zoom_width < 25000) { 
      g->set_line_width(4); 
      
      for (const auto& line : subway_lines) {
          g->set_color(line.color);
          
          for (const auto& track : line.tracks) {
              for (size_t i = 0; i < track.size() - 1; ++i) {
                  g->draw_line(track[i], track[i + 1]);
              }
          }
      }
  }
  
  if (current_zoom_width < 15000) { 
      for (const auto& station : subway_stations) {
          g->set_color(ezgl::WHITE);
          g->fill_arc(station.position, 15, 0, 360);
          
          g->set_color(ezgl::BLACK);
          g->set_line_width(1);
          g->draw_arc(station.position, 15, 0, 360);

          if (current_zoom_width < 6000 && !station.name.empty()) {
              if (is_night_mode) g->set_color(ezgl::WHITE);
              else g->set_color(ezgl::BLACK);
              
              g->set_font_size(10);
              g->draw_text({station.position.x, station.position.y + 25}, station.name); 
          }
      }
  }

  if (is_night_mode) g->set_color(ezgl::WHITE);
  else g->set_color(ezgl::BLACK);
  
  g->set_font_size(9);
  std::unordered_set<std::string> names_drawn_this_frame;

  for (const auto &seg : streets) {
    float speed_kmh = seg.speedLimit * 3.6f;
    if (names_drawn_this_frame.count(seg.name)) continue;
    if (current_zoom_width > 15000) continue;
    if (current_zoom_width > 8000 && speed_kmh < 70) continue;
    if (current_zoom_width > 3000 && speed_kmh < 40) continue;
    if (seg.name == "<unknown>") continue;

    if (seg.points.size() >= 2) {
      size_t mid_idx = seg.points.size() / 2;
      ezgl::point2d p1 = seg.points[mid_idx - 1];
      ezgl::point2d p2 = seg.points[mid_idx];

      if (visible_world.contains(p1) || visible_world.contains(p2)) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double seg_len = std::sqrt(dx * dx + dy * dy);
        if (seg_len < (seg.name.length() * 7)) continue;

        double angle = atan2(dy, dx) * 180 / M_PI;
        if (angle > 90) angle -= 180;
        else if (angle < -90) angle += 180;

        g->set_text_rotation(angle);
        g->draw_text({(p1.x + p2.x) / 2, (p1.y + p2.y) / 2}, seg.name);
        names_drawn_this_frame.insert(seg.name);
      }
    }
  }
  g->set_text_rotation(0);

    for (int id : highlighted_intersections) {
    ezgl::point2d c = {intersections[id].x, intersections[id].y};

    if (!visible_world.contains(c)) continue;

    // Magenta halo
    g->set_color(ezgl::color(255, 0, 255, 60));
    g->fill_arc(c, 70, 0, 360);

    // White inner circle
    g->set_color(ezgl::WHITE);
    g->fill_arc(c, 25, 0, 360);

    // Magenta ring
    g->set_color(ezgl::color(255, 0, 255));
    g->set_line_width(3);
    g->draw_arc(c, 40, 0, 360);
    }

  if (selected_intersection != -1) {
    ezgl::point2d center = {intersections[selected_intersection].x, intersections[selected_intersection].y};
    g->set_color(ezgl::RED);
    g->fill_arc(center, 3, 0, 360);
  }

   if (visible_world.width() < 5000) {
    // Adaptive sizing based on zoom
    double base_radius = 16.0;
    double label_threshold = 1200.0;
    double cluster_distance = 80.0;
    
    if (visible_world.width() > 3000) {
      base_radius = 18.0;
      cluster_distance = 120.0;
      label_threshold = 999999.0; // Don't show labels when far
    } else if (visible_world.width() > 1500) {
      base_radius = 16.0;
      cluster_distance = 40.0;
      label_threshold = 2000.0;
    } else {
      base_radius = 14.0;
      cluster_distance = 0.0;
      label_threshold = 1200.0;
    }
    
    // Track drawn POIs to prevent overlap
    std::vector<std::pair<ezgl::point2d, double>> drawn_pois; // pos, radius
    
    // Count POIs by category for clustering
    struct ClusterInfo {
      ezgl::point2d center;
      std::vector<int> poi_indices;
      PoiCategory category;
    };
    std::vector<ClusterInfo> clusters;
    std::vector<bool> poi_processed(Mypois.size(), false);
    
    // First pass: Create clusters
    for (int i = 0; i < (int)Mypois.size(); i++) {
      if (poi_processed[i]) continue;
      if (!visible_world.contains(Mypois[i].x, Mypois[i].y)) continue;
      
      PoiCategory cat = classify_poi(getPOIType(i));
      
      // Start new cluster
      ClusterInfo cluster;
      cluster.center = {Mypois[i].x, Mypois[i].y};
      cluster.poi_indices.push_back(i);
      cluster.category = cat;
      poi_processed[i] = true;
      
      // Find nearby POIs of same category
      for (int j = i + 1; j < (int)Mypois.size(); j++) {
        if (poi_processed[j]) continue;
        
        PoiCategory cat2 = classify_poi(getPOIType(j));
        
        // Only cluster same category
        if (cat.label != cat2.label) continue;
        
        double dx = Mypois[j].x - Mypois[i].x;
        double dy = Mypois[j].y - Mypois[i].y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < cluster_distance) {
          cluster.poi_indices.push_back(j);
          poi_processed[j] = true;
          
          // Update cluster center (average position)
          double sum_x = 0, sum_y = 0;
          for (int idx : cluster.poi_indices) {
            sum_x += Mypois[idx].x;
            sum_y += Mypois[idx].y;
          }
          cluster.center.x = sum_x / cluster.poi_indices.size();
          cluster.center.y = sum_y / cluster.poi_indices.size();
        }
      }
      
      clusters.push_back(cluster);
    }
    
    // Second pass: Draw clusters
    for (const auto& cluster : clusters) {
      int count = cluster.poi_indices.size();

      // When zoomed in tightly, skip clusters of size > 1 and draw individuals instead
      // (they will each be their own cluster of 1 at this zoom level naturally,
      //  but we reduce cluster_distance above — so this just draws every dot cleanly)

      double radius = base_radius;
      // No scaling by count — always same size dot

      // Check overlap with already drawn
      bool overlaps = false;
      for (const auto& drawn : drawn_pois) {
        double dx = cluster.center.x - drawn.first.x;
        double dy = cluster.center.y - drawn.first.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < (radius + drawn.second + 10)) {
          overlaps = true;
          break;
        }
      }

      if (overlaps) continue;

      // Draw plain color dot — no count badge
      g->set_color(cluster.category.color);
      g->fill_arc(cluster.center, radius, 0, 360);

      // White border
      g->set_color(ezgl::WHITE);
      g->set_line_width(2);
      g->draw_arc(cluster.center, radius, 0, 360);

      // Draw label when zoomed in close on individual POIs
      if (visible_world.width() < label_threshold && count == 1) {
        std::string name = Mypois[cluster.poi_indices[0]].name;
        if (!name.empty() && name != "<unknown>") {
          if (is_night_mode) g->set_color(ezgl::WHITE);
          else g->set_color(ezgl::BLACK);
          g->set_font_size(9);
          g->draw_text({cluster.center.x, cluster.center.y + radius + 10}, name);
        }
      }

      drawn_pois.push_back({cluster.center, radius});
    }

    if (search_result_is_poi) {
    ezgl::point2d c = {search_result_x, search_result_y};
    g->set_color(ezgl::color(255, 0, 255, 80));  // soft halo
    g->fill_arc(c, 60, 0, 360);
    g->set_color(ezgl::WHITE);                  // inner
    g->fill_arc(c, 20, 0, 360);
    g->set_color(ezgl::color(255, 0, 255));     // ring
    g->set_line_width(3);
    g->draw_arc(c, 30, 0, 360);

    } else if (!search_result_is_poi && search_result_intersection >= 0) {

    float sx = intersections[search_result_intersection].x;
    float sy = intersections[search_result_intersection].y;
    ezgl::point2d c = {sx, sy};
    g->set_color(ezgl::color(255, 0, 255, 80));
    g->fill_arc(c, 60, 0, 360);
    g->set_color(ezgl::WHITE);
    g->fill_arc(c, 20, 0, 360);
    g->set_color(ezgl::color(255, 0, 255));
    g->set_line_width(3);
    g->draw_arc(c, 30, 0, 360);
  }
}
}

void drawMap() {
  load_map_data();

  ezgl::application::settings settings;
  settings.main_ui_resource = "libstreetmap/resources/main.ui";
  settings.window_identifier = "MainWindow";
  settings.canvas_identifier = "MainCanvas";

  ezgl::application application(settings);

  ezgl::rectangle initial_world(
      {xFromLon(global_minLon), yFromLat(global_minLat)},
      {xFromLon(global_maxLon), yFromLat(global_maxLat)}
  );
  g_map_world = initial_world;

  application.add_canvas("MainCanvas", draw_main_canvas, initial_world);
  application.run(initial_setup, act_on_mouse_click, nullptr, nullptr);
}