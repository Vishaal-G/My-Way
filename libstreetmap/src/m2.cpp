/* * Copyright 2026 University of Toronto ... (Header omitted for brevity) */

#include "m2.h"
#include "m1.hpp"
#include <gtk/gtk.h>
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <unordered_set>
#include <filesystem>
#include <set>

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

// Coordinate conversion declarations
float xFromLon(float lon);
float yFromLat(float lat);
float lonFromX(float x);
float latFromY(float y);
void act_on_mouse_click(ezgl::application *app, GdkEventButton *event, double x, double y);

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

    // Save the type directly instead of resolving the color here
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
      G_TYPE_STRING, // COL_NAME
      G_TYPE_INT,    // COL_TYPE
      G_TYPE_INT     // COL_IDX
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
    gtk_list_store_append(autocomplete_store, &iter);
    gtk_list_store_set(autocomplete_store, &iter,
                       COL_NAME, Mypois[i].name.c_str(),
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

  if (type == 0) {
    auto street_ids = findStreetIdsFromPartialStreetName(name ? name : "");
    highlighted_intersections.clear();
    selected_intersection = -1;
    if (!street_ids.empty()) {
      auto ints = findIntersectionsOfStreet(street_ids[0]);
      highlighted_intersections = ints;
      if (!ints.empty()) selected_intersection = ints[0];
    }
  } else if (type == 1) {
    selected_intersection = -1;
    highlighted_intersections.clear();
    ezgl::canvas *c = app->get_canvas("MainCanvas");
    if (c && idx < (int)Mypois.size()) {
      float cx = Mypois[idx].x;
      float cy = Mypois[idx].y;
      float half = 500.0f;
      ezgl::rectangle zoom_to({cx - half, cy - half}, {cx + half, cy + half});
      c->get_camera().set_world(zoom_to);
    }
  } else if (type == 2) {
    selected_intersection = idx;
    highlighted_intersections.clear();
    std::stringstream ss;
    ss << "Intersection: " << intersections[idx].name;
    app->update_message(ss.str());
  }

  if (name) g_free(name);
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
  LatLon clicked_pos(latFromY(y), lonFromX(x));
  selected_intersection = findClosestIntersection(clicked_pos);
  if (selected_intersection != -1) {
    std::stringstream ss;
    ss << "Intersection Clicked: " << intersections[selected_intersection].name;
    app->update_message(ss.str());
  }
  app->refresh_drawing();
}

static void find_and_highlight(const std::string &street1, const std::string &street2) {
  highlighted_intersections.clear();
  auto s1 = findStreetIdsFromPartialStreetName(street1);
  auto s2 = findStreetIdsFromPartialStreetName(street2);

  if (s1.empty() || s2.empty()) { std::cout << "Street not found.\n"; return; }

  for (int id1 : s1) {
    for (int id2 : s2) {
      auto ints = findIntersectionsOfTwoStreets(id1, id2);
      if (!ints.empty()) {
        highlighted_intersections = std::move(ints);
        std::cout << "Found " << highlighted_intersections.size() << " intersections.\n";
        return;
      }
    }
  }
  std::cout << "Found 0 intersections.\n";
}

static void find_button(GtkWidget *, gpointer data) {
  auto *app = static_cast<ezgl::application *>(data);
  GtkEntry *e1 = GTK_ENTRY(app->get_object("Street1Entry"));
  GtkEntry *e2 = GTK_ENTRY(app->get_object("Street2Entry"));

  std::string s1 = gtk_entry_get_text(e1);
  std::string s2 = gtk_entry_get_text(e2);

  find_and_highlight(s1, s2);
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

  bool load_success = loadMap(new_map_path);
  if (!load_success) {
    std::cerr << "Failed to load map: " << new_map_path << std::endl;
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

static void on_night_mode_toggled(GObject *object, GParamSpec *pspec, gpointer data) {
    // Update our global state to match the switch
    is_night_mode = gtk_switch_get_active(GTK_SWITCH(object));
    
    // Force the map to redraw with the new colors
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
    // Sync the switch to our default state (false/off)
    gtk_switch_set_active(GTK_SWITCH(night_switch), is_night_mode);
    
    // Connect the signal that fires when the user toggles it
    g_signal_connect(night_switch, "notify::active", G_CALLBACK(on_night_mode_toggled), app);
  }

  build_autocomplete_store();

  GtkEntry *top_search = GTK_ENTRY(app->get_object("TopSearch"));
  GtkEntry *e1 = GTK_ENTRY(app->get_object("Street1Entry"));
  GtkEntry *e2 = GTK_ENTRY(app->get_object("Street2Entry"));

  if (top_search) attach_autocomplete(top_search, app);
  if (e1) attach_autocomplete(e1, app);
  if (e2) attach_autocomplete(e2, app);

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

}

ezgl::color get_feature_color(FeatureType type, bool night) {
    if (night) {
        switch (type) {
            case PARK: case GREENSPACE: case GOLFCOURSE: return ezgl::color(35, 75, 45); // Dark green
            case LAKE: case RIVER: case STREAM: return ezgl::color(25, 50, 90); // Dark blue
            case BEACH: return ezgl::color(90, 80, 50); // Dark sand
            case ISLAND: return ezgl::color(40, 40, 40); // Dark land
            case BUILDING: return ezgl::color(60, 60, 60); // Dark grey
            case GLACIER: return ezgl::color(150, 150, 180); // Dark ice
            default: return ezgl::color(50, 50, 50); // Dark generic
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

// ----------------------------------------------------------------------------
// Core Drawing Function
// ----------------------------------------------------------------------------

void draw_main_canvas(ezgl::renderer *g) {
  ezgl::rectangle visible_world = g->get_visible_world();

  // 1. Background Color
  if (is_night_mode) g->set_color(30, 30, 30);
  else g->set_color(240, 240, 240);
  
  g->fill_rectangle(visible_world);

  // 2. Feature Colors
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

  // 3. Street Colors
  double current_zoom_width = visible_world.width();
  for (const auto &seg : streets) {
    float speed_kmh = seg.speedLimit * 3.6f;
    if (current_zoom_width > 15000 && speed_kmh <= 50) continue;
    if (current_zoom_width > 5000 && speed_kmh <= 30) continue;

    if (is_night_mode) {
      if (speed_kmh >= 80) { g->set_color(ezgl::color(180, 100, 20)); g->set_line_width(3); } // Dark orange
      else if (speed_kmh >= 60) { g->set_color(ezgl::color(50, 100, 180)); g->set_line_width(2); } // Dark blue
      else { g->set_color(ezgl::color(100, 100, 100)); g->set_line_width(3); } // Dark grey
    } else {
      if (speed_kmh >= 80) { g->set_color(ezgl::ORANGE); g->set_line_width(3); }
      else if (speed_kmh >= 60) { g->set_color(ezgl::BLUE); g->set_line_width(2); }
      else { g->set_color(250, 250, 250); g->set_line_width(3); }
    }

    for (size_t i = 0; i < seg.points.size() - 1; i++) {
      g->draw_line(seg.points[i], seg.points[i + 1]);
    }
  }

  // 4. Text Colors
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

  g->set_color(ezgl::YELLOW);
  for (int id : highlighted_intersections) {
    ezgl::point2d center = {intersections[id].x, intersections[id].y};
    g->fill_arc(center, 50, 0, 360);
  }

  if (selected_intersection != -1) {
    ezgl::point2d center = {intersections[selected_intersection].x, intersections[selected_intersection].y};
    g->set_color(ezgl::RED);
    g->fill_arc(center, 3, 0, 360);
  }

  if (visible_world.width() < 5000) {
    g->set_color(ezgl::BLUE);
    for (const auto &poi : Mypois) {
      if (visible_world.contains(poi.x, poi.y)) {
        g->fill_arc({poi.x, poi.y}, 20, 0, 360);
        if (visible_world.width() < 1000) {
          g->set_color(ezgl::BLACK);
          g->draw_text({poi.x, poi.y + 25}, poi.name);
          g->set_color(ezgl::BLUE);
        }
      }
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