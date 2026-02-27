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
#include <filesystem> // Added for finding maps
#include <set>        // Added for map deduplication

double cos_lat_avg;
void draw_main_canvas (ezgl::renderer *g);

struct Intersection{
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
    ezgl::color color;
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

// Coordinate conversion declarations
float xFromLon(float lon);
float yFromLat(float lat);
float lonFromX(float x);
float latFromY(float y);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);

// ----------------------------------------------------------------------------
// Coordinate Conversion Functions
// ----------------------------------------------------------------------------
float xFromLon(float lon){
   return kEarthRadiusInMeters * (lon * kDegreeToRadian) * cos_lat_avg;
}
float yFromLat(float lat){
   return kEarthRadiusInMeters * (lat * kDegreeToRadian);
}
float lonFromX(float x){
   return x / (kEarthRadiusInMeters * kDegreeToRadian * cos_lat_avg);
}
float latFromY(float y){
   return y / (kEarthRadiusInMeters * kDegreeToRadian);
}

// ----------------------------------------------------------------------------
// Data Loading & Map Discovery Functions
// ----------------------------------------------------------------------------

// Helper to check file extensions
bool ends_with(const std::string &text, const std::string &suffix) {
    if(suffix.size() > text.size()) return false;
    return text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Scans directories for map files
void discover_map_paths() {
    namespace fs = std::filesystem;
    std::set<std::string> unique_paths;
    discovered_map_paths.clear();

    auto add_maps_under = [&](const fs::path &root, int max_depth) {
        std::error_code ec;
        if(!fs::exists(root, ec)) return;
        fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec);
        fs::recursive_directory_iterator end;
        while(it != end) {
            if(ec) { ec.clear(); it.increment(ec); continue; }
            if(it.depth() > max_depth) { it.disable_recursion_pending(); it.increment(ec); continue; }
            if(it->is_regular_file(ec)) {
                std::string file_name = it->path().filename().string();
                if(ends_with(file_name, ".streets.bin")) {
                    std::string normalized = it->path().lexically_normal().string();
                    if(unique_paths.insert(normalized).second) {
                        discovered_map_paths.push_back(normalized);
                    }
                }
            }
            it.increment(ec);
        }
    };

    // Standard UofT ECE297 map locations
    add_maps_under(fs::path("."), 4);
    add_maps_under(fs::path("/cad2/ece297s/public/maps"), 2);
    
    // Sort alphabetically
    std::sort(discovered_map_paths.begin(), discovered_map_paths.end(), [](const std::string &a, const std::string &b) {
        return fs::path(a).filename().string() < fs::path(b).filename().string();
    });
}

// Extracts the heavy lifting out of drawMap() so it can be called on reload
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
    for(int i = 0; i < getNumIntersections(); ++i){
       intersections[i].position = getIntersectionPosition(i);
       intersections[i].name = getIntersectionName(i); 

       global_maxLat = std::max(global_maxLat, intersections[i].position.latitude()); 
       global_minLat = std::min(global_minLat, intersections[i].position.latitude()); 
       global_maxLon = std::max(global_maxLon, intersections[i].position.longitude()); 
       global_minLon = std::min(global_minLon, intersections[i].position.longitude()); 
    }

    double avgLat = ((global_maxLat + global_minLat) / 2.0) * kDegreeToRadian;
    cos_lat_avg = cos(avgLat);

    for(int i = 0; i < getNumIntersections(); ++i){
       intersections[i].x = xFromLon(intersections[i].position.longitude());
       intersections[i].y = yFromLat(intersections[i].position.latitude());
    }

    streets.clear(); 
    int numSegments = getNumStreetSegments();
    streets.resize(numSegments);
    for (int i = 0; i < numSegments; i++){
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

        switch (type) {
            case PARK: case GREENSPACE: case GOLFCOURSE: feat.color = ezgl::color(200, 238, 200); break;
            case LAKE: case RIVER: case STREAM: feat.color = ezgl::color(170, 218, 255); break;
            case BEACH: feat.color = ezgl::color(255, 240, 180); break;
            case ISLAND: feat.color = ezgl::color(240, 240, 240); break;
            case BUILDING: feat.color = ezgl::color(220, 220, 220); break;
            case GLACIER: feat.color = ezgl::color(255, 255, 255); break;
            default: feat.color = ezgl::color(230, 230, 230); break;
        }
        
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
}

// ----------------------------------------------------------------------------
// Event Handlers & Callbacks
// ----------------------------------------------------------------------------

void act_on_mouse_click(ezgl::application* app, GdkEventButton*, double x, double y) {
  LatLon clicked_pos(latFromY(y), lonFromX(x));
  selected_intersection = findClosestIntersection(clicked_pos);
  if(selected_intersection != -1){       
      std::stringstream ss; 
      ss << "Intersection Clicked: " << intersections[selected_intersection].name;
      app->update_message(ss.str());
   }
  app->refresh_drawing();
}

static void find_and_highlight(const std::string& street1, const std::string& street2) {
    highlighted_intersections.clear();
    auto s1 = findStreetIdsFromPartialStreetName(street1);
    auto s2 = findStreetIdsFromPartialStreetName(street2);

    if(s1.empty() || s2.empty()) { std::cout << "Street not found.\n"; return; }

    for(int id1 : s1) {
        for(int id2 : s2) {
            auto ints = findIntersectionsOfTwoStreets(id1, id2);
            if(!ints.empty()) {
                highlighted_intersections = std::move(ints);
                std::cout << "Found " << highlighted_intersections.size() << " intersections.\n";
                return;
            }
        }
    }
    std::cout << "Found 0 intersections.\n";
}

static void find_button(GtkWidget*, gpointer data) {
    auto* app = static_cast<ezgl::application*>(data);
    GtkEntry* e1 = GTK_ENTRY(app->get_object("Street1Entry"));
    GtkEntry* e2 = GTK_ENTRY(app->get_object("Street2Entry"));

    std::string s1 = gtk_entry_get_text(e1);
    std::string s2 = gtk_entry_get_text(e2);

    find_and_highlight(s1, s2);
    app->refresh_drawing();
}

// Map Loading Callback
void load_selected_map(GtkWidget* /*widget*/, gpointer data) {
    auto* app = static_cast<ezgl::application*>(data);
    
    // Get the combobox from Glade
    GtkComboBoxText* combo = GTK_COMBO_BOX_TEXT(app->get_object("MapCombo"));
    if (!combo) return;

    // Figure out which map the user selected
    int active_idx = gtk_combo_box_get_active(GTK_COMBO_BOX(combo));
    if (active_idx < 0 || active_idx >= static_cast<int>(discovered_map_paths.size())) return;

    std::string new_map_path = discovered_map_paths[active_idx];
    
    // Unload current map, load new map
    closeMap();
    
    bool load_success = loadMap(new_map_path);
    if (!load_success) {
        std::cerr << "Failed to load map: " << new_map_path << std::endl;
        return;
    }

    // Refresh all global data
    load_map_data();

    // Recalculate the boundaries and reset the camera
    ezgl::rectangle new_world({xFromLon(global_minLon), yFromLat(global_minLat)}, 
                              {xFromLon(global_maxLon), yFromLat(global_maxLat)});
    
    ezgl::canvas* main_canvas = app->get_canvas("MainCanvas");
    if (main_canvas) {
        main_canvas->get_camera().set_world(new_world);
    }
    
    app->refresh_drawing();
}

void initial_setup(ezgl::application* app, bool) {
    // Connect Find Button
    GtkWidget* find_btn = GTK_WIDGET(app->get_object("FindButton"));
    if (find_btn) g_signal_connect(find_btn, "clicked", G_CALLBACK(find_button), app);

    // Populate Map Dropdown
    discover_map_paths();
    GtkComboBoxText* map_combo = GTK_COMBO_BOX_TEXT(app->get_object("MapCombo"));
    GtkWidget* load_map_btn = GTK_WIDGET(app->get_object("LoadMapButton"));

    if (map_combo) {
        for (const auto& path : discovered_map_paths) {
            std::string filename = std::filesystem::path(path).filename().string();
            gtk_combo_box_text_append_text(map_combo, filename.c_str());
        }
    }

    // Connect Load Map Button
    if (load_map_btn) {
        g_signal_connect(load_map_btn, "clicked", G_CALLBACK(load_selected_map), app);
    }
}

// ----------------------------------------------------------------------------
// Core Drawing Function
// ----------------------------------------------------------------------------
void draw_main_canvas(ezgl::renderer *g) {
    // (Keep your exact draw_main_canvas logic here as it was)
    auto startTime = std::chrono::high_resolution_clock::now();
    ezgl::rectangle visible_world = g->get_visible_world();
    
    g->set_color(240, 240, 240); 
    g->fill_rectangle(visible_world);

    for (const auto& feat : features) {
        g->set_color(feat.color);
        if (feat.is_closed && feat.points.size() > 2) {
            g->fill_poly(feat.points);
        } else {
            g->set_line_width(1);
            for (size_t i = 0; i < feat.points.size() - 1; i++) {
                g->draw_line(feat.points[i], feat.points[i+1]);
            }
        }
    }

    double current_zoom_width = visible_world.width();
    for (const auto& seg : streets) {
        float speed_kmh = seg.speedLimit * 3.6f;
        if (current_zoom_width > 15000 && speed_kmh <= 50) continue; 
        if (current_zoom_width > 5000 && speed_kmh <= 30) continue;
        if (speed_kmh >= 80) { g->set_color(ezgl::ORANGE); g->set_line_width(3); } 
        else if (speed_kmh >= 60) { g->set_color(ezgl::BLUE); g->set_line_width(2); } 
        else { g->set_color(250, 250, 250); g->set_line_width(3); }

        for (size_t i = 0; i < seg.points.size() - 1; i++) {
            g->draw_line(seg.points[i], seg.points[i+1]);
        }
    }

    g->set_color(ezgl::BLACK);
    g->set_font_size(9);
    std::unordered_set<std::string> names_drawn_this_frame;

    for (const auto& seg : streets) {
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
                double seg_len = std::sqrt(dx*dx + dy*dy);
                if (seg_len < (seg.name.length() * 7)) continue; 

                double angle = atan2(dy, dx) * 180 / M_PI;
                if (angle > 90) angle -= 180;
                else if (angle < -90) angle += 180;

                g->set_text_rotation(angle);
                g->draw_text({(p1.x + p2.x)/2, (p1.y + p2.y)/2}, seg.name);
                names_drawn_this_frame.insert(seg.name);
            }
        }
    }
    g->set_text_rotation(0); 

    g->set_color(0, 0, 0);  
    for(size_t i = 0; i < intersections.size(); ++i){
       float x = intersections[i].x; 
       float y = intersections[i].y; 
       float width = 0; float height = width; 
       ezgl::point2d inter_loc = {x - width / 2.0f, y - height / 2.0f};
       g->fill_rectangle(inter_loc, width, height);
    }

    g->set_color(ezgl::YELLOW);
    for(int id : highlighted_intersections) {
       ezgl::point2d center = { intersections[id].x, intersections[id].y };
       g->fill_arc(center, 50, 0, 360);
    }

    if(selected_intersection != -1) {
       ezgl::point2d center = { intersections[selected_intersection].x, intersections[selected_intersection].y };
       g->set_color(ezgl::RED);
       g->fill_arc(center, 3, 0, 360);
    }

    if (visible_world.width() < 5000) { 
        g->set_color(ezgl::BLUE);
        for (const auto& poi : Mypois) {
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
   // 1. Initial Load of the currently active map (loaded in main.cpp)
   load_map_data();

   // 2. Setup ezgl
   ezgl::application::settings settings;
   settings.main_ui_resource = "libstreetmap/resources/main.ui"; 
   settings.window_identifier = "MainWindow";
   settings.canvas_identifier = "MainCanvas";
   
   ezgl::application application(settings);

   // 3. Set Initial Boundaries based on our global max/mins calculated in load_map_data()
   ezgl::rectangle initial_world({xFromLon(global_minLon), yFromLat(global_minLat)}, 
                                 {xFromLon(global_maxLon), yFromLat(global_maxLat)});
   
   application.add_canvas("MainCanvas", draw_main_canvas, initial_world);
   application.run(initial_setup, act_on_mouse_click, nullptr, nullptr);
}