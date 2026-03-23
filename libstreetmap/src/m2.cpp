/*
 * Copyright 2026 University of Toronto
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * with the Software in accordance with the terms of the ECE297 course.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 */

/*
 * m2.cpp
 *
 * This file implements the graphical user interface and visualization logic
 * for our ECE297 mapping application.
 *
 * The purpose of this file is to connect the backend map data and functions
 * (from Milestone 1) with a fully-interactive graphical interface using
 * the ezgl graphics library.
 *
 * Main responsibilities of this file include:
 *  - Rendering map features (streets, intersections, POIs, features)
 *  - Handling mouse clicks and user interaction
 *  - Implementing street, POI, and intersection search with autocomplete
 *  - Implementing the "Find Intersections" button feature between two streets
 *  - Highlighting intersections and search results
 *  - Displaying additional information in the message bar
 *  - Supporting night mode visualization
 *  - Rendering subway lines and stations from OSM data
 *  - Managing UI elements such as buttons, dialogs, and popups
 *  - Loading maps of other cities 
 *
 * This file relies on:
 *  - m1.hpp and m1.cpp for map query functions and data access
 *  - ezgl for rendering and UI management
 *  - m2.hpp for declarations related to Milestone 2
 *  - tts.h for text-to-speech functionality
 *
 */

#include <gtk/gtk.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <queue>

// Header project files
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m1.hpp"
#include "m2.hpp"
#include "m3.hpp"
#include "tts.h"

// Global UI State Variables
IntersectionIdx start_intersection_id = -1;
IntersectionIdx dest_intersection_id = -1;
std::vector<StreetSegmentIdx> current_drive_route; 
std::vector<StreetSegmentIdx> current_walk_route; 
bool is_walk_mode = false;
const double DEFAULT_TURN_PENALTY = 15.0; // Usually provided by the autotester, but good to have a default for UI

// Popup box state for directions
static GtkWidget* active_route_dialog = nullptr;

// Global tracker for the help dialog so we don't open 100 of them
static GtkWidget* active_help_dialog = nullptr;

// Constants for projections
double cos_lat_avg;

// Main Map Data (precomputed for drawing/UI)
std::vector<MyFeature> features;
std::vector<streetSegments> streets;
std::vector<MyPOI> Mypois;
std::vector<Intersection> intersections;

// Subway data from OSM
std::vector<SubwayLine> subway_lines;
std::vector<SubwayStation> subway_stations;

// Hash maps to quickly look up OSM pointers by their ID
std::unordered_map<OSMID, const OSMWay*> osm_ways_map;
std::unordered_map<OSMID, const OSMNode*> osm_nodes_map;

// Selection and highlight state
static int selected_intersection = -1;
static std::vector<int> highlighted_intersections;

// Map loading and world boundaries
std::vector<std::string> discovered_map_paths;
double global_maxLat, global_minLat, global_maxLon, global_minLon;
static ezgl::rectangle g_map_world;

// Autocomplete state
static GtkListStore* autocomplete_store = nullptr;
enum { COL_NAME = 0, COL_TYPE, COL_IDX, N_COLS };

static int search_result_intersection = -1;
static float search_result_x = -1;
static float search_result_y = -1;
static bool search_result_is_poi = false;
static bool search_just_selected = false;

// Find Popup box state
static GtkWidget* active_find_dialog = nullptr;

// Helper Function declaration
static PoiCategory classify_poi(const std::string& type);

//Path finding function to find the route 
void calculate_and_display_route(ezgl::application* app);

//Displays the information about the path found 
std::string generate_directions_text(const std::vector<StreetSegmentIdx>& route, bool is_walk);;
void show_directions_dialog(ezgl::application* app, const std::string& text);

// Coordinate Conversion Functions
float xFromLon(float lon) {
  return kEarthRadiusInMeters * (lon * kDegreeToRadian) * cos_lat_avg;
}
float yFromLat(float lat) {
  return kEarthRadiusInMeters * (lat * kDegreeToRadian);
}
float lonFromX(float x) {
  return x / (kEarthRadiusInMeters * kDegreeToRadian * cos_lat_avg);
}
float latFromY(float y) { return y / (kEarthRadiusInMeters * kDegreeToRadian); }

// Data Loading & Map Discovery Functions
bool ends_with(const std::string& text, const std::string& suffix) {
  if (suffix.size() > text.size()) return false;
  return text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

static void show_error_dialog(ezgl::application* app, const std::string& message);

IntersectionIdx get_intersection_from_casual_string(std::string query);

// Helper function to standard GTK error popup
static void show_error_dialog(ezgl::application* app, const std::string& message) {
    GtkWidget* dialog = gtk_message_dialog_new(
        GTK_WINDOW(app->get_object("MainWindow")),
        GTK_DIALOG_DESTROY_WITH_PARENT,
        GTK_MESSAGE_ERROR,
        GTK_BUTTONS_CLOSE,
        "%s", message.c_str()
    );
    gtk_window_set_title(GTK_WINDOW(dialog), "Input Error");
    
    // Safely close and destroy the dialog when the user clicks "Close"
    g_signal_connect(dialog, "response", G_CALLBACK(+[](GtkDialog* d, gint, gpointer) {
        gtk_widget_destroy(GTK_WIDGET(d));
    }), nullptr);

    gtk_widget_show_all(dialog);
    
    // Also log it to the message bar at the bottom
    app->update_message("Error: " + message);
}

//Helper to find an intersection ID from exact string name
// Helper to find intersection from exact match OR a casual string like "street1 & street2"
IntersectionIdx get_intersection_from_casual_string(std::string query) {
    //Check if it's an exact match first (from clicking an autocomplete suggestion)
    for (int i = 0; i < (int)intersections.size(); ++i) {
        if (intersections[i].name == query) return i;
    }

    //Look for an '&' or 'and' to split the string into two streets
    size_t split_pos = query.find("&");
    if (split_pos == std::string::npos) split_pos = query.find(" and ");

    if (split_pos != std::string::npos) {
        std::string s1 = query.substr(0, split_pos);
        
        // Offset by length of the delimiter ("&" is 1, " and " is 5)
        size_t offset = (query[split_pos] == '&') ? 1 : 5;
        std::string s2 = query.substr(split_pos + offset);

        // Trim leading and trailing spaces
        s1.erase(0, s1.find_first_not_of(" \t"));
        s1.erase(s1.find_last_not_of(" \t") + 1);
        s2.erase(0, s2.find_first_not_of(" \t"));
        s2.erase(s2.find_last_not_of(" \t") + 1);

        // Find the intersection of these two streets
        auto s1_ids = findStreetIdsFromPartialStreetName(s1);
        auto s2_ids = findStreetIdsFromPartialStreetName(s2);

        for (int id1 : s1_ids) {
            for (int id2 : s2_ids) {
                auto common_ints = findIntersectionsOfTwoStreets(id1, id2);
                if (!common_ints.empty()) {
                    return common_ints[0]; // Return the first valid intersection found
                }
            }
        }
    }
    return -1; // Not found
}

// Generate directions from a vector of street segments
std::string generate_directions_text(const std::vector<StreetSegmentIdx>& route, bool is_walk) {
    if (route.empty()) return "";
    
    std::ostringstream text;
    text << (is_walk ? "🚶 WALKING DIRECTIONS 🚶\n" : "🚗 DRIVING DIRECTIONS 🚗\n");
    text << "------------------------\n";

    StreetSegmentInfo prev_info = getStreetSegmentInfo(route[0]);
    std::string current_street = getStreetName(prev_info.streetID);
    double current_length = findStreetSegmentLength(route[0]);
    int current_speed = std::round(prev_info.speedLimit * 3.6);

    // Loop through the route and group segments with the same name
    for (size_t i = 1; i < route.size(); ++i) {
        StreetSegmentInfo info = getStreetSegmentInfo(route[i]);
        std::string street_name = getStreetName(info.streetID);

        if (street_name == current_street) {
            // Still on the same street, just add to the distance
            current_length += findStreetSegmentLength(route[i]);
        } else {
            // Street changed. Print the instruction for the street we just finished
            text << "• Continue on " << (current_street == "<unknown>" ? "Unknown Street" : current_street)
                 << " for " << std::round(current_length) << " m "
                 << "(Limit: " << current_speed << " km/h)\n";
            
            text << "• Turn onto " << (street_name == "<unknown>" ? "Unknown Street" : street_name) << "\n";

            // Reset trackers for the new street
            current_street = street_name;
            current_length = findStreetSegmentLength(route[i]);
            current_speed = std::round(info.speedLimit * 3.6);
        }
    }
    
    // Print the final leg of the journey
    text << "• Continue on " << (current_street == "<unknown>" ? "Unknown Street" : current_street)
         << " for " << std::round(current_length) << " m "
         << "(Limit: " << current_speed << " km/h)\n";
    text << "🏁 Arrive at destination.\n\n";

    return text.str();
}

// Create and show a scrollable GTK dialog with the directions
void show_directions_dialog(ezgl::application* app, const std::string& text) {
    // Close existing dialog if one is already open
    if (active_route_dialog != nullptr) {
        gtk_widget_destroy(active_route_dialog);
        active_route_dialog = nullptr;
    }

    // Create the dialog window
    active_route_dialog = gtk_dialog_new_with_buttons(
        "Turn-by-Turn Directions",
        GTK_WINDOW(app->get_object("MainWindow")),
        GTK_DIALOG_DESTROY_WITH_PARENT, 
        "Close", GTK_RESPONSE_CLOSE, 
        nullptr);

    gtk_window_set_default_size(GTK_WINDOW(active_route_dialog), 450, 500);

    // Create a scrollable area
    GtkWidget* scroll = gtk_scrolled_window_new(nullptr, nullptr);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);

    // Create the text view
    GtkWidget* text_view = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(text_view), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(text_view), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(text_view), GTK_WRAP_WORD);
    
    // Set margins for better reading
    gtk_text_view_set_left_margin(GTK_TEXT_VIEW(text_view), 10);
    gtk_text_view_set_right_margin(GTK_TEXT_VIEW(text_view), 10);
    gtk_text_view_set_top_margin(GTK_TEXT_VIEW(text_view), 10);

    // Inject our directions text
    GtkTextBuffer* buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(text_view));
    gtk_text_buffer_set_text(buf, text.c_str(), -1);

    // Pack it all together
    gtk_container_add(GTK_CONTAINER(scroll), text_view);
    gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(active_route_dialog))), scroll, TRUE, TRUE, 0);

    // Handle the close button safely
    g_signal_connect(active_route_dialog, "response", G_CALLBACK(+[](GtkDialog* d, gint, gpointer) {
        gtk_widget_destroy(GTK_WIDGET(d));
        active_route_dialog = nullptr;
    }), nullptr);

    gtk_widget_show_all(active_route_dialog);
}


// Extracted routing logic
void calculate_and_display_route(ezgl::application* app) {
    if (start_intersection_id == -1 || dest_intersection_id == -1) return;

    if (!is_walk_mode) {
        current_walk_route.clear(); 
        current_drive_route = findPathBetweenIntersections(DEFAULT_TURN_PENALTY, 
                                                           {start_intersection_id, dest_intersection_id});
    } else {
        GtkEntry* speed_entry = GTK_ENTRY(app->get_object("WalkSpeedEntry"));
        GtkEntry* time_entry = GTK_ENTRY(app->get_object("WalkTimeEntry"));
        
        double walk_speed = 1.5;   
        double walk_time = 300.0;  
        
        try {
            if (speed_entry) {
                std::string speed_str = gtk_entry_get_text(speed_entry);
                if (!speed_str.empty()) walk_speed = std::stod(speed_str);
            }
            if (time_entry) {
                std::string time_str = gtk_entry_get_text(time_entry);
                if (!time_str.empty()) walk_time = std::stod(time_str);
            }
        } catch (...) {
            std::cout << "Invalid speed/time input. Using defaults." << std::endl;
        }

        auto paths = findPathWithWalkToPickUp(start_intersection_id, dest_intersection_id, 
                                              DEFAULT_TURN_PENALTY, walk_speed, walk_time);
        current_walk_route = paths.first;
        current_drive_route = paths.second;
    }

    if (current_drive_route.empty() && current_walk_route.empty()) {
        app->update_message("Error: No valid path could be found.");
    } else {
        // Update the message bar briefly
        app->update_message("Path found! Displaying turn-by-turn directions.");
        
        // Generate the text
        std::string full_directions = "";
        if (!current_walk_route.empty()) {
            full_directions += generate_directions_text(current_walk_route, true);
        }
        if (!current_drive_route.empty()) {
            full_directions += generate_directions_text(current_drive_route, false);
        }

        // Pop up the window
        show_directions_dialog(app, full_directions);
    }
}

// The GTK callback for the Route button
// 3. The GTK callback for the Route button
static void on_route_button_clicked(GtkWidget*, gpointer data) {
    auto* app = static_cast<ezgl::application*>(data);

    GtkEntry* start_entry = GTK_ENTRY(app->get_object("StartEntry"));
    GtkEntry* dest_entry = GTK_ENTRY(app->get_object("DestEntry"));

    if (start_entry && dest_entry) {
        std::string start_str = gtk_entry_get_text(start_entry);
        std::string dest_str = gtk_entry_get_text(dest_entry);

        // --- NEW ERROR HANDLING ---
        if (start_str.empty() || dest_str.empty()) {
            show_error_dialog(app, "Please enter both a Start and Destination location.");
            return;
        }

        IntersectionIdx id_start = get_intersection_from_casual_string(start_str);
        if (id_start != -1) {
            start_intersection_id = id_start;
            gtk_entry_set_text(start_entry, intersections[id_start].name.c_str());
        } else {
            show_error_dialog(app, "Invalid Start Location: '" + start_str + "'\nMake sure to include an '&' between two valid crossing streets.");
            return;
        }

        IntersectionIdx id_dest = get_intersection_from_casual_string(dest_str);
        if (id_dest != -1) {
            dest_intersection_id = id_dest;
            gtk_entry_set_text(dest_entry, intersections[id_dest].name.c_str());
        } else {
            show_error_dialog(app, "Invalid Destination Location: '" + dest_str + "'\nMake sure to include an '&' between two valid crossing streets.");
            return;
        }
    }

    calculate_and_display_route(app);
    app->refresh_drawing();
}


static bool is_night_mode = false;

//Returns the colour that the subway lines should be 
ezgl::color findSubwayColour(std::string hex_str) {
  if (hex_str.empty()) return ezgl::color(0, 100, 200);

  //If given text for colour rather than hexcode 
  if (hex_str == "green") return ezgl::color(0, 150, 50, 200); 
  if (hex_str == "yellow") return ezgl::color(250, 210, 0, 200);
  if (hex_str == "purple") return ezgl::color(150, 0, 150, 200); 
  if (hex_str == "orange") return ezgl::color(255, 140, 0, 200);
  if (hex_str == "red") return ezgl::color(230, 30, 30, 200);
  if (hex_str == "blue") return ezgl::color(0, 100, 250, 200);
  if (hex_str == "mediumorchid") return ezgl::color(186, 85, 211, 200);

  //Get rid of the # 
  if (hex_str[0] == '#') hex_str = hex_str.substr(1);

  //For hex codes 
  if (hex_str.length() == 6) {
    try {
      unsigned long val = std::stoul(hex_str, nullptr, 16);
      uint8_t r = (val >> 16) & 0xFF;//Shift 16 bits 
      uint8_t g = (val >> 8) & 0xFF; //Shift 8 bits 
      uint8_t b = val & 0xFF;
      return ezgl::color(r, g, b, 200);
    } catch (...) {
      //If any crashes 
    }
  }
  
  //Default blue 
  return ezgl::color(0, 100, 200);
}

// Discover map files in the current directory and the provided maps directory,
// and store their paths
// Provides the list of map files in alphabetical order 
void findMapFiles() {
    namespace fs = std::filesystem;
    std::set<std::string> unique_maps;
    discovered_map_paths.clear();

    // Custom Breadth-First Search (BFS) directory traversal engine
    auto bfs_search = [&](const std::string& root_dir, int max_depth) {
        std::error_code ec;
        if (!fs::exists(root_dir, ec)) return;

        // The queue stores pairs of: {current_directory_path, current_depth_level}
        std::queue<std::pair<fs::path, int>> search_queue;
        search_queue.push({root_dir, 0});

        while (!search_queue.empty()) {
            auto [current_path, depth] = search_queue.front();
            search_queue.pop();

            // Open the directory, safely skipping folders we don't have permission to read
            fs::directory_iterator dir_it(current_path, fs::directory_options::skip_permission_denied, ec);
            if (ec) {
                ec.clear();
                continue; 
            }

            // Manually evaluate every item inside the current folder
            for (const auto& entry : dir_it) {
                if (entry.is_regular_file(ec)) {
                    std::string filepath = entry.path().string();
                    
                    // Check if the file ends with the required map extension
                    std::string target_ext = ".streets.bin";
                    if (filepath.size() >= target_ext.size() && 
                        filepath.compare(filepath.size() - target_ext.size(), target_ext.size(), target_ext) == 0) {
                        unique_maps.insert(filepath);
                    }
                } 
                // If it's a folder and we haven't hit our depth limit, queue it up for the next level
                else if (entry.is_directory(ec) && depth < max_depth) {
                    search_queue.push({entry.path(), depth + 1});
                }
            }
        }
    };

    // Execute our custom search on both required directories
    bfs_search(".", 4);
    bfs_search("/cad2/ece297s/public/maps", 2);

    // Transfer unique results to the global vector
    discovered_map_paths.assign(unique_maps.begin(), unique_maps.end());

    // Sort the final list alphabetically by filename only (ignoring the folder paths)
    std::sort(discovered_map_paths.begin(), discovered_map_paths.end(),
              [](const std::string& a, const std::string& b) {
                  return fs::path(a).filename().string() < fs::path(b).filename().string();
              });
}

void load_map_data() {
  //For highlighted intersections 
  selected_intersection = -1;
  highlighted_intersections.clear();

  search_result_intersection = -1;
  search_result_x = -1;
  search_result_y = -1;
  search_result_is_poi = false;
  search_just_selected = false;

  if (getNumIntersections() == 0) return;

  //To draw intersections 
  global_maxLat = getIntersectionPosition(0).latitude();
  global_minLat = global_maxLat;
  global_maxLon = getIntersectionPosition(0).longitude();
  global_minLon = global_maxLon;

  //Get the boundaries of the city to draw 
  intersections.clear();
  intersections.resize(getNumIntersections());
  for (int i = 0; i < getNumIntersections(); ++i) {
    intersections[i].position = getIntersectionPosition(i);
    intersections[i].name = getIntersectionName(i);

    global_maxLat =
        std::max(global_maxLat, intersections[i].position.latitude());
    global_minLat =
        std::min(global_minLat, intersections[i].position.latitude());
    global_maxLon =
        std::max(global_maxLon, intersections[i].position.longitude());
    global_minLon =
        std::min(global_minLon, intersections[i].position.longitude());
  }

  double avgLat = ((global_maxLat + global_minLat) / 2.0) * kDegreeToRadian;
  cos_lat_avg = cos(avgLat);

  //Project Lat and Lon for intersection
  //Iterate through each intersection  
  for (int i = 0; i < getNumIntersections(); ++i) {
    intersections[i].x = xFromLon(intersections[i].position.longitude());
    intersections[i].y = yFromLat(intersections[i].position.latitude());
  }

  //Iterate through all the streets segments and get their from to and curve points to draw 
  streets.clear();
  int numSegments = getNumStreetSegments();
  streets.resize(numSegments);
  for (int i = 0; i < numSegments; i++) {
    StreetSegmentInfo info = getStreetSegmentInfo(i);//Gets the information about the street 
    streets[i].speedLimit = info.speedLimit;
    streets[i].name = getStreetName(info.streetID);
    //Gets the position of the "from" of the street 
    LatLon fromPos = getIntersectionPosition(info.from);
    streets[i].points.push_back(
        {xFromLon(fromPos.longitude()), yFromLat(fromPos.latitude())});

    //Goes through each curve position of the street so the drawing also draws the curves 
    for (int j = 0; j < info.numCurvePoints; j++) {
      LatLon curvePos = getStreetSegmentCurvePoint(i, j);
      streets[i].points.push_back(
          {xFromLon(curvePos.longitude()), yFromLat(curvePos.latitude())});
    }
    //Gets the "to" of the street 
    LatLon toPos = getIntersectionPosition(info.to);
    streets[i].points.push_back(
        {xFromLon(toPos.longitude()), yFromLat(toPos.latitude())});
  }

  //Load all POIs to draw 
  Mypois.clear();
  int numPOIs = getNumPointsOfInterest();
  Mypois.resize(numPOIs);
  for (int i = 0; i < numPOIs; i++) {
    Mypois[i].position = getPOIPosition(i);
    Mypois[i].name = getPOIName(i);
    Mypois[i].x = xFromLon(Mypois[i].position.longitude());
    Mypois[i].y = yFromLat(Mypois[i].position.latitude());
  }

  //Load all features 
  features.clear();
  int numFeatures = getNumFeatures();
  for (int i = 0; i < numFeatures; i++) {
    //Identifies the feature 
    MyFeature feat;
    FeatureType type = getFeatureType(i);

    feat.type = type;
    //Iterating through the points that make up the feature's boundaries 
    int numPoints = getNumFeaturePoints(i);
    for (int j = 0; j < numPoints; j++) {
      LatLon pos = getFeaturePoint(i, j);
      feat.points.push_back(
          {xFromLon(pos.longitude()), yFromLat(pos.latitude())});
    }
    //Checks if the feature is closed
    //Use g->fill if closed, or g->drawline if not closed 
    feat.is_closed = (feat.points.size() > 2 &&
                      feat.points.front().x == feat.points.back().x &&
                      feat.points.front().y == feat.points.back().y);

    features.push_back(feat);
  }

  // Load Subway Lines from OSM
  subway_lines.clear();
  osm_nodes_map.clear();
  osm_ways_map.clear();

  //Store all nodes of the OSM 
  for (int i = 0; i < getNumberOfNodes(); ++i) {
    const OSMNode* node = getNodeByIndex(i);
    osm_nodes_map[node->id()] = node;
  }
  //Store all ways of the OSM 
  for (int i = 0; i < getNumberOfWays(); ++i) {
    const OSMWay* way = getWayByIndex(i);
    osm_ways_map[way->id()] = way;
  }
  //Subways are "relations"
  for (int i = 0; i < getNumberOfRelations(); ++i) {
    const OSMRelation* rel = getRelationByIndex(i);

    bool is_subway = false;
    std::string name = "Unknown Line";
    std::string hex_color = "";

    //Tag count is the number of attributes the subway has 
    int tagCount = getTagCount(rel);
    
    //Checks the OSM to see if relation it is subway 
    for (int j = 0; j < tagCount; ++j) {
      std::pair<std::string, std::string> tag = getTagPair(rel, j);

      if (tag.first == "route" && tag.second == "subway") is_subway = true;
      if (tag.first == "name") name = tag.second;
      if (tag.first == "colour") hex_color = tag.second;
      
    }
    //If the relation is a subway 
    if (is_subway) {
      SubwayLine line;
      line.name = name;
      line.color = findSubwayColour(hex_color); //Calls findSubwayColour to get the colour of the subway line 

      std::vector<TypedOSMID> members = getRelationMembers(rel);
      std::vector<std::string> roles = getRelationMemberRoles(rel); //role is the role of the subway (stop, station, platform)

      //Iterate through all parts (members) of the subway relation
      for (size_t k = 0; k < members.size(); ++k) {
        const auto& member = members[k];
        const std::string& role = roles[k];

        //If the member is a Way, it is a line 
        //If the member is a Way, extract coordinate points to draw the physical tracks
        if (member.type() == TypedOSMID::Way) {
          auto way_it = osm_ways_map.find(member);//Find the way in the hashmap 
          if (way_it != osm_ways_map.end()) {
            const OSMWay* way = way_it->second;
            //Stores all the track points of the subway to draw 
            std::vector<ezgl::point2d> track_points;

            const std::vector<OSMID>& nds = getWayMembers(way);
            //Go through every node in the way 
            for (OSMID nd_id : nds) {
              auto node_it = osm_nodes_map.find(nd_id);
              if (node_it != osm_nodes_map.end()) {
                //Store the track points of the way 
                LatLon ll = getNodeCoords(node_it->second);
                track_points.push_back(
                    {xFromLon(ll.longitude()), yFromLat(ll.latitude())});
              }
            }

            if (!track_points.empty()) {
              line.tracks.push_back(track_points);
            }
          }
        
        //If the member is a Node, check if it's a station/platform and save its position
        } else if (member.type() == TypedOSMID::Node) {
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
              subway_stations.push_back(st);//Store the subway stations 
            }
          }
        }
      }
      subway_lines.push_back(line);
    }
  }

  //Set the global map bounding box using the min/max coordinates calculated earlier
  g_map_world =
      ezgl::rectangle({xFromLon(global_minLon), yFromLat(global_minLat)},
                      {xFromLon(global_maxLon), yFromLat(global_maxLat)});
}

// Build the autocomplete store for POIs, streets, and intersections
void build_autocomplete_store() {
  // Clear previous store if it exists
  if (autocomplete_store) {
    g_object_unref(autocomplete_store);
  }

  // Create list store with columns (Name, Type, Index)
  autocomplete_store =
      gtk_list_store_new(N_COLS, G_TYPE_STRING, G_TYPE_INT, G_TYPE_INT);

  GtkTreeIter iter;  // Iterator for adding rows

  // Add streets to autocomplete (type 0)
  std::unordered_set<std::string> seen_streets;
  for (int i = 0; i < (int)streets.size(); i++) {
    const std::string& name = streets[i].name;
    if (name == "<unknown>" || seen_streets.count(name)) continue;
    seen_streets.insert(name);
    gtk_list_store_append(autocomplete_store, &iter);
    gtk_list_store_set(autocomplete_store, &iter, COL_NAME, name.c_str(),
                       COL_TYPE, 0, COL_IDX, i, -1);
  }

  // Add POIs to autocomplete (type 1)
  for (int i = 0; i < (int)Mypois.size(); i++) {
    std::string display_name = Mypois[i].name;
    int nearest = findClosestIntersection(Mypois[i].position);
    if (nearest >= 0 && nearest < (int)intersections.size()) {
      display_name += " (" + intersections[nearest].name + ")";
    }
    gtk_list_store_append(autocomplete_store, &iter);
    gtk_list_store_set(autocomplete_store, &iter, COL_NAME,
                       display_name.c_str(), COL_TYPE, 1, COL_IDX, i, -1);
  }

  // Add intersections to autocomplete (type 2)
  std::unordered_set<std::string> seen_ints;
  for (int i = 0; i < (int)intersections.size(); i++) {
    const std::string& name = intersections[i].name;
    if (name == "<unknown>" || seen_ints.count(name)) continue;
    seen_ints.insert(name);
    gtk_list_store_append(autocomplete_store, &iter);
    gtk_list_store_set(autocomplete_store, &iter, COL_NAME, name.c_str(),
                       COL_TYPE, 2, COL_IDX, i, -1);
  }
}

// After a user selects an autocomplete match, highlight it and show details (No Auto-Zoom)
static gboolean on_autocomplete_match_selected(GtkEntryCompletion*,
                                               GtkTreeModel* model,
                                               GtkTreeIter* iter,
                                               gpointer data) {
  auto* app = static_cast<ezgl::application*>(data);

  // Get the selected item's details
  gchar* name = nullptr;
  gint type = 0;
  gint idx = 0;
  gtk_tree_model_get(model, iter, COL_NAME, &name, COL_TYPE, &type, COL_IDX,
                     &idx, -1);

  // Reset previous search result state
  search_result_intersection = -1;
  search_result_x = -1;
  search_result_y = -1;
  search_result_is_poi = false;

  // Street selected
  if (type == 0) {
    auto street_ids = findStreetIdsFromPartialStreetName(name ? name : "");
    highlighted_intersections.clear();
    selected_intersection = -1;

    // If multiple streets match
    if (!street_ids.empty()) {
      auto ints = findIntersectionsOfStreet(street_ids[0]);
      highlighted_intersections = ints;

      // Select the first intersection of the first matching street (No zoom)
      if (!ints.empty()) {
        selected_intersection = ints[0];
        search_result_is_poi = false;
        search_result_intersection = ints[0];
      }
    }

    // Announce street name with TTS
    if (name) {
      speak(std::string("Street: ") + name);
    }
  }

  // POI selected
  else if (type == 1) {
    selected_intersection = -1;
    highlighted_intersections.clear();

    // Select the POI (No zoom)
    if (idx < (int)Mypois.size()) {
      search_result_is_poi = true;
      search_result_x = Mypois[idx].x;
      search_result_y = Mypois[idx].y;
    }

    // Show POI details in the message bar
    std::string raw_type = getPOIType(idx);
    PoiCategory cat = classify_poi(raw_type);

    std::stringstream ss;
    ss << "POI: " << Mypois[idx].name
       << " | Type: " << cat.label;  // Show POI category label

    int nearest = findClosestIntersection(Mypois[idx].position);  // Find nearest intersection to POI
    if (nearest >= 0 && nearest < (int)intersections.size()) {
      ss << " | Near: " << intersections[nearest].name;
    }

    app->update_message(ss.str());

    // TTS announcement for POI
    if (name) {
      std::string clean = Mypois[idx].name;
      speak(std::string("Point of interest: ") + clean);
    }
  }

  // Intersection selected
  else if (type == 2) {
    selected_intersection = idx;
    highlighted_intersections.clear();

    // Select the intersection (No zoom)
    if (idx < (int)intersections.size()) {
      search_result_is_poi = false;
      search_result_intersection = idx;
    }

    // Show intersection details in the message bar
    std::stringstream ss;
    ss << "Intersection: " << intersections[idx].name;
    app->update_message(ss.str());

    // TTS announcement for intersection
    speak(std::string("Intersection: ") + intersections[idx].name);
  }

  // Show basic details for location searched (street/POI/intersection)
  std::stringstream ss;

  if (type == 0) {
    ss << "STREET: " << (name ? name : "");
  } else if (type == 1) {
    ss << "POI: " << (name ? name : "");
  } else if (type == 2) {
    ss << "INTERSECTION: " << (name ? name : "")
       << " (click intersection for details)";
  }

  // For POIs, also show category and nearest intersection in the message bar
  if (type == 1) {
    std::string raw_type = getPOIType(idx);
    PoiCategory cat = classify_poi(raw_type);

    std::stringstream ss2;
    ss2 << "POI: " << Mypois[idx].name << " | Type: " << cat.label;

    int nearest = findClosestIntersection(Mypois[idx].position);
    if (nearest >= 0 && nearest < (int)intersections.size()) {
      ss2 << " | Near: " << intersections[nearest].name;
    }

    ss.str("");
    ss.clear();
    ss << ss2.str();
  }

  // Update message bar with search result details
  app->update_message(ss.str());

  // Clean up
  if (name) g_free(name);
  search_just_selected = true;
  app->refresh_drawing();
  return FALSE;
}


// Attach autocomplete functionality to a GtkEntry widget
static void attach_autocomplete(GtkEntry* entry, ezgl::application* app) {
  GtkEntryCompletion* completion = gtk_entry_completion_new();

  gtk_entry_completion_set_model(completion,
                                 GTK_TREE_MODEL(autocomplete_store));
  gtk_entry_completion_set_text_column(completion, COL_NAME);
  gtk_entry_completion_set_minimum_key_length(completion, 2);
  gtk_entry_completion_set_popup_completion(completion, TRUE);
  gtk_entry_completion_set_inline_completion(completion, FALSE);

  g_signal_connect(completion, "match-selected",
                   G_CALLBACK(on_autocomplete_match_selected), app);

  gtk_entry_set_completion(entry, completion);
  g_object_unref(completion);
}

void act_on_mouse_click(ezgl::application* app, GdkEventButton* /*event*/, double x, double y) {
    // Convert the click x/y canvas coordinates back to LatLon using your helper functions
    float lat = latFromY(y);
    float lon = lonFromX(x);
    LatLon click_latlon(lat, lon);
    
    // Find the closest intersection using your M1 API
    IntersectionIdx clicked_id = findClosestIntersection(click_latlon);

    // State Machine Logic
    if (start_intersection_id == -1) {
        // First click: Set the start point
        start_intersection_id = clicked_id;
        std::string msg = "Start set: " + getIntersectionName(clicked_id);
        std::cout << msg << std::endl;
        app->update_message(msg);
        
    } else if (dest_intersection_id == -1) {
        // Second click: Set the destination FIRST
        dest_intersection_id = clicked_id; 
        
        // Calculate the route using our new helper
        calculate_and_display_route(app);
        

    } else {
        // Third click: Reset everything and treat this as a new start point
        start_intersection_id = clicked_id;
        dest_intersection_id = -1;
        
        // Clear BOTH route vectors
        current_drive_route.clear();
        current_walk_route.clear();
        
        std::string msg = "Resetting search. New start set: " + getIntersectionName(clicked_id);
        std::cout << msg << std::endl;
        app->update_message(msg);
    }

    // Force the screen to redraw so we can see the highlighted points and path
    app->refresh_drawing();
}

// Find and highlight intersections between two streets given their names
// (partial allowed)
static void find_and_highlight(const std::string& street1,
                               const std::string& street2) {
  highlighted_intersections.clear();

  // Find street IDs matching the partial names
  auto s1 = findStreetIdsFromPartialStreetName(street1);
  auto s2 = findStreetIdsFromPartialStreetName(street2);
  if (s1.empty() || s2.empty()) return;

  std::unordered_set<int> uniq;  // To store unique intersection IDs

  // For each pair of matching streets, find their intersections and add to
  // unique set
  for (int id1 : s1) {
    for (int id2 : s2) {
      auto ints = findIntersectionsOfTwoStreets(id1, id2);
      for (int inter : ints) uniq.insert(inter);
    }
  }

  highlighted_intersections.assign(uniq.begin(),
                                   uniq.end());  // Convert back to vector
}

// Classify POI types into categories for coloring and labeling
static void find_button(GtkWidget*, gpointer data) {
  auto* app = static_cast<ezgl::application*>(data);

  // Close any previous find popup (always)
  if (active_find_dialog != nullptr) {
    gtk_widget_destroy(active_find_dialog);
    active_find_dialog = nullptr;
  }

  // Get street names from the entry fields
  GtkEntry* e1 = GTK_ENTRY(app->get_object("Street1Entry"));
  GtkEntry* e2 = GTK_ENTRY(app->get_object("Street2Entry"));

  // Validate input
  std::string s1 = gtk_entry_get_text(e1);
  std::string s2 = gtk_entry_get_text(e2);

  if (s1.empty() || s2.empty()) {
      show_error_dialog(app, "Please enter both street names to find an intersection.");
      return;
  }

  auto s1_ids = findStreetIdsFromPartialStreetName(s1);
  auto s2_ids = findStreetIdsFromPartialStreetName(s2);

  if (s1_ids.empty()) {
      show_error_dialog(app, "Invalid Street 1: Could not find any street matching '" + s1 + "'.");
      return;
  }
  if (s2_ids.empty()) {
      show_error_dialog(app, "Invalid Street 2: Could not find any street matching '" + s2 + "'.");
      return;
  }

  // If either field is empty, show an error message and return
  find_and_highlight(s1, s2);
  const int MATCH_CAP = 50;

  int n_matches = (int)highlighted_intersections.size();

  // If too many matches, clear highlights and show message asking for more
  // specific input
  if (n_matches > MATCH_CAP) {
    highlighted_intersections.clear();  // Clear highlights to avoid performance
                                        // issues with drawing too many
    selected_intersection = -1;

    // Show message in the message bar
    std::string msg = "Too many matches (" + std::to_string(n_matches) +
                      "). Please type 1–2 more letters.";
    app->update_message(msg);
    speak(msg);
    app->refresh_drawing();
    return;
  }
  std::stringstream ss;

  // Show results in message bar
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

  // TTS announcement for find results
  if (!highlighted_intersections.empty()) {
    std::ostringstream msg;
    msg << "Found " << highlighted_intersections.size()
        << (highlighted_intersections.size() == 1 ? " intersection"
                                                  : " intersections")
        << " between " << s1 << " and " << s2;
    speak(msg.str());
  } else {
    speak(std::string("No intersections found between ") + s1 + " and " + s2);
  }

  // If multiple intersections found, show them in a draggable popup dialog with
  // a scrollable list
  if (highlighted_intersections.size() > 1) {
    // Make sure highlights are visible first
    app->refresh_drawing();

    // Create a new dialog to show the list of intersections
    active_find_dialog = gtk_dialog_new_with_buttons(
        (std::to_string(highlighted_intersections.size()) +
         " Possible Intersections: " + s1 + " & " + s2)
            .c_str(),
        GTK_WINDOW(app->get_object("MainWindow")),
        GTK_DIALOG_DESTROY_WITH_PARENT, "Close", GTK_RESPONSE_CLOSE, nullptr);

    // Set default size and create scrollable text view to list intersections
    gtk_window_set_default_size(GTK_WINDOW(active_find_dialog), 420, 360);
    GtkWidget* scroll = gtk_scrolled_window_new(nullptr, nullptr);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll),
                                   GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);
    GtkWidget* text_view = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(text_view), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(text_view), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(text_view), GTK_WRAP_WORD);
    GtkTextBuffer* buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(text_view));

    std::ostringstream
        list_text;  // Build the list of intersections to show in the dialog

    // List each intersection with its name in the dialog
    for (int i = 0; i < (int)highlighted_intersections.size(); i++) {
      list_text << (i + 1) << ".  "
                << intersections[highlighted_intersections[i]].name << "\n";
    }

    // Set the text of the text view to the list of intersections
    gtk_text_buffer_set_text(buf, list_text.str().c_str(), -1);

    // Add the text view to the scrollable area, and the scrollable area to the
    // dialog
    gtk_container_add(GTK_CONTAINER(scroll), text_view);
    gtk_box_pack_start(
        GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(active_find_dialog))),
        scroll, TRUE, TRUE, 0);

    // Non-blocking close behavior + reset pointer
    g_signal_connect(active_find_dialog, "response",
                     G_CALLBACK(+[](GtkDialog* d, gint, gpointer) {
                       gtk_widget_destroy(GTK_WIDGET(d));
                       active_find_dialog = nullptr;
                     }),
                     nullptr);

    gtk_widget_show_all(
        active_find_dialog);  // Show the dialog with the list of intersections
  }

  app->refresh_drawing();
}

// Zoom fit button handler
static void zoom_fit_button(GtkWidget*, gpointer data) {
  auto* app = static_cast<ezgl::application*>(data);
  ezgl::canvas* c = app->get_canvas("MainCanvas");
  if (!c) return;

  c->get_camera().set_world(g_map_world);
  app->refresh_drawing();
}

// Zoom out button handler
// Zooms out by a factor, but not beyond the full map world
static void zoom_out_button(GtkWidget*, gpointer data) {
  auto* app = static_cast<ezgl::application*>(data);
  ezgl::canvas* c = app->get_canvas("MainCanvas");
  if (!c) return;

  ezgl::rectangle cur = c->get_camera().get_world();

  // Zoom out by a factor 1.25, but ensure we don't exceed the map world
  double factor = 1.25;
  ezgl::point2d center = cur.center();
  double new_w = cur.width() * factor;
  double new_h = cur.height() * factor;

  ezgl::rectangle next({center.x - new_w / 2.0, center.y - new_h / 2.0},
                       {center.x + new_w / 2.0, center.y + new_h / 2.0});

  if (next.width() >= g_map_world.width() ||
      next.height() >= g_map_world.height()) {
    c->get_camera().set_world(g_map_world);
  } else {
    c->get_camera().set_world(next);
  }

  app->refresh_drawing();
}

// Load map button handler
void load_selected_map(GtkWidget*, gpointer data) {
  auto *app = static_cast<ezgl::application *>(data);

  //Makes app search the UI file for MapCombo widget 
  GtkComboBoxText *combo = GTK_COMBO_BOX_TEXT(app->get_object("MapCombo"));
  if (!combo) return;
  //Gets index that the user selected 
  int active_idx = gtk_combo_box_get_active(GTK_COMBO_BOX(combo));
  if (active_idx < 0 || active_idx >= static_cast<int>(discovered_map_paths.size())) return;

  //Get the new map to draw based on the map index 
  std::string new_map_path = discovered_map_paths[active_idx];

  //Close the current map 
  closeMap();

  //Load the new map 
  bool load_success = loadMap(new_map_path);

  if (!load_success) {
    std::cerr << "Failed to load map: " << new_map_path << std::endl;
    show_error_dialog(app, "Failed to load map file:\n" + new_map_path + "\n\nThe file might be missing or corrupted.");
    return;
  }

  //Extract the data into our vectors
  load_map_data();
  build_autocomplete_store();

  //Update UI components
  //Updates the autocomplete data to the search functions 
  GtkEntry *top_search = GTK_ENTRY(app->get_object("TopSearch"));
  GtkEntry *start_entry = GTK_ENTRY(app->get_object("StartEntry"));
  GtkEntry *dest_entry = GTK_ENTRY(app->get_object("DestEntry"));

  if (top_search) {
    GtkEntryCompletion *c = gtk_entry_get_completion(top_search);
    if (c) gtk_entry_completion_set_model(c, GTK_TREE_MODEL(autocomplete_store));
  }
  if (start_entry) {
    GtkEntryCompletion *c = gtk_entry_get_completion(start_entry);
    if (c) gtk_entry_completion_set_model(c, GTK_TREE_MODEL(autocomplete_store));
  }
  if (dest_entry) {
    GtkEntryCompletion *c = gtk_entry_get_completion(dest_entry);
    if (c) gtk_entry_completion_set_model(c, GTK_TREE_MODEL(autocomplete_store));
  }

  //Reset camera boundaries
  ezgl::rectangle new_world({xFromLon(global_minLon), yFromLat(global_minLat)},
                            {xFromLon(global_maxLon), yFromLat(global_maxLat)});
  g_map_world = new_world;

  ezgl::canvas *main_canvas = app->get_canvas("MainCanvas");
  if (main_canvas) {
    main_canvas->get_camera().set_world(new_world);
  }

  app->refresh_drawing();
}

// Night mode toggle handler
static void on_night_mode_toggled(GObject* object, GParamSpec*, gpointer data) {
  is_night_mode = gtk_switch_get_active(GTK_SWITCH(object));
  auto* app = static_cast<ezgl::application*>(data);
  app->refresh_drawing();
}

static void on_mode_switch_toggled(GObject* object, GParamSpec*, gpointer data) {
    is_walk_mode = gtk_switch_get_active(GTK_SWITCH(object));
    auto* app = static_cast<ezgl::application*>(data);
    
    if (is_walk_mode) {
        app->update_message("Mode: Walk + Drive. Please ensure Speed and Time Limit are set.");
    } else {
        app->update_message("Mode: Drive Only.");
    }
}

// Callback to generate and show the Help window
static void on_help_button_clicked(GtkWidget*, gpointer data) {
    auto* app = static_cast<ezgl::application*>(data);

    // Close any existing help dialog
    if (active_help_dialog != nullptr) {
        gtk_widget_destroy(active_help_dialog);
        active_help_dialog = nullptr;
    }

    // Create the dialog window
    active_help_dialog = gtk_dialog_new_with_buttons(
        "🗺️ Map Features & Controls",
        GTK_WINDOW(app->get_object("MainWindow")),
        GTK_DIALOG_DESTROY_WITH_PARENT, 
        "Got it!", GTK_RESPONSE_CLOSE, 
        nullptr);

    gtk_window_set_default_size(GTK_WINDOW(active_help_dialog), 500, 450);

    // Create a scrollable area
    GtkWidget* scroll = gtk_scrolled_window_new(nullptr, nullptr);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);

    // Create the text view
    GtkWidget* text_view = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(text_view), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(text_view), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(text_view), GTK_WRAP_WORD);
    
    // Set margins for nice formatting
    gtk_text_view_set_left_margin(GTK_TEXT_VIEW(text_view), 15);
    gtk_text_view_set_right_margin(GTK_TEXT_VIEW(text_view), 15);
    gtk_text_view_set_top_margin(GTK_TEXT_VIEW(text_view), 15);
    gtk_text_view_set_bottom_margin(GTK_TEXT_VIEW(text_view), 15);

    //The actual help text
    std::string help_text = 
        "🔍 FIND INTERSECTION\n"
        "• Top Search Bar: Type a street, POI, or intersection. Click the drop-down to select and highlight it.\n"
        "• Between Two Streets: Enter two partial street names and click 'Find Intersection'. The map will automatically highlight the intersection(s).\n\n"
        
        "🛣️ FIND PATH (ROUTING)\n"
        "• Mouse Clicks: Click two intersections on the map to instantly draw a route. Click a third time to reset.\n"
        "• Search Boxes: Type two intersections (e.g., 'Don Mills & Sheppard'). Use the drop-down to lock them in, then click 'Find Route'\n"
        "For autocomplete, enter two intersections; each intersection containing two partial street names seperated by '&' (eg. 'ed & un').\n"
        "• Walk + Drive Mode: Toggle the switch to include walking. Set your walk speed and time limit in the provided boxes.\n"
        "• Directions: A turn-by-turn navigation window will appear automatically when a route is found!\n\n"
        
        "🌙 VISUALS & DISPLAY\n"
        "• Night Mode: Toggle the switch for a dark UI theme.\n"
        "• Subway Lines: Colored transit lines and stations become visible as you zoom in.\n"
        "• Map Switcher: Use the drop-down menu and 'Load Map' button to travel to different cities.\n"
        "• POI Clusters: Points of Interest cluster together when zoomed out to prevent clutter, and reveal their names when zoomed in.\n\n"
        
        "🖱️ NAVIGATION\n"
        "• Scroll Wheel: Zoom in and out.\n"
        "• Left Click & Drag: Pan across the map.\n"
        "• Zoom Fit: Instantly return to the full city view.\n"
        "• Zoom Out: Step the camera back smoothly.";

    // Inject text into the buffer
    GtkTextBuffer* buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(text_view));
    gtk_text_buffer_set_text(buf, help_text.c_str(), -1);

    // Pack it all together
    gtk_container_add(GTK_CONTAINER(scroll), text_view);
    gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(active_help_dialog))), scroll, TRUE, TRUE, 0);

    // Handle the close button safely
    g_signal_connect(active_help_dialog, "response", G_CALLBACK(+[](GtkDialog* d, gint, gpointer) {
        gtk_widget_destroy(GTK_WIDGET(d));
        active_help_dialog = nullptr;
    }), nullptr);

    gtk_widget_show_all(active_help_dialog);
}

// Initial setup function to connect signals and prepare the application
void initial_setup(ezgl::application* app, bool) {
  GtkWidget* find_btn = GTK_WIDGET(app->get_object("FindButton"));

  GtkWidget* help_btn = GTK_WIDGET(app->get_object("HelpButton"));
  if (help_btn) {
      g_signal_connect(help_btn, "clicked", G_CALLBACK(on_help_button_clicked), app);
  }

  GtkWidget* mode_switch = GTK_WIDGET(app->get_object("ModeSwitch"));
  if (mode_switch) {
      g_signal_connect(mode_switch, "notify::active", G_CALLBACK(on_mode_switch_toggled), app);
  }

  // Connect the "Find" button to its handler
  if (find_btn)
    g_signal_connect(find_btn, "clicked", G_CALLBACK(find_button), app);

  GtkWidget* zoom_fit_btn = GTK_WIDGET(app->get_object("ZoomFitButton"));
  if (zoom_fit_btn)
    g_signal_connect(zoom_fit_btn, "clicked", G_CALLBACK(zoom_fit_button), app);

  GtkWidget* zoom_out_btn = GTK_WIDGET(app->get_object("ZoomOutButton"));
  if (zoom_out_btn)
    g_signal_connect(zoom_out_btn, "clicked", G_CALLBACK(zoom_out_button), app);

  GtkWidget* night_switch = GTK_WIDGET(app->get_object("NightModeSwitch"));
  if (night_switch) {
    gtk_switch_set_active(GTK_SWITCH(night_switch), is_night_mode);
    //Call on_night_mode_toggled when the switch is toggled 
    g_signal_connect(night_switch, "notify::active",
                     G_CALLBACK(on_night_mode_toggled), app);
  }

  build_autocomplete_store();

  // Attach autocomplete to the top search entry
  GtkEntry* top_search = GTK_ENTRY(app->get_object("TopSearch"));
  GtkEntry* start_entry = GTK_ENTRY(app->get_object("StartEntry"));
  GtkEntry* dest_entry = GTK_ENTRY(app->get_object("DestEntry"));

  if (top_search) attach_autocomplete(top_search, app);
  if (start_entry) attach_autocomplete(start_entry, app);
  if (dest_entry) attach_autocomplete(dest_entry, app);

  GtkWidget* route_btn = GTK_WIDGET(app->get_object("RouteButton"));
  if (route_btn) {
      g_signal_connect(route_btn, "clicked", G_CALLBACK(on_route_button_clicked), app);
  }

  // Populate the map selection combo box with discovered maps
  findMapFiles();
  GtkComboBoxText* map_combo = GTK_COMBO_BOX_TEXT(app->get_object("MapCombo"));
  GtkWidget* load_map_btn = GTK_WIDGET(app->get_object("LoadMapButton"));

  // If the combo box exists, add the discovered map paths to it
  if (map_combo) {
    for (const auto& path : discovered_map_paths) {
      std::string filename = std::filesystem::path(path).filename().string();
      gtk_combo_box_text_append_text(map_combo, filename.c_str());
    }
  }

  // Connect the "Load Map" button to its handler
  if (load_map_btn) {
    g_signal_connect(load_map_btn, "clicked", G_CALLBACK(load_selected_map),
                     app);
  }

  auto draw_legend_swatch = [](GtkWidget* /*widget*/, cairo_t* cr,
                               gpointer color_ptr) -> gboolean {
    ezgl::color* col = static_cast<ezgl::color*>(color_ptr);
    cairo_set_source_rgb(cr, col->red / 255.0, col->green / 255.0,
                         col->blue / 255.0);
    cairo_arc(cr, 8, 8, 6, 0, 2 * M_PI);
    cairo_fill(cr);
    return FALSE;
  };

  // Define colors for each POI category
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

  // Legend
  GtkWidget* legend_food = GTK_WIDGET(app->get_object("LegendFood"));
  GtkWidget* legend_health = GTK_WIDGET(app->get_object("LegendHealth"));
  GtkWidget* legend_edu = GTK_WIDGET(app->get_object("LegendEducation"));
  GtkWidget* legend_finance = GTK_WIDGET(app->get_object("LegendFinance"));
  GtkWidget* legend_shop = GTK_WIDGET(app->get_object("LegendShopping"));
  GtkWidget* legend_emergency = GTK_WIDGET(app->get_object("LegendEmergency"));
  GtkWidget* legend_transport = GTK_WIDGET(app->get_object("LegendTransport"));
  GtkWidget* legend_accommodation =
      GTK_WIDGET(app->get_object("LegendAccommodation"));
  GtkWidget* legend_services = GTK_WIDGET(app->get_object("LegendServices"));
  GtkWidget* legend_religion = GTK_WIDGET(app->get_object("LegendReligion"));

  if (legend_food)
    g_signal_connect(legend_food, "draw", G_CALLBACK(+draw_legend_swatch),
                     &food_col);
  if (legend_health)
    g_signal_connect(legend_health, "draw", G_CALLBACK(+draw_legend_swatch),
                     &health_col);
  if (legend_edu)
    g_signal_connect(legend_edu, "draw", G_CALLBACK(+draw_legend_swatch),
                     &edu_col);
  if (legend_finance)
    g_signal_connect(legend_finance, "draw", G_CALLBACK(+draw_legend_swatch),
                     &finance_col);
  if (legend_shop)
    g_signal_connect(legend_shop, "draw", G_CALLBACK(+draw_legend_swatch),
                     &shop_col);
  if (legend_emergency)
    g_signal_connect(legend_emergency, "draw", G_CALLBACK(+draw_legend_swatch),
                     &emergency_col);
  if (legend_transport)
    g_signal_connect(legend_transport, "draw", G_CALLBACK(+draw_legend_swatch),
                     &transport_col);
  if (legend_accommodation)
    g_signal_connect(legend_accommodation, "draw",
                     G_CALLBACK(+draw_legend_swatch), &accommodation_col);
  if (legend_services)
    g_signal_connect(legend_services, "draw", G_CALLBACK(+draw_legend_swatch),
                     &services_col);
  if (legend_religion)
    g_signal_connect(legend_religion, "draw", G_CALLBACK(+draw_legend_swatch),
                     &religion_col);
}

ezgl::color get_feature_color(FeatureType type, bool night) {
  if (night) {
    switch (type) {
      case PARK:
      case GREENSPACE:
      case GOLFCOURSE:
        return ezgl::color(35, 75, 45);
      case LAKE:
      case RIVER:
      case STREAM:
        return ezgl::color(25, 50, 90);
      case BEACH:
        return ezgl::color(90, 80, 50);
      case ISLAND:
        return ezgl::color(40, 40, 40);
      case BUILDING:
        return ezgl::color(60, 60, 60);
      case GLACIER:
        return ezgl::color(150, 150, 180);
      default:
        return ezgl::color(50, 50, 50);
    }
  } else {
    switch (type) {
      case PARK:
      case GREENSPACE:
      case GOLFCOURSE:
        return ezgl::color(200, 238, 200);
      case LAKE:
      case RIVER:
      case STREAM:
        return ezgl::color(170, 218, 255);
      case BEACH:
        return ezgl::color(255, 240, 180);
      case ISLAND:
        return ezgl::color(240, 240, 240);
      case BUILDING:
        return ezgl::color(220, 220, 220);
      case GLACIER:
        return ezgl::color(255, 255, 255);
      default:
        return ezgl::color(230, 230, 230);
    }
  }
}

// Struct to hold POI category information (color and label)
static PoiCategory classify_poi(const std::string& type) {
  static const std::unordered_set<std::string> food = {
      "restaurant", "cafe",      "fast_food", "bar",    "pub",
      "food_court", "ice_cream", "bakery",    "butcher"};
  static const std::unordered_set<std::string> health = {
      "hospital", "clinic", "doctors", "dentist", "pharmacy", "veterinary"};
  static const std::unordered_set<std::string> education = {
      "school", "university", "college", "kindergarten", "library"};
  static const std::unordered_set<std::string> transport = {
      "bus_station", "taxi",    "car_rental",
      "fuel",        "parking", "charging_station"};
  static const std::unordered_set<std::string> finance = {"bank", "atm",
                                                          "bureau_de_change"};
  static const std::unordered_set<std::string> shopping = {
      "marketplace", "convenience", "supermarket", "mall"};
  static const std::unordered_set<std::string> accommodation = {
      "hotel", "motel", "hostel"};
  static const std::unordered_set<std::string> services = {
      "post_office", "laundry", "beauty", "hairdresser"};
  static const std::unordered_set<std::string> religion = {
      "place_of_worship", "church", "mosque", "synagogue", "temple"};
  static const std::unordered_set<std::string> emergency = {"police",
                                                            "fire_station"};

  // Distinct vibrant colors
  if (food.count(type)) return {ezgl::color(230, 50, 30), "Food & Drink"};
  if (health.count(type)) return {ezgl::color(220, 20, 120), "Healthcare"};
  if (education.count(type)) return {ezgl::color(20, 100, 220), "Education"};
  if (transport.count(type)) return {ezgl::color(100, 100, 100), "Transport"};
  if (finance.count(type)) return {ezgl::color(10, 180, 60), "Finance"};
  if (shopping.count(type)) return {ezgl::color(200, 30, 200), "Shopping"};
  if (accommodation.count(type))
    return {ezgl::color(230, 140, 0), "Accommodation"};
  if (services.count(type)) return {ezgl::color(50, 150, 200), "Services"};
  if (religion.count(type)) return {ezgl::color(140, 70, 180), "Religion"};
  if (emergency.count(type)) return {ezgl::color(200, 0, 0), "Emergency"};

  return {ezgl::color(120, 120, 120), "Other"};
}


void draw_main_canvas(ezgl::renderer *g) {
//Get what part of the map is on the screen 
  ezgl::rectangle visible_world = g->get_visible_world();

//Night mode colours 
  if (is_night_mode) g->set_color(30, 30, 30);
  else g->set_color(240, 240, 240);
  
  g->fill_rectangle(visible_world);

//Colour in the features and choose map 
  for (const auto &feat : features) {
    g->set_color(get_feature_color(feat.type, is_night_mode));
    if (feat.is_closed && feat.points.size() > 2) {
      g->fill_poly(feat.points); //Closed feature 
    } else {
      g->set_line_width(1);
      for (size_t i = 0; i < feat.points.size() - 1; i++) {
        g->draw_line(feat.points[i], feat.points[i + 1]);//Open features 
      }
    }
  }

  double current_zoom_width = visible_world.width();
  
  //Stores one way segments 
  std::vector<std::pair<StreetSegmentIdx, const streetSegments*>> oneway_segments;
  
  //Draws the streets colours and width 
  for (int idx = 0; idx < (int)streets.size(); ++idx) {
    const auto &seg = streets[idx];
    float speed_kmh = seg.speedLimit * 3.6f;
    //If statements used to control the zoom level 
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
    //Draw street segment lines 
    for (size_t i = 0; i < seg.points.size() - 1; i++) {
      g->draw_line(seg.points[i], seg.points[i + 1]);
    }
    //Store the one way streets 
    StreetSegmentInfo info = getStreetSegmentInfo(idx);
    if (info.oneWay && speed_kmh < 80) {
      oneway_segments.push_back({idx, &seg});
    }
  }

//Arrows for one way 
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
      
      const streetSegments* seg = pair.second;
      
      
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

//For subway lines 
  if (current_zoom_width < 100000) { 
      g->set_line_width(4); 
      //Goes through each subway line 
      for (const auto& line : subway_lines) {
          g->set_color(line.color);
          //Go through the track segments 
          for (const auto& track : line.tracks) {
            //Go through the nodes of the track 
              for (size_t i = 0; i < track.size() - 1; ++i) {
                  g->draw_line(track[i], track[i + 1]);
              }
          }
      }
  }
  //Draw the circles for the subway stations 
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

  // Each entry is {center_x, center_y, half_width, half_height} of a drawn label
  struct LabelBox { double cx, cy, hw, hh; };
  std::vector<LabelBox> drawn_label_boxes;

  // Estimate how many world-units one character occupies at this zoom
  // font size 9 => ~7px per char; scale by zoom so boxes are in world coords
  // visible_world.width() / canvas_pixel_width ≈ zoom scale
  // We don't have canvas pixel width here, so use a fixed pixel estimate (1000px typical)
  double px_per_world = 1000.0 / current_zoom_width; // pixels per world unit
  double char_w_world = 7.0 / px_per_world;          // world units per character
  double char_h_world = 12.0 / px_per_world;         // world units for label height

  auto overlaps_any = [&](double cx, double cy, double hw, double hh) -> bool {
    for (const auto& box : drawn_label_boxes) {
      if (std::abs(cx - box.cx) < (hw + box.hw) &&
          std::abs(cy - box.cy) < (hh + box.hh)) {
        return true;
      }
    }
    return false;
  };

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

        double label_cx = (p1.x + p2.x) / 2.0;
        double label_cy = (p1.y + p2.y) / 2.0;
        double abs_angle = std::abs(angle);
        double angle_rad = abs_angle * M_PI / 180.0;

        // For a rotated label, the axis-aligned bounding box shrinks in one
        // dimension and grows in the other. Approximate with trig:
        double text_len = seg.name.length() * char_w_world;
        double label_hw = (text_len * std::cos(angle_rad) + char_h_world * std::sin(angle_rad)) / 2.0;
        double label_hh = (text_len * std::sin(angle_rad) + char_h_world * std::cos(angle_rad)) / 2.0;

        double pad = char_w_world * 0.6;

        if (overlaps_any(label_cx, label_cy, label_hw + pad, label_hh + pad)) continue;

        drawn_label_boxes.push_back({label_cx, label_cy, label_hw, label_hh});

        g->set_text_rotation(angle);
        g->draw_text({label_cx, label_cy}, seg.name);
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
    double label_threshold = 800.0;
    double cluster_distance = 80.0;
    
    if (visible_world.width() > 2000) {
      base_radius = 16.0;
      cluster_distance = 100.0;
      label_threshold = 0.0; // Don't show labels when far
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
          double lx = cluster.center.x;
          double ly = cluster.center.y + radius + 10;
          double lhw = (name.length() * char_w_world) / 2.0;
          double lhh = char_h_world / 2.0;

          // Only draw if it doesn't overlap any street label already placed
          if (!overlaps_any(lx, ly, lhw + char_w_world, lhh + char_h_world * 0.5)) {
            // Also check it's not too close to any subway line track
            bool near_subway = false;
            for (const auto& line : subway_lines) {
              for (const auto& track : line.tracks) {
                for (size_t ti = 0; ti + 1 < track.size(); ++ti) {
                  // Simple AABB check: is label center within ~30 world units of segment
                  double min_tx = std::min(track[ti].x, track[ti+1].x) - 30;
                  double max_tx = std::max(track[ti].x, track[ti+1].x) + 30;
                  double min_ty = std::min(track[ti].y, track[ti+1].y) - 30;
                  double max_ty = std::max(track[ti].y, track[ti+1].y) + 30;
                  if (lx >= min_tx && lx <= max_tx && ly >= min_ty && ly <= max_ty) {
                    near_subway = true;
                    break;
                  }
                }
                if (near_subway) break;
              }
              if (near_subway) break;
            }

            if (!near_subway) {
              if (is_night_mode) g->set_color(ezgl::WHITE);
              else g->set_color(ezgl::BLACK);
              g->set_font_size(9);
              g->draw_text({lx, ly}, name);
              // Register this label so future POI labels don't overlap it either
              drawn_label_boxes.push_back({lx, ly, lhw, lhh});
            }
          }
        }
      }

      drawn_pois.push_back({cluster.center, radius});
    }
  }  // ← closes if (visible_world.width() < 5000)

  // Search result highlight — always draw regardless of zoom
  if (search_result_is_poi) {
    ezgl::point2d c = {search_result_x, search_result_y};
    g->set_color(ezgl::color(255, 0, 255, 80));
    g->fill_arc(c, 60, 0, 360);
    g->set_color(ezgl::WHITE);
    g->fill_arc(c, 20, 0, 360);
    g->set_color(ezgl::color(255, 0, 255));
    g->set_line_width(3);
    g->draw_arc(c, 30, 0, 360);
  } else if (search_result_intersection >= 0) {
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

  //Get the dimensions of the map that is being drawn on the screen  
  double screen_width_meters = visible_world.width();
  double screen_height_meters = visible_world.height();

  //Determine the scale of the bar, make it 15% 
  double ideal_scale = screen_width_meters * 0.15; 
  double scale_val; 

  //Describe the increments of the scale
  if (ideal_scale > 10000) scale_val = 10000;      // 10 km
  else if (ideal_scale > 5000) scale_val = 5000;   // 5 km
  else if (ideal_scale > 2000) scale_val = 2000;   // 2 km
  else if (ideal_scale > 1000) scale_val = 1000;   // 1 km
  else if (ideal_scale > 500) scale_val = 500;     // 500 m
  else if (ideal_scale > 100) scale_val = 100;     // 100 m
  else if (ideal_scale > 50) scale_val = 50;       // 50 m
  else scale_val = 10;                             // 10 m

  //pad_x and pad_y store the 
  //The distance between right edge of screen to end of scale bar 
  double pad_x = screen_width_meters * 0.05; 
  //The distance bottom of screen to edge of scale bar 
  double pad_y = screen_height_meters * 0.05;

  //Find the boundaries of the current camera view
  double right_edge = visible_world.center_x() + (screen_width_meters / 2.0);
  double bottom_edge = visible_world.center_y() - (screen_height_meters / 2.0);

  double start_x = right_edge - pad_x - scale_val;
  double end_x = right_edge - pad_x;
  double y_pos = bottom_edge + pad_y;

  //Draw the horizontal line of the scale  
  if (is_night_mode) g->set_color(ezgl::WHITE);
  else g->set_color(ezgl::BLACK);

  g->set_line_width(3);
  g->draw_line({start_x, y_pos}, {end_x, y_pos});

  //Draw the vertical lines 
  double tick_height = screen_height_meters * 0.01;//Tick height is the percentage of the screens height
  g->draw_line({start_x, y_pos - tick_height}, {start_x, y_pos + tick_height});
  g->draw_line({end_x, y_pos - tick_height}, {end_x, y_pos + tick_height});

  //Write the text label above the bar, based on km or m 
  std::string scale_text = (scale_val >= 1000) ? 
                          std::to_string((int)(scale_val / 1000)) + " km" : 
                          std::to_string((int)scale_val) + " m";
                          
  g->set_font_size(10);
  // Shift the text slightly up (positive Y) so it floats above the line
  g->draw_text({(start_x + end_x) / 2.0, y_pos + (tick_height * 2.5)}, scale_text);

  /// Calculate a dynamic radius so the markers are always visible regardless of zoom
    double marker_radius = current_zoom_width * 0.01; 
    if (marker_radius < 15.0) marker_radius = 15.0;    // Minimum size
    if (marker_radius > 150.0) marker_radius = 150.0;  // Maximum size

    // Highlight the Start Intersection
    if (start_intersection_id != -1) {
        ezgl::point2d start_center = {intersections[start_intersection_id].x, intersections[start_intersection_id].y};
        
        // White border for contrast
        g->set_color(ezgl::WHITE);
        g->fill_arc(start_center, marker_radius + (current_zoom_width * 0.002), 0, 360);
        
        // Bright Green inner circle
        g->set_color(ezgl::color(0, 200, 0));
        g->fill_arc(start_center, marker_radius, 0, 360); 

        // Text Label
        if (is_night_mode) g->set_color(ezgl::WHITE);
        else g->set_color(ezgl::BLACK);
        g->set_font_size(12);
        g->draw_text({start_center.x, start_center.y + marker_radius + 30}, "START");
    }

    // Highlight the Destination Intersection
    if (dest_intersection_id != -1) {
        ezgl::point2d dest_center = {intersections[dest_intersection_id].x, intersections[dest_intersection_id].y};
        
        // White border for contrast
        g->set_color(ezgl::WHITE);
        g->fill_arc(dest_center, marker_radius + (current_zoom_width * 0.002), 0, 360);
        
        // Bright Red inner circle
        g->set_color(ezgl::color(220, 0, 0));
        g->fill_arc(dest_center, marker_radius, 0, 360); 

        // Text Label
        if (is_night_mode) g->set_color(ezgl::WHITE);
        else g->set_color(ezgl::BLACK);
        g->set_font_size(12);
        g->draw_text({dest_center.x, dest_center.y + marker_radius + 30}, "DEST");
    }

    // Draw the Route
    if (!current_walk_route.empty()) {
        g->set_color(ezgl::color(100, 150, 255, 200)); // Lighter blue for walking
        g->set_line_width(4);
        g->set_line_dash(ezgl::line_dash::asymmetric_5_3); // Make it a dashed line!

        for (StreetSegmentIdx seg_id : current_walk_route) {
            StreetSegmentInfo info = getStreetSegmentInfo(seg_id);
            LatLon from_latlon = getIntersectionPosition(info.from);
            ezgl::point2d prev_point = {xFromLon(from_latlon.longitude()), yFromLat(from_latlon.latitude())};
            
            for (int i = 0; i < info.numCurvePoints; ++i) {
                LatLon curve_latlon = getStreetSegmentCurvePoint(seg_id, i);
                ezgl::point2d curr_point = {xFromLon(curve_latlon.longitude()), yFromLat(curve_latlon.latitude())};
                g->draw_line(prev_point, curr_point);
                prev_point = curr_point;
            }
            LatLon to_latlon = getIntersectionPosition(info.to);
            ezgl::point2d final_point = {xFromLon(to_latlon.longitude()), yFromLat(to_latlon.latitude())};
            g->draw_line(prev_point, final_point);
        }
        g->set_line_dash(ezgl::line_dash::none); // Reset dash so we don't break other lines
    }

    // Draw the Driving Route (e.g., Solid Dark Blue)
    if (!current_drive_route.empty()) {
        g->set_color(ezgl::BLUE);
        g->set_line_width(3); 
        g->set_line_cap(ezgl::line_cap::round);

        for (StreetSegmentIdx seg_id : current_drive_route) {
            StreetSegmentInfo info = getStreetSegmentInfo(seg_id);
            LatLon from_latlon = getIntersectionPosition(info.from);
            ezgl::point2d prev_point = {xFromLon(from_latlon.longitude()), yFromLat(from_latlon.latitude())};
            
            for (int i = 0; i < info.numCurvePoints; ++i) {
                LatLon curve_latlon = getStreetSegmentCurvePoint(seg_id, i);
                ezgl::point2d curr_point = {xFromLon(curve_latlon.longitude()), yFromLat(curve_latlon.latitude())};
                g->draw_line(prev_point, curr_point);
                prev_point = curr_point;
            }
            LatLon to_latlon = getIntersectionPosition(info.to);
            ezgl::point2d final_point = {xFromLon(to_latlon.longitude()), yFromLat(to_latlon.latitude())};
            g->draw_line(prev_point, final_point);
        }
    }
}


// Main function to initialize and run the application
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