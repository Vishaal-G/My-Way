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


#pragma once

// Forward declare GTK types so this header compiles without GTK on the include path.
// The .cpp file includes <gtk/gtk.h> directly.
typedef struct _GtkWidget      GtkWidget;
typedef struct _GdkEventButton GdkEventButton;
typedef void*                  gpointer;

#include <string>
#include <vector>
#include <unordered_map>

#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m1.hpp"
#include "m3.hpp"

// Constants
extern double cos_lat_avg;

// Structs

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

struct SubwayLine {
    std::string name;
    ezgl::color color;
    std::vector<std::vector<ezgl::point2d>> tracks;
};

struct SubwayStation {
    std::string name;
    ezgl::point2d position;
};

// Global Data
extern std::vector<MyFeature> features;
extern std::vector<streetSegments> streets;
extern std::vector<MyPOI> Mypois;
extern std::vector<Intersection> intersections;

extern std::vector<SubwayLine> subway_lines;
extern std::vector<SubwayStation> subway_stations;

extern std::unordered_map<OSMID, const OSMWay*> osm_ways_map;
extern std::unordered_map<OSMID, const OSMNode*> osm_nodes_map;

extern double global_maxLat, global_minLat, global_maxLon, global_minLon;

// Coordinate Conversion
float xFromLon(float lon);
float yFromLat(float lat);
float lonFromX(float x);
float latFromY(float y);

ezgl::color findSubwayColour(std::string hex_str);
void findMapFiles(); 

// Internal Helpers
bool ends_with(const std::string& text, const std::string& suffix);
ezgl::color parse_hex_color(std::string hex_str);

// Map Loading
void discover_map_paths();
void load_map_data();
void build_autocomplete_store();

// Drawing
void draw_main_canvas(ezgl::renderer* g);
ezgl::color get_feature_color(FeatureType type, bool night);

// UI / Event Handlers
void initial_setup(ezgl::application* app, bool new_window);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);
void load_selected_map(GtkWidget* widget, gpointer data);

// Entry Point
void drawMap();