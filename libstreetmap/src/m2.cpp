/* 
 * Copyright 2026 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "m2.h"
#include "m1.hpp" 
#include <gtk/gtk.h>
//These are the graphics libraries
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <sstream>

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

// Global vector to hold POI data
std::vector<MyPOI> Mypois; 

// Global variables
static int selected_intersection = -1;
std::vector<Intersection> intersections; //Store all the intersections 
static std::vector<int> highlighted_intersections;

float xFromLon(float lon);
float yFromLat(float lat);

float lonFromX(float x);
float latFromY(float y);

void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);

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

void act_on_mouse_click(ezgl::application* app, GdkEventButton*, double x, double y) {
  LatLon clicked_pos(latFromY(y), lonFromX(x));
  selected_intersection = findClosestIntersection(clicked_pos);
  if (selected_intersection != -1) {
        // 1. Print to the terminal
        std::cout << "Clicked: " << intersections[selected_intersection].name << "\n"; 
        
        // 2. Print to the EZGL UI bottom status bar
        std::stringstream ss; 
        ss << "Intersection Clicked: " << intersections[selected_intersection].name;
        app->update_message(ss.str());
    }
  app->refresh_drawing();
}

void draw_main_canvas (ezgl::renderer *g){
   auto startTime = std::chrono::high_resolution_clock::now();
   ezgl::rectangle visible_world = g->get_visible_world();
   std::cout << "Drawing canvas with " << intersections.size() << " intersections!" << std::endl;
   g->fill_rectangle(g->get_visible_world());
   g->set_color(0, 0, 0);  

   for(size_t i = 0; i < intersections.size(); ++i){

      //make sure to scale these using xfromLon & yFromLat
      float x = intersections[i].x; 
      float y = intersections[i].y; 

      float width = 50; 
      float height = width; 

      //Starting at {x,y} and draw until {x+width, y+height}
      g->fill_rectangle({x, y}, {x+width, y+height});
   }
   auto currTime = std::chrono::high_resolution_clock::now(); 
   auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currTime - startTime);
   std::cout << "Canvas took " << wallClock.count() << " seconds \n";

// Drawing highed intersections from the find feature
g->set_color(ezgl::YELLOW);

for(int id : highlighted_intersections) {
    ezgl::point2d center = {
        intersections[id].x,
        intersections[id].y
    };
    g->fill_arc(center, 50, 0, 360);
}

   // For clicking the intersection
   if(selected_intersection != -1) {
  g->set_color(ezgl::RED);
ezgl::point2d center = {
    intersections[selected_intersection].x,
    intersections[selected_intersection].y
};

g->set_color(ezgl::RED);
g->fill_arc(center, 60, 0, 360);
}
if (visible_world.width() < 5000) { 
        g->set_color(ezgl::BLUE);
        
        for (const auto& poi : Mypois) {
            // Visibility check: skip if off-screen
            if (visible_world.contains(poi.x, poi.y)) {
                // Draw a small circle/arc for the POI
                g->fill_arc({poi.x, poi.y}, 20, 0, 360);
                
                // Draw text if zoomed in even further
                if (visible_world.width() < 1000) {
                    g->set_color(ezgl::BLACK);
                    g->draw_text({poi.x, poi.y + 25}, poi.name);
                    g->set_color(ezgl::BLUE); 
                }
            }
        }
    }


} 

static void find_and_highlight(const std::string& street1,
                                  const std::string& street2) {

   // Clear highlights from previous use
    highlighted_intersections.clear();

   // Get all street IDs matching (or partial) with the street inputs
    auto s1 = findStreetIdsFromPartialStreetName(street1);
    auto s2 = findStreetIdsFromPartialStreetName(street2);

   // Exit if no matches
    if(s1.empty() || s2.empty()) {
        std::cout << "Street not found.\n";
        return;
    }

   // Checking combinations of matching input IDs
    for(int id1 : s1) {
        for(int id2 : s2) {

           // Find intersections shared by the two streets
            auto ints = findIntersectionsOfTwoStreets(id1, id2);

            // If intersections exists, store and exit 
            if(!ints.empty()) {
                highlighted_intersections = std::move(ints);
                std::cout << "Found "
                          << highlighted_intersections.size()
                          << " intersections.\n";
                return;
            }
        }
    }
   // Else no intersections found for this pair
    std::cout << "Found 0 intersections.\n";
}

static void find_button(GtkWidget*, gpointer data) {
    auto* app = static_cast<ezgl::application*>(data);

   // Access street name entry fields  from UI
    GtkEntry* e1 = GTK_ENTRY(app->get_object("Street1Entry"));
    GtkEntry* e2 = GTK_ENTRY(app->get_object("Street2Entry"));

   // Get text entered by the user
    std::string s1 = gtk_entry_get_text(e1);
    std::string s2 = gtk_entry_get_text(e2);

   // Find + highlight intersections of the streets user entered (if any)
    find_and_highlight(s1, s2);

    // Refreshing canvas
    app->refresh_drawing();
}

void initial_setup(ezgl::application* app, bool) {
    GtkWidget* find_btn = GTK_WIDGET(app->get_object("FindButton"));
    g_signal_connect(find_btn, "clicked", G_CALLBACK(find_button), app);
}

void drawMap() {
   
   //Stores the max and min coordinate system that you need to paint 
   double maxLat = getIntersectionPosition(0).latitude(); 
   double minLat = maxLat; 
   double maxLon = getIntersectionPosition(0).longitude(); 
   double minLon = maxLon; 

   //To draw intersections 
   intersections.clear();
   intersections.resize(getNumIntersections()); 
   for(int i = 0; i < getNumIntersections(); ++i){
      intersections[i].position = getIntersectionPosition(i);
      intersections[i].name = getIntersectionName(i); 

      maxLat = std::max(maxLat, intersections[i].position.latitude()); 
      minLat = std::min(minLat, intersections[i].position.latitude()); 
      maxLon = std::max(maxLon, intersections[i].position.longitude()); 
      minLon = std::min(minLon, intersections[i].position.longitude()); 
   }
   double avgLat = ((maxLat + minLat) / 2.0) * kDegreeToRadian;
   cos_lat_avg = cos(avgLat);

   for(int i = 0; i < getNumIntersections(); ++i){
      intersections[i].x = xFromLon(intersections[i].position.longitude());
      intersections[i].y = yFromLat(intersections[i].position.latitude());

   }

   //Loading POI
   Mypois.clear();
    int numPOIs = getNumPointsOfInterest();
    Mypois.resize(numPOIs);
    
    for (int i = 0; i < numPOIs; i++) {
        Mypois[i].position = getPOIPosition(i);
        Mypois[i].name = getPOIName(i);
        // Project coordinates using the already-calculated cos_lat_avg
        Mypois[i].x = xFromLon(Mypois[i].position.longitude());
        Mypois[i].y = yFromLat(Mypois[i].position.latitude());
    }

   ezgl::application::settings settings;
   settings.main_ui_resource = "libstreetmap/resources/main.ui"; 
   settings.window_identifier = "MainWindow";
   settings.canvas_identifier = "MainCanvas";
    
   //Create the ezgl application 
   ezgl::application application(settings);


   //Creates canvas of 1000x1000 
   ezgl::rectangle initial_world({xFromLon(minLon), yFromLat(minLat)}, {xFromLon(maxLon), yFromLat(maxLat)});
   application.add_canvas("MainCanvas", draw_main_canvas, initial_world);

   //Run the ezgl application 
   application.run(initial_setup, act_on_mouse_click, nullptr, nullptr);
}
