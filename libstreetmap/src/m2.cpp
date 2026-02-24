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
//These are the graphics libraries
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <cmath>
#include <algorithm>

double cos_lat_avg;
void draw_main_canvas (ezgl::renderer *g);


struct Intersection{
   LatLon position;
   std::string name; 
   float x; 
   float y; 
};
//Store all the intersections 
std::vector<Intersection> intersections; 

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

void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y){
   std::cout << "Mouse clicked at"  << x << " " << y; 

   LatLon pos = LatLon(latFromY(y), lonFromX(x)); 
   int id = findClosestIntersection(pos); 

   float width = 50; 
   float height = width; 

   ezgl::renderer *g = app->get_renderer();
   g->set_color(ezgl::RED); 
   g->fill_rectangle({intersections[id].x, intersections[id].y}, width, height); 

   std::cout << "Closest position " << intersections[id].name; 
}

void draw_main_canvas (ezgl::renderer *g){
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
} 

void drawMap() {
   
   //Stores the max and min coordinate system that you need to paint 
   double maxLat = getIntersectionPosition(0).latitude(); 
   double minLat = maxLat; 
   double maxLon = getIntersectionPosition(0).longitude(); 
   double minLon = maxLon; 

   //To draw intersections 
   intersections.resize(getNumIntersections()); 
   for(int i = 0; i < getNumIntersections(); ++i){
      intersections[i].position = getIntersectionPosition(i);
      intersections[i].name = getIntersectionName(i); 

      maxLat = std::max(maxLat, intersections[i].position.latitude()); 
      minLat = std::min(minLat, intersections[i].position.latitude()); 
      maxLon = std::max(maxLon, intersections[i].position.longitude()); 
      minLon = std::min(minLon, intersections[i].position.longitude()); 
   }
   //THIS DOES NOT WORK, supposed to calculate the average cos lat ?? to scale the map correctly  
   double avgLat = ((maxLat + minLat) / 2.0) * kDegreeToRadian;
   cos_lat_avg = cos(avgLat);

   for(int i = 0; i < getNumIntersections(); ++i){
      intersections[i].x = xFromLon(intersections[i].position.longitude());
      intersections[i].y = yFromLat(intersections[i].position.latitude());

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
   application.run(initial_setup, act_on_mouse_click, act_on_mouse_move, act_on_key_press); 
}
