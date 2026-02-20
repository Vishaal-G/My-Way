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
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"

void draw_main_canvas (ezgl::renderer *g);

void draw_main_canvas (ezgl::renderer *g){
   g->set_color(ezgl::BLACK);
   g->draw_rectangle({0, 0},{1000, 1000});
}

void drawMap() {
    // 1. Setup EZGL settings (Make sure the UI path matches your repo structure!)
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui"; 
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
    
    // 2. Create the application object
    ezgl::application application(settings);
    
   ezgl::rectangle initial_world({0, 0}, {1000,1000});
   application.add_canvas("MainCanvas", draw_main_canvas, initial_world);

   application.run(nullptr, nullptr, nullptr, nullptr);

}
