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
#include <iostream>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include <map> 
#include <vector> 
#include <string> 
#include <algorithm> 
#include <utility>
#include <cctype> 


// loadMap will be called with the name of the file that stores the "layer-2"
// map data accessed through StreetsDatabaseAPI: the street and intersection 
// data that is higher-level than the raw OSM data). 
// This file name will always end in ".streets.bin" and you 
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 (StreetsDatabase) API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1 
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the 
// name of the ".osm.bin" file that matches your map -- just change 
// ".streets" to ".osm" in the map_streets_database_filename to get the proper
// name.

//Global to store street name and the street index 
std::vector<std::pair<std::string, StreetIdx>> streetNametoId;
//HELPER FUNCTION, removes spaces and makes string all lowercase 
std::string cleanName(std::string name){
    std::string cleaned; 
    for(char c : name){
        if(c != ' '){
            cleaned += std::tolower(c);
        }
    }
    return cleaned; 
}

bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully

    std::cout << "loadMap: " << map_streets_database_filename << std::endl;

    //
    // Load your map related data structures here.
    //
    //For the function findStreetIdsFromPartialStreetName, need to first create structure to make look up time faster
    
    streetNametoId.clear(); //Clean up the map on each run 
    streetNametoId.reserve(getNumStreets());
    for(int i = 0; i < getNumStreets(); i++){
        std::string inputName = getStreetName(i); //Gets all street names 
        std::string streetName = cleanName(inputName); //Gets rid of spaces and all lowercase
        streetNametoId.push_back({streetName, i}); 
    }
    std::sort(streetNametoId.begin(), streetNametoId.end()); //Sorts the street names by alphabetical order 
    

    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here
    
}

double findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> points){
    //Convert both sets of latitutde and longitude to radians
    double lat1 = points.first.latitude() * kDegreeToRadian;
    double lon1 = points.first.longitude() * kDegreeToRadian;

    double lat2 = points.second.latitude() * kDegreeToRadian;
    double lon2 = points.second.longitude() * kDegreeToRadian;

    //Find average of latitudes
    double latAvg = (lat1 + lat2) / 2.0;

    //Compute projected x,y coordinates
    double x1 = kEarthRadiusInMeters * lon1 * cos(latAvg);
    double y1 = kEarthRadiusInMeters * lat1;

    double x2 = kEarthRadiusInMeters * lon2 * cos(latAvg);
    double y2 = kEarthRadiusInMeters * lat2;

    //Compute distance and return it
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));

}

double findStreetSegmentLength(StreetSegmentIdx street_segment_id){
    return 0;
}

double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id){
    return 0;
}

double findStreetSegmentTurnAngle(StreetSegmentIdx dst_street_segment_id, StreetSegmentIdx src_street_segment_id){
    return 0;
}

std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id){
    return {};
}

IntersectionIdx findClosestIntersection(LatLon my_position){
    return 0;
}

std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id){
    return {};
}

std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(StreetIdx street_id1, StreetIdx street_id2){
    return {};
}

std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix){
    std::vector<StreetIdx> answer;
    if(street_prefix.empty()){ //If parameter is empty, return empty vector 
        return answer; 
    }
    //Make the parameter all lowercase and get rid of spaces 
    std::string prefix = cleanName(street_prefix);


    //Create a target to search for
    //-1 is less than any index, to ensure that the iterator starts at the first prefix
    std::pair<std::string, StreetIdx> target = std::make_pair(prefix, -1);
    //Create iterator it to iterate through streetNametoId  
    std::vector<std::pair<std::string, StreetIdx>>::iterator it; 
    //lower_bound searches through streetNametoId to get the first element >= target
    it = std::lower_bound(streetNametoId.begin(), streetNametoId.end(), target); 

    //Compares all keys in streetNametoId where prefix is the same, and adds it to answer 
    while(it != streetNametoId.end()){
        if(it->first.compare(0, prefix.length(), prefix) != 0){//Compares prefix 
            break; 
        }
        answer.push_back(it->second);//Insert street index if same prefix 
        ++it; 
    }
    return answer; 
}

LatLonBounds findStreetBoundingBox (StreetIdx street_id){
    LatLonBounds bounds{};
    return bounds;

}

std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id){
    return {};
}

double findStreetLength(StreetIdx street_id){
    return 0;
}

POIIdx findClosestPOI(LatLon my_position, std::string poi_name){
    return 0;
}

double findFeatureArea(FeatureIdx feature_id){
    return 0;
}

double findWayLength(OSMID way_id){
    return 0;
}

std::string getOSMNodeTagValue(OSMID osm_id, std::string key){
    return "";
}

