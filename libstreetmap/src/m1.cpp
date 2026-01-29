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
#include "OSMDatabaseAPI.h"
#include <cmath>
#include <map> 
#include <vector> 
#include <string> 
#include <algorithm> 
#include <utility>
#include <cctype> 
#include <limits> 
#include <unordered_map> 
#include <iterator>


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

// Global variable declarations
std::vector<std::pair<std::string, StreetIdx>> streetNametoId; //Used for findStreetIdsFromPartialStreetName
                                                               //Global to store street name and the street index 
static std::vector<std::vector<IntersectionIdx>> street_to_intersections;

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

//Used for findClosestIntersection 
std::vector<LatLon> pointsOfIntersections;

//Used for findClosestPOI
std::vector<LatLon> POIPositions; //Stores LatLon of all the POIs
std::unordered_map<std::string, std::vector<POIIdx>> POIbyName; //Allows you to look up all Indexes of a POI by name 

//Used for findWayLength 
std::unordered_map<OSMID, const OSMWay*> OSMWayFromID;
std::unordered_map<OSMID, LatLon> LatLonFromOSMID;


bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = loadStreetsDatabaseBIN(map_streets_database_filename);
    if (!load_successful) {
        return false; //Indicates whether the map has loaded successfully
    } 

    std::cout << "loadMap: " << map_streets_database_filename << std::endl;

  
    //For findStreetIdsFromPartialStreetName, need to first create structure to make look up time faster
    streetNametoId.clear(); //Clean up the map on each run 
    streetNametoId.reserve(getNumStreets());
    for(int i = 0; i < getNumStreets(); i++){
        std::string inputName = getStreetName(i); //Gets all street names 
        std::string streetName = cleanName(inputName); //Gets rid of spaces and all lowercase
        streetNametoId.push_back({streetName, i}); 
    }
    std::sort(streetNametoId.begin(), streetNametoId.end()); //Sorts the street names by alphabetical order 
    
    //For findClosestIntersection, stores all the LatLons of the intersections for faster lookup time 
    int numOfIntersections = getNumIntersections(); 
    pointsOfIntersections.resize(numOfIntersections); 
    for(int i = 0; i < numOfIntersections; ++i){
        pointsOfIntersections[i] = getIntersectionPosition(i); 
    }

    //For findClosestPOI
    int numberOfPOIs = getNumPointsOfInterest(); 
    POIPositions.resize(numberOfPOIs);
    POIbyName.clear(); //Clear the hashmap 
    POIbyName.reserve(numberOfPOIs); 

    for(int i = 0; i < numberOfPOIs; ++i){
        //Stores all the LatLon of POIs
        POIPositions[i] = getPOIPosition(i);

        //Stores name and groups names of POI with associated index 
        std:: string POIName = getPOIName(i); 
        POIbyName[POIName].push_back(i); 
    }

    // For findIntersectionsOfStreet, findIntersectionsOfTwoStreets
    street_to_intersections.clear();
    street_to_intersections.resize(getNumStreets());

    for (StreetSegmentIdx i = 0; i < getNumStreetSegments(); i++) {
        StreetSegmentInfo info = getStreetSegmentInfo(i);

        // Every segment touches two intersections
        street_to_intersections[info.streetID].push_back(info.from);
        street_to_intersections[info.streetID].push_back(info.to);
    }
    for (size_t i = 0; i < street_to_intersections.size(); i++) { 

        // Removing duplicate intersections in the vector
        std::vector<IntersectionIdx> &intersections= street_to_intersections[i];
        std::sort(intersections.begin(), intersections.end()); // Sort all streets so duplicates are next to eachother
        intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end()); // Deletes streets that are not duplicates
    }

    //For findWayLength 
    //This is to replace the streets file with the osm file 
    std::string osm_filename = map_streets_database_filename; 
    std::string replace = ".streets.bin";
    std::string replaceWith = ".osm.bin";
    //Loads the OSM database 
    size_t find = osm_filename.find(replace);
    osm_filename.replace(find, replace.length(), replaceWith); //Gets the OSM database 
    bool osm_load_successful = loadOSMDatabaseBIN(osm_filename);
    if (!osm_load_successful) {
        return false; //Indicates whether the map has loaded successfully
    }
    //Populate the LatLonFromOSMID hashmap 
    int numNodes = getNumberOfNodes(); 
    LatLonFromOSMID.clear(); 
    LatLonFromOSMID.reserve(numNodes); 

    for(int i = 0; i < numNodes; ++i){
        const OSMNode* node = getNodeByIndex(i); 
        LatLonFromOSMID[node->id()] = getNodeCoords(node); 
    }

    //Populate the OSMWayFromID hashmap 
    int numWays = getNumberOfWays();
    OSMWayFromID.clear();
    OSMWayFromID.reserve(numWays);

    for (int i = 0; i < numWays; ++i) {
        const OSMWay* way = getWayByIndex(i);
        OSMWayFromID[way->id()] = way;
    }


    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here

    //Deallocate the memory 
    OSMWayFromID.clear();
    LatLonFromOSMID.clear();
    
    //Close the OSM Database 
    closeOSMDatabase();
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
    //Getting from and to ID's
    StreetSegmentInfo info = getStreetSegmentInfo(street_segment_id);

    //Get Lat/Lon coordinates of the start and finish
    LatLon start = getIntersectionPosition(info.from);
    LatLon finish = getIntersectionPosition(info.to);

    //Compute distance based on how many curve points there are
    double totalDistance = 0;
    
    //Set starting location to start coordinate 
    LatLon prev = start;

    //Loop till number of curve points are there, so we can calculate each segment
    for (int i = 0; i < info.numCurvePoints; i++){
        //Get current coordinate of the curve and compute distance of line segment
        LatLon current = getStreetSegmentCurvePoint(street_segment_id, i);
        totalDistance += findDistanceBetweenTwoPoints({prev,current});
        prev = current;
    }

    //Loop doesnt account for the line segment between last point --> finish, so include it
    totalDistance += findDistanceBetweenTwoPoints({prev, finish});
    return totalDistance;

}

double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id){
    // Load street data
    StreetSegmentInfo info = getStreetSegmentInfo(street_segment_id);

    // Finding time using distance/speed
    double length = findStreetSegmentLength(street_segment_id); // m
    double speed = info.speedLimit; // metres/s

    return length / speed;
}

double findStreetSegmentTurnAngle(StreetSegmentIdx dst_street_segment_id, StreetSegmentIdx src_street_segment_id){
    return 0;
}

std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id){
    return {};
}

IntersectionIdx findClosestIntersection(LatLon my_position){
    
    //Edge case where there are no points of intersections 
    if(pointsOfIntersections.empty())
        return -1; 

    IntersectionIdx answer; 
    double minDistance = std::numeric_limits<double>::max(); //Makes minDistance largest possible number 

    //Gets distance from my_position to every intersection to find the lowest distance one
    for(int i = 0; i < pointsOfIntersections.size(); i++){
        double distance = findDistanceBetweenTwoPoints(std::make_pair(my_position, pointsOfIntersections[i])); 
        if(distance < minDistance){
            minDistance = distance; 
            answer = i; 
        }   
    }   
    return answer;
}

std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id){
    return street_to_intersections[street_id];
}

std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(StreetIdx street_id1, StreetIdx street_id2){ 

    // Find intersections of street 1
    std::vector<IntersectionIdx> &street1_intersections =
        street_to_intersections[street_id1];

    // Find intersections of street 2
    std::vector<IntersectionIdx> &street2_intersections =
        street_to_intersections[street_id2];

    std::vector<IntersectionIdx> result; // Store the intersections  of both streets

    // Find common intersections of the two streets
    std::set_intersection(
        street1_intersections.begin(), street1_intersections.end(),
        street2_intersections.begin(), street2_intersections.end(),
        std::back_inserter(result) // Adds common intersection to result list
    );

    return result;
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

    //it is iterator of hashmap, finds the list of POIs with the given name 
    auto it = POIbyName.find(poi_name); 
    if(it == POIbyName.end())
        return -1; 

    //Use & so it doesn't make a copy, and foundPOIs stores the vector of POIIdxs 
    const std::vector<POIIdx>& foundPOIs = it->second; 
    POIIdx answer = -1; 
    double minDistance = std::numeric_limits<double>::max(); 

    for(POIIdx POIIndex : foundPOIs){//Loops through all the indexes of the found POIs
        LatLon POIPosition = POIPositions[POIIndex]; //Get the LatLon of index  
        double distance = findDistanceBetweenTwoPoints(std::make_pair(my_position, POIPosition)); 
        
        if(distance < minDistance){
            minDistance = distance; 
            answer = POIIndex; 
        }
    }
    return answer;
}

double findFeatureArea(FeatureIdx feature_id){
    return 0;
}

double findWayLength(OSMID way_id){
    auto way_it = OSMWayFromID.find(way_id);
    if (way_it == OSMWayFromID.end()) {
        return 0.0;
    }
    //Gets the way from the id 
    const OSMWay* way = way_it->second;

    const std::vector<OSMID>& members = getWayMembers(way);
    if (members.size() < 2) {
        return 0;
    }

    double totalLength = 0;

    //Iterate through nodes and sum the distances between two nodes 
    for (size_t i = 0; i < members.size() - 1; ++i) {
        OSMID id1 = members[i];
        OSMID id2 = members[i+1];

        //Gets the iterator for both ids 
        auto node1it = LatLonFromOSMID.find(id1);
        auto node2it = LatLonFromOSMID.find(id2);

        if(node1it != LatLonFromOSMID.end() && node2it != LatLonFromOSMID.end()) {
            //Get the LatLon of both ids 
            LatLon node1Position = node1it->second;
            LatLon node2Position = node2it->second;
            
            // Add the distance between these two points
            totalLength += findDistanceBetweenTwoPoints({node1Position, node2Position});
        }
    }

    return totalLength;

}

std::string getOSMNodeTagValue(OSMID osm_id, std::string key){
    return "";
}

