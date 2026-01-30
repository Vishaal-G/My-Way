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
#include <tuple> 


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
std::vector<std::vector<StreetSegmentIdx>> intersection_to_segments; //Mimicks Hashmap used to map all ID's of street segments connected to an intersection

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

//Used for getOSMNodeTagValue
std::unordered_map<OSMID, const OSMNode*> OSMNodeFromID;

//Used for computing street length
static std::vector<double> g_street_lengths;

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

    // For findStreetSegmentsOfIntersection, we resize array on load when another intersection is added
    intersection_to_segments.clear();
    intersection_to_segments.resize(getNumIntersections());

    //For findStreetLength, we resize array on load
    g_street_lengths.clear();
    g_street_lengths.resize(getNumStreets(), 0.0);

    
    for (StreetSegmentIdx seg = 0; seg < getNumStreetSegments(); seg++) {
        StreetSegmentInfo info = getStreetSegmentInfo(seg);

        //Mapping each street ID of a segment to a length
        g_street_lengths[info.streetID] += findStreetSegmentLength(seg);


        // street -> intersections
        street_to_intersections[info.streetID].push_back(info.from);
        street_to_intersections[info.streetID].push_back(info.to);

        // intersection -> segments
        //Take from and to intersections and add the street segment to it
        intersection_to_segments[info.from].push_back(seg);
        intersection_to_segments[info.to].push_back(seg);
    }

    for (size_t i = 0; i < street_to_intersections.size(); i++) { 

        // Removing duplicate intersections in the vector
        std::vector<IntersectionIdx> &intersections= street_to_intersections[i];
        std::sort(intersections.begin(), intersections.end()); // Sort all streets so duplicates are next to eachother
        intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end()); // Deletes streets that are not duplicates
    }

    //For findWayLength & getOSMNodeTagValue 
    std::string osm_filename = map_streets_database_filename; 
    std::string replace = ".streets.bin";
    std::string replaceWith = ".osm.bin";

    // Loads the OSM database 
    size_t find = osm_filename.find(replace);
    if (find != std::string::npos) {
        osm_filename.replace(find, replace.length(), replaceWith); // Gets the OSM database filename
    }

    bool osm_load_successful = loadOSMDatabaseBIN(osm_filename);
    if (!osm_load_successful) {
        return false; // Indicates whether the map has loaded successfully
    }

    // Populate the LatLonFromOSMID and OSMNodeFromID hashmap 
    int numNodes = getNumberOfNodes(); 
    LatLonFromOSMID.clear(); 
    LatLonFromOSMID.reserve(numNodes); 
    OSMNodeFromID.clear(); 
    OSMNodeFromID.reserve(numNodes); 

    for(int i = 0; i < numNodes; ++i){
        const OSMNode* node = getNodeByIndex(i); 
        LatLonFromOSMID[node->id()] = getNodeCoords(node); 

        OSMNodeFromID[node->id()] = node; 
    }

    // Populate the OSMWayFromID hashmap 
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
    OSMWayFromID.clear(); 
    LatLonFromOSMID.clear(); 
    OSMNodeFromID.clear(); 

    closeOSMDatabase(); 
    //Clean-up your map related data structures here
    //Cleaning streetLength data here
    g_street_lengths.clear();
    g_street_lengths.shrink_to_fit();
    
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
    //Get information about curve points, intersection ID's
    StreetSegmentInfo src = getStreetSegmentInfo(src_street_segment_id);
    StreetSegmentInfo dst = getStreetSegmentInfo(dst_street_segment_id);

    //Check if the two segments share an intersection, if not, impossible to turn
    bool shares_src_from = (src.from == dst.from) || (src.from == dst.to);
    bool shares_src_to   = (src.to   == dst.from) || (src.to   == dst.to);

    //If not meet, then return no angle
    if (!shares_src_from && !shares_src_to){
        return NO_ANGLE;
    }

    //Find out which intersection is shared 
    IntersectionIdx shared;

    if (shares_src_from){
        shared = src.from;
    }
    else{
        shared = src.to;
    }

    //If intersection is to part of line segment, use the last curved point to determine angle
    //If intersection to from part of line segment, use the first curved point to determine angle
    //We get the closest points right beside the intersection on each line segment
    LatLon A;
    if (shared == src.from) {
        if (src.numCurvePoints > 0){
            A = getStreetSegmentCurvePoint(src_street_segment_id, 0);
        }
        else {
            A = getIntersectionPosition(src.to);
        }
    } else {
        if (src.numCurvePoints > 0){
            A = getStreetSegmentCurvePoint(src_street_segment_id, src.numCurvePoints - 1);
        }
        else {
            A = getIntersectionPosition(src.from);
        }
    }
    
    //Same logic for destination line segment
    LatLon B;
    if (shared == dst.from) {
        if (dst.numCurvePoints > 0) {
            B = getStreetSegmentCurvePoint(dst_street_segment_id, 0);
        }
        else {
            B = getIntersectionPosition(dst.to);
        }
    } else {
        if (dst.numCurvePoints > 0) {
            B = getStreetSegmentCurvePoint(dst_street_segment_id, dst.numCurvePoints - 1);
        }
        else {
            B = getIntersectionPosition(dst.from);
        }
    }

    LatLon S = getIntersectionPosition(shared);

    //We create two vectors A -> Intersection and Intersection --> B
    //Store all latitudes and longitudes of points
    double latA = A.latitude() * kDegreeToRadian;
    double lonA = A.longitude() * kDegreeToRadian;

    double latS = S.latitude() * kDegreeToRadian;
    double lonS = S.longitude() * kDegreeToRadian;

    double latB = B.latitude() * kDegreeToRadian;
    double lonB = B.longitude() * kDegreeToRadian;

    //Calculate average latitude and project to x y coordinates to do vector math

    //Conversion for Starting line segment -> Intersection
    double latAvg_in = (latA + latS) / 2.0;
    double Ax   = kEarthRadiusInMeters * lonA * cos(latAvg_in);
    double Ay   = kEarthRadiusInMeters * latA;
    double Sx_in = kEarthRadiusInMeters * lonS * cos(latAvg_in);
    double Sy_in = kEarthRadiusInMeters * latS;

    //Conversion for Ending line segment -> Intersection
    double latAvg_out = (latS + latB) / 2.0;
    double Sx_out= kEarthRadiusInMeters * lonS * cos(latAvg_out);
    double Sy_out= kEarthRadiusInMeters * latS;
    double Bx   = kEarthRadiusInMeters * lonB * cos(latAvg_out);
    double By   = kEarthRadiusInMeters * latB;

    //Build vector Starting Line Segment -> Intersection
    double inx = Sx_in - Ax;
    double iny = Sy_in - Ay;

    //Build vector Intersection -> Destination Line Segment
    double outx = Bx - Sx_out;
    double outy = By - Sy_out;

    //Compute angle
    //Dot tells me the angle mangitude and cross gives me the direction
    //Using atan2 is faster way of combining the cross and dot together at the end using if statements 
    double dot   = inx * outx + iny * outy;
    double cross = inx * outy - iny * outx;
    return atan2(cross, dot);
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
    //Set minimum to biggest possible number initially 
    //Set maximum to smallest possible number intially 
    double minLat = std::numeric_limits<double>::max();
    double maxLat = std::numeric_limits<double>::lowest();
    double minLon = std::numeric_limits<double>::max();
    double maxLon = std::numeric_limits<double>::lowest();

    //Loop through every street segment and only process the ones that belong to street_id
    for (StreetSegmentIdx seg = 0; seg < getNumStreetSegments(); seg++){
        //Get info about current street segment
        StreetSegmentInfo info = getStreetSegmentInfo(seg);

        //Check if this street segment belongs on the street we want
        if (info.streetID != street_id){
            continue;
        }

        //Get endpoints (intersections)
        LatLon fromPos = getIntersectionPosition(info.from);
        LatLon toPos   = getIntersectionPosition(info.to);

        //Update min/max values checking the starting point of segment
        minLat = std::min(minLat, fromPos.latitude());
        maxLat = std::max(maxLat, fromPos.latitude());
        minLon = std::min(minLon, fromPos.longitude());
        maxLon = std::max(maxLon, fromPos.longitude());

        //Update min/max values checking the destination point of segment
        minLat = std::min(minLat, toPos.latitude());
        maxLat = std::max(maxLat, toPos.latitude());
        minLon = std::min(minLon, toPos.longitude());
        maxLon = std::max(maxLon, toPos.longitude());

        //Check if curve points have new max/min long/lat
        for (int i = 0; i < info.numCurvePoints; i++){
            LatLon cp = getStreetSegmentCurvePoint(seg, i);

            minLat = std::min(minLat, cp.latitude());
            maxLat = std::max(maxLat, cp.latitude());
            minLon = std::min(minLon, cp.longitude());
            maxLon = std::max(maxLon, cp.longitude());
        }
    }

    //Create a new LatLonBounds object to update max/min values
    LatLonBounds bounds;
    bounds.min = LatLon(minLat, minLon);
    bounds.max = LatLon(maxLat, maxLon);
    return bounds;
}


std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id){
    //Return street segments corresponding to correct intersection in hashmap
    return intersection_to_segments[intersection_id];
}

double findStreetLength(StreetIdx street_id){
    return g_street_lengths[street_id];
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
    // Gets the way from the id 
    const OSMWay* way = way_it->second;

    const std::vector<OSMID>& members = getWayMembers(way);
    if (members.size() < 2) {
        return 0;
    }

    double totalLength = 0;

    // Iterate through nodes and sum the distances between two nodes 
    for (size_t i = 0; i < members.size() - 1; ++i) {
        OSMID id1 = members[i];
        OSMID id2 = members[i+1];

        // Gets the iterator for both ids 
        auto node1it = LatLonFromOSMID.find(id1);
        auto node2it = LatLonFromOSMID.find(id2);

        if(node1it != LatLonFromOSMID.end() && node2it != LatLonFromOSMID.end()) {
            // Get the LatLon of both ids 
            LatLon node1Position = node1it->second;
            LatLon node2Position = node2it->second;
            
            // Add the distance between these two points
            totalLength += findDistanceBetweenTwoPoints({node1Position, node2Position});
        }
    }
    return totalLength;
}

std::string getOSMNodeTagValue(OSMID osm_id, std::string key){
    auto it = OSMNodeFromID.find(osm_id); 

    if(it == OSMNodeFromID.end())
        return "";
    
    //Get the OSMNode of the ID 
    const OSMNode* node = it->second; 

    //Looks for the tag 
    int numberOfTag = getTagCount(node); 
    for(int i = 0; i < numberOfTag; ++i){
        std::string currentKey, value; 
        std::tie(currentKey, value) = getTagPair(node, i);

        if(currentKey == key){
            return value; 
        }
    }
    return ""; 
}

