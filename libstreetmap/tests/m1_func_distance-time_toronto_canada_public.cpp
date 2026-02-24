
#include "m1.h"
#include "unit_test_util.h"

#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"

#include <UnitTest++/UnitTest++.h>

#include <random>
#include <algorithm>
#include <set>

using ece297test::relative_error;
using ece297test::sorted;

SUITE(distance_time_queries_public_toronto_canada) {

    struct BaseMapFixture {
        BaseMapFixture() {
            //Load the map
            try {
                loadMap("/cad2/ece297s/public/maps/toronto_canada.streets.bin");
            } catch (...) {
                std::cout << "!!!! BaseMapFixture test setup: loadMap threw an exception !!!!" << std::endl;
                throw; // re-throw exception
            }
        }
    
        ~BaseMapFixture() {
            //Clean-up
            try {
                closeMap();
            } catch (const std::exception& e) {
                std::cout << "!!!! BaseMapFixture test teardown: closeMap threw an exception. what(): " << e.what() << " !!!!" << std::endl;
                std::terminate(); // we're in a destructor
            } catch (...) {
                std::cout << "!!!! BaseMapFixture test teardown: closeMap threw an exception !!!!" << std::endl;
                std::terminate(); // we're in a destructor
            }
        }
    };


    struct MapFixture : BaseMapFixture {};

    TEST_FIXTURE(MapFixture, distance_between_two_points) {
        double expected;

        expected = 2356.64505946482040599;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.79279708862304688, -79.40152740478515625), LatLon(43.78813934326171875, -79.37289428710937500))), 0.001000000);

        expected = 2427.80270119967008213;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.71529388427734375, -79.55553436279296875), LatLon(43.73324203491210938, -79.53834533691406250))), 0.001000000);

        expected = 2949.60709968117043900;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.59196853637695312, -79.37844085693359375), LatLon(43.59408187866210938, -79.41493988037109375))), 0.001000000);

        expected = 3548.76272402025188057;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.82280731201171875, -79.18561553955078125), LatLon(43.83702087402343750, -79.14601898193359375))), 0.001000000);

        expected = 7082.94972707004035328;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.64250946044921875, -79.35969543457031250), LatLon(43.60358810424804688, -79.29006958007812500))), 0.001000000);

        expected = 9273.46528542371379444;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.54529571533203125, -79.10992431640625000), LatLon(43.56031417846679688, -79.22308349609375000))), 0.001000000);

        expected = 9502.16744354860202293;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.71036148071289062, -79.60935211181640625), LatLon(43.65697860717773438, -79.70158386230468750))), 0.001000000);

        expected = 10259.27792697115364717;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.59600830078125000, -79.11286163330078125), LatLon(43.52816772460937500, -79.19910430908203125))), 0.001000000);

        expected = 10811.38925209360422741;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.87495803833007812, -79.56263732910156250), LatLon(43.85248565673828125, -79.43147277832031250))), 0.001000000);

        expected = 14256.83954341826029122;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.61819839477539062, -79.22166442871093750), LatLon(43.72824859619140625, -79.31252288818359375))), 0.001000000);

        expected = 14466.80948114287093631;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.77970123291015625, -79.60786437988281250), LatLon(43.71545028686523438, -79.45131683349609375))), 0.001000000);

        expected = 16151.57112973360381147;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.73253250122070312, -79.09116363525390625), LatLon(43.58732604980468750, -79.09315490722656250))), 0.001000000);

        expected = 17244.38714028189860983;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.56804275512695312, -79.77233886718750000), LatLon(43.69831848144531250, -79.65620422363281250))), 0.001000000);

        expected = 23088.23867834949487587;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.84590911865234375, -79.17972564697265625), LatLon(43.66650009155273438, -79.32427978515625000))), 0.001000000);

        expected = 26697.57579962744057411;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.77907943725585938, -79.08943939208984375), LatLon(43.88920593261718750, -79.38510131835937500))), 0.001000000);

        expected = 28346.94087889042202733;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.57031250000000000, -79.68901824951171875), LatLon(43.77373886108398438, -79.47676086425781250))), 0.001000000);

        expected = 29222.10386375499729184;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.65841293334960938, -79.78363800048828125), LatLon(43.66227722167968750, -79.42051696777343750))), 0.001000000);

        expected = 30851.74677904916461557;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.77691650390625000, -79.58846282958984375), LatLon(43.56536865234375000, -79.34043121337890625))), 0.001000000);

        expected = 31806.78265816414932488;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.71936416625976562, -79.52052307128906250), LatLon(43.89463806152343750, -79.20742797851562500))), 0.001000000);

        expected = 34409.99452569775894517;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.74295806884765625, -79.03385162353515625), LatLon(43.64626312255859375, -79.44029235839843750))), 0.001000000);

        expected = 36763.18711706031172071;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.89926528930664062, -79.00794982910156250), LatLon(43.57030868530273438, -79.05247497558593750))), 0.001000000);

        expected = 38140.77731261219742009;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.87306594848632812, -79.04129028320312500), LatLon(43.88062286376953125, -79.51689147949218750))), 0.001000000);

        expected = 38384.63474691180454101;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.89250946044921875, -79.44643402099609375), LatLon(43.57640457153320312, -79.25479125976562500))), 0.001000000);

        expected = 40275.83257943762146169;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.83801651000976562, -79.22557830810546875), LatLon(43.91736984252929688, -79.71572113037109375))), 0.001000000);

        expected = 41625.99285252964182291;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.56082916259765625, -79.01220703125000000), LatLon(43.72190475463867188, -79.47900390625000000))), 0.001000000);

        expected = 44870.51818089283915469;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.87591171264648438, -79.01165008544921875), LatLon(43.90194320678710938, -79.57025146484375000))), 0.001000000);

        expected = 45647.26265787136071594;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.64857482910156250, -79.20729827880859375), LatLon(43.71326065063476562, -79.76768493652343750))), 0.001000000);

        expected = 50284.51768783960142173;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.90982818603515625, -79.60358428955078125), LatLon(43.48199844360351562, -79.40149688720703125))), 0.001000000);

        expected = 51319.73420976037596120;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.65852737426757812, -79.65677642822265625), LatLon(43.54740142822265625, -79.03836059570312500))), 0.001000000);

        expected = 60998.75850568081659731;
        ECE297_CHECK_RELATIVE_ERROR(expected, findDistanceBetweenTwoPoints(std::make_pair(LatLon(43.74923324584960938, -79.02179718017578125), LatLon(43.81575775146484375, -79.77580261230468750))), 0.001000000);

    } //distance_between_two_points

    TEST_FIXTURE(MapFixture, street_segment_length) {
        double expected;

        expected = 4.61592946477725263;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(158504), 0.001000000);

        expected = 10.76485543251505383;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(248257), 0.001000000);

        expected = 12.62964549966396177;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(74825), 0.001000000);

        expected = 16.82786356359765989;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(246389), 0.001000000);

        expected = 17.18461870426369131;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(216073), 0.001000000);

        expected = 17.57102788288468531;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(187962), 0.001000000);

        expected = 17.79751386779991407;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(251756), 0.001000000);

        expected = 18.55303183339434980;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(159866), 0.001000000);

        expected = 19.93899379937786520;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(267111), 0.001000000);

        expected = 20.26265297766093454;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(189727), 0.001000000);

        expected = 32.04655836486701048;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(207976), 0.001000000);

        expected = 32.88165816082470627;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(137633), 0.001000000);

        expected = 36.06008049228044143;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(274424), 0.001000000);

        expected = 40.80067210629675145;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(119406), 0.001000000);

        expected = 41.44911067512509106;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(219683), 0.001000000);

        expected = 43.17472156082976653;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(175447), 0.001000000);

        expected = 43.40716439792700498;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(77474), 0.001000000);

        expected = 52.91784990677152933;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(54042), 0.001000000);

        expected = 54.58360349419594115;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(196254), 0.001000000);

        expected = 54.96054393067900889;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(18), 0.001000000);

        expected = 57.17009719947726865;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(200691), 0.001000000);

        expected = 59.67044106303386286;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(64212), 0.001000000);

        expected = 71.66905217904457004;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(151779), 0.001000000);

        expected = 90.26606037639763258;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(223956), 0.001000000);

        expected = 93.23240188840350129;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(194552), 0.001000000);

        expected = 103.55948702294864461;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(71052), 0.001000000);

        expected = 107.34295340501533644;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(149053), 0.001000000);

        expected = 132.37495525691997500;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(49755), 0.001000000);

        expected = 202.56914256566443555;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(250632), 0.001000000);

        expected = 222.04124301032774724;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentLength(245687), 0.001000000);

    } //street_segment_length

    TEST_FIXTURE(MapFixture, street_length) {
        double expected;

        expected = 117.30524065955242463;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(15418), 0.001000000);

        expected = 128.25795627606464677;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(22301), 0.001000000);

        expected = 134.71432006267588122;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(15949), 0.001000000);

        expected = 150.79824676211828205;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(6081), 0.001000000);

        expected = 172.54094952511536576;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(5219), 0.001000000);

        expected = 193.19898090504727861;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(17559), 0.001000000);

        expected = 225.07554710505962703;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(19966), 0.001000000);

        expected = 246.42941147802443425;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(20367), 0.001000000);

        expected = 255.31563764844435127;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(21707), 0.001000000);

        expected = 274.79473535755045077;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(15275), 0.001000000);

        expected = 276.74891011285103559;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(17852), 0.001000000);

        expected = 281.27443206473941473;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(12113), 0.001000000);

        expected = 281.99794405739947933;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(18200), 0.001000000);

        expected = 289.46171271959246951;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(15810), 0.001000000);

        expected = 296.97209624137900619;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(12881), 0.001000000);

        expected = 330.42944760073049792;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(11185), 0.001000000);

        expected = 366.18288078397040408;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(9704), 0.001000000);

        expected = 367.92775723120917064;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(4392), 0.001000000);

        expected = 368.25507001704477261;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(20023), 0.001000000);

        expected = 400.10183360245162021;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(20459), 0.001000000);

        expected = 404.68251922060517245;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(4044), 0.001000000);

        expected = 409.67423847780366941;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(12992), 0.001000000);

        expected = 412.67333212279771715;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(5774), 0.001000000);

        expected = 485.33504000828702374;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(12334), 0.001000000);

        expected = 486.93725414958362308;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(14258), 0.001000000);

        expected = 567.82752663257008408;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(20174), 0.001000000);

        expected = 865.83765722719999758;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(16309), 0.001000000);

        expected = 1135.35493113092184103;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(16901), 0.001000000);

        expected = 2294.67084119445416945;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(6296), 0.001000000);

        expected = 43871.46413885730726179;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetLength(2), 0.001000000);

    } //street_length

    TEST_FIXTURE(MapFixture, street_segment_travel_time) {
        double expected;

        expected = 0.33234691132153937;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(158504), 0.001000000);

        expected = 0.64589127667330948;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(248257), 0.001000000);

        expected = 0.75777867216591310;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(74825), 0.001000000);

        expected = 0.88378038087254807;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(216073), 0.001000000);

        expected = 1.21160613960374652;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(246389), 0.001000000);

        expected = 1.26511396895948303;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(187962), 0.001000000);

        expected = 1.28142095937572953;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(251756), 0.001000000);

        expected = 1.33581825123845532;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(159866), 0.001000000);

        expected = 1.43560750974394202;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(267111), 0.001000000);

        expected = 1.45891096986915780;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(189727), 0.001000000);

        expected = 2.30735213185562582;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(207976), 0.001000000);

        expected = 2.44804013960716471;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(119406), 0.001000000);

        expected = 2.59632571621061903;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(274424), 0.001000000);

        expected = 2.95934909336146257;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(137633), 0.001000000);

        expected = 2.98433587753430318;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(219683), 0.001000000);

        expected = 3.10857985751341470;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(175447), 0.001000000);

        expected = 3.12531574127367717;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(77474), 0.001000000);

        expected = 3.93001933164743589;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(196254), 0.001000000);

        expected = 4.11624687274447787;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(200691), 0.001000000);

        expected = 4.76260626451067370;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(54042), 0.001000000);

        expected = 4.79480918771576459;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(194552), 0.001000000);

        expected = 4.94644871789605567;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(18), 0.001000000);

        expected = 5.16017159941526948;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(151779), 0.001000000);

        expected = 5.37033943959530813;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(64212), 0.001000000);

        expected = 6.49915614876212366;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(223956), 0.001000000);

        expected = 7.72869240930012946;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(149053), 0.001000000);

        expected = 8.10276508443466170;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(250632), 0.001000000);

        expected = 9.32035338763629539;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(71052), 0.001000000);

        expected = 15.88499341890157801;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(49755), 0.001000000);

        expected = 15.98696900886002226;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(245687), 0.001000000);

    } //street_segment_travel_time

    TEST_FIXTURE(MapFixture, feature_area) {
        double expected;

        expected = 0.00000000000000000;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(45922), 0.001000000);

        expected = 0.00000000000000000;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(66432), 0.001000000);

        expected = 0.00000000000000000;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(69042), 0.001000000);

        expected = 0.00000000000000000;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(653164), 0.001000000);

        expected = 104.10145252250318038;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(71009), 0.001000000);

        expected = 232.33227628435909651;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(58609), 0.001000000);

        expected = 247.98491802170619280;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(435), 0.001000000);

        expected = 334.97735453603900169;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(6298), 0.001000000);

        expected = 624.48246597222328091;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(2099), 0.001000000);

        expected = 714.05399252652807718;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(680811), 0.001000000);

        expected = 716.98252221352402103;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(23385), 0.001000000);

        expected = 786.34065602448242771;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(38373), 0.001000000);

        expected = 786.43985179525043350;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(2231), 0.001000000);

        expected = 796.67505177905525215;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(391), 0.001000000);

        expected = 884.31451525271654646;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(9906), 0.001000000);

        expected = 1132.80217547627057684;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(47924), 0.001000000);

        expected = 1336.48490960331901078;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(595418), 0.001000000);

        expected = 1342.05790836102050889;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(655801), 0.001000000);

        expected = 1391.63550370107554954;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(24574), 0.001000000);

        expected = 2297.56342473457061715;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(74563), 0.001000000);

        expected = 2919.41720278143839096;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(65820), 0.001000000);

        expected = 3589.34936736015924907;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(37183), 0.001000000);

        expected = 3989.35180408405722119;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(11831), 0.001000000);

        expected = 14246.39965640041373263;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(4119), 0.001000000);

        expected = 19538.17389507560801576;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(58001), 0.001000000);

        expected = 22504.81730254002832226;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(29621), 0.001000000);

        expected = 40892.17597194760310231;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(11589), 0.001000000);

        expected = 46051.56894563402602216;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(69887), 0.001000000);

        expected = 46625.31953820634953445;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(633), 0.001000000);

        expected = 76439.27350512814882677;
        ECE297_CHECK_RELATIVE_ERROR(expected, findFeatureArea(5431), 0.001000000);

    } //feature_area

} //distance_time_queries_public_toronto_canada

