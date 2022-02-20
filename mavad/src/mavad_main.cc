/**
 * @file mavad_main.cc
 * @author Ojit Mehta (f20170372@goa.bits-pilani.ac.in)
 * @brief Exucatable file for simulation,  contains the main function
 * @version 0.1
 * @date 2021-03-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "planner_ns3.h"
#include "planner_ns3_utils.h"
#include "planner_config.h"

using namespace rnl;
using namespace ns3;

/**
 * Initializing static variables to a fixed value
 */
// ns3::Vector3D rnl::Planner::fire_centre = ns3::Vector3D (400,400,20); 
// ns3::Vector3D rnl::Planner::disas_centre = ns3::Vector3D (400,400,20); 
// ns3::Vector3D rnl::Planner::disas_centre = ns3::Vector3D (250,250,20); 
ns3::Vector3D rnl::Planner::disas_centre = ns3::Vector3D (10,10,3);
// float rnl::Planner::fire_rad = 70; 
// float rnl::Planner::disas_rad = 70; 
float rnl::Planner::disas_rad = 2; // not used
NS_LOG_COMPONENT_DEFINE ("Mavad_main");

int main(int argc, char **argv){

    ros::init(argc, argv, "ros_ns3_planner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    /**
     * Create an object of properties, give in phyMode, rssi value and number of nodes 
     */
    // Properties prop ("DsssRate11Mbps",-80, 20);
    // Properties prop ("DsssRate11Mbps",-80, 15);
    // Properties prop ("DsssRate11Mbps",-80, 13);
    Properties prop ("DsssRate11Mbps",-80, 8);
    prop.initialize(true, true); /**< Initializing wihtout realtime simulation and with checksum disabled*/
    // prop.setWifi (false, false); /**<Set wifi without debug and disable pcap */
    prop.setWifi (false, true); /**<Set wifi without debug and disable pcap */
    prop.setInternet (); /**< Set IP*/

    /**
     * Create and start a Planner 
     */
    // rnl::Planner plan (prop, 20, 0.2, 0.1, 2500.0);
    // rnl::Planner plan (nh, nh_private, prop, 15, 0.2, 0.1, 2500.0);
    rnl::Planner plan (nh, nh_private, prop, 8, 0.2, 0.1, 2500.0);
    // rnl::Planner plan (prop, 13, 0.2, 0.1, 2500.0);
    plan.initializeSockets ();
    plan.startSimul();
    return 0;
}