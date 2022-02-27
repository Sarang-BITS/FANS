/**
 * @brief Exucatable file for simulation,  contains the main function
 */

#include "planner_ns3.h"
#include "planner_ns3_utils.h"
#include "planner_config.h"

using namespace rnl;
using namespace ns3;

/**
 * Initializing static variables to a fixed value
 */
ns3::Vector3D rnl::Planner::disas_centre = ns3::Vector3D (10,10,3);

NS_LOG_COMPONENT_DEFINE ("Mavad_main");

int main(int argc, char **argv){

    ros::init(argc, argv, "ros_ns3_planner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    /**
     * Create an object of properties, give in phyMode, rssi value and number of nodes 
     */
    Properties prop ("DsssRate11Mbps",-80, 8);
    prop.initialize(true, true); /**< Initializing wihtout realtime simulation and with checksum disabled*/
    prop.setWifi (false, true); /**<Set wifi without debug and disable pcap */
    prop.setInternet (); /**< Set IP*/

    /**
     * Create and start a Planner 
     */
    rnl::Planner plan (nh, nh_private, prop, 8, 0.2, 0.1, 2500.0);
    plan.initializeSockets ();
    plan.startSimul();
    return 0;
}
