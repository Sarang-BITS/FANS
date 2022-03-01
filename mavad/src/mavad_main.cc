/**
 * @brief Executable file for simulation, contains the main function 
 */
#include "planner_ns3.h"

using namespace rnl;
using namespace ns3;

/**
 * Initializing static variables to a fixed value
 */
ns3::Vector3D rnl::Planner::disas_centre = ns3::Vector3D (250,250,20);

NS_LOG_COMPONENT_DEFINE ("Mavad_main");

int main(int argc, char **argv)
{
    /**
     * Create an object of properties, give phyMode, rss value and number of nodes 
     */
    Properties prop ("DsssRate11Mbps",-80, 15); // rss = -80dBm [Deprecated]
    prop.initialize(false, true); /**< Initializing without realtime simulation and with checksum disabled*/
    prop.setWifi (false, true); /**<Set wifi without debug and enable pcap and ascii tracing */
    prop.setInternet (); /**< Set IP*/

    /**
     * Create and start a Planner 
     */
    rnl::Planner plan (prop, 15, 0.2, 0.1, 2500.0);
    plan.initializeSockets ();
    plan.initializeMobility();
    plan.setLeaderExplorePath ();
    plan.startSimul();
    return 0;
}
