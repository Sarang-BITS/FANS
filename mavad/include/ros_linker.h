#pragma once

#include <sstream>
#include <vector>
#include <string>

#include "ns3/core-module.h"


#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/internet-stack-helper.h"

#include "ns3/netanim-module.h"
/*Linker namespace is rnl

  Classes: 
    Interfaces: creates a middleware for communication between drones.
            The packet transmission happens through sockets using WLAN 802.11 protocol
            Each uav consists of a ns3 node and planner node which is responsible for data transmission 
            between drones. A flow diagram of data transfer between drones
            n-1 , n and n + 1 is shown:

                      To px4 drone n
                          ^
                          |
                          | 
         ----------------------------------------
        | Planner Control Interface (PCI node n) |
         ----------------------------------------
                       ^     |
        Drone Control  |     |  Feedback
                       |     v
                 ------------------
                | Planner Node (n) |     
                 ------------------
                       ^     |
        Data from n-1  |     | Data of n
            drone      |     |    drone
                       |     v
 ----------        ---------------------         -----------
|planner   |----> | ros_ns3_linker node | ----> | planner   |
|node (n-1)|       ---------------------        | node (n+1)|
 ----------                                      -----------


*/
namespace rnl {

    void updateReceiver (ns3::Ptr<ns3::Socket>, int ID);
    
    class Interface {
        public:
            
            Interface(
                    std::string _phyMode, 
                    double _rss,  // -dBm
                    uint32_t _packetSize, // bytes
                    uint32_t _numPackets,
                    double _interval, // seconds
                    bool _verbose,
                    double _stopTime,
                    int _num_nodes);
            
            /*********************************************************/
            /*NS3 Functions*/

            /*For Realtime simulation set realtime and checksum*/
            void initialize(bool realtime, bool checksum);
            
            /*Default wifi options are set in this function, can be changed*/
            void setWifi();
            
            /*Setting the mobility of the ns3 nodes*/
            void setMobility();
            
            /*Setting the internet options and assigning IPs*/
            void setInternet();
            
            /*Sets the properties of reciever sockets for the nodes*/
            void setReceivers();
            
            /*Sets the properties of sender sockets for the nodes*/
            void setSenders();

            /*Callback functions on packet reception by ns3 node n as receivePacketn*/
            void receivePacket1(ns3::Ptr<ns3::Socket> socket1);
            void receivePacket2(ns3::Ptr<ns3::Socket> socket2);
            void receivePacket3(ns3::Ptr<ns3::Socket> socket3);

            /*Functions for Sending packets by ns3 node n as sendPacketn*/
            void sendPacket1(ns3::Ptr<ns3::Socket> socket, uint32_t pktSize,
                            ns3::Time pktInterval);
            void sendPacket2(ns3::Ptr<ns3::Socket> socket, uint32_t pktSize,
                            ns3::Time pktInterval);
            
            /*Simulation Setup. Start, Stop time*/
            void startSimul();

            /*********************************************************/
            
            
        private:

            std::ostringstream msg1;
            std::ostringstream msg2;
            std::ostringstream msg3;

            std::string phy_mode; 
            double rss;  // -dBm
            uint32_t packet_size; // bytes
            uint32_t num_packets;
            double interval; // seconds
            bool verbose;
            double stop_time;
            int num_nodes;

            ns3::Time inter_packet_interval; 
            ns3::Time broadcast_interval; 
  
            
            ns3::NodeContainer c;

            // The below set of helpers will help us to put together the wifi NICs we want
            ns3::WifiHelper wifi;

            ns3::YansWifiPhyHelper wifiPhy;

            ns3::YansWifiChannelHelper wifiChannel;

            // Add a mac and disable rate control
            ns3::WifiMacHelper wifiMac;
            
            ns3::NetDeviceContainer devices;

            // Note that with FixedRssLossModel, the positions below are not
            // used for received signal strength.
            ns3::MobilityHelper mobility;
  
            ns3::InternetStackHelper internet;

            ns3::Ipv4AddressHelper ipv4;
            ns3::Ipv4InterfaceContainer i;

            ns3::TypeId tid;
            
            ns3::Ptr<ns3::Socket> recv_sink1, recv_sink2, recv_sink3 = nullptr;
            
            ns3::Ptr<ns3::Socket> source1, source2 = nullptr;
    };
};
