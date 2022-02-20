/**
 * @file planner_ns3.h
 * @author Ojit Mehta (f20170372@goa.bits-pilani.ac.in)
 * @brief Planner class for the project 
 * @version 0.1
 * @date 2021-04-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include "planner_config.h"
#include "planner_ns3_utils.h"
#include "ns3/core-module.h"
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

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

#include "ns3/ipv4-static-routing-helper.h"

void TraceSink (std::size_t index, ns3::Ptr<const ns3::Packet> p, const ns3::Address& a);

/**
 * @namespace 
 */
namespace rnl{

 //    ns3::Vector3D pos0(disas_centre.x + RC, disas_centre.y, disas_centre.z);
	// ns3::Vector3D pos1(disas_centre.x + RC, disas_centre.y - RC, disas_centre.z);
	// ns3::Vector3D pos2(disas_centre.x + RC, disas_centre.y + RC, disas_centre.z);
	// ns3::Vector3D pos3(disas_centre.x, disas_centre.y, disas_centre.z);
	// ns3::Vector3D pos4(disas_centre.x, disas_centre.y - RC, disas_centre.z);
	// ns3::Vector3D pos5(disas_centre.x, disas_centre.y + RC, disas_centre.z);
	// ns3::Vector3D pos6(disas_centre.x - RC, disas_centre.y, disas_centre.z);
	// ns3::Vector3D pos7(disas_centre.x - RC, disas_centre.y - RC, disas_centre.z);
	// ns3::Vector3D pos8(disas_centre.x + RC, disas_centre.y + RC, disas_centre.z);
    /**
     * @brief Drone Socket common for planning and communication
     */
    struct DroneSoc
    {   
        /**
         * @brief Construct a new Drone Soc object
         * 
         */
        DroneSoc ();

        /**
         * @brief send Packet after every pktInterval. \n
         *  This registers a callback which is periodically called
         * 
         * @param pktInterval Packet interval at which this needs to be called. \n
         * The packet will be called after n * pktInterval 
         * 
         * @param n number of nodes in the swarm 
         */
        void sendPacket (ns3::Time pktInterval, int n);

        /**
         * @brief Send a broadcast packet
         * 
         * @param pktInterval Packet interval at which this needs to be called. \n
         * The packet will be called after n * pktInterval 
         * 
         * @param n number of nodes in the swarm 
         */
        void sendBcPacket (ns3::Time pktInterval, int n);
        
        /**
         * @brief Socket receiving callback. \n
         * This function will be called as an interrupt if something is received at the socket end 
         * 
         * @param soc Socket at which the message will be received 
         */
        void receivePacket (ns3::Ptr<ns3::Socket> soc);

        /**
         * @brief Terminate all sockets and send shut down command for this nodea
         */
        void closeSender ();
        
        /**
         * @brief update send message with the correct parent location and parent id to follow
         */
        void updateSendMsg ();
        
        /**
         * @brief Setup the node for broadcasting 
         * 
         * @param node Node to setup broadcasting for
         * @param tid Type id
         */
        void setBcSender (ns3::Ptr<ns3::Node> node, ns3::TypeId tid);
        
        /**
         * @brief Initialize the sender
         * 
         * @param node node 
         * @param tid type id
         * @param ip IP of the sending socket
         */
        void setSender (ns3::Ptr<ns3::Node> node, ns3::TypeId tid, const std::string& ip);

        void setSenderTCP (ns3::Ptr<ns3::Node> node, const std::string& self_ip, const std::string& remote_ip, ns3::Time startTime);

        void setRecv   (ns3::Ptr<ns3::Node> node, ns3::TypeId tid);

        void setRecvTCP (ns3::Ptr<ns3::Node> node, const std::string& ip, int num_nodes, ns3::Time stopTime);
        
        ns3::Ptr<ns3::Socket>         source; /**< Socket for sending unicast messages */
        ns3::Ptr<ns3::Socket>         source_bc; /**< Socket for sending broadcast messaged */

        ns3::Ptr<ns3::Socket>         recv_sink; /**< Receiving sink socket */
        int                           id; /**< Id of this drone soc */
        int                           anch_id; /**< Anchoring Id if any */
        int                           circle_dir; /**Circling direction */
        ns3::Vector3D                 anch_pos; /**< Anchoring position */
        rnl::USMsg                    msg_send; /**< message to send */
        rnl::URMsg                    msg_rec; /**< Message received */
        rnl::Nbt                      nbt; /**< Neighbout table */
        std::vector<ns3::Vector3D>    wpts; /**< Waypoints that drone needs to follow */
        ns3::Vector3D                 pos; /**< Current position of the drone */
        int                           lookaheadindex; /**< Look ahead index for the drone */
        int                           toggle_bc; /**< toggle broadcast on */
        
        /**
         * @brief Set if more drone required at the boundary
         */

        ros::Publisher                drone_lk_ahead_pub;
        ros::Subscriber               drone_pos_sub;

        void initializeRosParams(ros::NodeHandle& nh);
        void publishLookAhead ();
        void posSubCb (const geometry_msgs::PoseStamped& _pos);
        void setReqDrone (); 

    };

    /**
     * @class 
     * @brief Wifi properties set in this class and passed to planner
     * 
     */
    class Properties
    {
       public:
            
            /**
             * @brief Construct a new Properties object
             * 
             * @param _phyMode Physical layer mode
             * @param _rss Deprecated 
             * @param _num_nodes Number of nodes in the swarm
             */
            Properties(
                    std::string _phyMode, 
                    double _rss, 
                    int _num_nodes
                    );
            
            /*********************************************************/
            /*NS3 Functions*/

            /**
             * @brief For Realtime simulation set realtime and checksum 
             * 
             * @param realtime Simulation will run realtime
             * @param checksum Checksum is required for setting data checks 
             */
            void initialize(bool realtime, bool checksum);
            
            /**
             * @brief Set the Wifi object
             * 
             * @param verb If verbose required
             * @param pcap_enable If pcap is enabled
             */
            void setWifi(bool verb, bool pcap_enable);

            /**
             * @brief Set the Internet
             * 
             */
            void setInternet();

            void SetStaticRoute(ns3::Ptr<ns3::Node> n, const char* destination, const char* nextHop, uint32_t interface);

            ns3::NodeContainer c; /**< Node container containing all the ns3 */
            
            /**
             * @brief Get type id value
             * 
             * @return ns3::TypeId 
             */
            ns3::TypeId  tid_val () const;

            /**
             * @brief Set type id value
             * 
             * @return ns3::TypeId& 
             */
            ns3::TypeId& tid_val ();
            
            
        private:
            ns3::WifiHelper wifi; /**< Wifi Helper */

            ns3::YansWifiPhyHelper wifiPhy; /**< YansWifiHelper for ease of use */

            ns3::YansWifiChannelHelper wifiChannel; /**< channel level properties */ 

            ns3::WifiMacHelper wifiMac; /**< Adding mac layer */
            
            ns3::NetDeviceContainer devices; /**< Virtual net device to be used */

            ns3::InternetStackHelper internet; /**< InternetStackHelper */

            ns3::Ipv4StaticRoutingHelper staticRouting;

            ns3::Ipv4AddressHelper ipv4; /**< Ipv4AddressHelper used for setting ips to node */
            ns3::Ipv4InterfaceContainer i; /**< This is ips assigned to nodes */ 

            ns3::TypeId tid; /**< Type ID being used */

            std::string phy_mode; /**< Phy Mode */
            double rss;  // -dBm /**< Rss value Deprecated*/
            int num_nodes; /**< Number of nodes */

            ns3::AsciiTraceHelper ascii;

            // The below set of helpers will help us to put together the wifi NICs we want
            
    };

    /**
     * @brief Initial neighbour table to be set. Since we are initializing the nodes in linear fashion \n 
     * Setting it requires only index of the node. \n
     * Set to id + 1 and id - 1 as one hop neighbours
     * 
     * @param id Index
     * @param n number of nodes
     * @return rnl::Nbt 
     */
    rnl::Nbt   setinitialNbt (int id, int n);
    
    /**
     * @brief Set the initial message to be sent to successor
     * 
     * @param nbt Initial neighbour table
     * @param id Destination address sent to id + 1
     * @param n number number of nodes
     * @return rnl::USMsg 
     */
    rnl::USMsg setinitialSMsg (rnl::Nbt nbt, int id, int n);


    /**
     * @brief Planner class. Flow \n
     * 1. Initialze Dronesoc for every UAV \n
     * 2. Initialize the mobility (Positions) of each UAV \n
     * 3. set the initial exploration path or leader \n
     * 4. Start simulation 
     */
    class Planner{
        public:
            /**
             * @brief Construct a new Planner object
             * 
             * @param p wifi properties
             * @param _no_nodes number of nodes
             * @param _pkt_int packet interval
             * @param pos_int position interval, this is to be used for broadcasting positions once UAV reaches desired location
             * @param _stp Stop time for simulation
             */
            Planner (ros::NodeHandle& _nh, ros::NodeHandle& _nh_private, rnl::Properties& p, int _no_nodes, float _pkt_int, float pos_int, float _stp);
            
            /**
             * @brief Initialize positions of each UAV here 
             */
            void initializeMobility ();

            /**
             * @brief Initialize sockets here
             * 
             */
            void initializeSockets ();

            /**
             * @brief Set the Initial Leader Explore Path. Currently just a straight path towards assumed fire node
             */
            void setLeaderExplorePath ();
            /**
             * @brief Check if boundary reached
             * 
             * @param node_pos position 
             * @return true if position in or at the boundary edge
             * @return false if node_pos exterior point of your boundary
             */
            // static bool fireDetected (ns3::Vector3D node_pos);
            static bool siteReached (ns3::Vector3D node_pos, int ID);

            /**
             * @brief Start simulation
             * 
             */
            void startSimul ();

            /**
             * @brief Increment look ahead point. (Assuming no dynamics), spawning nodes at distances
             * 
             */
            void incLookAhead ();

            /**
             * @brief Increment of child index by leader if it has commanded the previous id to the direction of the boundary
             */
            // void incLeaderChildId ();
            
            /**
             * @brief Function where leader needs to take in the SGDRONEREQ param and send commands to incoming UAV
             * 
             */
            // void updateStateofLeader ();
            void updateStateofCentre ();

            /**
             * @brief Update the position of UAVs after one time step
             */
            void updatePosSocs();

            /**
             * @brief update waypoints of the node with index id \n
             * Here we check for the State of the UAV. This function acts \n
             * as a FSM, and decides what next action the UAV must take depending on \n
             * the commands from its successor
             * 
             * @param id index of UAV
             */
            void updateWpts(int id);

            /**
             * @brief Just calls updateWaypoints for every node in the swarm
             * 
             */
            void updateSocsfromRec ();

            /**
             * @brief Get the Nearest Drone object
             * 
             * @param id index to which nearest drone object is required
             * @return std::pair<int, ns3::Vector3D> 
             */
            std::pair<int, ns3::Vector3D> getNearestDrone (int id);

            /**
             * @brief check if anchor has been updated. Transition from one node to \n
             * another in phase 2 happens here 
             * 
             * @param id current index
             * @return std::pair<int, ns3::Vector3D> pair of id and position of the new anchor point
             */
            // std::pair<int, ns3::Vector3D> checkUpdateAnchor (int id);

            /**
             * @brief Check SGDRONEREQ for id, if more drones are required at the boundary.
             * 
             * @param i index 
             */
            // void checkDroneReq(int i);

            void doLawnMoverScanning(ns3::Time interval, int i, ns3::Vector3D pos0);
            
            /**
             * @brief Check if phase 2 direction of 2 UAVS is the same
             * 
             * @param id1 index of UAV
             * @param id2 index of UAV
             * @return true if same
             * @return false otherwise
             */
            // bool isDirectionSame (int id1, int id2);

            /**
             * @brief Update senders after one time step
             * 
             */
            // void updateSocsSenders  ();
            void updateSocs  ();

            /**
             * @brief Advance the position, runs repeatedly after interval time
             * 
             * @param interval frequency at which to advanceposition. Will determine the speed of simulation 
             */
            void advancePos (ns3::Time interval);
            void takeOFF (double _t);
            bool withinThreshold (const rnl::DroneSoc* _soc);
            
            // static ns3::Vector3D       fire_centre;
            // static float               fire_rad;

            /**
             * @brief Deprecated
             * 
             */
            // void takeOff ();
            
            // static ns3::Vector3D       fire_centre; /**< Centre of the fire */
            static ns3::Vector3D       disas_centre;
            // static float               fire_rad; /**< Radius of the fire */
            static float               disas_rad; /**< Radius of the fire */

            // ns3::Vector3D pos0(disas_centre.x + RC, disas_centre.y,         disas_centre.z);
            // ns3::Vector3D pos1(disas_centre.x + RC, disas_centre.y - RC,    disas_centre.z);
            // ns3::Vector3D pos2(disas_centre.x + RC, disas_centre.y + RC,    disas_centre.z);
            // ns3::Vector3D pos3(disas_centre.x,      disas_centre.y,         disas_centre.z);
            // ns3::Vector3D pos4(disas_centre.x,      disas_centre.y - RC,    disas_centre.z);
            // ns3::Vector3D pos5(disas_centre.x,      disas_centre.y + RC,    disas_centre.z);
            // ns3::Vector3D pos6(disas_centre.x - RC, disas_centre.y,         disas_centre.z);
            // ns3::Vector3D pos7(disas_centre.x - RC, disas_centre.y - RC,    disas_centre.z);
            // ns3::Vector3D pos8(disas_centre.x + RC, disas_centre.y + RC,    disas_centre.z);

        private:
            ros::NodeHandle nh;
            ros::NodeHandle nh_private;
            rnl::Properties            wifi_prop; /**< wifi properties object */
            std::vector<rnl::DroneSoc> nsocs; /**< UAV Dronesocs in the simulation, Each DroneSoc represents a UAV */
            int                        num_nodes; /**< number of nodes */

            ns3::MobilityHelper        mobility; /**< Mobility helper to set the initial mobility of the nodes */
            ns3::Time                  pkt_interval; /**< Unicast packet interval */
            ns3::Time                  pos_interval; /**< Broadcast packet interval */
            ns3::Time                  stopTime; /**< Stop time */
            int                        leader_id; /**< Leader index */
            int                        ldirec_flag; /**< Direction at which leader is sending the Drone currently, this value is +1 or -1 */
            int                        lchild_id; /**< Child index */
            int                        tail_id; /**< Child index */
    };
};
