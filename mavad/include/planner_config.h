/**
 * @brief Configuration file for setting the serializing & parsing of ns3 data
 * to other formats. Contains the datatype for data encoding & decoding and other
 * static data variables.
 */
#pragma once

#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/netanim-module.h"

/**
 * @namespace rnl
 */
namespace rnl{
    static std::string DELIM           = "\n"; /** Delimiter used for serializing message to stringstream @see rnl::USMsg::serialize() @see URMsg::serialize() */
    static std::string DELIM_NBTHOP    = "|"; /** Delimiter for specifiying neighbouring hops @see Nbt::serialize*/
    static std::string DELIM_NBTPOS    = ".";  /** Delimiter for specifiying neighbouring hops @see Nbt::serialize*/
    static std::string DELIM_NBTID_POS = ","; /** Delimiter for specifiying neighbouring hops @see Nbt::serialize*/
    static std::string DELIM_NBTMHOP   = "~~"; /** Delimiter seperating neighbours based on hop count @see Nbt::serialize*/
    static std::string DELIM_POS       = ":"; /** Delimiter seperating coordinate value of robot pose @see URMsg::parseUnicast*/
    static std::string IP_BASE         = "10.1.1."; /** IP Base*/
    static int         BASEID          = 50; /** Base Station IP Address */
    static double      STEP            = 0.3; /** Step Size for discretizing */
    static double      RC              = 4.0; /** RC Distance as specified in the paper. Ideal distance of seperation between two nodes */


    /**
     * @enum 
     * @brief Specifying state of a node and swarm as known by the node. Used as a bitfield. \n
     * eg. If I am online and swarm leader has reached site, my state will be 2 (SONLINE) + 64 (SGSITEREACHED)
     */
    enum state
    {
        SSITEREACHED    = 1,    // SITE REACHED
        SONLINE         = 2,    // ON STRAIGHT LINE
        SANCHORING      = 4,    // ANCHORING (GOING LEFT,RIGHT,BEHIND)
        SLEFT           = 8,    // LEFT NEIGHBOUR
        SRIGHT          = 16,   // RIGHT NEIGHBOUR
        SCENTRE         = 32,   // CENTRE NEIGHBOUR
        SGSITEREACHED   = 64,   // [GLOBAL] SITE REACHED
        SGDRONEREQ      = 128,  // [GLOBAL] DRONE REQUIRED (LESS THAN 9 DRONES OVER SITE)
        SLAWNMOVERING   = 256,  // LAWNMOVERING
        SCHANGEPAR      = 512   // CHANGING PARENT
    };

    /**
     * @enum 
     * @brief Control command
     */
    enum control
    {
        CHOLDRC         = 1,    // HOLD RC DISTANCE FROM PARENT
        CCHANGEPAR      = 2,    // CHANGE PARENT
        CLTOP           = 4,    // GO LEFT TOWARDS PARENT
        CRTOP           = 8,    // GO RIGHT TOWARDS PARENT
        CBTOP           = 16    // GO BEHIND OF PARENT
    };

    /**
     * @struct Nbt
     * @brief For parsing and serializing data of neighbouring nodes
     */
    struct Nbt
    {
        /**
         * @brief Construct a new Nbt object
         */
        Nbt ();

        std::vector<std::pair<int, ns3::Vector3D>> one_hop; /**< ID and Location of One Hop Neighbour */ 
        std::vector<std::pair<int, ns3::Vector3D>> two_hop; /**< ID and Location of Two Hop Neighbour */
        
        /**
         * @brief Parse one hop neighbours here
         * 
         * @param msg contains encoded data of one hop neighbours 
         */
        void parseSingleNb(std::string msg); 

        /**
         * @brief Serializes one_hop and two_hop neighbours to string
         * 
         * @param dst destination pointer where these need to be written to
         */
        void serialize(std::string* dst);
    };

    /**
    * @struct USMsg
    * 
    * @brief
    * This Structure Will be Serialized and sent as Unicast Message. The \n
    * communication will have this format. Each drone sends \n
    * Unicast Message in this format. The serializer will make a \n
    * delimiter seperated string which will be passed onto the sockets \n
    * for transmission. \n
    * 
    * * msg_type (char) - Is the message bc/unicast (u/b) \n
    * * source_id  (int)- The ID which I have while Sending the message \n
    * * dst_id (int)    - The Destination ID, Message will be sent to this ID \n
    * * nbs    (string) - The Neighbour Table Of this Node, \n
                          Format of String is | seperated as follows: \n
                          (.. | ID(int), hop(int), pos(ns3::Vector) | ..) \n 
                          hop - 1,2,3 ...  \n
                          pos - x (float) : y (float) : z (float) \n
    * * control (int)   - This is a bit field which consists of all possible \n
                          control commands a particular drone can give. \n
                          1   -> Drone Should Hold Pos at RC from Parent \n
                          2   -> Drone Should Change Parent to p_id \n
                          4   -> Drone Should Go Left of the p_id \n
                          8   -> Drone Should Go Right of the p_id \n
                          16  -> Drone Should Go Behind of the p_id \n
                          
    * * state (int)     - This is a bit field consisting of the state this \n
                          drone is in. \n

                          1   -> Site Reached \n
                          2   -> On Straight Line \n
                          4   -> Going Left/Right/Behind \n
                          8   -> Left Neighbour \n
                          16  -> Right Neighbour \n
                          32  -> Centre Neighbour \n
                          64  -> (Global) Site Reached \n
                          128 -> (Global) Drone required \n
                          256 -> Lawnmovering \n
                          512 -> Changing Parent \n

    * * p_id (int)      - Parent Id \n
    * * neigh_cnt (int) - Neighbour Count \n
    * * p_loc (ns3::Vector3D)  - Location of the Parent \n
    * * bc_nbs (string)        - Broadcast Neighbours \n
    */
    struct USMsg
    {
        std::string       msg_type = "u"; /**< Message type. "u" for unicast, "b" for broadcast */
        int               source_id; /**< source index from which this message originated */
        int               dst_id; /**< Destination index to which this message was intended to be sent */
        std::string       nbs; /**< Information about my neighbours */
        int               control; /**< Control information to receiver */
        int               state; /**< Self State information to receiver */
        int               p_id; /**< My Parent Index */
        int               neigh_cnt;/**< Neighbour Count */
        ns3::Vector3D     p_loc; /**< My Parent Location */
        std::string       bc_nbs; /**< Neighbours broadcasting to me */
        
        /**
         * @brief Construct a new USMsg object
         */
        USMsg () ;

        /**
         * @brief Construct a new USMsg object
         * 
         * @param id source index
         * @param dst destination index
         * @param nbs serialized neighbour table
         * @param co control value to destination node
         * @param st my state 
         * @param p my parent index
         * @param p my neighbour count
         * @param _ploc my parent location
         */
        USMsg 
        (int                     id,
        int                      dst,
        const std::string&       nbs,
        int                      co, 
        int                      st,
        int                      p,
        int                      neigh_count,
        ns3::Vector3D            _ploc
        );

        /**
         * @brief Serialize the member attributes of this structure to a Unicast Message
         * 
         * @param loc Pointer to string to which this needs to be serialized
         */
        void serialize (std::string * loc);

        /**
         * @brief Serialize the member attributes of this structure to a broadcast message.
         * No personal member info serialized
         * 
         * @param loc Pointer to string to which this needs to be serialized
         * @param id index
         * @param pos Position 
         */
        void serializeBC (std::string * loc, int id, ns3::Vector3D pos);
    };

    /**
    * @struct URMsg
    * 
    * @brief This Structure Will be parsed as a Unicast Message. The \n
    * communication will have this format. Each drone receives \n
    * Unicast Message in this format. The parser will convert \n
    * the received message into struct members. \n
    * 
    * * source_id  (int)    - The ID which I have while Sending the message \n
    * * dst_id (int)    - The Destination ID, Message will be sent to this ID \n
    * * nbs    (string) - The Neighbour Table Of this Node, \n
                          Format of String is | seperated as follows: \n
                          (.. | ID(int), hop(int), pos(ns3::Vector) | ..) \n 
                          hop - 1,2,3 ... \n
                          pos - x (float) : y (float) : z (float) \n
    * * control (int)   - This is a bit field which consists of all possible \n
                          control commands a particular drone can give. \n
                          1   -> Drone Should Hold Pos at RC from Parent \n
                          2   -> Drone Should Change Parent to p_id \n
                          4   -> Drone Should Go Left of the p_id \n
                          8   -> Drone Should Go Right of the p_id \n
                          16  -> Drone Should Go Behind of the p_id \n
                          
    * * state (int)     - This is a bit field consisting of the state this \n
                          drone is in. \n

                          1   -> Site Reached \n
                          2   -> On Straight Line \n
                          4   -> Going Left/Right/Behind \n
                          8   -> Left Neighbour \n
                          16  -> Right Neighbour \n
                          32  -> Centre Neighbour \n
                          64  -> (Global) Site Reached \n
                          128 -> (Global) Drone required \n
                          256 -> Lawnmovering \n
                          512 -> Changing Parent \n

    * * p_id (int)      - Parent Id \n
    * * neigh_cnt (int) - Neighbour Count \n
    * * p_loc (ns3::Vector3D)  - Location of the Parent \n
    * * bc_nbs (string)        - Broadcast Neighbours \n
    */
    struct URMsg
    {
        int               source_id; /**< Source id from where the message originated */
        int               dst_id; /**< Destination ID, the desired location the message was sent to */
        std::string       nbs; /**< Neighbourhood of the source ID */
        int               control; /**< Control Command from source id to destination id */
        int               state; /**< State of the source */
        int               p_id; /**< Parent ID to be followed by the dst_id */
        int               neigh_cnt; /**< Neighbour Count */
        ns3::Vector3D     p_loc; /**< Location of Parent */
        std::string       bc_nbs; /**< Broadcast neighbours of Source ID, known if rnl::USMsg::msg_type is "u"*/

        /**
         * @brief Construct a new URMsg object
         */
        URMsg ();
        
        /**
         * @brief Construct a new URMsg object
         * 
         * @param id source id 
         * @param dst destination id
         * @param nbs neighbour table
         * @param co control bit
         * @param st state bit
         * @param p parent id
         * @param neigh_count neighbour count
         * @param _ploc parent location
         */
        URMsg 
        (int                     id,
        int                      dst,
        const std::string&       nbs,
        int                      co,
        int                      st,
        int                      p,
        int                      neigh_count,
        ns3::Vector3D            _ploc
        );
        
        /**
         * @brief Parse an incoming Message 
         * 
         * @param msg Incoming message on the socket converted to string and passed here
         */
        void parse (std::string& msg);

        /**
         * @brief Parse an incoming Broadcast Message
         * 
         * @param msg If Incoming message is Broadcast parsed here 
         */
        void parseBroadcast (std::string& msg);

        /**
         * @brief Parse an incoming Unicast Message
         * 
         * @param msg If Incoming message is Unicast parsed here
         */
        void parseUnicast   (std::string& msg);

    };
};
