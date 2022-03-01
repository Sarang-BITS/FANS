/**
 * @brief Utility functions for Swarm Planner
 */
#pragma once
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "ns3/core-module.h"

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/netanim-module.h"

/**
 * @namespace
 */
namespace rnl {

    /**
     * @brief Set the Position of an ns3 node
     * 
     * @param node  Node Pointer for which the position needs to be set
     * @param pos   position to set it to 
     */
    void setPosition
    (
        ns3::Ptr<ns3::Node> node,
        ns3::Vector3D       pos
    );
    
    /**
     * @brief Get the Position object
     * 
     * @param node get the position of object at this node 
     * 
     * @return ns3::Vector3D 
     */
    ns3::Vector3D getPosition
    (
        ns3::Ptr<ns3::Node> node
    );

    /**
     * @brief Get a Trajectory for UAV
     * 
     * @param wpts Waypoints pointer to be filled by final trajectory locations
     * @param start_pos Starting position for the robot
     * @param end_pos Ending position for the robot
     * @param step step size at which point seperation is required in the trajectory
     * 
     * @return true if trajectory found else false
     */
    bool getTrajectory
    (
        std::vector<ns3::Vector3D>* wpts,
        ns3::Vector3D start_pos, 
        ns3::Vector3D end_pos,
        double        step 
    );

    bool getTrajectoryContinue
    (
        std::vector<ns3::Vector3D>* wpts,
        ns3::Vector3D start_pos, 
        ns3::Vector3D end_pos,
        double        step 
    );

    /**
     * @brief Get the To Circle Range object
     * 
     * @param wpts Waypoints pointer to be filled by final trajectory locations
     * @param _anch_p Anchor position
     * @param _my_p Start Position
     * @param cr Circling Radius required
     * @param step Step size at which to add waypoints
     * 
     * @return true if succeeded else false
     */
    bool getToCircleRange
    (
        std::vector<ns3::Vector3D>* wpts,
        ns3::Vector3D              _anch_p,
        ns3::Vector3D              _my_p,
        float                      cr,
        float                      step
    );

    /**
     * @brief Offset to reach from child location to anchor location
     * for getting to Circling radius of anchor 
     * 
     * @param anch Anchor location
     * @param child Child location
     * @param _cr Circling Radius
     * 
     * @return float Offset returned
     */
    float circlingOffset
    (
        ns3::Vector3D anch,
        ns3::Vector3D child,
        float         _cr
    );

    /**
     * @brief Waypoints passed for Position Hold requirement
     * 
     * @param wpts Waypoints pointer to be filled by final trajectory locations
     * @param pos Position at which to hold position
     */
    void posHold
    (

        std::vector<ns3::Vector3D>* wpts,
        ns3::Vector3D       pos
    );
};
