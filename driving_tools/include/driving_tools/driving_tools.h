/*!
 * \driving_tools.h
 * \brief Manipulation Planning for SRC2 Rovers
 *
 * Manipulation creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * * \date April 28, 2020
 */

#ifndef DRIVING_TOOLS_H
#define DRIVING_TOOLS_H

#include <math.h>
#include <stdio.h> 

// ROS includes.
#include <ros/ros.h>
#include <ros/console.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <motion_control/WheelGroup.h>
#include <motion_control/SteeringGroup.h>
#include <driving_tools/Stop.h>
#include <driving_tools/RotateInPlace.h>
#include <driving_tools/CirculateBaseStation.h>
#include <driving_tools/MoveForward.h>
#include <driving_tools/TurnWheelsSideways.h>
#include <driving_tools/MoveSideways.h>
#include <std_msgs/Int64.h>

// ConstantsS
#define PI 3.14159265

/*!
 * \def MAX_SPEED
 *
 * The maximum wheel rotational speed.
 */
#define MAX_SPEED 1.07

/*!
 * \def WHEEL_RADIUS
 *
 * The wheel radius.
 */
#define WHEEL_RADIUS 0.17

/*!
 * \def MAX_STEERING_ANGLE
 *
 * The maximum steering angle.
 */
#define MAX_STEERING_ANGLE PI/2

/*!
 * \def SEMI_CHASSIS_LENGTH
 *
 * Half of the wheelbase length.
 */
#define SEMI_CHASSIS_LENGTH 1.8/2

/*!
 * \def SEMI_CHASSIS_WIDTH
 *
 * Half of the track length.
 */
#define SEMI_CHASSIS_WIDTH 1.5/2

#define STOP_MODE 0
#define CRAB_MODE 1
#define DACK_MODE 2
#define TIPP_MODE 3
#define INACTIVE_MODE 4

class DrivingTools
{
private:

    // Node Handle
    ros::NodeHandle nh_;

    // Publishers
    ros::Publisher pubWheelVels;
    ros::Publisher pubSteeringAngles;

    // Service Servers
    ros::ServiceServer stopServer;
    ros::ServiceServer rotateInPlaceServer;
    ros::ServiceServer circBaseStationServer;
    ros::ServiceServer moveForwardServer;
    ros::ServiceServer turnWheelsSidewaysServer;
    ros::ServiceServer moveSidewaysServer;

    // Publishers
    ros::Publisher pubDrivingMode;

    // Create output messages
    motion_control::WheelGroup w;       /*!< Publisher of motor efforts */
    motion_control::SteeringGroup s;    /*!< Publisher of steering angles */   
    double s1 = 0; double s2 = 0; double s3 = 0; double s4 = 0;   /*!< Init steering variables */
    double w1 = 0; double w2 = 0; double w3 = 0; double w4 = 0;   /*!< Init driving variables */
    int driving_mode_;

public:
    DrivingTools();
    bool Stop(driving_tools::Stop::Request  &req, driving_tools::Stop::Response &res);
    bool RotateInPlace(driving_tools::RotateInPlace::Request  &req, driving_tools::RotateInPlace::Response &res);
    bool CirculateBaseStation(driving_tools::CirculateBaseStation::Request  &req, driving_tools::CirculateBaseStation::Response &res);
    bool MoveForward(driving_tools::MoveForward::Request  &req, driving_tools::MoveForward::Response &res);
    bool TurnWheelsSideways(driving_tools::TurnWheelsSideways::Request  &req, driving_tools::TurnWheelsSideways::Response &res);
    bool MoveSideways(driving_tools::MoveSideways::Request  &req, driving_tools::MoveSideways::Response &res);
};


#endif // DRIVING_TOOLS_H