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
#include <motion_control/MotorGroup.h>
#include <motion_control/SteeringGroup.h>
#include <driving_tools/Stop.h>
#include <driving_tools/RotateInPlace.h>
#include <driving_tools/CirculateBaseStation.h>
#include <driving_tools/MoveForward.h>

// ConstantsS
#define PI 3.14159265

/*!
 * \def MAX_MOTOR_EFFORT
 *
 * The maximum translational velocity.
 */
#define MAX_MOTOR_EFFORT 75.0

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


class DrivingTools
{
private:

    // Node Handle
    ros::NodeHandle nh_;

    // Publishers
    ros::Publisher pubMotorEfforts;
    ros::Publisher pubSteeringEfforts;

    // Service Servers
    ros::ServiceServer stopServer;
    ros::ServiceServer rotateInPlaceServer;
    ros::ServiceServer circBaseStationServer;
    ros::ServiceServer moveForwardServer;

    // Create output messages
    motion_control::MotorGroup m;       /*!< Publisher of motor efforts */
    motion_control::SteeringGroup s;    /*!< Publisher of steering angles */   
    double s1 = 0; double s2 = 0; double s3 = 0; double s4 = 0;   /*!< Init steering variables */
    double m1 = 0; double m2 = 0; double m3 = 0; double m4 = 0;   /*!< Init driving variables */

public:
    DrivingTools();
    bool Stop(driving_tools::Stop::Request  &req, driving_tools::Stop::Response &res);
    bool RotateInPlace(driving_tools::RotateInPlace::Request  &req, driving_tools::RotateInPlace::Response &res);
    bool CirculateBaseStation(driving_tools::CirculateBaseStation::Request  &req, driving_tools::CirculateBaseStation::Response &res);
    bool MoveForward(driving_tools::MoveForward::Request  &req, driving_tools::MoveForward::Response &res);
};


#endif // DRIVING_TOOLS_H