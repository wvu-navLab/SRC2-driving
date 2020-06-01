/*!
 * \wheel_control.h
 * \brief wheel_control (...).
 *
 * Wheel control (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
 * \author Chris Tatsch, WVU - ca0055@wvu.mix.edu
 * \date June 01, 2020
 */

#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

// Include cpp important headers
#include <math.h>
#include <stdio.h> 

// ROS headers
#include <ros/ros.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <motion_control/MotorGroup.h>
#include <driving_control/WheelVels.h>

class WheelControl
{
private:
    // Node Handle
    ros::NodeHandle & nh_;
    // Publisher
    ros::Publisher pubMotorEfforts;

    // Subscriber
    ros::Subscriber subWheelVels;
    
public:
    WheelControl(ros::NodeHandle & nh);

    // Callback function for subscriber.
    void wheelVelsCallback(const motion_control::MotorGroup::ConstPtr &msg);
};



#endif // WHEEL_CONTROL_H