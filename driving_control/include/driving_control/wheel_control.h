/*!
 * \wheel_control.h
 * \brief wheel_control (...).
 *
 * Wheel control (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Chris Tatsch, WVU - ca0055@mix.wvu.edu
 * \author Rogerio Lima, WVU - 	rrl00003@mix.wvu.edu
 * \date June 01, 2020
 */

#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

// Include cpp important headers
#include <math.h>
#include <stdio.h> 

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <motion_control/MotorGroup.h>
#include <driving_control/WheelVelCmds.h>
#include <sensor_msgs/JointState.h>

class WheelControl
{
private:
    // Node Handle
    ros::NodeHandle & nh_;
    // Publisher
    ros::Publisher pubMotorEfforts;

    // Subscriber
    ros::Subscriber subWheelVelCmds;
    ros::Subscriber subJointStates;

    // void doCalculations();
    // Callback function for subscribers.
    void wheelVelCmdsCallback(const driving_control::WheelVelCmds::ConstPtr &msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

    double w1_current_ = 0.0;
    double w2_current_ = 0.0;
    double w3_current_ = 0.0;
    double w4_current_ = 0.0;
 
    double w1_cmd_ = 0.0;
    double w2_cmd_ = 0.0;
    double w3_cmd_ = 0.0;
    double w4_cmd_ = 0.0;

    // PID gains
    double Kp_ = 50;

public:
    WheelControl(ros::NodeHandle & nh);

    void controlMotors();

};



#endif // WHEEL_CONTROL_H