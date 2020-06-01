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

// ROS headers
#include <ros/ros.h>

class WheelControl
{
private:
    // Node Handle
    ros::NodeHandle & nh_;
    
public:
    WheelControl(ros::NodeHandle & nh);
    ~WheelControl();
};



#endif // WHEEL_CONTROL_H