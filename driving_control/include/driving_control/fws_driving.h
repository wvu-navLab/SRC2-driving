/*!
 * \fws_driving.h
 * \brief 4ws_driving (...).
 *
 * Four wheel steering driving (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
 * \author Chris Tatsch, WVU - ca0055@wvu.mix.edu
 * \date June 01, 2020
 */

#ifndef FWS_DRIVING_H
#define FWS_DRIVING_H

// Include cpp important headers
#include <math.h>
#include <stdio.h> 

// ROS headers
#include <ros/ros.h>

class FourWheelSteeringDriving
{
private:
    // Node Handle
    ros::NodeHandle & nh_;
    
public:
    FourWheelSteeringDriving(ros::NodeHandle & nh);
};


#endif // FWS_DRIVING_H