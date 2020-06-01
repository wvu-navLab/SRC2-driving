/*!
 * \fws_driving.cpp
 * \brief 4ws_driving (...).
 *
 * Four wheel steering driving (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
 * \author Chris Tatsch, WVU - ca0055@wvu.mix.edu
 * \date June 01, 2020
 */

#include "driving_control/fws_driving.h"

FourWheelSteeringDriving::FourWheelSteeringDriving(ros::NodeHandle & nh)
: nh_(nh)
{
}

/*!
 * \brief Creates and runs the fws_driving node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fws_driving");
    ros::NodeHandle nh("");
    ros::Rate rate(50);

    ROS_INFO("Find Rover Node initializing...");
    FourWheelSteeringDriving fws_driving(nh);

    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}