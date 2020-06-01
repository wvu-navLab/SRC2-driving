
#include "driving_control/wheel_control.h"

WheelControl::WheelControl(ros::NodeHandle & nh)
: nh_(nh)
{
}

WheelControl::~WheelControl()
{
}

/*!
 * \brief Creates and runs the wheel_control node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_control");
    ros::NodeHandle nh("");
    ros::Rate rate(50);

    ROS_INFO("Find Rover Node initializing...");
    WheelControl wheel_control(nh);

    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}