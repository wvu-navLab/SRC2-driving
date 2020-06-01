/*!
 * \wheel_control.cpp
 * \brief wheel_control (...).
 *
 * Wheel control (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
 * \author Chris Tatsch, WVU - ca0055@wvu.mix.edu
 * \date June 01, 2020
 */

#include "driving_control/wheel_control.h"

WheelControl::WheelControl(ros::NodeHandle & nh)
: nh_(nh)
{
    subJointStates = nh_.subscribe("joint_states", 1000, &WheelControl::jointStatesCallback, this);
    subWheelVels = nh_.subscribe("wheel_velocities", 1000, &WheelControl::wheelVelsCallback, this);
    pubMotorEfforts = nh_.advertise<motion_control::MotorGroup>("motor_efforts", 1000);
}

void WheelControl::wheelVelsCallback(const motion_control::MotorGroup::ConstPtr &msg)
{
    ROS_INFO_STREAM("Message: " << msg);
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