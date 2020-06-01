/*!
 * \wheel_control.cpp
 * \brief wheel_control (...).
 *
 * Wheel control (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
 * \author Chris Tatsch, WVU - ca0055@wvu.mix.edu
 * \author Rogerio Lima, WVU - 	rrl00003@mix.wvu.edu
 * \date June 01, 2020
 */

#include "driving_control/wheel_control.h"

WheelControl::WheelControl(ros::NodeHandle & nh)
: nh_(nh)
{
    subJointStates = nh_.subscribe("joint_states", 1000, &WheelControl::jointStatesCallback, this);
    subWheelVelCmds = nh_.subscribe("wheel_velocities", 1000, &WheelControl::wheelVelCmdsCallback, this);
    pubMotorEfforts = nh_.advertise<motion_control::MotorGroup>("motor_efforts", 1000);
}

void WheelControl::wheelVelCmdsCallback(const driving_control::WheelVelCmds::ConstPtr &msg)
{
    w1_cmd_ = msg->w1;
    w2_cmd_ = msg->w2;
    w3_cmd_ = msg->w3;
    w4_cmd_ = msg->w4;
  // ROS_INFO("Joint commands updated.");
  // ROS_INFO_STREAM("Values "<< w1_cmd_<<" "<< w2_cmd_<< " "<< w3_cmd_<< " "<< w4_cmd_);
}

void WheelControl::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    w1_current_ = msg->position[15];
    w2_current_ = msg->position[0];
    w3_current_ = msg->position[8];
    w4_current_ = msg->position[7];
  // ROS_INFO("Joint states updated.");
  // ROS_INFO_STREAM("Values "<< q1_pos_<<" "<< q2_pos_<< " "<< q3_pos_<< " "<< q4_pos_);
}

void WheelControl::controlMotors()
{
    double m1 = Kp_*(w1_cmd_ - w1_current_);
    double m2 = Kp_*(w2_cmd_ - w2_current_);
    double m3 = Kp_*(w3_cmd_ - w3_current_);
    double m4 = Kp_*(w4_cmd_ - w4_current_);

    motion_control::MotorGroup m;

    m.m1 = m1; 
    m.m2 = m2; 
    m.m3 = m3; 
    m.m4 = m4;

    pubMotorEfforts.publish(m);
  // ROS_INFO("Joint states updated.");
  // ROS_INFO_STREAM("Values "<< q1_pos_<<" "<< q2_pos_<< " "<< q3_pos_<< " "<< q4_pos_);
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
        wheel_control.controlMotors();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}