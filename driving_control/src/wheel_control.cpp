/*!
 * \wheel_control.cpp
 * \brief wheel_control (...).
 *
 * Wheel control (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Chris Tatsch, WVU - ca0055@mix.wvu.edu
 * \author Rogerio Lima, WVU - 	rrl00003@mix.wvu.edu
 * \date June 01, 2020
 */

#include "driving_control/wheel_control.h"

WheelControl::WheelControl(ros::NodeHandle & nh)
: nh_(nh)
{
    // Subscribers
    subJointStates = nh_.subscribe("joint_states", 1000, &WheelControl::jointStatesCallback, this);
    subWheelVelCmds = nh_.subscribe("driving/wheel_vel_cmds", 1000, &WheelControl::wheelVelCmdsCallback, this);
    // Publishers
    pubMotorEfforts = nh_.advertise<motion_control::MotorGroup>("driving/motor_efforts", 1000);
}

void WheelControl::wheelVelCmdsCallback(const driving_control::WheelVelCmds::ConstPtr &msg)
{
    w1_cmd_ = msg->w1;
    w2_cmd_ = msg->w2;
    w3_cmd_ = msg->w3;
    w4_cmd_ = msg->w4;

    // new_state_or_setpt_ = true;
    // ROS_INFO("Joint commands updated.");
    // ROS_INFO_STREAM("Values "<< w1_cmd_<<" "<< w2_cmd_<< " "<< w3_cmd_<< " "<< w4_cmd_);
}

void WheelControl::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    int fr_wheel_joint_idx;
    int br_wheel_joint_idx;
    int fl_wheel_joint_idx;
    int bl_wheel_joint_idx;
    
    //get robot name
    for(int i=0; i<msg->name.size(); i++) 
    {
        if(msg->name[i] == "fr_wheel_joint") 
        {
            fr_wheel_joint_idx = i;
        }
        if(msg->name[i] == "br_wheel_joint") 
        {
            br_wheel_joint_idx = i;
        }
        if(msg->name[i] == "fl_wheel_joint") 
        {
            fl_wheel_joint_idx = i;
        }
        if(msg->name[i] == "bl_wheel_joint") 
        {
            bl_wheel_joint_idx = i;
        }
    }

    w1_current_ = msg->velocity[fr_wheel_joint_idx]; //fr_wheel_joint
    w2_current_ = msg->velocity[br_wheel_joint_idx]; //br_wheel_joint
    w3_current_ = msg->velocity[fl_wheel_joint_idx]; //fl_wheel_joint
    w4_current_ = msg->velocity[bl_wheel_joint_idx]; //bl_wheel_joint

    // ROS_INFO("Joint states updated.");
    // ROS_INFO_STREAM("Values "<< w1_current_<<" "<< w2_current_<< " "<< w3_current_<< " "<< w4_current_);
}

void WheelControl::controlMotors()
{

    // See https://bitbucket.org/AndyZe/pid/src/master/src/pid.cpp
    // for a better implementation
        
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
    // ROS_INFO("Motor efforts published.");
    // ROS_INFO_STREAM("Values "<< m1<<" "<< m2<< " "<< m3<< " "<< m4);
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
    ros::Rate rate(100);

    ROS_INFO("Wheel Velocity Controller initializing...");
    WheelControl wheel_control(nh);

    while(ros::ok()) 
    {
        wheel_control.controlMotors();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}