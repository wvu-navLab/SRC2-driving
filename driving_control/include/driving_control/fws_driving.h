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

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <geometry_msgs/Twist.h>
#include <motion_control/SteeringGroup.h>
#include <driving_control/WheelVelCmds.h>
#include <driving_tools/Stop.h>

#include <driving_control/speed_limiter.h>

class FourWheelSteeringDriving
{
public:
    FourWheelSteeringDriving(ros::NodeHandle & nh);

    void starting(const ros::Time& time);
    
    void stopping(const ros::Time& /*time*/);

    void update(const ros::Time& time, const ros::Duration& period);

    void updateCommand(const ros::Time& time, const ros::Duration& period);

    void brake();

    void cmdVelCallback(const geometry_msgs::Twist& command);
    
private:
    // Node Handle
    ros::NodeHandle & nh_;

    // Subscriber
    ros::Subscriber subCmdVel;

    // Publisher
    ros::Publisher pubWheelVelCmds;
    ros::Publisher pubSteeringAngles;

    // Client
    ros::ServiceClient clientStop;

    // Name string
    std::string name_;

    /// Velocity command related:
    struct Command
    {
      ros::Time stamp;

      Command() : stamp(0.0) {}
    };

    struct CommandTwist : Command
    {
      double lin_x;
      double lin_y;
      double ang;

      CommandTwist() : lin_x(0.0), lin_y(0.0), ang(0.0) {}
    };

    realtime_tools::RealtimeBuffer<CommandTwist> command_twist_;
    CommandTwist command_struct_twist_;

    /// Whether the control is make with four_wheel_steering msg or twist msg:
    bool enable_twist_cmd_;

    /// Wheel separation (or track), distance between left and right wheels (from the midpoint of the wheel width):
    double wheel_separation_width_;

    /// Distance between a wheel joint (from the midpoint of the wheel width) and the associated steering joint:
    /// We consider that the distance is the same for every wheel
    double wheel_steering_y_offset_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Wheel base (distance between front and rear wheel):
    double wheel_separation_length_;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Extra params 
    double joint_state_controller_publish_rate_;
    double max_velocity_;

    /// Speed limiters:
    CommandTwist last1_cmd_;
    CommandTwist last0_cmd_;
    SpeedLimiter limiter_lin_;
    SpeedLimiter limiter_ang_;
};


#endif // FWS_DRIVING_H