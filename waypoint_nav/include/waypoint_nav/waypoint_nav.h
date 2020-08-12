/*!
 * \waypoint_nav.h
 * \brief waypoint_nav (...).
 *
 * Waypoint navigation (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Matteo De Petrillo, WVU - madepetrillo@mix.wvu.edu
 * \date June 01, 2020
 */

#ifndef WAYPOINT_NAV_H
#define WAYPOINT_NAV_H

// Include cpp important headers
#include <math.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <termios.h>

// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


class WaypointNavigation
{
public:
    WaypointNavigation(ros::NodeHandle & nh);

    void commandVelocity();

private:
    // Node Handle
    ros::NodeHandle & nh_;

    // Subscriber
    ros::Subscriber subOdom;
    ros::Subscriber subOdomTruth;
    ros::Subscriber subGoal;
    ros::Subscriber subAvoidDirection;

    // Publisher
    ros::Publisher pubCmdVel;

    double avoid_angle_ = 0.0;
    double Kp_yaw_ = 5.0;
    bool rr_ = false;
    bool ll_ = false;

    geometry_msgs::Pose localPos_curr_;
    geometry_msgs::Twist localVel_curr_;
    geometry_msgs::Pose goalPos_;

    void odometryTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void goalCallback(const geometry_msgs::Pose& msg);
    void avoidObstacleCallback(const std_msgs::Float64::ConstPtr& msg);
};


#endif // WAYPOINT_NAV_H
