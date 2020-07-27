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
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <waypoint_nav/SetGoal.h>
#include <waypoint_nav/Interrupt.h>

#define ARRIVED 0
#define GOING 1
#define UNREACHABLE 2

#define INIT_STATE 0
#define TRAV_STATE 1
#define PLAN_STATE 2
#define VOLH_STATE 3
#define LOST_STATE 4

class WaypointNavigation
{
public:
    WaypointNavigation(ros::NodeHandle & nh);

    void commandVelocity();

    bool active_ = false;

private:
    // Node Handle
    ros::NodeHandle & nh_;

    // Subscriber
    ros::Subscriber subOdom;
    ros::Subscriber subGoal;
    ros::Subscriber subAvoidDirection;
    ros::Subscriber subSmach;

    // Publisher
    ros::Publisher pubCmdVel;
    ros::Publisher pubNavStatus;
    ros::Publisher pubWaypointUnreachable;
    ros::Publisher pubArrivedAtWaypoint;

    ros::ServiceServer srv_set_goal_;
    ros::ServiceServer srv_interrupt_;

    double avoid_angle_ = 0.0;
    double Kp_yaw_ = 5.0;
    bool rr_ = false;
    bool ll_ = false;

    bool firstOdom_;
    bool firstGoal_;

    geometry_msgs::Pose localPos_curr_;
    geometry_msgs::Twist localVel_curr_;
    geometry_msgs::Pose goalPos_;

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void smachCallback(const std_msgs::Int64::ConstPtr& msg);
    void goalCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void avoidObstacleCallback(const std_msgs::Float64::ConstPtr& msg);

    bool setGoal(waypoint_nav::SetGoal::Request &req, waypoint_nav::SetGoal::Response &res);
    bool interrupt(waypoint_nav::Interrupt::Request &req, waypoint_nav::Interrupt::Response &res);
};


#endif // WAYPOINT_NAV_H
