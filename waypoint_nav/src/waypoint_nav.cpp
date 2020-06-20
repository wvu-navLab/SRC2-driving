/*!
 * \waypoint_nav.cpp
 * \brief waypoint_nav (...).
 *
 * Waypoint navigation (...).
 * 
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Matteo Petrillo, WVU - madepetrillo@mix.wvu.edu
 * \date June 01, 2020
 */

#include "waypoint_nav/waypoint_nav.h"

WaypointNavigation::WaypointNavigation(ros::NodeHandle & nh)
    : nh_(nh)
{
    // Subscriber
    subOdom = nh_.subscribe("dead_reckoning/odometry", 1, &WaypointNavigation::odometryCallback, this);
    subOdomTruth = nh_.subscribe("odometry/truth", 1, &WaypointNavigation::odometryTruthCallback, this);
    subGoal = nh_.subscribe("/volatile_pos", 1, &WaypointNavigation::goalCallback, this);
    subAvoidDirection = nh_.subscribe("direction", 100, &WaypointNavigation::avoidObstacleCallback, this);

    // Publisher
    pubCmdVel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
}

void WaypointNavigation::odometryTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    localPos_curr_ = msg->pose.pose;
    localVel_curr_ = msg->twist.twist;
    // ROS_INFO_STREAM("odometryTruth"<<localPos_curr_);
    
}

void WaypointNavigation::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    localPos_curr_ = msg->pose.pose;
    localVel_curr_ = msg->twist.twist;
}

void WaypointNavigation::goalCallback(const geometry_msgs::Pose& msg)
{
    goalPos_ = msg;
    // ROS_INFO_STREAM("Goal pos"<<goalPos_);
}

void WaypointNavigation::avoidObstacleCallback(const std_msgs::Float64::ConstPtr& msg)
{
   if (msg->data > 0 && msg->data < 0.758 ) 
   {
        rr_ = true;
        ll_ = false;
        avoid_angle_ = 0.758;
    }
    else if(msg->data > 0 && msg->data > 0.758) {
        ll_ = true;
        rr_ = false;
        avoid_angle_ = msg->data;
    }
    else if(msg->data < 0 && msg->data > -0.758) {
        ll_ = true;
        rr_ = false;
        avoid_angle_ = -0.758;
    }
    else if(msg->data < 0 && msg->data < -0.758) {
        ll_ = true;
        rr_ = false;
        avoid_angle_ = msg->data;
    }
    else if(msg->data == 0) {
        rr_ = false;
        ll_ = false;
    }
}

void WaypointNavigation::commandVelocity()
{
    geometry_msgs::Twist cmd_vel;
    double ex, ey, et, ephi, phi_d;
    double vd, vFB, ev;
    double roll, pitch, yaw;

    tf::Quaternion q(
                    localPos_curr_.orientation.x,
                    localPos_curr_.orientation.y,
                    localPos_curr_.orientation.z,
                    localPos_curr_.orientation.w
                    );

    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    double pf;

    ex = goalPos_.position.x - localPos_curr_.position.x;
    ey = goalPos_.position.y - localPos_curr_.position.y;
    pf = std::hypot(ex, ey);

    vd = 1;
    if (vd<1) 
    {
        vd=2;
    }
    vFB = std::hypot(localVel_curr_.linear.x, localVel_curr_.linear.y);
    ev = vd-vFB;

    if (pf > 0.2) 
    {
        phi_d = atan2(ey,ex);
        ephi = phi_d - yaw;
        et = atan2(sin(ephi), cos(ephi));

        if (signbit(et) == 1) 
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.2;
            // ROS_INFO_STREAM("Rotate in place");
        }
        else
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = -0.2;
            // ROS_INFO_STREAM("Rotate in place");
        }
        if (abs(et) < 0.07) 
        {
            cmd_vel.linear.x = ev;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = Kp_yaw_*et;
            // ROS_INFO_STREAM("Double Ackermann");
        }
        if (rr_ == true && ll_ == false)
        {
            cmd_vel.linear.x = ev*cos(avoid_angle_);
            cmd_vel.linear.y = ev*sin(avoid_angle_);
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;
            // ROS_INFO_STREAM("Crab motion");
        }
        if (rr_ == false && ll_ == true)
        {
            cmd_vel.linear.x = ev*cos(avoid_angle_);
            cmd_vel.linear.y = ev*sin(avoid_angle_);
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;
            // ROS_INFO_STREAM("Crab motion");
        }
    }
    else
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    ROS_INFO_STREAM("Commanded vel" << cmd_vel);
    pubCmdVel.publish(cmd_vel);
}

/*!
 * \brief Creates and runs the waypoint_nav node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_nav");
    ros::NodeHandle nh("");
    
    ros::Rate rate(200.0);
    
    ROS_INFO("Waypoint Nav Node initializing...");
    WaypointNavigation waypoint_nav(nh);


    while(ros::ok()) 
    {
        waypoint_nav.commandVelocity();
        ros::spinOnce();
        rate.sleep();
    }
                
    return 0;
}