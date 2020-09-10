/*!
 * \waypoint_nav.cpp
 * \brief waypoint_nav (...).
 *
 * Waypoint navigation (...).
 *
 * \author Matteo Petrillo, WVU - madepetrillo@mix.wvu.edu
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date June 20, 2020
 */

#include "waypoint_nav/waypoint_nav.h"

WaypointNavigation::WaypointNavigation(ros::NodeHandle & nh)
    : nh_(nh)
{
    // Subscriber
    subOdom = nh_.subscribe("localization/odometry/sensor_fusion", 10, &WaypointNavigation::odometryCallback, this);
    subSmach = nh_.subscribe("/state_machine/state", 10, &WaypointNavigation::smachCallback, this);
    subAvoidDirection = nh_.subscribe("driving/direction", 10, &WaypointNavigation::avoidObstacleCallback, this);

    // Publisher
    pubCmdVel = nh_.advertise<geometry_msgs::Twist>("driving/cmd_vel", 10);
    pubNavStatus = nh_.advertise<std_msgs::Int64>("navigation/status", 10);
    pubWaypointUnreachable = nh_.advertise<std_msgs::Bool>("/state_machine/waypoint_unreachable", 10);
    pubArrivedAtWaypoint = nh_.advertise<std_msgs::Bool>("/state_machine/arrived_at_waypoint", 10);

    // Service Servers
    srv_set_goal_ = nh_.advertiseService("navigation/set_goal", &WaypointNavigation::setGoal, this);
    srv_interrupt_ = nh_.advertiseService("navigation/interrupt", &WaypointNavigation::interrupt, this);

}

void WaypointNavigation::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (firstOdom_ == false)
    {
        firstOdom_ = true;
    }
    localPos_curr_ = msg->pose.pose;
    localVel_curr_ = msg->twist.twist;
}

bool WaypointNavigation::interrupt(waypoint_nav::Interrupt::Request &req, waypoint_nav::Interrupt::Response &res)
{
    if (req.interrupt == true)
    {
        geometry_msgs::Twist cmd_vel;

        active_ = false;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        pubCmdVel.publish(cmd_vel);
        ROS_INFO_STREAM("WAYPOINT NAV INTERRUPT");
    }
    else
    {
        active_ = true;
    }
    res.success = true;
    return true;
}

bool WaypointNavigation::setGoal(waypoint_nav::SetGoal::Request &req, waypoint_nav::SetGoal::Response &res)
{
    if (req.start == true)
    {
        if (firstGoal_ == false)
        {
            firstGoal_ = true;
        }

        goalPos_ = req.goal;
        // ROS_INFO_STREAM("New goal "<< goalPos_);
        res.success = false;
    }
    else
    {
        goalPos_ = localPos_curr_;
        res.success = false;
    }
    return true;
}

void WaypointNavigation::smachCallback(const std_msgs::Int64::ConstPtr & msg)
{
    if (msg->data == TRAV_STATE)
    {
        active_ = true;
    }
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
    std_msgs::Int64 status;
    std_msgs::Bool arrived;
    std_msgs::Bool unreachable;
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
    // ROS_INFO_STREAM("error:"<<pf);
    // ROS_INFO_STREAM("goalPos_.position"<<goalPos_);
    // ROS_INFO_STREAM("localPos_curr_.position"<<localPos_curr_);
    // ROS_INFO_STREAM("yaw: "<<yaw);

    vd = 1.5;
    // vd = 0.4;
    // if (vd<1)
    // {
    //     vd=2;
    // }
    // vFB = std::hypot(localVel_curr_.linear.x, localVel_curr_.linear.y);
    // ev = vd-vFB;
    ev = vd/2;

    if (pf > 0.2)
    {
        phi_d = atan2(ey,ex);

        // ROS_INFO_STREAM("phi_d: "<<phi_d);
        ephi = phi_d - yaw;
        et = atan2(sin(ephi), cos(ephi));
        // ROS_INFO_STREAM("et: "<<et);

        if (signbit(et) == 1)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = -0.4;
            // ROS_INFO_STREAM("Rotate in place");
        }
        else
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.4;
            // ROS_INFO_STREAM("Rotate in place");
        }
        if (abs(et) < 0.3)
        {
            cmd_vel.linear.x = vd;
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
        status.data = GOING;
        arrived.data = false;
    }
    else
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        status.data = ARRIVED;
        arrived.data = true;
    }
    // ROS_INFO_STREAM("Commanded vel" << cmd_vel);

    unreachable.data = false;

    if (firstGoal_ && firstOdom_)
    {
        pubCmdVel.publish(cmd_vel);
        pubNavStatus.publish(status);
        pubArrivedAtWaypoint.publish(arrived);
        pubWaypointUnreachable.publish(unreachable);
    }
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

    ros::Rate rate(100);

    ROS_INFO("Waypoint Nav Node initializing...");
    WaypointNavigation waypoint_nav(nh);


    while(ros::ok())
    {
        if (waypoint_nav.active_ == true)
        {
            waypoint_nav.commandVelocity();
        }
        // ROS_INFO_THROTTLE(1,"active_%d",waypoint_nav.active_);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
