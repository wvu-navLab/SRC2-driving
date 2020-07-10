/*!
 * \waypoint_nav.h
 * \brief waypoint_nav (...).
 *
 * Waypoint navigation (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date June 01, 2020
 */

#ifndef IMU_RPY_NOISE_H
#define IMU_RPY_NOISE_H


// Include cpp important headers
#include <math.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <termios.h>
#include <vector>
#include <numeric>

// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>

class ImuRPYNoise
{
public:

    ImuRPYNoise(ros::NodeHandle & nh);

private:

    ros::NodeHandle & nh_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    ros::Subscriber subImu;

    ros::Publisher pubRoll;
    ros::Publisher pubPitch;
    ros::Publisher pubYaw;

    double roll_, pitch_, yaw_;
    std::vector<double> v_roll_, v_pitch_, v_yaw_;
 };

#endif //IMU_RPY_NOISE_H