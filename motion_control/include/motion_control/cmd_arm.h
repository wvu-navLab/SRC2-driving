#ifndef CMD_ARM_H
#define CMD_ARM_H

// ROS includes.
#include "ros/ros.h"
#include "std_msgs/Float64.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "motion_control/ArmGroup.h"


class ArmCommander
{
public:
  //! Constructor.
  ArmCommander(ros::NodeHandle & nh);

  //! Destructor.
  ~ArmCommander();

  //! Callback function for subscriber.
  void messageCallback(const motion_control::ArmGroup::ConstPtr &msg);

  //! Joint angles
  std_msgs::Float64 q1;
  std_msgs::Float64 q2;
  std_msgs::Float64 q3;
  std_msgs::Float64 q4;

private:
  // Node Handle
  ros::NodeHandle & nh_;
  
  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Publisher pub3_;
  ros::Publisher pub4_;

  ros::Subscriber sub_;  
};

#endif // CMD_ARM_H
