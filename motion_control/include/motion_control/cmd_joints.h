#ifndef CMD_JOINTS_H
#define CMD_JOINTS_H

// ROS includes.
#include "ros/ros.h"
#include "std_msgs/Float64.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "motion_control/JointGroup.h"


class JointsCommander
{
public:
  //! Constructor.
  JointsCommander(ros::NodeHandle & nh);

  //! Destructor.
  ~JointsCommander();

  //! Callback function for subscriber.
  void messageCallback(const motion_control::JointGroup::ConstPtr &msg);

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

#endif // CMD_JOINTS_H
