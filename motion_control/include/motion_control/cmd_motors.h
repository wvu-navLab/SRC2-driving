#ifndef CMD_MOTORS_H
#define CMD_MOTORS_H

// ROS includes.
#include "ros/ros.h"
#include "std_msgs/Float64.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "motion_control/MotorGroup.h"


class MotorsCommander
{
public:
  //! Constructor.
  MotorsCommander(ros::NodeHandle & nh);

  //! Destructor.
  ~MotorsCommander();

  //! Callback function for subscriber.
  void messageCallback(const motion_control::MotorGroup::ConstPtr &msg);

  //! Joint angles
  std_msgs::Float64 m1, m2, m3, m4;

private:
  // Node Handle
  ros::NodeHandle & nh_;
  
  ros::Publisher pub1_, pub2_, pub3_, pub4_;
  ros::Subscriber sub_;  
};

#endif // CMD_MOTORS_H
