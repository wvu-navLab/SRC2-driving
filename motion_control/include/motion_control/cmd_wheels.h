#ifndef CMD_WHEELS_H
#define CMD_WHEELS_H

// ROS includes.
#include "ros/ros.h"
#include "std_msgs/Float64.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "motion_control/WheelGroup.h"


class WheelsCommander
{
public:
  //! Constructor.
  WheelsCommander(ros::NodeHandle & nh);

  //! Destructor.
  ~WheelsCommander();

  //! Callback function for subscriber.
  void messageCallback(const motion_control::WheelGroup::ConstPtr &msg);

  //! Joint angles
  std_msgs::Float64 w1, w2, w3, w4;

private:
  // Node Handle
  ros::NodeHandle & nh_;
  
  ros::Publisher pub1_, pub2_, pub3_, pub4_;
  ros::Subscriber sub_;  
};

#endif // CMD_WHEELS_H
