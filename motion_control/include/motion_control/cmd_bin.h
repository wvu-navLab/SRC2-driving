#ifndef CMD_BIN_H
#define CMD_BIN_H

// ROS includes.
#include "ros/ros.h"
#include "std_msgs/Float64.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "motion_control/BinJoint.h"

class BinCommander
{
public:
  //! Constructor.
  BinCommander(ros::NodeHandle & nh);

  //! Destructor.
  ~BinCommander();

  //! Callback function for subscriber.
  void messageCallback(const motion_control::BinJoint::ConstPtr &msg);

  //! Joint angles
  std_msgs::Float64 b1;

private:
  // Node Handle
  ros::NodeHandle & nh_;
  
  ros::Publisher pub_;
  ros::Subscriber sub_;  
};

#endif // CMD_BIN_H
