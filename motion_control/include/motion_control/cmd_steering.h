#ifndef CMD_STEERING_H
#define CMD_STEERING_H

// ROS includes.
#include "ros/ros.h"
#include "std_msgs/Float64.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "motion_control/SteeringGroup.h"


class SteeringCommander
{
public:
  //! Constructor.
  SteeringCommander(ros::NodeHandle & nh);

  //! Destructor.
  ~SteeringCommander();

  //! Callback function for subscriber.
  void messageCallback(const motion_control::SteeringGroup::ConstPtr &msg);

  //! Joint angles
  std_msgs::Float64 s1, s2, s3, s4;

private:
  // Node Handle
  ros::NodeHandle & nh_;
  
  ros::Publisher pub1_, pub2_, pub3_, pub4_;
  ros::Subscriber sub_;  
};

#endif // CMD_STEERING_H
