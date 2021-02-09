#ifndef CMD_SENSOR_H
#define CMD_SENSOR_H

// ROS includes.
#include "ros/ros.h"
#include "std_msgs/Float64.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "motion_control/SensorGroup.h"

class SensorCommander
{
public:
  //! Constructor.
  SensorCommander(ros::NodeHandle & nh);

  //! Destructor.
  ~SensorCommander();

  //! Callback function for subscriber.
  void messageCallback(const motion_control::SensorGroup::ConstPtr &msg);

  //! Joint angles
  std_msgs::Float64 j1;
  std_msgs::Float64 j2;

private:
  // Node Handle
  ros::NodeHandle & nh_;
  
  ros::Publisher pub1_;
  ros::Publisher pub2_; 
  ros::Subscriber sub_;  
};

#endif // CMD_SENSOR_H
