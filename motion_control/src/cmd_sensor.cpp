#include "motion_control/cmd_sensor.h"

/*--------------------------------------------------------------------
 * SensorCommander()
 * Constructor.
 *------------------------------------------------------------------*/

SensorCommander::SensorCommander(ros::NodeHandle & nh)
: nh_(nh)
{    
 
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  sub_ = nh_.subscribe("control/sensor/joint_angles", 10, &SensorCommander::messageCallback, this);

  // Node publishes individual joint positions
  pub1_ = nh_.advertise<std_msgs::Float64>("sensor/pitch/command/position", 10);
  pub2_ = nh_.advertise<std_msgs::Float64>("sensor/yaw/command/position", 10);
} // end SensorCommander()

/*--------------------------------------------------------------------
 * ~SensorCommander()
 * Destructor.
 *------------------------------------------------------------------*/

SensorCommander::~SensorCommander()
{
} // end ~SensorCommander()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void SensorCommander::messageCallback(const motion_control::SensorGroup::ConstPtr &msg)
{
  j1.data = msg->j1;
  j2.data = msg->j2;

  pub1_.publish(j1);
  pub2_.publish(j2);

} // end publishCallback()

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cmd_sensor");
  ros::NodeHandle nh("");

  // Create a new SensorCommander object.
  SensorCommander sensor_commander(nh);
  
  ros::spin();
  
  return 0;

} // end main()
