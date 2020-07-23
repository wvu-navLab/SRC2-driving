#include "motion_control/cmd_motors.h"

/*--------------------------------------------------------------------
 * MotorsCommander()
 * Constructor.
 *------------------------------------------------------------------*/

MotorsCommander::MotorsCommander(ros::NodeHandle & nh)
: nh_(nh)
{      
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  sub_ = nh_.subscribe("driving/motor_efforts", 1000, &MotorsCommander::messageCallback, this);

  // Node publishes individual joint positions
  pub1_ = nh_.advertise<std_msgs::Float64>("fr_wheel_controller/command", 1000);
  pub2_ = nh_.advertise<std_msgs::Float64>("br_wheel_controller/command", 1000);
  pub3_ = nh_.advertise<std_msgs::Float64>("fl_wheel_controller/command", 1000);
  pub4_ = nh_.advertise<std_msgs::Float64>("bl_wheel_controller/command", 1000);
} // end MotorsCommander()

/*--------------------------------------------------------------------
 * ~MotorsCommander()
 * Destructor.
 *------------------------------------------------------------------*/

MotorsCommander::~MotorsCommander()
{
} // end ~MotorsCommander()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void MotorsCommander::messageCallback(const motion_control::MotorGroup::ConstPtr &msg)
{
  m1.data = msg->m1;
  m2.data = msg->m2;
  m3.data = msg->m3;
  m4.data = msg->m4;

  pub1_.publish(m1);
  pub2_.publish(m2);
  pub3_.publish(m3);
  pub4_.publish(m4);

} // end publishCallback()

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cmd_motors");
  ros::NodeHandle nh("");

  // Create a new MotorsCommander object.
  MotorsCommander motors_commander(nh);
  
  ros::spin();
  
  return 0;

} // end main()
