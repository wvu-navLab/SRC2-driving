#include "motion_control/cmd_wheels.h"

/*--------------------------------------------------------------------
 * WheelsCommander()
 * Constructor.
 *------------------------------------------------------------------*/

WheelsCommander::WheelsCommander(ros::NodeHandle & nh)
: nh_(nh)
{      
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  sub_ = nh_.subscribe("control/drive/wheel_velocities", 1000, &WheelsCommander::messageCallback, this);

  // Node publishes individual joint positions
  pub1_ = nh_.advertise<std_msgs::Float64>("front_right_wheel/drive/command/velocity", 1000);
  pub2_ = nh_.advertise<std_msgs::Float64>("back_right_wheel/drive/command/velocity", 1000);
  pub3_ = nh_.advertise<std_msgs::Float64>("front_left_wheel/drive/command/velocity", 1000);
  pub4_ = nh_.advertise<std_msgs::Float64>("back_left_wheel/drive/command/velocity", 1000);
} // end WheelsCommander()

/*--------------------------------------------------------------------
 * ~WheelsCommander()
 * Destructor.
 *------------------------------------------------------------------*/

WheelsCommander::~WheelsCommander()
{
} // end ~WheelsCommander()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void WheelsCommander::messageCallback(const motion_control::WheelGroup::ConstPtr &msg)
{
  w1.data = msg->w1;
  w2.data = msg->w2;
  w3.data = msg->w3;
  w4.data = msg->w4;

  pub1_.publish(w1);
  pub2_.publish(w2);
  pub3_.publish(w3);
  pub4_.publish(w4);

} // end publishCallback()

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cmd_wheels");
  ros::NodeHandle nh("");

  // Create a new WheelsCommander object.
  WheelsCommander wheels_commander(nh);
  
  ros::spin();
  
  return 0;

} // end main()
