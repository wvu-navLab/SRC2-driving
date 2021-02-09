#include "motion_control/cmd_steering.h"

/*--------------------------------------------------------------------
 * SteeringCommander()
 * Constructor.
 *------------------------------------------------------------------*/

SteeringCommander::SteeringCommander(ros::NodeHandle & nh)
: nh_(nh)
{    
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  sub_ = nh_.subscribe("control/steering/joint_angles", 1000, &SteeringCommander::messageCallback, this);

  // Node publishes individual joint positions
  pub1_ = nh_.advertise<std_msgs::Float64>("front_right_wheel/steer/command/position", 1000);
  pub2_ = nh_.advertise<std_msgs::Float64>("back_right_wheel/steer/command/position", 1000);
  pub3_ = nh_.advertise<std_msgs::Float64>("front_left_wheel/steer/command/position", 1000);
  pub4_ = nh_.advertise<std_msgs::Float64>("back_left_wheel/steer/command/position", 1000);
} // end SteeringCommander()

/*--------------------------------------------------------------------
 * ~SteeringCommander()
 * Destructor.
 *------------------------------------------------------------------*/

SteeringCommander::~SteeringCommander()
{
} // end ~SteeringCommander()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void SteeringCommander::messageCallback(const motion_control::SteeringGroup::ConstPtr &msg)
{
  s1.data = msg->s1;
  s2.data = msg->s2;
  s3.data = msg->s3;
  s4.data = msg->s4;

  pub1_.publish(s1);
  pub2_.publish(s2);
  pub3_.publish(s3);
  pub4_.publish(s4);

} // end publishCallback()

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cmd_steering");
  ros::NodeHandle nh("");

  // Create a new SteeringCommander object.
  SteeringCommander steering_commander(nh);
  
  ros::spin();
  
  return 0;

} // end main()
