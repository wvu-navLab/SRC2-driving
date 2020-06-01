#include "motion_control/cmd_bin.h"

/*--------------------------------------------------------------------
 * BinCommander()
 * Constructor.
 *------------------------------------------------------------------*/

BinCommander::BinCommander(ros::NodeHandle & nh)
: nh_(nh)
{     
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  sub_ = nh_.subscribe("binJointAngle", 1000, &BinCommander::messageCallback, this);

  // Node publishes individual joint positions
  pub_ = nh_.advertise<std_msgs::Float64>("bin_controller/command", 1000);
} // end BinCommander()

/*--------------------------------------------------------------------
 * ~BinCommander()
 * Destructor.
 *------------------------------------------------------------------*/

BinCommander::~BinCommander()
{
} // end ~BinCommander()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void BinCommander::messageCallback(const motion_control::BinJoint::ConstPtr &msg)
{
  b1.data = msg->b1;

  pub_.publish(b1);

} // end publishCallback()

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cmd_bin");
  ros::NodeHandle nh("");

  // Create a new BinCommander object.
  BinCommander bin_commander(nh);
  
  ros::spin();
  
  return 0;

} // end main()
