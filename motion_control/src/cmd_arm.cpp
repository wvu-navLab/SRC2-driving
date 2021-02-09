#include "motion_control/cmd_arm.h"

/*--------------------------------------------------------------------
 * ArmCommander()
 * Constructor.
 *------------------------------------------------------------------*/

ArmCommander::ArmCommander(ros::NodeHandle & nh)
: nh_(nh)
{

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  sub_ = nh_.subscribe("control/arm/joint_angles", 1000, &ArmCommander::messageCallback, this);

  // Node publishes individual joint positions
  pub1_ = nh_.advertise<std_msgs::Float64>("arm/shoulder_yaw/command/position", 1000);
  pub2_ = nh_.advertise<std_msgs::Float64>("arm/shoulder_pitch/command/position", 1000);
  pub3_ = nh_.advertise<std_msgs::Float64>("arm/elbow_pitch/command/position", 1000);
  pub4_ = nh_.advertise<std_msgs::Float64>("arm/wrist_pitch/command/position", 1000);
} // end ArmCommander()

/*--------------------------------------------------------------------
 * ~ArmCommander()
 * Destructor.
 *------------------------------------------------------------------*/

ArmCommander::~ArmCommander()
{
} // end ~ArmCommander()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void ArmCommander::messageCallback(const motion_control::ArmGroup::ConstPtr &msg)
{
  q1.data = msg->q1;
  q2.data = msg->q2;
  q3.data = msg->q3;
  q4.data = msg->q4;

  // // Note that these are only set to INFO so they will print to a terminal for example purposes.
  // // Typically, they should be DEBUG.
  // ROS_INFO("Goal Joint Angle 1 is ", msg->q1);
  // ROS_INFO("Goal Joint Angle 2 is ", msg->q2);
  // ROS_INFO("Goal Joint Angle 3 is ", msg->q3);
  // ROS_INFO("Goal Joint Angle 4 is ", msg->q4);

  pub1_.publish(q1);
  pub2_.publish(q2);
  pub3_.publish(q3);
  pub4_.publish(q4);

} // end publishCallback()

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cmd_arm");
  ros::NodeHandle nh("");

  // Create a new ArmCommander object.
  ArmCommander arm_commander(nh);

  ros::spin();

  return 0;

} // end main()
