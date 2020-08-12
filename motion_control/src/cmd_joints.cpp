#include "motion_control/cmd_joints.h"

/*--------------------------------------------------------------------
 * JointsCommander()
 * Constructor.
 *------------------------------------------------------------------*/

JointsCommander::JointsCommander(ros::NodeHandle & nh)
: nh_(nh)
{    
  
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  sub_ = nh_.subscribe("arm_joint_angles", 1000, &JointsCommander::messageCallback, this);

  // Node publishes individual joint positions
  pub1_ = nh_.advertise<std_msgs::Float64>("mount_joint_controller/command", 1000);
  pub2_ = nh_.advertise<std_msgs::Float64>("basearm_joint_controller/command", 1000);
  pub3_ = nh_.advertise<std_msgs::Float64>("distalarm_joint_controller/command", 1000);
  pub4_ = nh_.advertise<std_msgs::Float64>("bucket_joint_controller/command", 1000);
} // end JointsCommander()

/*--------------------------------------------------------------------
 * ~JointsCommander()
 * Destructor.
 *------------------------------------------------------------------*/

JointsCommander::~JointsCommander()
{
} // end ~JointsCommander()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void JointsCommander::messageCallback(const motion_control::JointGroup::ConstPtr &msg)
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
  ros::init(argc, argv, "cmd_joints");
  ros::NodeHandle nh("");

  // Create a new JointsCommander object.
  JointsCommander joints_commander(nh);
  
  ros::spin();
  
  return 0;

} // end main()
