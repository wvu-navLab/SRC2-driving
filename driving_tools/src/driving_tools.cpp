/*!
 * \driving_tools.cpp
 * \brief Driving Tools for SRC2 Rovers
 *
 * Manipulation creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
 * \date May 04, 2020
 */

#include <driving_tools/driving_tools.h>

DrivingTools::DrivingTools()
{
  // Publishers
  pubWheelVels = nh_.advertise<motion_control::WheelGroup>("control/drive/wheel_velocities", 1000);
  pubSteeringAngles = nh_.advertise<motion_control::SteeringGroup>("control/steering/joint_angles", 1000);
  pubDrivingMode = nh_.advertise<std_msgs::Int64>("driving/driving_mode", 1);

  // Service Servers
  stopServer = nh_.advertiseService("driving/stop", &DrivingTools::Stop, this);
  rotateInPlaceServer = nh_.advertiseService("driving/rotate_in_place", &DrivingTools::RotateInPlace, this);
  circBaseStationServer = nh_.advertiseService("driving/circ_base_station", &DrivingTools::CirculateBaseStation, this);
  moveForwardServer = nh_.advertiseService("driving/move_forward", &DrivingTools::MoveForward, this);
  turnWheelsSidewaysServer = nh_.advertiseService("driving/turn_wheels_sideways", &DrivingTools::TurnWheelsSideways, this);
  moveSidewaysServer = nh_.advertiseService("driving/move_sideways", &DrivingTools::MoveSideways, this);
}

bool DrivingTools::RotateInPlace(driving_tools::RotateInPlace::Request &req, driving_tools::RotateInPlace::Response &res)
{
  // ROS_INFO("Rotate-in-Place Service requested.");

  driving_mode_ = TIPP_MODE;
  // Rotate wheels at 45 deg
  s1 = MAX_STEERING_ANGLE/2;
  s2 = -MAX_STEERING_ANGLE/2;
  s3 = -MAX_STEERING_ANGLE/2;
  s4 = MAX_STEERING_ANGLE/2;
  s.s1 = s1;
  s.s2 = s2;
  s.s3 = s3;
  s.s4 = s4;

  double throttle = req.throttle;
  // Grab the directional data
  w1 = - throttle * MAX_MOTOR_EFFORT;
  w2 = - throttle * MAX_MOTOR_EFFORT;
  w3 = throttle * MAX_MOTOR_EFFORT;
  w4 = throttle * MAX_MOTOR_EFFORT;

  w.w1 = w1;
  w.w2 = w2;
  w.w3 = w3;
  w.w4 = w4;

  std_msgs::Int64 mode;
  mode.data = driving_mode_;
  pubDrivingMode.publish(mode);
  pubWheelVels.publish(w);
  pubSteeringAngles.publish(s);

  res.success = true;
  return true;
}

bool DrivingTools::CirculateBaseStation(driving_tools::CirculateBaseStation::Request  &req, driving_tools::CirculateBaseStation::Response &res)
{
  // ROS_INFO("Circulate Base Station Service requested.");
  driving_mode_ = CRAB_MODE;
  double R = req.radius;
  double alpha_i = atan2(R, SEMI_CHASSIS_WIDTH);
  double alpha_o = atan2(R+SEMI_CHASSIS_LENGTH, SEMI_CHASSIS_WIDTH);
  // Rotate wheels at 45 deg
  s1 = - alpha_i;
  s2 = - alpha_o;
  s3 = alpha_i;
  s4 = alpha_o;

  s.s1 = s1;
  s.s2 = s2;
  s.s3 = s3;
  s.s4 = s4;

  double throttle = req.throttle;
  // Grab the directional data
  w1 = throttle * MAX_MOTOR_EFFORT * (R+SEMI_CHASSIS_LENGTH)/R;
  w2 = throttle * MAX_MOTOR_EFFORT * (R-SEMI_CHASSIS_LENGTH)/R;
  w3 = - throttle * MAX_MOTOR_EFFORT * (R-SEMI_CHASSIS_LENGTH)/R;
  w4 = - throttle * MAX_MOTOR_EFFORT * (R+SEMI_CHASSIS_LENGTH)/R;

  w.w1 = w1;
  w.w2 = w2;
  w.w3 = w3;
  w.w4 = w4;

  std_msgs::Int64 mode;
  mode.data = driving_mode_;
  pubDrivingMode.publish(mode);
  pubWheelVels.publish(w);
  pubSteeringAngles.publish(s);
  res.success = true;
  return true;
}

bool DrivingTools::MoveForward(driving_tools::MoveForward::Request  &req, driving_tools::MoveForward::Response &res)
{
  driving_mode_ = DACK_MODE;

  // ROS_INFO("Move Forward Service requested.");
  // Rotate wheels at 45 deg
  s1 = 0;
  s2 = 0;
  s3 = 0;
  s4 = 0;

  s.s1 = s1;
  s.s2 = s2;
  s.s3 = s3;
  s.s4 = s4;

  double throttle = req.throttle;
  // Grab the directional data
  w1 = throttle * MAX_MOTOR_EFFORT;
  w2 = throttle * MAX_MOTOR_EFFORT;
  w3 = throttle * MAX_MOTOR_EFFORT;
  w4 = throttle * MAX_MOTOR_EFFORT;

  w.w1 = w1;
  w.w2 = w2;
  w.w3 = w3;
  w.w4 = w4;

  std_msgs::Int64 mode;
  mode.data = driving_mode_;
  pubDrivingMode.publish(mode);
  pubWheelVels.publish(w);
  pubSteeringAngles.publish(s);

  res.success = true;
  return true;
}


bool DrivingTools::TurnWheelsSideways(driving_tools::TurnWheelsSideways::Request  &req, driving_tools::TurnWheelsSideways::Response &res)
{
  driving_mode_ = CRAB_MODE;

  // ROS_INFO("Move Forward Service requested.");
  // Rotate wheels at 45 deg
  s1 = -MAX_STEERING_ANGLE;
  s2 = -MAX_STEERING_ANGLE;
  s3 = -MAX_STEERING_ANGLE;
  s4 = -MAX_STEERING_ANGLE;

  s.s1 = s1;
  s.s2 = s2;
  s.s3 = s3;
  s.s4 = s4;

  std_msgs::Int64 mode;
  mode.data = driving_mode_;
  pubDrivingMode.publish(mode);
  pubSteeringAngles.publish(s);

  res.success = true;
  return true;
}

bool DrivingTools::MoveSideways(driving_tools::MoveSideways::Request  &req, driving_tools::MoveSideways::Response &res)
{
  driving_mode_ = DACK_MODE;

  // ROS_INFO("Move Forward Service requested.");
  // Rotate wheels at 45 deg
  s1 = -MAX_STEERING_ANGLE;
  s2 = -MAX_STEERING_ANGLE;
  s3 = -MAX_STEERING_ANGLE;
  s4 = -MAX_STEERING_ANGLE;

  s.s1 = s1;
  s.s2 = s2;
  s.s3 = s3;
  s.s4 = s4;

  double throttle = req.throttle;
  // Grab the directional data
  w1 = throttle * MAX_MOTOR_EFFORT;
  w2 = throttle * MAX_MOTOR_EFFORT;
  w3 = throttle * MAX_MOTOR_EFFORT;
  w4 = throttle * MAX_MOTOR_EFFORT;

  w.w1 = w1;
  w.w2 = w2;
  w.w3 = w3;
  w.w4 = w4;

  std_msgs::Int64 mode;
  mode.data = driving_mode_;
  pubDrivingMode.publish(mode);
  pubWheelVels.publish(w);
  pubSteeringAngles.publish(s);

  res.success = true;
  return true;
}

bool DrivingTools::Stop(driving_tools::Stop::Request  &req, driving_tools::Stop::Response &res)
{
  driving_mode_ = STOP_MODE;
  if (req.enableStop)
  {
    // ROS_INFO("Stop Service requested.");

    s.s1 = 0;
    s.s2 = 0;
    s.s3 = 0;
    s.s4 = 0;

    w.w1 = 0;
    w.w2 = 0;
    w.w3 = 0;
    w.w4 = 0;

    std_msgs::Int64 mode;
    mode.data = driving_mode_;
    pubDrivingMode.publish(mode);
    pubWheelVels.publish(w);
    pubSteeringAngles.publish(s);

    res.success = true;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driving_tools");

  DrivingTools driving_tools;
  ROS_INFO("Driving tools node initialized.");
  
  ros::spin();

  return 0;
}
