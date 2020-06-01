/*!
 * \fws_driving.cpp
 * \brief 4ws_driving (...).
 *
 * Four wheel steering driving (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
 * \author Chris Tatsch, WVU - ca0055@wvu.mix.edu
 * \date June 01, 2020
 */

#include "driving_control/fws_driving.h"

FourWheelSteeringDriving::FourWheelSteeringDriving(ros::NodeHandle & nh)
    : nh_(nh)
    , command_struct_twist_()
    , wheel_separation_width_(0.0)
    , wheel_steering_y_offset_(0.0)
    , wheel_radius_(0.0)
    , wheel_separation_length_(0.0)
    , cmd_vel_timeout_(0.5)
    , enable_twist_cmd_(true)
{

    name_ = "fws_driving";

    nh_.getParam("joint_state_controller/publish_rate", joint_state_controller_publish_rate_);
    nh_.getParam("/max_velocity", max_velocity_);
    nh_.getParam("/wheel_radius", wheel_radius_);
    nh_.getParam("/wheel_separation_length", wheel_separation_length_);
    nh_.getParam("/wheel_separation_width", wheel_separation_width_);

    clientStop = nh_.serviceClient<driving_tools::Stop>("stop");

    // Subscriber
    subCmdVel = nh_.subscribe("cmd_vel", 1, &FourWheelSteeringDriving::cmdVelCallback, this);

    // Publisher
    pubWheelVelCmds = nh_.advertise<driving_control::WheelVelCmds>("wheel_vel_cmds", 1000);
    pubSteeringAngles = nh_.advertise<motion_control::SteeringGroup>("steering_joint_angles", 1000);
}


void FourWheelSteeringDriving::update(const ros::Time& time, const ros::Duration& period)
{
    updateCommand(time, period);
}

void FourWheelSteeringDriving::starting(const ros::Time& time)
{
    brake();
}

void FourWheelSteeringDriving::stopping(const ros::Time& /*time*/)
{
    brake();
}

void FourWheelSteeringDriving::brake()
{
    ROS_INFO("Stopping.");
    driving_tools::Stop srv;
    srv.request.enableStop = true;
    bool success = clientStop.call(srv);
}

void FourWheelSteeringDriving::cmdVelCallback(const geometry_msgs::Twist& command)
{
    if(std::isnan(command.angular.z) || std::isnan(command.linear.x))
    {
    ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
    return;
    }
    command_struct_twist_.ang   = command.angular.z;
    command_struct_twist_.lin_x   = command.linear.x;
    command_struct_twist_.lin_y   = command.linear.y;
    command_struct_twist_.stamp = ros::Time::now();
    command_twist_.writeFromNonRT (command_struct_twist_);
    ROS_DEBUG_STREAM_NAMED(name_,
                            "Added values to command. "
                            << "Ang: "   << command_struct_twist_.ang << ", "
                            << "Lin x: " << command_struct_twist_.lin_x << ", "
                            << "Lin y: " << command_struct_twist_.lin_y << ", "
                            << "Stamp: " << command_struct_twist_.stamp);
}


void FourWheelSteeringDriving::updateCommand(const ros::Time& time, const ros::Duration& period)
{
    // Retreive current velocity command and time step:
    Command* cmd;
    CommandTwist curr_cmd_twist = *(command_twist_.readFromRT());

    const double dt = (time - cmd->stamp).toSec();
    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd_twist.lin_x = 0.0;
      curr_cmd_twist.lin_y = 0.0;
      curr_cmd_twist.ang = 0.0;
    }

    const double cmd_dt(period.toSec());
    const double steering_track = wheel_separation_width_-2*wheel_steering_y_offset_;

    double vel_left_front = 0, vel_right_front = 0;
    double vel_left_rear = 0, vel_right_rear = 0;
    double front_left_steering = 0, front_right_steering = 0;
    double rear_left_steering = 0, rear_right_steering = 0;

    if(enable_twist_cmd_ == true)
    {
      // Limit velocities and accelerations:
      limiter_lin_.limit(curr_cmd_twist.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
      limiter_ang_.limit(curr_cmd_twist.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);
      last1_cmd_ = last0_cmd_;
      last0_cmd_ = curr_cmd_twist;

      // Compute wheels velocities:
      if(fabs(curr_cmd_twist.lin_x) > 0.001)
      {
        const double vel_steering_offset = (curr_cmd_twist.ang*wheel_steering_y_offset_)/wheel_radius_;
        const double sign = copysign(1.0, curr_cmd_twist.lin_x);
        vel_left_front  = sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),
                                            (wheel_separation_length_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                          - vel_steering_offset;
        vel_right_front = sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),
                                            (wheel_separation_length_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                          + vel_steering_offset;
        vel_left_rear = sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),
                                          (wheel_separation_length_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                        - vel_steering_offset;
        vel_right_rear = sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),
                                           (wheel_separation_length_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                         + vel_steering_offset;
      }

      // Compute steering angles
      if(fabs(2.0*curr_cmd_twist.lin_x) > fabs(curr_cmd_twist.ang*steering_track))
      {
        front_left_steering = atan(curr_cmd_twist.ang*wheel_separation_length_ /
                                    (2.0*curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track));
        front_right_steering = atan(curr_cmd_twist.ang*wheel_separation_length_ /
                                     (2.0*curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track));
      }
      else if(fabs(curr_cmd_twist.lin_x) > 0.001)
      {
        front_left_steering = copysign(M_PI_2, curr_cmd_twist.ang);
        front_right_steering = copysign(M_PI_2, curr_cmd_twist.ang);
      }
      rear_left_steering = -front_left_steering;
      rear_right_steering = -front_right_steering;
    }

    ROS_DEBUG_STREAM_THROTTLE(1, "vel_left_rear "<<vel_left_rear<<" front_right_steering "<<front_right_steering);

    // Set wheels velocities:
    driving_control::WheelVelCmds w;
    w.w1 = vel_left_front;
    w.w2 = vel_right_front;
    w.w3 = vel_left_rear;
    w.w4 = vel_right_rear;
    pubWheelVelCmds.publish(w);

    /// TODO check limits to not apply the same steering on right and left when saturated !
    motion_control::SteeringGroup s;
    s.s1 = front_left_steering;
    s.s2 = front_right_steering;
    s.s3 = rear_left_steering;
    s.s4 = rear_right_steering;
    pubSteeringAngles.publish(s);
}

/*!
 * \brief Creates and runs the fws_driving node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fws_driving");
    ros::NodeHandle nh("");
    ros::Rate rate(50);

    ROS_INFO("Find Rover Node initializing...");
    FourWheelSteeringDriving fws_driving(nh);

    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}