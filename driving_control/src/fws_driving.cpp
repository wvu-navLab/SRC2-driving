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

    ROS_INFO_STREAM("joint_state_controller/publish_rate" << joint_state_controller_publish_rate_);
    ROS_INFO_STREAM("/max_velocity" << max_velocity_);
    ROS_INFO_STREAM("/wheel_radius" << wheel_radius_);
    ROS_INFO_STREAM("/wheel_separation_length" << wheel_separation_length_);
    ROS_INFO_STREAM("/wheel_separation_width" << wheel_separation_width_);

    clientStop = nh_.serviceClient<driving_tools::Stop>("stop");

    // Subscriber
    subCmdVel = nh_.subscribe("cmd_vel", 1, &FourWheelSteeringDriving::cmdVelCallback, this);

    // Publisher
    pubWheelVelCmds = nh_.advertise<driving_control::WheelVelCmds>("wheel_vel_cmds", 1000);
    pubSteeringAngles = nh_.advertise<motion_control::SteeringGroup>("steering_joint_angles", 1000);


    ROS_INFO_STREAM("cmd stamp:"<<command_struct_twist_.stamp);
}

void FourWheelSteeringDriving::starting(const ros::Time& time)
{
    ROS_INFO("Starting.");
    brake();
}

void FourWheelSteeringDriving::stopping(const ros::Time& time)
{    
    ROS_INFO("Stopping.");
    brake();
}

void FourWheelSteeringDriving::brake()
{
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
    cmd = &curr_cmd_twist;

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
        bool vx_flag = (fabs(curr_cmd_twist.lin_x) > 0.001);
        bool vy_flag = (fabs(curr_cmd_twist.lin_y) > 0.001);
        bool wz_flag = (fabs(curr_cmd_twist.ang) > 0.001);

        if (wz_flag && !vx_flag && !vy_flag) // Turn-in-place Driving
        {
            vel_right_front = curr_cmd_twist.ang * std::hypot(wheel_separation_length_,steering_track) / (2 * wheel_radius_);
            vel_right_rear = vel_right_front;
            vel_left_front  = - vel_right_front;
            vel_left_rear = - vel_right_front;

            front_right_steering = atan(wheel_separation_length_/steering_track);
            rear_right_steering = -front_right_steering;
            front_left_steering = -front_right_steering;
            rear_left_steering = front_right_steering;
        } 
        else if (vx_flag && vy_flag && !wz_flag) // All-Wheel Driving (crab motion)
        {
            const double sign_vx = copysign(1.0, curr_cmd_twist.lin_x);
            vel_right_front = sign_vx * std::hypot(curr_cmd_twist.lin_y,curr_cmd_twist.lin_x)/ wheel_radius_;
            vel_right_rear = vel_right_front;
            vel_left_front = vel_right_front;
            vel_left_rear = vel_right_front;

            // const double sign_vy = copysign(1.0, curr_cmd_twist.lin_y);
            front_right_steering = atan(curr_cmd_twist.lin_y/ curr_cmd_twist.lin_x);
            rear_right_steering = front_right_steering;
            front_left_steering = front_right_steering;
            rear_left_steering = front_right_steering;
        }
        else // Double Ackermann Driving
        {
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
    }

    // Set wheels velocities:
    driving_control::WheelVelCmds w;
    w.w1 = vel_right_front; //fr_wheel_joint
    w.w2 = vel_right_rear; //br_wheel_joint
    w.w3 = vel_left_front; //fl_wheel_joint
    w.w4 = vel_left_rear; //bl_wheel_joint
    pubWheelVelCmds.publish(w);

    // ROS_INFO_STREAM("Wheel velocities commanded" << w);

    /// TODO check limits to not apply the same steering on right and left when saturated !
    motion_control::SteeringGroup s;
    s.s1 = front_right_steering; //fr_steering_joint
    s.s2 = rear_right_steering; //br_steering_joint
    s.s3 = front_left_steering; //fl_steering_joint
    s.s4 = rear_left_steering; //bl_steering_joint
    pubSteeringAngles.publish(s);

    // ROS_INFO_STREAM("Steering angles commanded" << s);
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

    // This should be set in launch files as well
    nh.setParam("/use_sim_time", true);
    
    ROS_INFO("Four Wheel Steering Driving Node initializing...");
    FourWheelSteeringDriving fws_driving(nh);

    ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time internal_time(0);
    const ros::Duration dt = fws_driving.getPeriod();

    ROS_WARN_STREAM("Period: " << dt.toSec());

    // fws_driving.starting(internal_time);

    while(ros::ok()) 
    {
        fws_driving.updateCommand(internal_time, dt);

        rosgraph_msgs::Clock clock;
        clock.clock = ros::Time(internal_time);
        clock_publisher.publish(clock);
        internal_time += dt;
    }

    spinner.stop();

    fws_driving.stopping(internal_time);

    return 0;
}