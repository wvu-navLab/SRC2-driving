#include "waypoint_nav/imu_rpy_noise.h"


ImuRPYNoise::ImuRPYNoise(ros::NodeHandle & nh)
	: nh_(nh)
{
	subImu = nh_.subscribe("/scout_1/imu",10,&ImuRPYNoise::imuCallback,this);

    pubRoll = nh_.advertise<std_msgs::Float64>("/scout_1/imu_processed/roll",1);
    pubPitch = nh_.advertise<std_msgs::Float64>("/scout_1/imu_processed/pitch",1);
    pubYaw = nh_.advertise<std_msgs::Float64>("/scout_1/imu_processed/yaw",1);
}


void ImuRPYNoise::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

    tf::Quaternion q(
                    msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w
                    );

    tf::Matrix3x3 m(q);

    m.getRPY(roll_, pitch_, yaw_);
    v_roll_.push_back(roll_);
    v_pitch_.push_back(pitch_);
    v_yaw_.push_back(yaw_);

    std_msgs::Float64 roll, pitch, yaw;
    roll.data = roll_;    
    pitch.data = pitch_;
    yaw.data = yaw_;

    
    pubRoll.publish(roll);
    pubPitch.publish(pitch);
    pubYaw.publish(yaw);

    std::vector<double> v;

    double sum, mean, sq_sum, stdev;
    // Roll 
    v = v_roll_;
    sum = std::accumulate(v.begin(), v.end(), 0.0);
    mean = sum / v.size();
    sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size() - mean * mean);
    ROS_INFO_STREAM("ROLL mean: "<< mean <<", std deviation: " << stdev << ", data points: "<< v.size());

    // Pitch 
    v = v_pitch_;
    sum = std::accumulate(v.begin(), v.end(), 0.0);
    mean = sum / v.size();
    sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size() - mean * mean);
    ROS_INFO_STREAM("PITCH mean: "<< mean <<", std deviation: " << stdev << ", data points: "<< v.size());

    // Yaw
    v = v_yaw_;
    sum = std::accumulate(v.begin(), v.end(), 0.0);
    mean = sum / v.size();
    sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size() - mean * mean);

    ROS_INFO_STREAM("YAW mean: "<< mean <<", std deviation: " << stdev << ", data points: "<< v.size());
}






int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_rpy_noise");
	ros::NodeHandle nh("");

	ROS_INFO(" IMU RPY Noise measurements started ");

	ImuRPYNoise imu_rpy_noise(nh);

	ros::spin();

	return 0;
}