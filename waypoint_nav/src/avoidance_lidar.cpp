/*!
 * \avoidance_lidar.cpp
 * \brief Lidar Avoidance (...).
 *
 * Lidar Avoidance for waypoint navigation (...).
 * 
 * \author Matteo De Petrillo, WVU - madepetrillo@mix.wvu.edu
 * \date June 20, 2020
 */

#include <stdio.h>
#include <unistd.h>

// Include ROS headers

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <termios.h>
#include <math.h>
#include <stdint.h>



# define M_PI           3.14159265358979323846

using namespace std;

geometry_msgs::Pose pos_curr, localPos_curr;
geometry_msgs::Twist localVel_curr;

std_msgs::Float64 dir;
bool cclose = false;


void lidar_clud(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // cout << "range front "<<scan->ranges[50]<<endl;
  int c = 50;
  float right[50];
  float left[50];
  int l;
  int r;
  for (int i = 0;i<100;i++)
  {
    if (scan->ranges[i]<2.0)
    {
      cclose = true;
      for(int j=0;j<100;j++)
      {
        if(isinf(scan->ranges[c-j]) || scan->ranges[c-j]>8.0)
        {
          r=r+1;
        }
        if(isinf(scan->ranges[c+j]) || scan->ranges[c+j]>8.0)
        {
          l=l+1;
        }
      }
      // cout<<"number of rightINF "<<r<<endl;
      // cout<<"number of leftINf "<<l<<endl;
      if(r>l)
      {
        float rmax = scan->ranges[50];
        int idr=0;
        for(int s=50;s>0;s--)
        {
          if(scan->ranges[s] > rmax)
          {
            rmax = scan->ranges[s];
            idr = s;
          }
        }
        dir.data = (idr+4)*(-0.0262626260519);
        // cout<<dir.data<<endl;
        // cout<<idr<<endl;
      }
      else 
      {
        float lmax = scan->ranges[50];
        int idl=0;
        for(int sl=50;sl<100;sl++)
        {
          if(scan->ranges[sl] > lmax)
          {
            lmax = scan->ranges[sl];
            idl = sl;
          }
        }
        dir.data = (idl+4)*(0.0262626260519);
        // cout<<dir.data<<endl;
        // cout<<idl<<endl;
      }
    }
    else if(scan->ranges[i]>2.0 && cclose==false)
    {
      dir.data=0;
      // cout<<"dist > 2 meters"<<endl;
    }
    l = 0;
    r = 0;
  }
  int g = sizeof(scan->ranges)/sizeof(scan->ranges[0]);
}



int main(int argc, char **argv){

  ros::init(argc, argv, "avoidance_lidar");
  ros::NodeHandle nh("");

  ros::Subscriber lidar = nh.subscribe<sensor_msgs::LaserScan>("laser/scan",1, lidar_clud);
  ros::Publisher directionr = nh.advertise<std_msgs::Float64>("driving/direction",1000);

  ros::Rate rate(100);

  while (ros::ok()) 
  {
    directionr.publish(dir);
    cclose = false;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}