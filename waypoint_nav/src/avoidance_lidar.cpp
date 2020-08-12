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

typedef struct {
  double i;
  double distance;
  double x;
  double y;
  double slope;
} Vector;


typedef struct {
    double distance;
    int index;
    bool type; // 0 - first, 1 - end
} POI;



void lidar_clud(const sensor_msgs::LaserScan::ConstPtr& scan)
{


//have to choose threshold carefully
  // int length = scan->ranges.size();
  // double gradient_threshold = 0.5;
  // double increment = scan->angle_increment;
	// std::vector<double> gradient;
	// //Sets up x and y coordinates for gradient as well as creates
	// //vector of all gradients between all the points in the laserscan
	// for (int i = 0; i < (length - 1); i++) {
  //   if (scan->ranges[i] < 8.0 && scan->ranges < 8.0){
  // 		double x0 = scan->ranges[i] * cos(i*increment);
  // 		double y0 = scan->ranges[i] * sin(i*increment);
  // 		double x1 = scan->ranges[i+1] * cos((i+1)*increment);
  // 		double y1 = scan->ranges[i+1] * sin((i+1)*increment);
  //
  // 		Vector temp;
  // 		temp.x = x1 - x0;
  // 		temp.y = y1 - y0;
  // 		temp.i = i;
  // 		temp.distance = scan->ranges[i];
  // 		temp.slope = fabs(atan2(temp.y,temp.x));
  // 		gradients.push_back(temp);
  //   }
	// }
  //
  // double grad_ave_a = 0;
	// double grad_ave_b = 0;
	// int k = 10;
  // std::vector<POI> first_tier;
	// //Find the average gradient over span of 20 points for each point in Laserscan
	// while (k < (gradients.size() - 10)) {
	// 	for (int j = -10; j < 10; j++) {
	// 	    if(j < 0)
	// 	        grad_ave_a +=gradients[k+j].slope;
	// 	    else
	// 	        grad_ave_b +=gradients[k+j].slope;
	// 	}
	// 	grad_ave_a = grad_ave_a/10;
	// 	grad_ave_b = grad_ave_b/10;
  //
	// 	if((fabs(grad_ave_a - grad_ave_b) < gradient_threshold)){
	// 	    POI temp;
	// 	    temp.index = gradients[k].i; // temp.angle = i;
	// 	    temp.distance = gradients[k].distance;
	// 	    temp.type = 1;
  //
	// 	    first_tier.push_back(temp);
	// 	}
	// 	k++;
	// }
  //
  // int obsfront = 0;
  // bool frontClear = false;  // flag for checking if front is clear or not
  // bool frontedge = false;  // edges are the traversible but the come up as obstacles in lidar
  //
  // for (int i = 40; i < 60; i++){
  //   if (scan->ranges[i] < 8.0){
  //     obsfront += 1;
  //   }
  // }
  //
  // if(obsfront < 5){
  //   frontClear = true;
  // }
  //
  // for (int i=0; i < first_tier.size(); i++){
  //   if (first_tier[i].index == 50){
  //     frontedge = true;
  //   }
  // }

  //add the avoidance stuff here


  // std::vector <double> angles;
  // for(int i=0; i<90; i++){
  //   double gradient = 0.0;
  //   for(int j=0; j<9; j++){
  //     if (i+j <= 99){
  //       gradient += std::abs(scan->ranges[i+j] - scan->ranges[i+j+1]);
  //     }
  //   }
  //   if (gradient < threshold){
  //     double direction = (i+5)*(-0.0262626260519);
  //     angles.pushback(direction);
  //   }
  // }

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
  ros::Publisher directionr = nh.advertise<std_msgs::Float64>("direction",1000);

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
// cout << "range front "<<scan->ranges[50]<<endl;
// int sections{10};
//
// typedef struct{
//   int index;    //index of the section [1-5] or[0-4]
//   int start;    // index of the starting data point in that section
//   std::vector<double> range;     //vector of ranges of all data points in the section
//   std::vector<int> obsindex;  // vector of suspected obstacles in the section
//   double gradient;   // change in the range between successive suspected obstacles
//   int numobs;        // number of suspected obstacles
// } section;
//
//
// // Initialize all sections:
// std::vector<section> sections(5);
// int k = 0;
// for(int i=0; i<100; i++){
//   if (i%20 == 0){
//     section sect;
//     sect.index = k;
//     sect.start = i;
//     sect.gradient = 0.0;
//     sect.numobs = 0;
//     for (int j=0; j < 20; j++){
//       sect.range.push_back(scan->ranges[i+j]);
//       if(scan->ranges[i+j] < 8.0){
//         sect.numobs += 1;
//         sect.obsindex.pushback(i+j);
//       }
//     }
//     k+=1;
//     sections.push_back(sect);
//   }
// }

//compare the sections, take the section with the least number of obstacle and
// with the direction closest to the goal direction
