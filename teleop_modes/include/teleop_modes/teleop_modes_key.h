/*!
 * \TeleopModesKey.h
 * \brief Allows for control of the SRC2 Rovers with a keyboard.
 *
 * TeleopModesKey creates a ROS node that allows the control of the SRC2 Rovers
 * with a keyboard.
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Cagri Kilic, WVU - cakilic@mix.wvu.edu
 * * \date March 28, 2020
 */

#ifndef TELEOP_MODES_KEY_H_
#define TELEOP_MODES_KEY_H_

// Include cpp important headers
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <map>
#include <unistd.h>

// Include ROS headers
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <motion_control/ArmGroup.h>
#include <motion_control/WheelGroup.h>
#include <motion_control/SteeringGroup.h>
#include <motion_control/SensorGroup.h>
#include <motion_control/BinJoint.h>
#define PI 3.14159265

//Keycodes (see https://github.com/simlabrobotics/allegro_hand_ros/blob/master/allegro_hand_keyboard/src/AllegroHand_keyboard.cpp)
#define KEYCODE_a	0x61
#define KEYCODE_b	0x62
#define KEYCODE_c	0x63
#define KEYCODE_d	0x64
#define KEYCODE_e	0x65
#define KEYCODE_f	0x66
#define KEYCODE_g	0x67
#define KEYCODE_h	0x68
#define KEYCODE_i	0x69
#define KEYCODE_j	0x6A
#define KEYCODE_k	0x6B
#define KEYCODE_l	0x6C
#define KEYCODE_m	0x6D
#define KEYCODE_n	0x6E
#define KEYCODE_o	0x6F
#define KEYCODE_p	0x70
#define KEYCODE_q	0x71
#define KEYCODE_r	0x72
#define KEYCODE_s	0x73
#define KEYCODE_t	0x74
#define KEYCODE_u	0x75
#define KEYCODE_v	0x76
#define KEYCODE_w	0x77
#define KEYCODE_x	0x78
#define KEYCODE_y	0x79
#define KEYCODE_z	0x7A

#define KEYCODE_CTRL  0x11
#define KEYCODE_ALT   0x12

#define KEYCODE_SPACE 0x20

#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT  0x44

#define KEYCODE_0	0x30
#define KEYCODE_1	0x31
#define KEYCODE_2	0x32
#define KEYCODE_3	0x33
#define KEYCODE_4	0x34
#define KEYCODE_5	0x35
#define KEYCODE_6	0x36
#define KEYCODE_7	0x37
#define KEYCODE_8	0x38
#define KEYCODE_9	0x39

//Control modes
/*!
 * \def DRIVE_SKID_MODE
 *
 * Skid Steering Driving Mode.
 */
#define DRIVE_SKID_MODE 0
/*!
 * \def DRIVE_CRAB_MODE
 *
 * Crab Steering Driving Mode.
 */
//! @todo change to 1 when Crab Mode is on
#define DRIVE_CRAB_MODE 1
/*!
 * \def DRIVE_ACKE_MODE
 *
 * Ackermann Steering Driving Mode.
 */

#define DRIVE_ACKE_MODE 2

/*!
 * \def DRIVE_DACKE_MODE
 *
 * Double Ackermann Steering Driving Mode.
 */

#define DRIVE_DACKE_MODE 3

/*!
 * \def MAX_MOTOR_EFFORT
 *
 * The maximum translational velocity.
 */
#define MAX_MOTOR_EFFORT 75.0

/*!
 * \def MAX_STEERING_ANGLE
 *
 * The maximum steering angle.
 */
#define MAX_STEERING_ANGLE PI/2

/*!
 * \def JOINT LIMITS
 */

#define ARM1_MAX PI
#define ARM1_MIN -PI
#define ARM2_MAX 3*PI/8
#define ARM2_MIN -3*PI/8
#define ARM3_MAX 7*PI/8
#define ARM3_MIN -PI/4
#define ARM4_MAX 2*PI/3
#define ARM4_MIN -2*PI/3
#define SENSOR1_MAX PI
#define SENSOR1_MIN -PI
#define SENSOR2_MAX PI/3
#define SENSOR2_MIN -PI/3
#define BIN_MAX 3*PI/4
#define BIN_MIN 0

/*!
 * \class TeleopModesKey
 * \brief Allows for control of the the SRC2 Rovers with a keyboard.
 *
 * TeleopModesKey creates a ROS node that allows for the control of the
 * SRC2 Rovers with a keyboard, by reading keyboard input to a terminal.
 */
class TeleopModesKey
{
public:
  /*!
   * \brief Constructor
   *
   * Creates a TeleopModesKey object that can be used control the SRC2 Rovers
   * with a keyboard. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  TeleopModesKey(ros::NodeHandle & nh);

  // Teleop functions
  /*!
   * \brief Monitors the keyboard.
   */
  void GetKey();
  /*!
   * \brief (Monitors the keyboard) Publishes to the rover controllers whenever a key corresponding to a motion command is pressed.
   */
  void ResolveKey();




  char key = ' ';

private:
  /*!
   * \brief Publishes zero motor efforts to stop the robot if no key is pressed after a short time period.
   */
  void PublishAll();

  // Print functions;
  /*!
   * \brief Publishes to the rover controllers.
   */
  void PrintStatus();
  /*!
   * \brief Displays a help menu appropriate to the current mode.
   */
  void PrintCmdReminders();
  /*!
   * \brief Constrain angles.
   */
  double ConstrainAngle(double s_, double min_, double max_);
  
  // Node Handle
  ros::NodeHandle & nh_;
  
  // Create oublishers
  ros::Publisher pubArmAngles;      /*!< Publisher of joint angles */
  ros::Publisher pubWheelVels;     /*!< Publisher of joint angles */
  ros::Publisher pubSteeringAngles;  /*!< Publisher of joint angles */
  ros::Publisher pubSensorAngles;      /*!< Publisher of joint angles */
  ros::Publisher pubBinAngle;         /*!< Publisher of joint angles */

  // Create output messages
  motion_control::ArmGroup q;       /*!< Publisher of joint angles */
  motion_control::WheelGroup w;       /*!< Publisher of joint angles */
  motion_control::SteeringGroup s;    /*!< Publisher of joint angles */
  motion_control::SensorGroup j;      /*!< Publisher of joint angles */
  motion_control::BinJoint b;         /*!< Publisher of joint angles */

  int mode; /*!< the controller mode */
  int lights_mode = 0;
  int crab_rotate = 0;
  double throttle = 0;  /*!< Init factor for motor efforts variables */
  double step = 1;      /*!< Init factor for angle changes variables */

  double j1 = 0, j2 = 0;                                                /*!< Init sensor variable */
  double s1 = 0; double s2 = 0; double s3 = 0; double s4 = 0;   /*!< Init steering variables */
  double w1 = 0; double w2 = 0; double w3 = 0; double w4 = 0;   /*!< Init driving variables */
  double q1 = 0; double q2 = ARM2_MIN; double q3 = ARM3_MAX; double q4 = ARM4_MAX;   /*!< Init manipulation variables */
  double b1 = 0;                                                /*!< Init bin variables */

  // Map for mode keys
  std::map<char, std::vector<float>> modeBindings
  {
    {KEYCODE_1, {DRIVE_SKID_MODE}},
    {KEYCODE_2, {DRIVE_CRAB_MODE}},
    {KEYCODE_3, {DRIVE_ACKE_MODE}},
    {KEYCODE_4, {DRIVE_DACKE_MODE}}
  };

  // Map for sensor, bin and manipulator joints and light toggle keys
  std::map<char, std::vector<float>> sensorBindings
  {
    {KEYCODE_8, {1, 0}},
    {KEYCODE_9, {0, 0}},
    {KEYCODE_0, {-1, 0}},
    {KEYCODE_i, {0, 1}},
    {KEYCODE_o, {0, 0}},
    {KEYCODE_p, {0, -1}}
  };

  std::map<char, std::vector<float>> binBindings
  {
    {KEYCODE_y, {1}},
    {KEYCODE_h, {-1}}
  };
  
  std::map<char, std::vector<float>> manipulatorBindings
  {
    {KEYCODE_q, {1, 0, 0, 0}},
    {KEYCODE_w, {0, 1, 0, 0}},
    {KEYCODE_e, {0, 0, 1, 0}},
    {KEYCODE_r, {0, 0, 0, 1}},
    {KEYCODE_a, {-1, 0, 0, 0}},
    {KEYCODE_s, {0, -1, 0, 0}},
    {KEYCODE_d, {0, 0, -1, 0}},
    {KEYCODE_f, {0, 0, 0, -1}}
  };

  // Map for step keys
  std::map<char, std::vector<float>> stepBindings
  {
    {KEYCODE_5, {10}},
    {KEYCODE_6, {1.0}},
    {KEYCODE_7, {0.1}}
  };
  // Map for drive keys
  std::map<char, std::vector<float>> throttleBindings
  {
    {KEYCODE_j, {0.1}},
    {KEYCODE_m, {-0.1}}
  };

  // Maps for Skid Steering Steering
  std::map<char, std::vector<float>> skidMotorBindings
  {
    {KEYCODE_UP, {1, 1, 1, 1}},
    {KEYCODE_DOWN, {-1, -1, -1, -1}},
    {KEYCODE_LEFT, {1, 1, -1, -1}},  // or {'\x43', {1, 1, 0, 0}}
    {KEYCODE_RIGHT, {-1, -1, 1, 1}},   // or {'\x44', {0, 0, 1, 1}}
    {KEYCODE_SPACE, {0,0,0,0}}
  };

  // Maps for Crab Steering Steering
  std::map<char, std::vector<float>> crabMotorBindings
  {
    {KEYCODE_UP, {1, 1, 1, 1}},
    {KEYCODE_DOWN, {-1, -1, -1, -1}},
    {KEYCODE_x, {-1, -1, 1, 1}},
    {KEYCODE_c, {1, 1, -1, -1}},
    {KEYCODE_SPACE, {0, 0, 0, 0}}
  };
  std::map<char, std::vector<float>> crabSteeringBindings
  {
 //   {KEYCODE_UP, {1, 1, -1, -1}},//
 //   {KEYCODE_DOWN, {-1, -1, 1, 1}},//
    {KEYCODE_z, {1, -1, -1, 1}},
    //{KEYCODE_x, {-1, 1, 1, -1}},
    {KEYCODE_LEFT, {1, 1, 1, 1}},
    {KEYCODE_RIGHT, {-1, -1, -1, -1}}
  };

  // Maps for Ackermann Steering Steering
  std::map<char, std::vector<float>> ackeMotorBindings
  {
    {KEYCODE_UP, {1, 1, 1, 1}},
    {KEYCODE_DOWN, {-1, -1, -1, -1}},
    {KEYCODE_SPACE, {0,0,0,0}}
  };
  std::map<char, std::vector<float>> ackeSteeringBindings
  {
    {KEYCODE_LEFT, {1, 0, 1, 0}},
    {KEYCODE_RIGHT, {-1, 0, -1, 0}}
  };

  // Maps for Ackermann Steering Steering
  std::map<char, std::vector<float>> dAckeMotorBindings
  {
    {KEYCODE_UP, {1, 1, 1, 1}},
    {KEYCODE_DOWN, {-1, -1, -1, -1}},
    {KEYCODE_SPACE, {0,0,0,0}}
  };
  std::map<char, std::vector<float>> dAckeSteeringBindings
  {
    {KEYCODE_LEFT, {1, -1, 1, -1}},
    {KEYCODE_RIGHT, {-1, 1, -1, 1}}  
  };

};

/*!
 * \brief Creates and runs the TeleopModesKey node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //TELEOP_MODES_KEY_H_
