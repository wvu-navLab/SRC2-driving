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

#include <teleop_modes/teleop_modes_key.h>

TeleopModesKey::TeleopModesKey(ros::NodeHandle & nh)
: nh_(nh)
{
  // Node publishes the messages in appropriate topics
  pubArmAngles = nh_.advertise<motion_control::ArmGroup>("control/arm/joint_angles", 1000);
  pubWheelVels = nh_.advertise<motion_control::WheelGroup>("control/drive/wheel_velocities", 1000);
  pubSteeringAngles = nh_.advertise<motion_control::SteeringGroup>("control/steering/joint_angles", 1000);
  pubSensorAngles = nh_.advertise<motion_control::SensorGroup>("control/sensor/joint_angles", 1000);
  pubBinAngle = nh_.advertise<motion_control::BinJoint>("control/bin/joint_angle", 1000);

  // initialize driving mode
  mode = DRIVE_ACKE_MODE;

  // Initialize teleop with zero commands
  j.j1 = 0; j.j2 = 0;
  q.q1 = 0; q.q2 =  ARM2_MIN; q.q3 = ARM3_MAX; q.q4 = ARM4_MIN; // Arm will move to HOME_MODE
  w.w1 = 0; w.w2 = 0; w.w3 = 0; w.w4 = 0;
  s.s1 = 0; s.s2 = 0; s.s3 = 0; s.s4 = 0;
  b.b1 = 0;

  // ROS_INFO("Keyboard teleop started for %s", arm_name_.c_str());
}

double TeleopModesKey::ConstrainAngle(double s_, double min_, double max_)
{
  s_ = (s_ > max_)?  max_ : s_;
  s_ = (s_ < min_)? min_ : s_;  
  return s_;
}

void TeleopModesKey::GetKey()
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  key = ch;
}

void TeleopModesKey::PublishAll()
{
  pubArmAngles.publish(q);
  pubWheelVels.publish(w);
  pubSteeringAngles.publish(s);
  pubSensorAngles.publish(j);
  pubBinAngle.publish(b);
  PrintStatus();
  PrintCmdReminders();
}

void TeleopModesKey::ResolveKey()
{
  switch (mode)
  {
    case DRIVE_SKID_MODE:
    {
      // If the key corresponds to a Mode Key
      if (modeBindings.count(key) == 1)
      {
        mode = modeBindings[key][0];        
        s1 = 0; s2 = 0; s3 = 0; s4 = 0;
        s.s1 = s1; s.s2 = s2; s.s3 = s3; s.s4 = s4;
        w1 = 0; w2 = 0; w3 = 0; w4 = 0;
        w.w1 = w1; w.w2 = w2; w.w3 = w3; w.w4 = w4;
        PublishAll();
      }
      // If the key corresponds to a key in sensorBindings
      else if (sensorBindings.count(key) == 1)
      {
        if (key == KEYCODE_9)
        {
          // Grab the change and update the current joint angles message
          j1 = 0;
          j.j1 = j1; // Update the current sensor angle message
        }
        else if (key == KEYCODE_o)
        {
          // Grab the change and update the current joint angles message
          j2 = 0;
          j.j2 = j2; // Update the current sensor angle message
        }
        else
        {
          // Grab the change and update the current joint angles message
          j1 = ConstrainAngle(j1 + (sensorBindings[key][0]*step)*PI/180, SENSOR1_MIN, SENSOR1_MAX);
          j2 = ConstrainAngle(j2 + (sensorBindings[key][1]*step)*PI/180, SENSOR2_MIN, SENSOR2_MAX);
          j.j1 = j1; // Update the current sensor angle message
          j.j2 = j2; // Update the current sensor angle message
        }
        PublishAll();
      }
      else if (binBindings.count(key) == 1)
      {
        // Grab the change and update the current joint angles message
        b1 = ConstrainAngle(b1 + (binBindings[key][0]*step)*PI/180, BIN_MIN, BIN_MAX);
        b.b1 = b1; // Update the current bin angle message
        PublishAll();
      }
      // If the key corresponds to a key in manipulatorBindings
      else if (manipulatorBindings.count(key) == 1)
      {
        // Grab the change and update the current joint angles message
        q1 = ConstrainAngle(q1 + manipulatorBindings[key][0] * step * PI/180, ARM1_MIN, ARM1_MAX); // Update the current joint angles message
        q2 = ConstrainAngle(q2 + manipulatorBindings[key][1] * step * PI/180, ARM2_MIN, ARM2_MAX);
        q3 = ConstrainAngle(q3 + manipulatorBindings[key][2] * step * PI/180, ARM3_MIN, ARM3_MAX);
        q4 = ConstrainAngle(q4 + manipulatorBindings[key][3] * step * PI/180, ARM4_MIN, ARM4_MAX);
        q.q1 = q1;
        q.q2 = q2;
        q.q3 = q3;
        q.q4 = q4;
        PublishAll();
      }
      // If the key corresponds to a key in skidMotorBindings
      else if (skidMotorBindings.count(key) == 1)
      {
        // Grab the directional data
        w1 = skidMotorBindings[key][0] * throttle * MAX_MOTOR_EFFORT;
        w2 = skidMotorBindings[key][1] * throttle * MAX_MOTOR_EFFORT;
        w3 = skidMotorBindings[key][2] * throttle * MAX_MOTOR_EFFORT;
        w4 = skidMotorBindings[key][3] * throttle * MAX_MOTOR_EFFORT;

        w.w1 = w1;
        w.w2 = w2;
        w.w3 = w3;
        w.w4 = w4;
        PublishAll();
      }
      else if (stepBindings.count(key) == 1)
      {
        // Grab the throttle data
        step = stepBindings[key][0];
        PublishAll();
      }
      else if (throttleBindings.count(key) == 1)
      {
        // Grab the throttle data
        throttle = throttle + throttleBindings[key][0];
        throttle = (throttle>1)? 1 : throttle;
        throttle = (throttle<0)? 0 : throttle;
        PublishAll();
      }
      // Otherwise, set the robot to stop
      else
      {
        //w1 = 0; w2 = 0; w3 = 0; w4 = 0;
        //w.w1 = w1; w.w2 = w2; w.w3 = w3; w.w4 = w4;
        PublishAll(); 
      }
    }
    break;

    case DRIVE_CRAB_MODE:
    {
      // If the key corresponds to a Mode Key
      if (modeBindings.count(key) == 1)
      {
        mode = modeBindings[key][0];        
        s1 = 0; s2 = 0; s3 = 0; s4 = 0;
        s.s1 = s1; s.s2 = s2; s.s3 = s3; s.s4 = s4;
        w1 = 0; w2 = 0; w3 = 0; w4 = 0;
        w.w1 = w1; w.w2 = w2; w.w3 = w3; w.w4 = w4;
        PublishAll();
      }
      // If the key corresponds to a key in sensorBindings
      else if (sensorBindings.count(key) == 1)
      {
        if (key == KEYCODE_9)
        {
          // Grab the change and update the current joint angles message
          j1 = 0;
          j.j1 = j1; // Update the current sensor angle message
        }
        else if (key == KEYCODE_o)
        {
          // Grab the change and update the current joint angles message
          j2 = 0;
          j.j2 = j2; // Update the current sensor angle message
        }
        else
        {
          // Grab the change and update the current joint angles message
          j1 = ConstrainAngle(j1 + (sensorBindings[key][0]*step)*PI/180, SENSOR1_MIN, SENSOR1_MAX);
          j2 = ConstrainAngle(j2 + (sensorBindings[key][1]*step)*PI/180, SENSOR2_MIN, SENSOR2_MAX);
          j.j1 = j1; // Update the current sensor angle message
          j.j2 = j2; // Update the current sensor angle message
        }
        PublishAll();
      }
      else if (binBindings.count(key) == 1)
      {
        // Grab the change and update the current joint angles message
        b1 = ConstrainAngle(b1 + (binBindings[key][0]*step)*PI/180, BIN_MIN, BIN_MAX);
        b.b1 = b1; // Update the current bin angle message
        PublishAll();
      }
      // If the key corresponds to a key in manipulatorBindings
      else if (manipulatorBindings.count(key) == 1)
      {
        // Grab the change and update the current joint angles message
        q1 = ConstrainAngle(q1 + manipulatorBindings[key][0] * step * PI/180, ARM1_MIN, ARM1_MAX); // Update the current joint angles message
        q2 = ConstrainAngle(q2 + manipulatorBindings[key][1] * step * PI/180, ARM2_MIN, ARM2_MAX);
        q3 = ConstrainAngle(q3 + manipulatorBindings[key][2] * step * PI/180, ARM3_MIN, ARM3_MAX);
        q4 = ConstrainAngle(q4 + manipulatorBindings[key][3] * step * PI/180, ARM4_MIN, ARM4_MAX);
        q.q1 = q1;
        q.q2 = q2;
        q.q3 = q3;
        q.q4 = q4;
        PublishAll();
      }
      // If the key corresponds to a key in crabSteeringBindings
      else if (crabSteeringBindings.count(key) == 1)
      {
        int crab_rotate_ = (key == KEYCODE_z || key == KEYCODE_x || key == KEYCODE_c);
        switch (crab_rotate_)
        {
          case 0:
          {
            // Grab the directional data
            if (crab_rotate == 1)
            {
              s1 = 0; s2 = 0; s3 = 0; s4 = 0;
              s.s1 = s1; s.s2 = s2; s.s3 = s3; s.s4 = s4;
            }
            s1 = ConstrainAngle(s1 + crabSteeringBindings[key][0] * step * PI/180,-MAX_STEERING_ANGLE,MAX_STEERING_ANGLE);
            s2 = ConstrainAngle(s2 + crabSteeringBindings[key][1] * step * PI/180,-MAX_STEERING_ANGLE,MAX_STEERING_ANGLE);
            s3 = ConstrainAngle(s3 + crabSteeringBindings[key][2] * step * PI/180,-MAX_STEERING_ANGLE,MAX_STEERING_ANGLE);
            s4 = ConstrainAngle(s4 + crabSteeringBindings[key][3] * step * PI/180,-MAX_STEERING_ANGLE,MAX_STEERING_ANGLE);
            s.s1 = s1;
            s.s2 = s2;
            s.s3 = s3;
            s.s4 = s4;
            PublishAll();
          }
          break;

          case 1:
          {
            // rotate wheels 45 deg
            s1 = MAX_STEERING_ANGLE/2;
            s2 = -MAX_STEERING_ANGLE/2;
            s3 = -MAX_STEERING_ANGLE/2;
            s4 = MAX_STEERING_ANGLE/2;
            s.s1 = s1;
            s.s2 = s2;
            s.s3 = s3;
            s.s4 = s4;
            PublishAll();
          }
          break;
        }
        crab_rotate = crab_rotate_;
      }
      // If the key corresponds to a key in crabMotorBindings
      else if (crabMotorBindings.count(key) == 1)
      {
        // Grab the directional data
        w1 = crabMotorBindings[key][0] * throttle * MAX_MOTOR_EFFORT;
        w2 = crabMotorBindings[key][1] * throttle * MAX_MOTOR_EFFORT;
        w3 = crabMotorBindings[key][2] * throttle * MAX_MOTOR_EFFORT;
        w4 = crabMotorBindings[key][3] * throttle * MAX_MOTOR_EFFORT;

        w.w1 = w1;
        w.w2 = w2;
        w.w3 = w3;
        w.w4 = w4;
        PublishAll();
      }
      else if (stepBindings.count(key) == 1)
      {
        // Grab the throttle data
        step = stepBindings[key][0];
        PublishAll();
      }
      else if (throttleBindings.count(key) == 1)
      {
        // Grab the throttle data
        throttle = throttle + throttleBindings[key][0];
        throttle = (throttle>1)? 1 : throttle;
        throttle = (throttle<0)? 0 : throttle;
        PublishAll();
      }
      // Otherwise, set the robot to stop
      else
      {
        //s1 = 0; s2 = 0; s3 = 0; s4 = 0;
        //s.s1 = s1; s.s2 = s2; s.s3 = s3; s.s4 = s4;
        //w1 = 0; w2 = 0; w3 = 0; w4 = 0;
        //w.w1 = w1; w.w2 = w2; w.w3 = w3; w.w4 = w4;
        PublishAll();  
      }
    }
    break;

    case DRIVE_ACKE_MODE:
    {
      // If the key corresponds to a Mode Key
      if (modeBindings.count(key) == 1)
      {
        mode = modeBindings[key][0];        
        s1 = 0; s2 = 0; s3 = 0; s4 = 0;
        s.s1 = s1; s.s2 = s2; s.s3 = s3; s.s4 = s4;
        w1 = 0; w2 = 0; w3 = 0; w4 = 0;
        w.w1 = w1; w.w2 = w2; w.w3 = w3; w.w4 = w4;
        PublishAll();
      }
      else if (sensorBindings.count(key) == 1)
      {
        if (key == KEYCODE_9)
        {
          // Grab the change and update the current joint angles message
          j1 = 0;
          j.j1 = j1; // Update the current sensor angle message
        }
        else if (key == KEYCODE_o)
        {
          // Grab the change and update the current joint angles message
          j2 = 0;
          j.j2 = j2; // Update the current sensor angle message
        }
        else
        {
          // Grab the change and update the current joint angles message
          j1 = ConstrainAngle(j1 + (sensorBindings[key][0]*step)*PI/180, SENSOR1_MIN, SENSOR1_MAX);
          j2 = ConstrainAngle(j2 + (sensorBindings[key][1]*step)*PI/180, SENSOR2_MIN, SENSOR2_MAX);
          j.j1 = j1; // Update the current sensor angle message
          j.j2 = j2; // Update the current sensor angle message
        }
        PublishAll();
      }
      else if (binBindings.count(key) == 1)
      {
        // Grab the change and update the current joint angles message
        b1 = ConstrainAngle(b1 + (binBindings[key][0]*step)*PI/180, BIN_MIN, BIN_MAX);
        b.b1 = b1; // Update the current bin angle message
        PublishAll();
      }
      // If the key corresponds to a key in manipulatorBindings
      else if (manipulatorBindings.count(key) == 1)
      {
        // Grab the change and update the current joint angles message
        q1 = ConstrainAngle(q1 + manipulatorBindings[key][0] * step * PI/180, ARM1_MIN, ARM1_MAX); // Update the current joint angles message
        q2 = ConstrainAngle(q2 + manipulatorBindings[key][1] * step * PI/180, ARM2_MIN, ARM2_MAX);
        q3 = ConstrainAngle(q3 + manipulatorBindings[key][2] * step * PI/180, ARM3_MIN, ARM3_MAX);
        q4 = ConstrainAngle(q4 + manipulatorBindings[key][3] * step * PI/180, ARM4_MIN, ARM4_MAX);
        q.q1 = q1;
        q.q2 = q2;
        q.q3 = q3;
        q.q4 = q4;
        PublishAll();
      }
      // If the key corresponds to a key in ackeSteeringBindings
      else if (ackeSteeringBindings.count(key) == 1)
      {
        // Grab the directional data
        s1 = ConstrainAngle(s1 + ackeSteeringBindings[key][0] * step * PI/180, -MAX_STEERING_ANGLE/2, MAX_STEERING_ANGLE/2);
        s3 = ConstrainAngle(s3 + ackeSteeringBindings[key][2] * step * PI/180, -MAX_STEERING_ANGLE/2, MAX_STEERING_ANGLE/2);
        s.s1 = s1;
        s.s3 = s3;
        PublishAll();
      }
      // If the key corresponds to a key in ackeMotorBindings
      else if (ackeMotorBindings.count(key) == 1)
      {
        // Grab the directional data
        w1 = ackeMotorBindings[key][0] * throttle * MAX_MOTOR_EFFORT;
        w2 = ackeMotorBindings[key][1] * throttle * MAX_MOTOR_EFFORT;
        w3 = ackeMotorBindings[key][2] * throttle * MAX_MOTOR_EFFORT;
        w4 = ackeMotorBindings[key][3] * throttle * MAX_MOTOR_EFFORT;
        w.w1 = w1;
        w.w2 = w2;
        w.w3 = w3;
        w.w4 = w4;
        PublishAll();
      }
      // If the key corresponds to a key in stepBindings
      else if (stepBindings.count(key) == 1)
      {
        // Grab the throttle data
        step = stepBindings[key][0];
        PublishAll();
      }
      // If the key corresponds to a key in throttling
      else if (throttleBindings.count(key) == 1)
      {
        // Grab the throttle data
        throttle = throttle + throttleBindings[key][0];
        throttle = (throttle>1)? 1 : throttle;
        throttle = (throttle<0)? 0 : throttle;
        PublishAll();
      }
      // Otherwise, set the robot to stop
      else
      {
        //s1 = 0; s2 = 0; s3 = 0; s4 = 0;
        //s.s1 = s1; s.s2 = s2; s.s3 = s3; s.s4 = s4;
        //w1 = 0; w2 = 0; w3 = 0; w4 = 0;
        //w.w1 = w1; w.w2 = w2; w.w3 = w3; w.w4 = w4;
        PublishAll();
      }
    }
    break;

    case DRIVE_DACKE_MODE:
    {
      // If the key corresponds to a Mode Key
      if (modeBindings.count(key) == 1)
      {
        mode = modeBindings[key][0];        
        s1 = 0; s2 = 0; s3 = 0; s4 = 0;
        s.s1 = s1; s.s2 = s2; s.s3 = s3; s.s4 = s4;
        w1 = 0; w2 = 0; w3 = 0; w4 = 0;
        w.w1 = w1; w.w2 = w2; w.w3 = w3; w.w4 = w4;
        PublishAll();
      }
      // If the key corresponds to a key in sensorBindings
      else if (sensorBindings.count(key) == 1)
      {
        if (key == KEYCODE_9)
        {
          // Grab the change and update the current joint angles message
          j1 = 0;
          j.j1 = j1; // Update the current sensor angle message
        }
        else if (key == KEYCODE_o)
        {
          // Grab the change and update the current joint angles message
          j2 = 0;
          j.j2 = j2; // Update the current sensor angle message
        }
        else
        {
          // Grab the change and update the current joint angles message
          j1 = ConstrainAngle(j1 + (sensorBindings[key][0]*step)*PI/180, SENSOR1_MIN, SENSOR1_MAX);
          j2 = ConstrainAngle(j2 + (sensorBindings[key][1]*step)*PI/180, SENSOR2_MIN, SENSOR2_MAX);
          j.j1 = j1; // Update the current sensor angle message
          j.j2 = j2; // Update the current sensor angle message
        }
        PublishAll();
      }
      else if (binBindings.count(key) == 1)
      {
        // Grab the change and update the current joint angles message
        b1 = ConstrainAngle(b1 + (binBindings[key][0]*step)*PI/180, BIN_MIN, BIN_MAX);
        b.b1 = b1; // Update the current bin angle message
        PublishAll();
      }
      // If the key corresponds to a key in manipulatorBindings
      else if (manipulatorBindings.count(key) == 1)
      {
        // Grab the change and update the current joint angles message
        q1 = ConstrainAngle(q1 + manipulatorBindings[key][0] * step * PI/180, ARM1_MIN, ARM1_MAX); // Update the current joint angles message
        q2 = ConstrainAngle(q2 + manipulatorBindings[key][1] * step * PI/180, ARM2_MIN, ARM2_MAX);
        q3 = ConstrainAngle(q3 + manipulatorBindings[key][2] * step * PI/180, ARM3_MIN, ARM3_MAX);
        q4 = ConstrainAngle(q4 + manipulatorBindings[key][3] * step * PI/180, ARM4_MIN, ARM4_MAX);
        q.q1 = q1;
        q.q2 = q2;
        q.q3 = q3;
        q.q4 = q4;
        PublishAll();
      }
      // If the key corresponds to a key in ackeSteeringBindings
      else if (dAckeSteeringBindings.count(key) == 1)
      {
        // Grab the directional data
        s1 = ConstrainAngle(s1 + dAckeSteeringBindings[key][0] * step * PI/180, -MAX_STEERING_ANGLE/2, MAX_STEERING_ANGLE/2);
        s2 = ConstrainAngle(s2 + dAckeSteeringBindings[key][1] * step * PI/180, -MAX_STEERING_ANGLE/2, MAX_STEERING_ANGLE/2);
        s3 = ConstrainAngle(s3 + dAckeSteeringBindings[key][2] * step * PI/180, -MAX_STEERING_ANGLE/2, MAX_STEERING_ANGLE/2);
        s4 = ConstrainAngle(s4 + dAckeSteeringBindings[key][3] * step * PI/180, -MAX_STEERING_ANGLE/2, MAX_STEERING_ANGLE/2);
        s.s1 = s1;
        s.s2 = s2;
        s.s3 = s3;
        s.s4 = s4;
        PublishAll();
      }
      // If the key corresponds to a key in ackeMotorBindings
      else if (ackeMotorBindings.count(key) == 1)
      {
        // Grab the directional data
        w1 = dAckeMotorBindings[key][0] * throttle * MAX_MOTOR_EFFORT;
        w2 = dAckeMotorBindings[key][1] * throttle * MAX_MOTOR_EFFORT;
        w3 = dAckeMotorBindings[key][2] * throttle * MAX_MOTOR_EFFORT;
        w4 = dAckeMotorBindings[key][3] * throttle * MAX_MOTOR_EFFORT;
        w.w1 = w1;
        w.w2 = w2;
        w.w3 = w3;
        w.w4 = w4;
        PublishAll();
      }
      // If the key corresponds to a key in stepBindings
      else if (stepBindings.count(key) == 1)
      {
        // Grab the throttle data
        step = stepBindings[key][0];
        PublishAll();
      }
      // If the key corresponds to a key in throttling
      else if (throttleBindings.count(key) == 1)
      {
        // Grab the throttle data
        throttle = throttle + throttleBindings[key][0];
        throttle = (throttle>1)? 1 : throttle;
        throttle = (throttle<0)? 0 : throttle;
        PublishAll();
      }
      // Otherwise, set the robot to stop
      else
      {
        //s1 = 0; s2 = 0; s3 = 0; s4 = 0;
        //s.s1 = s1; s.s2 = s2; s.s3 = s3; s.s4 = s4;
        //w1 = 0; w2 = 0; w3 = 0; w4 = 0;
        //w.w1 = w1; w.w2 = w2; w.w3 = w3; w.w4 = w4;
        PublishAll();
      }
    }
    break;
  }
}

void TeleopModesKey::PrintCmdReminders()
{
  switch (mode)
  {
    case DRIVE_SKID_MODE:
    {
      puts("|-------------------------------------------------------------------|");
      puts("|   West Virginia University 2020  :  TAKE ME HOME, COUNTRY ROADS!  |");
      puts("|-------------------------------------------------------------------|");
      puts("|  MODES  | <<SKID>>  | 2. CRAB  | 3. ACKERMANN  | 4. D. ACKERMANN  |");
      puts("|-------------------------------------------------------------------|");
      puts("|        Moving Base:       |           Moving Manipulator:         |");
      puts("|---------------------------|---------------------------------------|");
      puts("|  Use Directional Arrows   |        Mount | Base | Distal | Bucket |");
      puts("|---------------------------|--------------|------|--------|--------|");
      puts("|            FWD            |  UP   :   q  |   w  |    e   |    r   |");
      puts("|             ^             | DOWN  :   a  |   s  |    d   |    f   |");
      puts("|  Rotate  <     >   Rotate |---------------------------------------|");
      puts("|    CCW      v        CW   |   Moving Sensor:  |                   |");
      puts("|            RWD            |-------------------|-------------------|");
      puts("|---------------------------|  UP     :    8    |   RIGHT  :   i    |");
      puts("|     j  :  +10% throttle   |  RESET  :    9    |   RESET  :   o    |");
      puts("|     m  :  -10% throttle   |  DOWN   :    0    |   LEFT   :   p    |");
      puts("|---------------------------|---------------------------------------|");
      puts("|  space :  FULL STOP       | Angle Steps:  10/1/0.1 : Press 5/6/7  |");
      puts("|-------------------------------------------------------------------|");
    }
    break;  

    case DRIVE_CRAB_MODE:
    {
      puts("|-------------------------------------------------------------------|");
      puts("|   West Virginia University 2020  :  TAKE ME HOME, COUNTRY ROADS!  |");
      puts("|-------------------------------------------------------------------|");
      puts("|  MODES  | 1. SKID  | <<CRAB>>  | 3. ACKERMANN  | 4. D. ACKERMANN  |");
      puts("|-------------------------------------------------------------------|");
      puts("|        Moving Base:       |           Moving Manipulator:         |");
      puts("|---------------------------|---------------------------------------|");
      puts("|            FWD            |        Mount | Base | Distal | Bucket |");
      puts("|             ^             |--------------|------|--------|--------|");
      puts("|  Steer   <     >   Steer  |  UP   :   q  |   w  |    e   |    r   |");
      puts("|  Left       v      Right  | DOWN  :   a  |   s  |    d   |    f   |");
      puts("|            RWD            |---------------------------------------|");
      puts("|---------------------------|   Moving Sensor:  |                   |");
      puts("|     z  :   Turn-In-Place  |-------------------|-------------------|");
      puts("|    x-c :   Rotate CW-CCW  |  UP     :    8    |   RIGHT  :   i    |");
      puts("|     j  :  +10% throttle   |  RESET  :    9    |   RESET  :   o    |");
      puts("|     m  :  -10% throttle   |  DOWN   :    0    |   LEFT   :   p    |");
      puts("|---------------------------|---------------------------------------|");
      puts("|  space :  FULL STOP       | Angle Steps:  10/1/0.1 : Press 5/6/7  |");
      puts("|-------------------------------------------------------------------|");
    }
    break; 

    case DRIVE_ACKE_MODE:
    {
      puts("|-------------------------------------------------------------------|");
      puts("|   West Virginia University 2020  :  TAKE ME HOME, COUNTRY ROADS!  |");
      puts("|-------------------------------------------------------------------|");
      puts("|  MODES  | 1. SKID  | 2. CRAB  | <<ACKERMANN>>  | 4. D. ACKERMANN  |");
      puts("|-------------------------------------------------------------------|");
      puts("|        Moving Base:       |           Moving Manipulator:         |");
      puts("|---------------------------|---------------------------------------|");
      puts("|            FWD            |        Mount | Base | Distal | Bucket |");
      puts("|             ^             |--------------|------|--------|--------|");
      puts("|  Steer   <     >   Steer  |  UP   :   q  |   w  |    e   |    r   |");
      puts("|  Left       v      Right  | DOWN  :   a  |   s  |    d   |    f   |");
      puts("|            RWD            |---------------------------------------|");
      puts("|---------------------------|   Moving Sensor:  |                   |");
      puts("|      j  :  +10% throttle  |-------------------|-------------------|");
      puts("|      m  :  -10% throttle  |  UP     :    8    |   RIGHT  :   i    |");
      puts("|---------------------------|  RESET  :    9    |   RESET  :   o    |");
      puts("|   space :  FULL STOP      |  DOWN   :    0    |   LEFT   :   p    |");
      puts("|---------------------------|---------------------------------------|");
      puts("| West Virginia University  | Angle Steps:  10/1/0.1 : Press 5/6/7  |");
      puts("|-------------------------------------------------------------------|");
    }
    break;

    case DRIVE_DACKE_MODE:
    {
      puts("|-------------------------------------------------------------------|");
      puts("|   West Virginia University 2020  :  TAKE ME HOME, COUNTRY ROADS!  |");
      puts("|-------------------------------------------------------------------|");
      puts("|  MODES  | 1. SKID  | 2. CRAB  | 3. ACKERMANN  | <<D. ACKERMANN>>  |");
      puts("|-------------------------------------------------------------------|");
      puts("|        Moving Base:       |           Moving Manipulator:         |");
      puts("|---------------------------|---------------------------------------|");
      puts("|            FWD            |        Mount | Base | Distal | Bucket |");
      puts("|             ^             |--------------|------|--------|--------|");
      puts("|  Steer   <     >   Steer  |  UP   :   q  |   w  |    e   |    r   |");
      puts("|  Left       v      Right  | DOWN  :   a  |   s  |    d   |    f   |");
      puts("|            RWD            |---------------------------------------|");
      puts("|---------------------------|   Moving Sensor:  |                   |");
      puts("|      j  :  +10% throttle  |-------------------|-------------------|");
      puts("|      m  :  -10% throttle  |  UP     :    8    |   RIGHT  :   i    |");
      puts("|---------------------------|  RESET  :    9    |   RESET  :   o    |");
      puts("|   space :  FULL STOP      |  DOWN   :    0    |   LEFT   :   p    |");
      puts("|---------------------------|---------------------------------------|");
      puts("| West Virginia University  | Angle Steps:  10/1/0.1 : Press 5/6/7  |");
      puts("|-------------------------------------------------------------------|");
    }
    break;    
  }
}

void TeleopModesKey::PrintStatus()
{
  switch (mode)
  {
    case DRIVE_SKID_MODE:
    {
      puts("\033[2J\033[1;1H");
      ROS_WARN("Current Values\r");
      printf("\rManipulator = (%2.1f,%2.1f,%2.1f,%2.1f) deg\t Sensor = (%2.2f,%2.2f) deg\t Step = %2.1f deg\n Throttle %2.2f /100| Last command: %c .\n", q1*180/PI, q2*180/PI, q3*180/PI, q4*180/PI, j1*180/PI, j2*180/PI, step, throttle*100, key);
    }
    break;

    case DRIVE_CRAB_MODE:
    {
      puts("\033[2J\033[1;1H");
      ROS_WARN("Current Values\r");
        switch (crab_rotate)
        {
          case 0:
          {
            printf("\rManipulator = (%2.2f,%2.2f,%2.2f,%2.2f) deg\t Sensor = (%2.2f,%2.2f) deg\t Step = %2.2f deg\n Throttle %2.2f /100\t Steering = %2.2f | Last command: %c .\n", q1*180/PI, q2*180/PI, q3*180/PI, q4*180/PI, j1*180/PI, j2*180/PI, step, throttle*100, s1*180/PI, key);
          }
          break;

          case 1:
          {
            printf("\rManipulator = (%2.2f,%2.2f,%2.2f,%2.2f) deg\t Sensor = (%2.2f,%2.2f) deg\t Step = %2.2f deg\n Throttle %2.2f /100\t ROTATION | Last command: %c .\n", q1*180/PI, q2*180/PI, q3*180/PI, q4*180/PI, j1*180/PI, j2*180/PI, step, throttle*100, key);
          }
          break;
        }
    }
    break;

    case DRIVE_ACKE_MODE:
    {
      puts("\033[2J\033[1;1H");
      ROS_WARN("Current Values\r");
      printf("\rManipulator = (%2.2f,%2.2f,%2.2f,%2.2f) deg\t Sensor = (%2.2f,%2.2f) deg\t Step = %2.2f deg\n Throttle %2.2f /100\t Steering = %2.2f | Last command: %c .\n", q1*180/PI, q2*180/PI, q3*180/PI, q4*180/PI, j1*180/PI, j2*180/PI, step, throttle*100, s1*180/PI, key);
    }
    break;

    case DRIVE_DACKE_MODE:
    {
      puts("\033[2J\033[1;1H");
      ROS_WARN("Current Values\r");
      printf("\rManipulator = (%2.2f,%2.2f,%2.2f,%2.2f) deg\t Sensor = (%2.2f,%2.2f) deg\t Step = %2.2f deg\n Throttle %2.2f /100\t Steering = %2.2f | Last command: %c .\n", q1*180/PI, q2*180/PI, q3*180/PI, q4*180/PI, j1*180/PI, j2*180/PI, step, throttle*100, s1*180/PI, key);
    }
    break;
  }
}


int main(int argc, char** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "teleop_modes_key");
  ros::NodeHandle nh("");

  // initialize the keyboard controller
  TeleopModesKey teleop_modes_key(nh);

  printf("\rAwaiting command...");

  while(true){
    // Get the pressed key
    teleop_modes_key.GetKey();

    // If Ctrl+C was pressed, terminate the program
    if (teleop_modes_key.key == '\x03')
    {
      printf("\rExiting!");
      break;
    }

    // Resolve command
    teleop_modes_key.ResolveKey();
    ros::spinOnce();
  }

  return 0;
}
