#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <alex_msgs/MotorParamOut.h>
#include <alex_driver/send_tmotor_command.h>
#include <can_msgs/Frame.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Motor parameters - these should be set to actual parameters at some point
#define P_MIN -95.5f
#define P_MAX 95.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f

#define P_INCREMENT 5.0f
#define V_INCREMENT 5.0f
#define T_INCREMENT 2.0f
#define KD_INCREMENT 0.5f
#define KP_INCREMENT 50.0f

// Map for movement keys
std::map<char, int> poseBindings {
  {'1', -1},
  {'2', 1}
};

std::map<char, int> speedBindings {
  {'3', -1},
  {'4', 1}
};

std::map<char, int> torqueBindings {
  {'5', -1},
  {'6', 1}
};

std::map<char, int> kdBindings {
  {'7', -1},
  {'8', 1}
};

std::map<char, int> kpBindings {
  {'9', -1},
  {'0', 1}
};

std::map<char, int> specialBindings {
  {'i', 1}, //Enter motor control mode
  {'o', 2}, //Exit motor control mode
  {'p', 3}  //Zero Motor Pose
};

std::map<char, bool> motorBindings {
  {'m', true}
};

// Reminder message
const char* msg = R"(
  Decrease Position: 1
  Increase Position: 2
  Decrease Position: 3
  Increase Position: 4
  Decrease Torque: 5
  Increase Torque: 6
  Decrease kp: 7
  Increase kp: 8
  Decrease kd: 9
  Increase kd: 10

  CTRL-C to quit
)";

// Init variables
float pose(0);
float speed(0);
float torque(0);
float kd(0);
float kp(0);
int mode(0);
bool sendCommand = false;

char key(' ');

// For non-blocking keyboard inputs
int getch(void)
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

  return ch;
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "alex_keyboard_teleop_test_node");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<alex_driver::send_tmotor_command>("send_tmotor_command");
  alex_driver::send_tmotor_command srv;

  // Init cmd_vel publisher
  //ros::Publisher pub = nh.advertise<alex_msgs::MotorParamOut>("alex/motorParamOut", 1);

  // Create MotorParamOut message
  alex_msgs::MotorParamOut motorParamOut;

  printf("%s", msg);
  printf("\rCurrent: Pose: %f\tSpeed: %f\tTorque: %f\tkd: %f\tkd: %f | Awaiting command: %c   \r", pose, speed, torque, kd, kp);

  while(true){

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in poseBindings
    if (poseBindings.count(key) == 1)
    {
      // Grab the direction data
      pose = pose + poseBindings[key] * P_INCREMENT;
      if (pose <= P_MIN) {
        pose = P_MIN;
      } else if (pose >= P_MAX) {
        pose = P_MAX;
      }

      mode = 0;
      sendCommand = false;
      printf("\rCurrent: Pose: %f\tSpeed: %f\tTorque: %f\tkd: %f\tkd: %f | Last command: %c   \r", pose, speed, torque, kd, kp, key);
    }

    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      speed = speed + speedBindings[key] * V_INCREMENT;
      if (speed <= V_MIN) {
        speed = V_MIN;
      } else if (speed >= V_MAX) {
        speed = V_MAX;
      }

      mode = 0;
      sendCommand = false;
      printf("\rCurrent: Pose: %f\tSpeed: %f\tTorque: %f\tkd: %f\tkd: %f | Last command: %c   \r", pose, speed, torque, kd, kp, key);
    }

    else if (torqueBindings.count(key) == 1)
    {
      // Grab the direction data
      torque = torque + torqueBindings[key] * T_INCREMENT;
      if (torque <= T_MIN) {
        torque = T_MIN;
      } else if (torque >= T_MAX) {
        torque = T_MAX;
      }

      mode = 0;
      sendCommand = false;
      printf("\rCurrent: Pose: %f\tSpeed: %f\tTorque: %f\tkd: %f\tkd: %f | Last command: %c   \r", pose, speed, torque, kd, kp, key);
    }

    else if (kdBindings.count(key) == 1)
    {
      // Grab the speed data
      kd = kd + kdBindings[key] * KD_INCREMENT;
      if (kd <= KD_MIN) {
        kd = KD_MIN;
      } else if (kd >= KD_MAX) {
        kd = KD_MAX;
      }

      mode = 0;
      sendCommand = false;
      printf("\rCurrent: Pose: %f\tSpeed: %f\tTorque: %f\tkd: %f\tkd: %f | Last command: %c   \r", pose, speed, torque, kd, kp, key);
    }

    else if (kpBindings.count(key) == 1)
    {
      // Grab the speed data
      kp = kp + kpBindings[key] * KP_INCREMENT;
      if (kp <= KP_MIN) {
        kp = KP_MIN;
      } else if (kp >= KP_MAX) {
        kp = KP_MAX;
      }

      mode = 0;
      sendCommand = false;
      printf("\rCurrent: Pose: %f\tSpeed: %f\tTorque: %f\tkd: %f\tkd: %f | Last command: %c   \r", pose, speed, torque, kd, kp, key);
    }

    else if (specialBindings.count(key) == 1) {
      mode = specialBindings[key];
      pose = 0;
      speed = 0;
      torque = 0;
      kd = 0;
      kp = 0;
      sendCommand = true;

      printf("\rCurrent: Pose: %f\tSpeed: %f\tTorque: %f\tkd: %f\tkd: %f | Last command: %c   \r", pose, speed, torque, kd, kp, key);
    } else if (motorBindings.count(key) == 1) {
      mode = 0;
      sendCommand = true;
    }

    // Otherwise, set the robot to stop
    else
    {
      pose = 0;
      speed = 0;
      torque = 0;
      kd = 0;
      kp = 0;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\nBYE\n\n");
        break;
      }

      printf("\rCurrent: Pose: %f\tSpeed: %f\tTorque: %f\tkd: %f\tkd: %f | Invalid command: %c   \r", pose, speed, torque, kd, kp, key);
    }

    // Update the MotorParamOut message
    if (sendCommand) {
      motorParamOut.header.stamp = ros::Time::now();
      motorParamOut.header.frame_id = "/world";
      motorParamOut.pose = pose;
      motorParamOut.speed = speed;
      motorParamOut.torque = torque;
      motorParamOut.kd = kd;
      motorParamOut.kp = kp;
      motorParamOut.mode = mode;

      srv.request.motorParamOut = motorParamOut;
      client.call(srv);

      if (srv.response.success) {
        printf("Response; TRUE \r");
      } else {
        printf("Response: FALSE \r");
      }

      // Publish it and resolve any remaining callbacks
      //pub.publish(motorParamOut);
      ros::spinOnce();
    }
  }

  return 0;
}
