// Need to ad executable in cmake
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alex_msgs/MotorParamOut.h"
#include "alex_driver/send_tmotor_command.h"

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <alex_global/global_definitions.h>


int main (int argc, char **argv) {
  ros::init(argc, argv, "alex_receive_tmotor_reply_node");
  ros::NodeHandle n;

  ROS_INFO("Receive TMotor Reply Node");
  ros::spin();

  return 0;
}
