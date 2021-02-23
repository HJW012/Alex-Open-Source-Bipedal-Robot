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
#include <can_msgs/Frame.h>

#include <alex_global/global_definitions.h>

void unpackTmotorReply(const can_msgs::Frame::ConstPtr& msg) {
  int len = 6;
  unsigned int id = msg->data[0];
  unsigned int p_int = (msg->data[1] << 8) | msg->data[2];
  unsigned int v_int = (msg->data[3] << 4) | (msg->data[4] >> 4);
  unsigned int i_int = ((msg->data[4] & 0xF) << 8) | msg->data[5];

  double p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  double v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  double t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "alex_receive_tmotor_reply_node");
  ros::NodeHandle n;

  ros::Subscriber canSubscriber = n.subscribe("received_messages", 10, unpackTmotorReply);

  ROS_INFO("Receive TMotor Reply Node");
  ros::spin();

  return 0;
}
