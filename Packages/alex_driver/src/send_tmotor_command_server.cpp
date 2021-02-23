#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alex_msgs/MotorParamOut.h"
#include "alex_driver/send_tmotor_command.h"
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_interface/string.h>

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

int s;
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 300.0f;
float kd_in =  1.0f;
float t_in = 0.0f;
ros::Publisher can_topic_;


unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
   float span = x_max - x_min;
   float offset = x_min;
   unsigned int pgg = 0;
   if (bits == 12) {
      pgg = (unsigned int) ((x - offset) * 4095.0 / span);
   }
   if (bits == 16) {
      pgg = (unsigned int) ((x - offset) * 65535.0 / span);
   }

   return pgg;
}

float constrain(float in, float min, float max) {
  if (in >= max) {
     in = max;
  }
  if (in <= min)  {
     in = min;
  }

  return in;
}

bool sendTMotorCommand(alex_driver::send_tmotor_command::Request &req, alex_driver::send_tmotor_command::Response &res) {
  char buf[8];

  if (req.motorParamOut.mode == 0) {
    float p_des = constrain(req.motorParamOut.pose, P_MIN, P_MAX);
    float v_des = constrain(req.motorParamOut.speed, V_MIN, V_MAX);
    float kp = constrain(req.motorParamOut.kp, KP_MIN, KP_MAX);
    float kd = constrain(req.motorParamOut.kd, KD_MIN, KD_MAX);
    float t_ff = constrain(req.motorParamOut.torque, T_MIN, T_MAX);

    unsigned int p_int = float_to_uint(p_des, P_MIN,P_MAX, 16);
    unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    buf[0] = p_int >> 8;
    buf[1] = p_int & 0xFF;
    buf[2] = v_int >> 4;
    buf[3] = ((v_int & 0xF)  << 4) | (kp_int >> 8);
    buf[4] = kp_int & 0xFF;
    buf[5] = kd_int >> 4;
    buf[6] = ((kd_int & 0xF) << 4)  | (t_int >> 8);
    buf[7] = t_int & 0xFF;
  } else {
    switch (req.motorParamOut.mode) {
      case (1):
      buf[0] = 0xFF;
      buf[1] = 0xFF;
      buf[2] = 0xFF;
      buf[3] = 0xFF;
      buf[4] = 0xFF;
      buf[5] = 0xFF;
      buf[6] = 0xFF;
      buf[7] = 0xFC;
      break;

      case (2):
      buf[0] = 0xFF;
      buf[1] = 0xFF;
      buf[2] = 0xFF;
      buf[3] = 0xFF;
      buf[4] = 0xFF;
      buf[5] = 0xFF;
      buf[6] = 0xFF;
      buf[7] = 0xFD;
      break;

      case (3):
      buf[0] = 0xFF;
      buf[1] = 0xFF;
      buf[2] = 0xFF;
      buf[3] = 0xFF;
      buf[4] = 0xFF;
      buf[5] = 0xFF;
      buf[6] = 0xFF;
      buf[7] = 0xFE;
      break;
    }
  }
  can_msgs::Frame frame;
  if (req.motorParamOut.id == 4) {
    frame.id = 0x004;
  } else if (req.motorParamOut.id == 5) {
    frame.id = 0x005;
  }

  frame.dlc = 8;
  for (int i = 0; i < 8; i++) {
    frame.data[i] = buf[i];
  }
  /*if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    for (int i = 0; i < 8; i++) {
      frame.data[i] = buf[i];
    }
    res.success = true;
  } else {
    res.success = false;
  }*/

  can_topic_.publish(frame);

  return true;
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "alex_send_tmotor_command_node");
  ros::NodeHandle n;
  can_topic_ = n.advertise<can_msgs::Frame>("sent_messages", 10);
  ros::ServiceServer service = n.advertiseService("send_tmotor_command", sendTMotorCommand);
  ROS_INFO("Send TMotor Command Node");
  ros::spin();

  return 0;
}
