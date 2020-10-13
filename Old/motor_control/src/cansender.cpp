#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>



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

#define P_MIN -95.5f
#define P_MAX 95.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

int s;
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 100.0f;
float kd_in =  1.0f;
float t_in = 0.0f;

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

void pack_cmd() {
   char buf[8];
   
   float p_des = constrain(p_in, P_MIN, P_MAX);
   float v_des = constrain(v_in, V_MIN, V_MAX);
   float kp = constrain(kp_in, KP_MIN, KP_MAX);
   float kd = constrain(kd_in, KD_MIN, KD_MAX);
   float t_ff = constrain(t_in, T_MIN, T_MAX);
   
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
   
   struct can_frame frame;
   
   frame.can_id = 0x01;
   frame.can_dlc = 8;
   if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      perror("Write");
   }
}

void canSend(const geometry_msgs::Twist::ConstPtr& msg)
{
   
   v_in = msg->angular.z;
   //std::cout << "v_in: " << v_in << std::endl;
   
   char buf[8];
   
   float p_des = constrain(p_in, P_MIN, P_MAX);
   float v_des = constrain(v_in, V_MIN, V_MAX);
   float kp = constrain(kp_in, KP_MIN, KP_MAX);
   float kd = constrain(kd_in, KD_MIN, KD_MAX);
   float t_ff = constrain(t_in, T_MIN, T_MAX);
   //std::cout << "v_des: " << v_des << std::endl;
   
   unsigned int p_int = float_to_uint(p_des, P_MIN,P_MAX, 16);
   unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
   unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
   unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
   unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
   //std::cout << "v_int: " << v_int << std::endl;
   
   buf[0] = p_int >> 8;
   buf[1] = p_int & 0xFF;
   buf[2] = v_int >> 4;
   buf[3] = ((v_int & 0xF)  << 4) | (kp_int >> 8);
   buf[4] = kp_int & 0xFF;
   buf[5] = kd_int >> 4;
   buf[6] = ((kd_int & 0xF) << 4)  | (t_int >> 8);
   buf[7] = t_int & 0xFF;
   //std::cout << "buf[3]: " << (unsigned int)buf[3] << std::endl;
   
   struct can_frame frame;
   
   frame.can_id = 0x01;
   frame.can_dlc = 8;
   if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      perror("Write");
      std::cout << "Write Issue" << std::endl;
   }  
}

int main(int argc, char **argv)
{
   
   if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      perror("Socket");
      return -1;
   }

   struct ifreq ifr;
  
   strcpy(ifr.ifr_name, "can0");
   ioctl(s, SIOCGIFINDEX, &ifr);

   struct sockaddr_can addr;

   memset(&addr, 0, sizeof(addr));
   addr.can_family = AF_CAN;
   addr.can_ifindex = ifr.ifr_ifindex;
   
   if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) <0) {
      perror("Bind");
      return 1;
   }  
   
  ros::init(argc, argv, "cansender");

  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, canSend);

  ros::spin();

  return 0;
}
