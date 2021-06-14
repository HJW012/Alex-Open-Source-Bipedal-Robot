/*
   Control software for Teensy 4.0 - CAN transceiver module
   Advertises ROS node with one ROS service and one ROS topic
   ROS Service to transmit CAN frames
   ROS topic to publish received CAN frames
*/

#include "ros.h"
#include "can_msgs/Frame.h"
#include "std_msgs/Bool.h"
#include "alex_driver/send_can.h"
#include "FlexCAN_T4.h"

#define can_baud_rate 1000000 // Baud rate for target receiver (TMotor)
#define ros_publish_freq 30 // Hz
#define publish_delay 1000 / ros_publish_freq // Time delay to achieve ros publish frequency

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
ros::NodeHandle nh;
using alex_driver::send_can;

bool callback(const send_can::Request & req, send_can::Response & res) {
  digitalWrite(13, !digitalRead(13));
}

ros::ServiceServer<send_can::Request, send_can::Response> server("alex_send_can", &send_cmd);

can_msgs::Frame cmd_msg;
can_msgs::Frame reply_msg;

ros::Publisher reply_publisher("CAN_Reply", &reply_msg);

int prevTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  init_can();
  
  nh.initNode();
  nh.advertiseService(server);
  nh.advertise(reply_publisher);
}

void loop() {
  if (get_reply(reply_msg)) { 
    
  }

  if (millis() - prevTime >= publish_delay) {
    reply_publisher.publish( &reply_msg );
    nh.spinOnce();
    prevTime = millis();
  }
  
}

void send_cmd(const send_can::Request & req, send_can::Response & res) {
  CAN_message_t msg;
  convert_can_frame(req.msg, msg);
  
  can1.write(msg);
  res.success.data = true;
}

bool get_reply(can_msgs::Frame & msgOut) {
  CAN_message_t msgIn;
  
  if (can1.read(msgIn)) {
    convert_can_frame(msgIn, msgOut);
    return true;
  }
  return false;
}

void init_can() {
  can1.begin();
  can1.setBaudRate(1000000);
}

bool convert_can_frame(can_msgs::Frame msgIn, CAN_message_t & msgOut) {
  msgOut.id = msgIn.id;
  msgOut.flags.extended = msgIn.is_extended;
  msgOut.flags.remote = msgIn.is_rtr;
  msgOut.len = msgIn.dlc;
  
  for (int i = 0; i < 8; i++) {
    msgOut.buf[i] = msgIn.data[i];
  }
}

bool convert_can_frame(CAN_message_t msgIn, can_msgs::Frame &  msgOut) {
  msgOut.id = msgIn.id;
  msgOut.is_extended = msgIn.flags.extended;
  msgOut.is_rtr = msgIn.flags.remote;
  msgOut.dlc = msgIn.len;
  msgOut.header.stamp = nh.now();

  for (int i = 0; i < 8; i++) {
    msgOut.data[i] = msgIn.buf[i];
  }
}
