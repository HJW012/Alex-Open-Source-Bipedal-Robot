#include <ros/ros.h>
#include "std_msgs/String.h"
#include "alex_driver/Get_Dynamixel_Parameter.h"  // Get Parameter Service
#include "alex_msgs/GetDynamixelParameter.h"    // Get Parameter Msg
#include "alex_driver/Set_Dynamixel_Parameter.h"  // Set Parameter Service
#include "alex_msgs/SetDynamixelParameter.h"    // Set Parameter Msg
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "alex_global/global_definitions.h"

std::map<std::uint32_t, std::tuple<int8_t, std::string, uint32_t, uint32_t, float>> dynamixel_params = {
  // EEPROM Params - Don't change these too much, mostly just read them
  // ID, (size, name, min, max, multiplier)
  std::make_pair(7,   std::make_tuple(1, "ID", 0, 252, 1)),
  std::make_pair(8,   std::make_tuple(1, "Baud Rate", 0, 7, 1)),
  std::make_pair(9,   std::make_tuple(1, "Return Delay Time", 0, 254, 2)),
  std::make_pair(10,  std::make_tuple(1, "Drive Mode", 0, 5, 1)),
  std::make_pair(11,  std::make_tuple(1, "Operating Mode", 0, 16, 1)),
  std::make_pair(12,  std::make_tuple(1, "Secondary(Shadow) ID", 0, 252, 1)),
  std::make_pair(13,  std::make_tuple(1, "Protocol Type",	1, 2, 1)),
  std::make_pair(20,  std::make_tuple(4, "Homing Offset",	-1044479, 1044479, 1)),
  std::make_pair(24,  std::make_tuple(4, "Moving Threshold", 0, 1023, 0.229)),
  std::make_pair(31,  std::make_tuple(1, "Temperature Limit", 0, 100, 1)),
  std::make_pair(32,  std::make_tuple(2, "Max Voltage Limit", 95, 160, 0.1)),
  std::make_pair(34,  std::make_tuple(2, "Min Voltage Limit", 95, 160, 0.1)),
  std::make_pair(36,  std::make_tuple(2, "PWM Limit",	0, 885, 0.113)),
  std::make_pair(38,  std::make_tuple(2, "Current Limit",	0, 2047, 2.69)),
  std::make_pair(44,  std::make_tuple(4, "Velocity Limit", 0, 1023, 0.229)),
  std::make_pair(48,  std::make_tuple(4, "Max Position Limit", 0, 4095, 1)),
  std::make_pair(52,  std::make_tuple(4, "Min Position Limit", 0, 4095, 1)),
  std::make_pair(56,  std::make_tuple(1, "External Port Mode", 0, 3, 1)),
  std::make_pair(57,  std::make_tuple(1, "External Port Mode", 0, 3, 1)),
  std::make_pair(58,  std::make_tuple(1, "External Port Mode", 0, 3, 1)),
  std::make_pair(63,  std::make_tuple(1, "Shutdown", 0, 127, 1)),

  // RAM Params - These can be changed ltos, and must be set after every power cycle
  std::make_pair(64,	std::make_tuple(1, "Torque Enable", 0, 1, 1)),
  std::make_pair(65,	std::make_tuple(1, "LED", 0, 1, 1)),
  std::make_pair(68,	std::make_tuple(1, "Status Return Level", 0, 2, 1)),
  std::make_pair(69,	std::make_tuple(1, "Registered Instruction", 0, 1, 1)),
  std::make_pair(70,	std::make_tuple(1, "Hardware Error Status", 0, 127, 1)),
  std::make_pair(76,	std::make_tuple(2, "Velocity I Gain",	0, 16383, 1)),
  std::make_pair(78,	std::make_tuple(2, "Velocity P Gain",	0, 16383, 1)),
  std::make_pair(80,	std::make_tuple(2, "Position D Gain", 0, 16383, 1)),
  std::make_pair(82,	std::make_tuple(2, "Position I Gain", 0, 16383, 1)),
  std::make_pair(84,	std::make_tuple(2, "Position P Gain", 0, 16383, 1)),
  std::make_pair(88,	std::make_tuple(2, "Feedforward 2nd Gain", 0, 16383, 1)),
  std::make_pair(90,	std::make_tuple(2, "Feedforward 1st Gain", 0, 16383, 1)),
  std::make_pair(98,	std::make_tuple(1, "Bus Watchdog", 1, 127, 20)),
  std::make_pair(100,	std::make_tuple(2, "Goal PWM", -885, 885,	0.113)),
  std::make_pair(102,	std::make_tuple(2, "Goal Current", -2047, 2047,	2.69)),
  std::make_pair(104,	std::make_tuple(4, "Goal Velocity", -1023, 1023, 0.229)),
  std::make_pair(1081,	std::make_tuple(4, "Profile Acceleration - Velocity Based", 0, 32767, 214.577)),
  std::make_pair(1082,	std::make_tuple(4, "Profile Acceleration - Time Based", 0, 32737, 1)),
  std::make_pair(1121,	std::make_tuple(4, "Profile Velocity - Velocity Based", 0, 32767, 0.229)),
  std::make_pair(1122,	std::make_tuple(4, "Profile Velocity - Time Based", 0, 32737, 1)),
  std::make_pair(116,	std::make_tuple(4, "Goal Position", 0, 4095, 1)),
  std::make_pair(120,	std::make_tuple(2, "Realtime Tick", 0, 32767, 1)),
  std::make_pair(122,	std::make_tuple(1, "Moving", 0, 1, 1)),
  std::make_pair(123,	std::make_tuple(1, "Moving Status", 0, 127, 1)),
  std::make_pair(124,	std::make_tuple(2, "Present PWM", -855, 855, 0.113)),
  std::make_pair(126,	std::make_tuple(2, "Present Current", 0, 2047, 2.69)),
  std::make_pair(128,	std::make_tuple(4, "Present Velocity", -1023, 1023,	0.229)),
  std::make_pair(132,	std::make_tuple(4, "Present Position", -4095, 4095, 1)),
  std::make_pair(136,	std::make_tuple(4, "Velocity Trajectory", -1023, 1023, 0.229)),
  std::make_pair(140,	std::make_tuple(4, "Position Trajectory", 0, 4095, 1)),
  std::make_pair(144,	std::make_tuple(2, "Present Input Voltage", 95, 160, 0.1)),
  std::make_pair(146,	std::make_tuple(1, "Present Temperature", 0, 100,	1))
};

using namespace dynamixel;

// Control table addresses




PortHandler *portHandler;
PacketHandler *packetHandler;


// Get Parameter Services
/*bool getPresentPosition(alex_msgs::GetPresentPosition::Request &req, alex_msgs::GetPresentPosition::Response &res) {

}

bool getPresentVelocity(alex_msgs::GetPresentVelocity::Request &req, alex::msgs::GetPresentVelocity::Response &res) {

}T

bool getPresentCurrent(alex_msgs::GetPresentCurrent::Request &req, alex::msgs::GetPresentCurrent::Response &res) {

}

bool getMoving(alex_msgs::GetMoving::Request &req, alex::msgs::GetMoving::Response &res) {

}

bool getPresentTemperature(alex_msgs::GetPresentTemperature::Request &req, alex::msgs::GetPresentTemperature::Response &res) {

}

// Set Parameter Services
void setGoalPosition(alex_msgs::SetGoalPosition::Request &req, alex_msgs::SetGoalPosition::Response &res) {

}

void setGoalVelocity(alex_msgs::SetGoalVelocity::Request &req, alex_msgs::SetGoalVelocity::Response &res) {

}

void setGoalCurrent(alex_msgs::SetGoalCurrent::Request &req, alex_msgs::SetGoalCurrent::Response &res) {

}

void setVelocityPGain(alex_msgs::SetVelocityPGain::Request &req, alex_msgs::SetVelocityPGain::Response &res) {

}

void setVelocityIGain(alex_msgs::SetVelocityIGain::Request &req, alex_msgs::SetVelocityIGain::Response &res) {

}

void setPositionPGain(alex_msgs::SetPositionPGain::Request &req, alex_msgs::SetPositionPGain::Response &res) {

}

void setPositionIGain(alex_msgs::SetPositionIGain::Request &req, alex_msgs::SetPositionIGain::Response &res) {

}

void setPositionDGain(alex_msgs::SetPositionDGain::Request &req, alex_msgs::SetPositionDGain::Response &res) {

}

void setDriveMode(alex_msgs::SetDriveMode::Request &req, alex_msgs::SetDriveMode::Response &res) {

}

void setShutdown(alex_msgs::SetShutdown::Request &req, alex_msgs::SetShutdown::Response &res) {

}*/

// Check value is within limits
int32_t checkValue(int parmID, int32_t paramValue, int32_t paramMin, int32_t paramMax) {
  int32_t result = 0;
  if (paramValue > paramMax) {
    result = paramMax;
  } else if (paramValue < paramMin) {
    result = paramMin;
  } else {
    result = paramValue;
  }

  return result;
}

// Service callback
bool setParameter(alex_driver::Set_Dynamixel_Parameter::Request &req, alex_driver::Set_Dynamixel_Parameter::Response &res) {
  uint8_t dxl_error = 0;
  //int64_t value =
  return true;
}

bool getParameter(alex_driver::Get_Dynamixel_Parameter::Request &req, alex_driver::Get_Dynamixel_Parameter::Response &res) {

}



// Node main
int main(int argc, char **argv) {
  uint8_t dynamixel_error = 0;
  ros::init(argc, argv, "alex_dynamixel_command_server_node");
  ros::NodeHandle nh;

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (portHandler->openPort() == false) {
    ROS_ERROR("Failed to open Dynamixel port");
  }
  if (portHandler->setBaudRate(BAUDRATE) == false) {
    ROS_ERROR("Failed to set Dynamixel baud rate");
  }

  ros::ServiceServer set_parameter_srv = nh.advertiseService("set_dynamixel_parameter", setParameter);
  ros::ServiceServer get_parameter_srv = nh.advertiseService("get_dynamixel_parameter", getParameter);

  while (ros::ok()) {
    usleep(8 * 1000);

    ros::spin();
  }

  return 0;
}
