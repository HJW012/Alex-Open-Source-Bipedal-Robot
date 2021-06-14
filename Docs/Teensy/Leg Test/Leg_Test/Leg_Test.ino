#include <SPI.h>
#include <mcp2515.h>
#include <AccelStepper.h>
#include <FlexCAN_T4.h>


// Motor Limits
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

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> teensy_can;
CAN_message_t teensy_msg;

MCP2515 mcp2515(10);
struct can_frame mcp_msg;

bool deviceSelect = 0; //0 for SN65HVD230, 1 for MCP2515

// CAN Message Variables
uint32_t globalID;
uint8_t globalLen;
uint8_t globalBuf[8];

// Motor Variables - Send to Motor
float globalP = 0.0f; //Position
float globalV = 0.0f; //Speed
float globalKP = 100.0f; //Stiffness
float globalKD = 1.0f; //Damper
float globalT = 0.0f; //Torque

// Reply Variables
float outP = 0.0f;
float outV = 0.0f;
float outT = 0.0f;

String commands[] = {"EnterMotorMode", "ExitMotorMode", "h", "SingleStep", "PermaStep", "DisplayData"};
String openFunctions[] = {"<SetPose>", "<MoveTo>", "<StepSpeed>", "<SeriesStep>"};
String closeFunctions[] = {"</SetPose>", "</MoveTo>", "</StepSpeed>", "</SeriesStep>"};
String serialInput; 

void setup() {
  Serial.begin(9600);
  
  if (!deviceSelect) { //SN65HVD230
    teensy_can.begin();
    teensy_can.setBaudRate(125000);
  } else { //MCP2515
    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS);
    mcp2515.setNormalMode();
  }
}

void loop() {
  if (Serial.available()) {
    serialInput = Serial.readStringUntil(0xD); //JUST CARRIAGE RETURN IN SERIAL MONITOR
    parseSerialCommand(serialInput);
  }
}

bool parseSerialCommand(String commandIn) {
  bool commandFound = false;
  bool functionFound = false;
  bool functionError = false;
  
  for (auto command : commands) {
    if (commandIn == command) {
      commandFound = true;
    } else if (!commandFound) {
      
    } else if (!commandFound && !functionFound || !commandFound && functionError) {
      //Illegal command or function
    }
  }
  
}
/*

void pack_cmd() {
  byte buf[8];

  /// CAN COMMAND PACKET STRUCTURE ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit velocity command, betweem -30 and 30 rad/s
  /// 12 bit kp, between 0 and 500 Nm/rad
  /// 12 bit kd, between 0 and 100 Nms/rad
  /// 12 bit feed forward torque, between -18 and 18 Nm
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows: For each quantity, bit 0 is Least Significant Bit
  /// 0: [position[15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity{11-4]]
  /// 3: [velocity[3-0], kp[11-8]]
  /// 4: [kp[7-0]]
  /// 5: [kd[11-4]];
  /// 6: [kd[3-0], torque[11-8]]
  /// 7: [torque[7-0]]

  // Limit data to be within bounds
  float p_des = constrain(globalP, P_MIN, P_MAX);
  float v_des = constrain(globalV, V_MIN, V_MAX);
  float kp = constrain(globalKP, KP_MIN, KP_MAX);
  float kd = constrain(globalKD, KD_MIN, KD_MAX);
  float t_ff = constrain(globalT, T_MIN, T_MAX);

  // Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  // Pack ints into CAN buffer
  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;
  CAN.sendMsgBuf(0x01, 0, 8, buf);
}

void unpack_reply() {
  /// CAN REPLY PACKET STRUCTURE
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and 30 rad/s
  /// 12 bit current, between -40 and 40
  /// CAN Packet is 5 8-bit words
  /// Formatted as follows: For each quantity, bit 0 is Lease Significant Bit
  /// 0: [position[15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], current[11-8]]
  /// 4: [current[7-0]]

  byte len = 0;
  byte buf[8];
  CAN.readMsgBuf(&len, buf);

  unsigned long canId = CAN.getCanId();

  // Unpack ints from CAN buffer
  unsigned int id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];

  // Convert unsigned ints to floats
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
}

bool sendCan(uint32_t id, uint8_t len, uint8_t buf[8]) {
  bool sendSuccess = true;

  if (!deviceSelect) { //SN65HVD230
    teensy_msg.id = id;
    teensy_msg.len = len;
    for (int i = 0; i < 8; i++) {
      teensy_msg.buf[i] = buf[i];
    }
    teensy_can.write(teensy_msg);
  } else { //MCP2515
    mcp_msg.can_id = id;
    mcp_msg.can_dlc = len;
    for (int i = 0; i < 8; i++) {
      mcp_msg.data[i] = buf[i];
    }
    mcp2515.sendMessage(&mcp_msg);
  }

  return sendSuccess;
}


unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits
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

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  /// Converts unsigned int to float, given range and number of
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float) x_int) * span / 4095.0 + offset; 
  }
  if (bits == 16) {
    pgg = ((float) x_int) * span / 65535.0 + offset;
  }
  return pgg;
}

bool enterMotorMode(uint32_t motorID) {
  uint8_t buf[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}; 
  sendCan(motorID, 8, buf);

  return true;
}

bool exitMotorMode(uint32_t motorID) {
  uint8_t buf[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}; 
  sendCan(motorID, 8, buf);

  return true;
}

bool zeroMotorPose(uint32_t motorID) {
  uint8_t buf[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; 
  sendCan(motorID, 8, buf);

  return true;
} */
