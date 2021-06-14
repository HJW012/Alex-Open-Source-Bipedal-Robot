#include <FlexCAN_T4.h>

#define ID1 4
#define ID2 5

byte enterMotorModeBuf[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
byte exitMotorModeBuf[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
byte zeroMotorBuf[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

bool inMotorMode = false;

// Value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// Set Values - send to motor
float p_in = 0.0f; //Position
float v_in = 0.0f; //Speed
float kp_in = 100.0f; //Stiffness
float kd_in = 1.0f; //Damper
float t_in = 0.0f; //Torque

// Measured Values - received from motor
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

//float savedPoses[2][25];
//int poseIndex = 0;
#define savePoseButton 15
#define playButton 16
#define motorModeButton 17
#define ledPin 13
#define playbackSpeedMin 0
#define playbackSpeedMax 1000 // Potentiometer controlling how quickly it moves between poses

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
CAN_message_t canMsg2;
bool motorMode = false;
bool debounce1 = false;
bool debounce2 = false;
bool debounce3 = false;

uint32_t motorID[] = {0x004, 0x005};
float motorPose[] = {0, 0};
int poseIndex = 0;
float poses[2][5];

void setup() {
  Serial.begin(9600);

  pinMode(savePoseButton, INPUT_PULLUP);
  pinMode(playButton, INPUT_PULLUP);
  pinMode(motorModeButton, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  can1.begin();
  can1.setBaudRate(1000000);
  msg.id = 0x005; // 4 and 5
  msg.len = 8;
  delay(500);
}

void loop() {
  // On button press, save positions of eachmotor
  // use potentiometer to find playback speed
  // on play button press, send motor commands - check for pose - then send again

  /*Serial.print("ID: ");
    Serial.print(motorID[0]);
    Serial.print(": ");
    Serial.println(motorPose[0]);

    Serial.print("ID: ");
    Serial.print(motorID[1]);
    Serial.print(": ");
    Serial.println(motorPose[1]);*/


  if (!digitalRead(motorModeButton) && !debounce1) {
    debounce1 = true;
    toggleMotorMode(0x004);
    toggleMotorMode(0x005);
    motorMode = !motorMode;
  } else if (digitalRead(motorModeButton)) {
    debounce1 = false;
  }

  if (!digitalRead(savePoseButton) && !debounce2) {
    debounce2 = true;
    savePoses();
    //    getPositions();
    //    test();

  } else if (digitalRead(savePoseButton)) {
    debounce2 = false;
  }

  if (!digitalRead(playButton) && !debounce3) {
    debounce3 = true;
    playPoses();

  } else if (digitalRead(playButton)) {
    debounce3 = false;
  }

  delay(10);
}

void toggleMotorMode(uint32_t ID) {
  for (auto x : motorID) {
    msg.id = x;
    if (!motorMode) {
      msg.buf[0] = 0xFF;
      msg.buf[1] = 0xFF;
      msg.buf[2] = 0xFF;
      msg.buf[3] = 0xFF;
      msg.buf[4] = 0xFF;
      msg.buf[5] = 0xFF;
      msg.buf[6] = 0xFF;
      msg.buf[7] = 0xFC;
    } else {
      msg.buf[0] = 0xFF;
      msg.buf[1] = 0xFF;
      msg.buf[2] = 0xFF;
      msg.buf[3] = 0xFF;
      msg.buf[4] = 0xFF;
      msg.buf[5] = 0xFF;
      msg.buf[6] = 0xFF;
      msg.buf[7] = 0xFD;
    }
    can1.write(msg);
    delay(5);
  }
}

void playPoses() { // No status checkin
  Serial.println("Playing back Poses");
  if (poseIndex > 0) {
    for (int i = 0; i < poseIndex; i++) {
      for (int j = 0; j < 2; j++) {
        msg.id = motorID[j];
        p_in = poses[j][i];
        kp_in = 200;
        kd_in = 2.5;
        t_in = 18;
        pack_cmd();
//        t_in = 5;
        can1.write(msg);
        delay(2);
      }
      delay(100);
    }
  }
}

void savePoses() {
  Serial.println();

  for (int i = 0; i < 2; i++) {
    msg.id = motorID[i];
    p_in = 0;
    v_in = 0;
    kp_in = 0;
    kd_in = 0;
    t_in = 0;
    can1.write(msg);
    delay(5);
    unpack_reply();
    motorPose[i] = p_out;
    delay(5);
  }
  if (poseIndex < 5) {
    for (int i = 0; i < 2; i++) {
      poses[i][poseIndex] = motorPose[i];
      Serial.println("Poses saved");
      Serial.print(motorID[i]);
      Serial.print(": ");
      Serial.println(motorPose[i]);
    }
    poseIndex++;
  }
}

void test() {
  p_in = 0;
  v_in = 0;
  kp_in = 0;
  kd_in = 0;
  t_in = 0;
  pack_cmd();
  msg.id = 0x004;
  /*msg.buf[0] = 0x00;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;*/
  can1.write(msg);
}

void getPositions() {
  for (int i = 0; i < 2; i++) {
    //for (int j = 0; j < 2; j++) {
    msg.id = motorID[i];
    msg.buf[0] = 0x00;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    can1.write(msg);
    delay(5);
    unpack_reply();
    motorPose[i] = p_out;
    delay(5);
    //}
  }
  /*
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (auto x : motorID) {
          msg.id = x;
          msg.buf[0] = 0x00;
          msg.buf[1] = 0x00;
          msg.buf[2] = 0x00;
          msg.buf[3] = 0x00;
          msg.buf[4] = 0x00;
          msg.buf[5] = 0x00;
          msg.buf[6] = 0x00;
          msg.buf[7] = 0x00;
          can1.write(msg);

          unpack_reply();
          motorPose[i] = p_out;
          delay(5);
        }
      }
    }*/
}


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
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);

  // Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  // Pack ints into CAN buffer
  msg.buf[0] = p_int >> 8;
  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = v_int >> 4;
  msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  msg.buf[4] = kp_int & 0xFF;
  msg.buf[5] = kd_int >> 4;
  msg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  msg.buf[7] = t_int & 0xFF;

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
  byte buf[8];
  byte len = 0;
  if (can1.read(canMsg2)) {
    unsigned long canId = canMsg2.id;
    //Serial.print("Raw HEX Response: ");
    for (int i = 0; i < 8; i++) {
      buf[i] = canMsg2.buf[i];
      //Serial.print(buf[i], HEX);
      // Serial.print(" ");
    }
    // Serial.println();

    // Unpack ints from CAN buffer
    unsigned int id = buf[0];
    unsigned int p_int = (buf[1] << 8) | buf[2];
    unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
    unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];

    // Convert unsigned ints to floats
    p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
    v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
    t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    /*
      Serial.print("Pose: ");
      Serial.print(p_out);
      Serial.print("\tVel: ");
      Serial.print(v_out);
      Serial.print("\tTorque: ");
      Serial.println(t_out);*/
  }
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
