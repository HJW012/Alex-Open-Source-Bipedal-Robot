#include <FlexCAN_T4.h>

#define motor_button_pin 0
bool motor_button_debounce = false;
#define zero_button_pin 1
bool zero_button_debounce = false;
#define cw_button_pin 3
#define ccw_button_pin 2

#define red_pin 4
#define green_pin 5

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t tmotor_command;
CAN_message_t tmotor_reply;
#define motor_ID 0x001
#define can_length 8

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

// Set Values
float p_in = 0.0f;
float v_in = 1.0f;
float kp_in = 100.0f;
float kd_in = 1.0f;
float t_in = 1.0f;

// Measured Values
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

float p_step = 0.01;

bool motorMode = false;

void setup() {
  Serial.begin(9600);
  can1.begin();
  can1.setBaudRate(1000000);

  pinMode(motor_button_pin, INPUT_PULLUP);
  pinMode(zero_button_pin, INPUT_PULLUP);
  pinMode(cw_button_pin, INPUT_PULLUP);
  pinMode(ccw_button_pin, INPUT_PULLUP);
  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
}

long previousMillis = 0;
void loop() {
  buttonControl();
  ledState();
  pack_cmd();
  if (can1.read(tmotor_reply)) {
    unpack_reply();
  }
  //displayData();
  delay(1);
}

void buttonControl() {
  if (!digitalRead(motor_button_pin) && !motor_button_debounce) {
    motorMode = !motorMode;
    setMotorMode(motorMode);
    motor_button_debounce = true;
  } else if (digitalRead(motor_button_pin)) {
    motor_button_debounce = false;
  }

  if (motorMode) {
    if (!digitalRead(zero_button_pin) && !zero_button_debounce) {
      zeroMotorPose();
      zero_button_debounce = true;
    } else if (digitalRead(zero_button_pin)) {
      zero_button_debounce = false;
    }

    if (!digitalRead(cw_button_pin)) {
      p_in = p_in + p_step;
    }

    if (!digitalRead(ccw_button_pin)) {
      p_in = p_in - p_step;
    }
  }
  p_in = constrain(p_in, P_MIN, P_MAX);
}

void ledState() {
  if (motorMode) {
    digitalWrite(red_pin, LOW);
    digitalWrite(green_pin, HIGH);
  } else {
    digitalWrite(green_pin, LOW);
    digitalWrite(red_pin, HIGH);
  }
}

void displayData() {
  Serial.print(millis() - previousMillis);
  previousMillis = millis();
  Serial.print(" ");
  Serial.print(p_in);
  Serial.print(" ");
  Serial.print(p_out);
  Serial.print(" ");
  Serial.print(v_in);
  Serial.print(" ");
  Serial.print(v_out);
  Serial.print(" ");
  Serial.print(t_in);
  Serial.print(" ");
  Serial.print(t_out);
  Serial.println();

}

void setMotorMode(bool state) {
  tmotor_command.id = motor_ID;
  tmotor_command.len = can_length;
  if (state) {
    tmotor_command.buf[0] = 0xFF;
    tmotor_command.buf[1] = 0xFF;
    tmotor_command.buf[2] = 0xFF;
    tmotor_command.buf[3] = 0xFF;
    tmotor_command.buf[4] = 0xFF;
    tmotor_command.buf[5] = 0xFF;
    tmotor_command.buf[6] = 0xFF;
    tmotor_command.buf[7] = 0xFC;
  } else {
    tmotor_command.buf[0] = 0xFF;
    tmotor_command.buf[1] = 0xFF;
    tmotor_command.buf[2] = 0xFF;
    tmotor_command.buf[3] = 0xFF;
    tmotor_command.buf[4] = 0xFF;
    tmotor_command.buf[5] = 0xFF;
    tmotor_command.buf[6] = 0xFF;
    tmotor_command.buf[7] = 0xFD;
  }

  can1.write(tmotor_command);
  resetCommand();
}

void zeroMotorPose() {
  tmotor_command.id = motor_ID;
  tmotor_command.len = can_length;
  tmotor_command.buf[0] = 0xFF;
  tmotor_command.buf[1] = 0xFF;
  tmotor_command.buf[2] = 0xFF;
  tmotor_command.buf[3] = 0xFF;
  tmotor_command.buf[4] = 0xFF;
  tmotor_command.buf[5] = 0xFF;
  tmotor_command.buf[6] = 0xFF;
  tmotor_command.buf[7] = 0xFE;

  p_in = 0;
  can1.write(tmotor_command);
  resetCommand();
}

void pack_cmd() {
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
  p_in = 90.0f;
  v_in = 0.0f;
  kp_in = 500.0f;
  kd_in = 5.0f;
  t_in = 0.0f;
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
  tmotor_command.id = 0x001;
  tmotor_command.len = 8;
  tmotor_command.buf[0] = p_int >> 8;
  tmotor_command.buf[1] = p_int & 0xFF;
  tmotor_command.buf[2] = v_int >> 4;
  tmotor_command.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  tmotor_command.buf[4] = kp_int & 0xFF;
  tmotor_command.buf[5] = kd_int >> 4;
  tmotor_command.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  tmotor_command.buf[7] = t_int & 0xFF;

  for (int i = 0; i < 8; i++) {
    Serial.print(tmotor_command.buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
//  Serial.print(uint_to_float(p_int, P_MIN, P_MAX, 16));
//  Serial.print("  ");
//  Serial.print(uint_to_float(v_int, V_MIN, V_MAX, 12));
//  Serial.print("  ");
//  Serial.print(uint_to_float(kp_int, KP_MIN, KP_MAX, 12));
//  Serial.print("  ");
//  Serial.print(uint_to_float(kd_int, KD_MIN, KD_MAX, 12));
//  Serial.print("  ");
//  Serial.println(uint_to_float(t_int, T_MIN, T_MAX, 12));

  can1.write(tmotor_command);
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
  unsigned long canId = tmotor_reply.id;
  /*Serial.print("Raw HEX Response: ");
    for (int i = 0; i < 8; i++) {
    Serial.print(tmotor_reply[i], HEX);
    Serial.print(" ");
    }
    Serial.println();*/

  // Unpack ints from CAN buffer
  unsigned int id = tmotor_reply.buf[0];
  unsigned int p_int = (tmotor_reply.buf[1] << 8) | tmotor_reply.buf[2];
  unsigned int v_int = (tmotor_reply.buf[3] << 4) | (tmotor_reply.buf[4] >> 4);
  unsigned int i_int = ((tmotor_reply.buf[4] & 0xF) << 8) | tmotor_reply.buf[5];

  // Convert unsigned ints to floats
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
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
  //pgg = (unsigned int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
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

void resetCommand() {
  tmotor_command.buf[0] = 0x00;
  tmotor_command.buf[1] = 0x00;
  tmotor_command.buf[2] = 0x00;
  tmotor_command.buf[3] = 0x00;
  tmotor_command.buf[4] = 0x00;
  tmotor_command.buf[5] = 0x00;
  tmotor_command.buf[6] = 0x00;
  tmotor_command.buf[7] = 0x00;
}
