#include "ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "std_msgs/Header.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define ros_publish_freq 30 // Hz
#define publish_delay 1000 / ros_publish_freq // Time delay to achieve ros publish frequency

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
const int INTERRUPT_PIN[] = {2, 3};
#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady[] = {false, false};
uint8_t mpuIntStatus[2];   // holds actual interrupt status byte from MPU
uint8_t devStatus[2];      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize[2];    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount[2];     // count of all bytes currently in FIFO
uint8_t fifoBuffer[2][64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q[2];           // [w, x, y, z]         quaternion container
VectorInt16 aa[2];         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal[2];     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld[2];    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity[2];    // [x, y, z]            gravity vector
float euler[2][3];         // [psi, theta, phi]    Euler angle container
float ypr[2][3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt[] = {false, false}; // Used to show whether IMU interrupt pin has gone high
int idx = 0;
void dmpDataReady() {
  mpuInterrupt[idx] = true;
}

MPU6050 imu1(0x68); // AD0 low
MPU6050 imu2(0x69); // AD0 high
MPU6050 imu[] = {imu1, imu2};

// ROS Stuff
ros::NodeHandle nh;
geometry_msgs::Vector3Stamped vector3_msg;
geometry_msgs::QuaternionStamped quat_msg;
ros::Publisher vector3_pub("MPU6050", &vector3_msg);
ros::Publisher quat_pub("MPU6050", &quat_msg);

bool published = true;

int32_t lastTime = 0;

void setup() {
  Serial.begin(9600);

  nh.initNode();
#ifdef OUTPUT_READABLE_YAWPITCHROLL
  nh.advertise(vector3_pub);
#elif OUTPUT_READABLE_QUATERNION
  nh.advertise(quat_pub);
#endif

  // Join I2c bus
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  for (int i = 0; i < 2; i++) {
    idx = i;

    // Init IMUs
    Serial.println(F("Initializing IMU..."));
    imu[i].initialize();

    // verify IMU connections
    Serial.println(F("Testing IMU connections..."));
    Serial.println(imu[i].testConnection() ? F("IMU connection successful") : F("IMU connection failed"));

    // Load and configure DMP (digital motion processor)
    devStatus[i] = imu[i].dmpInitialize();

    // Gyro offsets to avoid error/drift
    imu[i].setXGyroOffset(220);
    imu[i].setYGyroOffset(76);
    imu[i].setZGyroOffset(-85);
    imu[i].setZAccelOffset(1788);

    pinMode(INTERRUPT_PIN[0], INPUT);
    if (devStatus[i] == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      imu[i].CalibrateAccel(6);
      imu[i].CalibrateGyro(6);
      imu[i].PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling IMU DMP..."));
      imu[i].setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling IMU interrupt detection (Teensy external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN[i]));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN[i]), dmpDataReady, RISING);
      mpuIntStatus[i] = imu[i].getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady[i] = true;

      // get expected DMP packet size for later comparison
      packetSize[i] = imu[i].dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("IMU DMP Initialization failed (code "));
      Serial.print(devStatus[i]);
      Serial.println(F(")"));
    }
  }

  pinMode(LED_PIN, OUTPUT);
  lastTime = millis();
}

void loop() {
  int imuCount = 0;
  double qwSum = 0;
  double qxSum = 0;
  double qySum = 0;
  double qzSum = 0;

  double rollSum = 0;
  double pitchSum = 0;
  double yawSum = 0;

  if (!dmpReady[0] && !dmpReady[1]) return;
  for (int i = 0; i < 2; i++) {
    if (dmpReady[i]) {
      if (imu[i].dmpGetCurrentFIFOPacket(fifoBuffer[i])) { // Get the Latest packet
        imuCount++;
#ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        imu[i].dmpGetQuaternion(&q[i], fifoBuffer[i]);

        qwSum += ypr[i].w;
        qxSum += ypr[i].x;
        qySum += ypr[i].y;
        qzSum += ypr[i].z;


        //          Serial.println(millis() - lastTime);
        //          lastTime = millis();

#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        imu[i].dmpGetQuaternion(&q[i], fifoBuffer[i]);
        imu[i].dmpGetGravity(&gravity[i], &q[i]);
        imu[i].dmpGetYawPitchRoll(ypr[i], &q[i], &gravity[i]);

        //        vector3_msg.header.frame_id = std_msgs::Header::_frame_id_type(i);
        //        vector3_msg.header.stamp = millis();
        rollSum += ypr[i][2];
        pitchSum += ypr[i][1];
        yawSum += ypr[i][0];

        //          Serial.println(millis() - lastTime);
        //          lastTime = millis();

#endif
      }
    }
  }

#ifdef OUTPUT_READABLE_QUATERNION
  if (imuCount > 0) {
    quat_msg.quaternion.w = qwSum / imuCount;
    quat_msg.quaternion.x = qxSum / imuCount;
    quat_msg.quaternion.y = qySum / imuCount;
    quat_msg.quaternion.z = qzSum / imuCount;

    Serial.print("quat\t");
    Serial.print(quat_msg.quaternion.w);
    Serial.print("\t");
    Serial.print(quat_msg.quaternion.x);
    Serial.print("\t");
    Serial.print(quat_msg.quaternion.y);
    Serial.print("\t");
    Serial.println(quat_msg.quaternion.z);

    
  }
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
  if (imuCount > 0) {
    vector3_msg.vector.x = rollSum / imuCount;
    vector3_msg.vector.y = pitchSum / imuCount;
    vector3_msg.vector.z = yawSum / imuCount;

    Serial.print("ypr\t");
    Serial.print(vector3_msg.vector.x * 180 / M_PI);
    Serial.print("\t");
    Serial.print(vector3_msg.vector.y * 180 / M_PI);
    Serial.print("\t");
    Serial.println(vector3_msg.vector.z * 180 / M_PI);

    
  }
#endif

  // blink LED to indicate activity
  if (millis() - lastTime >= publish_delay) {
    #ifdef OUTPUT_READABLE_QUATERNION
    quat_msg.header.stamp = nh.now();
    quat_pub.publish(&quat_msg);
    #endif
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    vector3_msg.header.stamp = nh.now();
    vector3_pub.publish(&vector3_msg);
    #endif
    nh.spinOnce();
    lastTime = millis();
  }
}
