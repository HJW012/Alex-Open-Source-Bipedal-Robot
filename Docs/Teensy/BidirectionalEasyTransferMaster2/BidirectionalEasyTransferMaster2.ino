#include <EasyTransfer.h>

/* DEFINITIONS */
#define ID 0 // 0 is master, any other ID is slave
#define initialHeartbeat true
#define transferFrequency 50 //This includes both transmit and receive (25Hz for each)
#define transferIncrement 1000 / transferFrequency // The time increment between transmit/receive operations
#define establishCommsInterval 1000 // The time between attempts to establish comms
#define sendReceiveDelay 25 // Time between sending data and checking for receiving data - avoids misreads of sent data
#define receiveTimeout 200 // Time between send and receive that will result in timeout and loss of comms

/* DATA and COMMS */
EasyTransfer network; // Allows transfer of datastructure data
struct data_structure {
  // Comms data
  bool heartbeat;
  int dev_ID;
  bool initComms = true;
  int timeStamp; // Calculatin this will be difficult as it will be based on time since the microcontroller powered on

  // Operation data
  int testInt = 1;
};
data_structure data;
data_structure blank_data;
bool prevHeartbeat;
bool commsEstablished = false;
bool prevCommsEstablished = false;
bool transmitCycle = true; // True = transmit, False = receive
int prevTransferTime;
int prevInitTime;
int prevReceiveCheckTime;
int knownSlaveID = 1;

void setup() {
  Serial.begin(9600); // Debug Serial Stream
  Serial1.begin(9600); // Data Transfer Serial Stream

  network.begin(details(data), &Serial1);
  prevHeartbeat = !initialHeartbeat; // Must invert previous heartbeat initially in order to not throw and error on startup
  prevTransferTime = millis();
  prevInitTime = millis();

  // Debug
  pinMode(13, OUTPUT);
}

void loop() {
  if (!commsEstablished) {
    if (prevCommsEstablished == commsEstablished) {
      // Comms were not JUST lost
      establishComms();
    } else {
      // Comms were JUST lost
      Serial.println("Comms lost: Attempting to reconnect");
      establishComms();
    }
  }

  if (commsEstablished) { // Seperate if statement to start transferring data on same cycle as establishing comms

  }
}

void establishComms() {
  if (millis() - prevInitTime >= establishCommsInterval) {
    data.initComms = true;
    data_structure temp_data = data;
    data = blank_data;
    network.sendData();
    data = temp_data;
    prevInitTime = millis();
    prevReceiveCheckTime = prevInitTime;
  }

  if (millis() - prevReceiveCheckTime >= sendReceiveDelay) {
    if (network.receiveData()) {
      if (data.dev_ID == knownSlaveID) {
        commsEstablished = true; 
      }
    }
  }

}
