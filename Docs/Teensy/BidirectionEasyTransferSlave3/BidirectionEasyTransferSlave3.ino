#include <EasyTransfer.h>

#define ID 1
#define masterID 0
#define transferFrequency 50 //Hz - this include receive and transfer
#define transferInterval 1000 / transferFrequency //ms - time between send/receive cycles
#define commsInitFrequency 2 //Hz - attempts to establish comms with slave
#define commsInitInterval  1000 / commsInitFrequency //ms
#define receiveDataDelay 20 //ms - delay in sending and receiving data

// COMMS and DATA
EasyTransfer network;
struct data_structure {
  // Comms
  int dev_ID;
  int target_dev_ID;
  bool heartbeat;
  bool initComms;
  int timeStamp;

  // Data
  int testInt;
};
data_structure data;
data_structure blank_Data;
bool commsEstablished = false;
bool prevCommsEstablished = false;
bool transferCycle = 0; // 1 is transmist, 0 is receive
bool prevHeartbeat = true;
bool dataSent = false;
bool commsHealthy = false;

int startTime;
int initTime;
int runTime;

void setup() {
  Serial.begin(9600); // Debug Serial Stream
  Serial1.begin(9600); // COmms Serial Stream

  network.begin(details(data), &Serial1);
  startTime = millis();

  // DEBUG
  pinMode(13, OUTPUT);
}

void loop() {
  /* DEBUG */
  if (commsEstablished && commsHealthy) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
  /* END DEBUG */

  if (!commsEstablished) {
    if (prevCommsEstablished) {
      // Comms were JUST lost
      Serial.println("Comms with master just lost: Attempting to reconnect");
      establishComms();
    } else {
      // Comms weren't JUST lost
      Serial.println("Attempting to Connect with master");
      establishComms();
    }
  }

  if (commsEstablished) {
    changeData();
    maintainComms();
  }
}

void establishComms() {
  if (network.receiveData()) {
    if (data.dev_ID = masterID && data.dev_ID == ID && data.initComms) {
      commsEstablished = true;
      delay(receiveDataDelay);
      network.sendData();
    }
  }
}

void maintainComms() {
  if (!transferCycle) { // Receive
    if (network.receiveData()) {
      transferCycle = true;
      dataSent = false;
      if (data.heartbeat == prevHeartbeat) {
        // Correct echoed heartbeat received from master
        data.heartbeat = !data.heartbeat;
        commsHealthy = true;
      } else {
        // Incorrect inverted heartbeat received from master
        Serial.println("Heartbeat unhealthy");
        commsHealthy = false;
      }
    }
  } else {  // Transmit
    if (!dataSent) {
      data.dev_ID = 1;
      network.sendData();
      prevHeartbeat = data.heartbeat;
      dataSent = true;
      transferCycle = false;
    }
  }
}

void changeData() {

}

void updateTimestamp() {

  runTime = millis() - startTime;

}
