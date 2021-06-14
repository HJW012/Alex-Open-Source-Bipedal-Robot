#include <EasyTransfer.h>

#define ID 0
#define transferFrequency 50 //Hz - this include receive and transfer
#define transferInterval 1000 / transferFrequency //ms - time between send/receive cycles
#define commsInitFrequency 2 //Hz - attempts to establish comms with slave
#define commsInitInterval  1000 / commsInitFrequency //ms
#define receiveDataDelay 20 //ms - delay in sending and receiving data

//COMMS and DATA
EasyTransfer network;
struct data_structure {
  // Comms
  int dev_ID = 0;
  int target_dev_ID = 1;
  bool heartbeat = true;
  bool initComms = true;
  int timeStamp;

  // Data
  int testInt = 1;
};
data_structure data;
data_structure blank_data;
bool commsEstablished = false;
bool prevCommsEstablished = false;
int prevTransferTime;
int prevInitTime;
int knownSlaveID = 1;
bool transferCycle = 1; // 1 is transmit, 0 is receive
bool prevHeartbeat = true;
bool dataSent = false; // Used to send message only once per cycle but check for receive many times
bool commsHealthy = false;

int startTime;
int initTime;
int runTime;

int sequence_step = 0;

void setup() {
  Serial.begin(9600); // Debug Serial Stream
  Serial1.begin(9600);  // Comms Serial Stream

  network.begin(details(data), &Serial1);
  startTime = millis();
  prevTransferTime = millis();
  prevInitTime = millis();

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
      Serial.println("Comms just lost: Attempting to reconnect");
      establishComms();
    } else {
      // Comms weren't JUST lost
      Serial.println("Attempting to Connect");
      establishComms();
    }
  }

  if (commsEstablished) { // Seperate if statement to allow data transfer in same cycle as establishing comms
    changeData();
    maintainComms();
  }
}

void establishComms() {
  if (millis() - prevInitTime >= commsInitInterval) {
    data_structure temp_data = data;
    data = blank_data;
    data.initComms = true;
    data.dev_ID = ID;
    network.sendData();
    data = temp_data;
    prevInitTime = millis();
  } else {
    if (millis() - prevInitTime >= receiveDataDelay) {
      if (network.receiveData()) {
        if (data.dev_ID == knownSlaveID) {
          commsEstablished = true;
          data.initComms = false;
          initTime = millis(); // For timestamp
          startTime = initTime;
        }
      }
    }
  }
}

void maintainComms() {
  if (millis() - prevTransferTime >= transferInterval) {
    transferCycle = !transferCycle;
    dataSent = false;
    prevTransferTime = millis();
    transferCycle = !transferCycle;
  }

  if (transferCycle) { // Transmit
    if (!dataSent) {
      data.dev_ID = 0;
      network.sendData();
      prevHeartbeat = data.heartbeat;
      dataSent = true;
    }
  } else {  // Receive
    if (network.receiveData()) {
      if (data.heartbeat != prevHeartbeat) {
        // Correct inverted heartbeat received from slave
        commsHealthy = true;
      } else {
        // Incorrect non-inverted heartbeat received from slave
        Serial.println("Heartbeat unhealthy");
        commsHealthy = false;
      }
    }
  }

  if (commsEstablished && commsHealthy) {
    digitalWrite(13, HIGH);
  }
}

void changeData() {

}

void updateTimestamp() {
  runTime = millis() - startTime;
  data.timeStamp = runTime;
}

void sequencer(int step) {
  // sequence step 0: Establish Connection
  // sequence step 1: Maintain Connection
  // sequence step 2: Send data
  // sequence step 3: Receive data
  // sequence step 4: Calculations
  // sequence step 5: Heartbeat Unhealthy
  // sequence step 6: Connection lost
  // sequence step 7: Reconnect

  switch (step) {
    case 0: // Establish Connection
      break;
    case 1: // Maintain Connection
      break;
    case 2: // Send data
      break;
    case 3: // Receive data
      break;
    case 4: // Calculations
      break;
    case 5: // Heartbeat Unhealthy
      break;
    case 6: // Connection lost
      break;
    case 7: // Reconnect
      break;
    default:
      break;
  }
}
