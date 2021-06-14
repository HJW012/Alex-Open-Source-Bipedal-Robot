#include <EasyTransfer.h>

#define ID 0
#define initialHeartbeat true
#define transferFrequency 50
#define responseFrequency 25
int transferIncrement = 1000 / transferFrequency; // In Millis
int responseIncrement = transferFrequency / responseFrequency; // In transfers (response Increment of 2 would respond every second transfer from the master)
int prevTime;
int currentTime;
int responseTimeout = 10; // Max wait time from slave in millis

struct data_structure {
  bool initComms = true;
  bool heartbeat = initialHeartbeat;
  bool requestResponse;
};

data_structure data;
data_structure backup_data;

bool prevHeartbeat;
EasyTransfer network;

bool role = 0; // Master: 0, Slave: 1

bool commsEstablished = false;
bool prevCommsEstablished = false;
int transferIndex = 0;

void setup() {
  Serial.begin(9600); // Debug serial stream
  Serial1.begin(9600); // EasyTransfer stream - this will also be used for RS485

  network.begin(details(data), &Serial1);
  prevHeartbeat = !initialHeartbeat;
  prevTime = millis();

  pinMode(13, OUTPUT);
}

// Operation Overview
// 1. Establish Connection
// 2. Maintain Connection
// 3. Manipulate data
// 4. Commms
//  3.1 Send Data
//  3.2 Receive Response

void loop() {
  digitalWrite(13, LOW);
  
  // 1: If no connection established, wait until connection is esteblaished
  while (!commsEstablished) {
    if (!prevCommsEstablished) {
      Serial.println("Attempting to establish connection");
    } else {
      Serial.println("Connection Lost: Attempting to reconnect");
    }
    establishConnection();
    delay(1000);
  }
  data.initComms = false;
  digitalWrite(13, HIGH);
  // 2: Maintain Connection
  

  // 3: Manipulate data

  // 4: Comms
  // 4.1: Send Data

  // 4.2: Receive Resposne


  // 3: Now that comms are established, begin transferring data between master and slaves
  if (commsEstablished) {
    maintainConnection();
  }


  prevCommsEstablished = commsEstablished;
}

void connectionLost() {
  backup_data = data;
  data_structure blankData;
  data = blankData;
}

void establishConnection() {
  data.initComms = true;
  network.sendData();
  delay(50);
  if (network.receiveData()) {
    commsEstablished = true;
    Serial.println("Connection Established");
  }
}

// Check heartbeat and continue transferring data
// Master sends and echos heartbeat from slave, slave inverts heartbeat and sends back
void maintainConnection() {
  int currentIncrement = millis() - prevTime;
  if (currentIncrement >= transferIncrement) {
    prevHeartbeat = data.heartbeat;
    network.sendData();
    transferIndex++;
    if (transferIndex >= responseIncrement) {
      data.requestResponse = true;
      transferIndex = 0;
    }
    if (data.requestResponse) {
      data.requestResponse = false;
      int responseWaitTime;
      bool timeout = false;
      if (!network.receiveData()) {
        responseWaitTime = millis();
      }
      while (!network.receiveData()) {
        if (millis() - responseWaitTime >= responseTimeout) {
          timeout = true;
          break;
        }
      }
      if (!timeout) {
        if (data.heartbeat != prevHeartbeat) {
          // Heartbeat healthy
          Serial.println("Heartbeat healthy");
        } else {
          // Heartbeat unhealthy
          Serial.println("Heartbeat unhealthy");
          commsEstablished = false;
        }

      } else {
        // Comms unhealthy due to repsonse timeout
        Serial.println("Response time unhealthy");
        commsEstablished = false;
      }
    }

  }

  if (data.requestResponse) {
    prevHeartbeat = data.heartbeat;

  }
}
