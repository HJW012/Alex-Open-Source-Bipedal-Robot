#include <EasyTransfer.h>

#define ID 1

//EasyTransfer ET;
bool master = true;

struct data_structure {
  bool commsEstablished = false;
  bool commsError = false;
  bool heartbeat = true;

  // Master to Slave data
  bool requestResponse = false;
  bool responseError = false; // Error in response from slave to master - instead of ack or nack
  int16_t slaveID = 1;

  // Slave to Master data
  bool responseSent = false;
  bool commandError = false; // Error in command from master to slave - instead of ack or nack
  int16_t masterID = 0;


  bool blink1 = false;
  bool blink2 = false;

};

class device {
  private:
    int _id = 0;
    
  public:
    bool connected = false;
    int connectedSlaves[10];
  
    void setID(int ID) {
      _id = ID;
    }
}

// Only 1 master in network
class master {
  private:
    
  public:
    master() {
      ET.begin(details(data), &Serial1);
    }

    // Connect to the slave ID in param
    void connect(int ID) {
      if (!connected && !data.commsEstablished) {
        if (ET.receiveData() && ) {
          
        }
      }
    }

    /*


    if (!data.commsEstablished) {
      Serial.println("Slave to Master Comms not Established");
      digitalWrite(13, LOW);
      if (ET.receiveData()) {
        data.commsEstablished = true;
        ET.sendData();
      }
    }

    */

    // Maintain connection with slave ID in param
    bool maintainConnection(int ID) {
      
    }
};

// Multiple slaves in network
class slave {
  private:

  public:
    slave() {

    }
};

data_structure data;
bool prevHeartbeat;
int buttonPin = 11;
bool buttonPressed = false;
int slaveIDs[] = {1};

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  




  
  ET.begin(details(data), &Serial1);
  pinMode(13, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  prevHeartbeat = !data.heartbeat;
}

void loop() {

  // Master Device
  if (master) {
    if (!data.commsEstablished) {
      ET.sendData();
      if (ET.receiveData()) {

      }
    }
    if (data.commsEstablished) {
      if (buttonPressed) {
        data.requestResponse = true;
        data.blink1 = true;
      } else {
        data.blink1 = false;
      }

      if (data.requestResponse) {
        prevHeartbeat = data.heartbeat;
        if (ET.receiveData()) {
          if (data.heartbeat != prevHeartbeat) {
            //Comm good
            Serial.print("HB: ");
            Serial.println(data.heartbeat);
            data.requestResponse = false;
          } else {
            //Comm error
            Serial.println("Master comm error");
          }
        }
      }
      ET.sendData();
    }
  }

  // Slave Device
  if (!master) {
    if (!data.commsEstablished) {
      if (ET.receiveData()) {
        data.commsEstablished = true;
        ET.sendData();
      }
    }

    if (data.commsEstablished) {
      if (ET.receiveData()) {
        if (data.requestResponse) {
          prevHeartbeat = data.heartbeat;
          if (data.heartbeat == prevHeartbeat) {
            data.heartbeat = !data.heartbeat;
            Serial.println("Response Requested");

            if (data.blink1) {
              if (!digitalRead(buttonPin)) {
                data.blink2 = true;
              } else {
                data.blink2 = false;
              }
            } else {
              data.blink2 = false;
            }
            ET.sendData();
          } else {
            //Comm error
            Serial.println("Slave comm error");
          }
        }
      }
    }
  }

  Serial.print("blink1: ");
  Serial.print(data.blink1);
  Serial.print("\t");
  Serial.print("blink2: ");
  Serial.println(data.blink2);
  delay(2);
}

void handleComms() {
  // Master Device
  if (master) {
    for (int x : slaveIDs) {

    }


    // If comms aren't established between master and slave, send a normal command
    // This will run the first time devices are connecting as well as every time comms are lost
    if (!data.commsEstablished) {
      Serial.println("Master to Slave Comms not Established");
      digitalWrite(13, LOW);
      ET.sendData();
      if (ET.receiveData()) {

      }
    }

    // If comms are established between master and slave, start normal commands
    if (data.commsEstablished) {
      Serial.println("Master to Slave Comms Established");
      digitalWrite(13, HIGH);
      // Command procedure

      // End Command Procedure

      // If master requested a response from slave, check for incoming messages
      if (data.requestResponse) {
        prevHeartbeat = data.heartbeat;
        if (ET.receiveData()) {
          if (data.heartbeat != prevHeartbeat && data.responseSent) {
            //Comm good
            data.requestResponse = false;
            Serial.println("Master to Slave Comms Healthy");
            digitalWrite(13, HIGH);
          } else {
            //Comm error
            Serial.println("Master to Slave Comms Unhealthy");
            digitalWrite(13, LOW);
          }
        }
      }
      ET.sendData();
    }
  }







  // Slave Device
  if (!master) {
    // If comms aren't established between slave and master, keep checking for incoming messages
    if (!data.commsEstablished) {
      Serial.println("Slave to Master Comms not Established");
      digitalWrite(13, LOW);
      if (ET.receiveData()) {
        data.commsEstablished = true;
        ET.sendData();
      }
    }

    if (data.commsEstablished) {
      Serial.println("Master to Slave Comms Established");
      if (ET.receiveData()) {
        if (data.requestResponse) {
          prevHeartbeat = data.heartbeat;
          if (data.heartbeat == prevHeartbeat) {
            data.heartbeat = !data.heartbeat;
            Serial.println("Response Requested");

            if (data.blink1) {
              if (!digitalRead(buttonPin)) {
                data.blink2 = true;
              } else {
                data.blink2 = false;
              }
            } else {
              data.blink2 = false;
            }
            ET.sendData();
          } else {
            //Comm error
            Serial.println("Slave comm error");
          }
        }
      }
    }
  }
}
