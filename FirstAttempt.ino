#include <BLEDevice.h>

//service looking for (i.e. HANDSET)
BLEUUID serviceUUID("b221d5b4-a5d0-41a4-816f-28bff361aa96");
//UUID of left and right joystick characteristics
BLEUUID leftUUID("c40eac0e-d5a7-4133-a947-79577311f05b");
BLEUUID rightUUID("c40eac0e-d5a7-4133-a947-79577311f060");

bool doConnect = false;
bool connected = false;
bool doScan = false;
static BLERemoteCharacteristic* leftJoystick;
static BLERemoteCharacteristic* rightJoystick;
static BLEAdvertisedDevice* Handset;
float left_joy = 0.0;
float right_joy = 0.0;

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient){
  }
  void onDisconnect(BLEClient* pclient){
    connected = false;
  }
};

bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println((*Handset).getAddress().toString().c_str());

  //create client
  BLEClient* Robot = BLEDevice::createClient();
  Serial.println("-> Created client");

  //connect to server
  (*Robot).setClientCallbacks(new ClientCallbacks());
  (*Robot).connect(Handset);
  Serial.println("-> Connected to Server");

  //check that the service matches the one that we're expecting and disconnect if not
  BLERemoteService* HandsetService = (*Robot).getService(serviceUUID);
  if (HandsetService == nullptr){
    Serial.print("---->Failed to find service ");
    Serial.println(serviceUUID.toString().c_str());
    (*Robot).disconnect();
    return false;
  }

  //check that the characteristics match up and disconnect if not
  leftJoystick = (*HandsetService).getCharacteristic(leftUUID);
  if(leftJoystick == nullptr) {
    Serial.print("---->Failed to find characteristic ");
    Serial.println(leftUUID.toString().c_str());
    (*Robot).disconnect();
    return false;
  }
  rightJoystick = (*HandsetService).getCharacteristic(rightUUID);
  if(rightJoystick == nullptr) {
    Serial.print("---->Failed to find characteristic ");
    Serial.println(rightUUID.toString().c_str());
    (*Robot).disconnect();
    return false;
  }
  Serial.println("-> Characteristics found");

  //now read the characteristics into variables
  if((*leftJoystick).canRead() && (*rightJoystick).canRead()){
    doConnect = false;
    //readCharacteristics();
  } else {
    Serial.println("Unable to read characteristics!");
  }
  connected = true;
  return true;
}

void readCharacteristics(){
  left_joy = (*leftJoystick).readFloat();
  right_joy = (*rightJoystick).readFloat();
  Serial.print("Left Joystick value = ");
  Serial.println(left_joy);
  Serial.print("Right Joystick value = ");
  Serial.println(right_joy);
}

class HandsetCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice){
    Serial.print("Advertising Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
    
    if(advertisedDevice.haveServiceUUID() and advertisedDevice.isAdvertisingService(serviceUUID)){
      BLEDevice::getScan()->stop();
      Handset = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  BLEDevice::init("Robot");
  BLEScan* Scan = BLEDevice::getScan();
  (*Scan).setAdvertisedDeviceCallbacks(new HandsetCallbacks());
  (*Scan).setInterval(1500);
  (*Scan).setWindow(500);
  (*Scan).setActiveScan(true);
  (*Scan).start(25, false);
}

void loop() {
  //connect to server if it has been found
  if(doConnect){
    if(connectToServer()){
      Serial.println("Connected to BLE Server");
    }
    else {
      Serial.println("Server connection failed!");
    }
    //doConnect = false;
  }
  else if (connected){
    readCharacteristics();
  }

  //update characteristics if connected
  if(connected) {
    //output to Tiva Board
  }
  else if (doScan){
    BLEDevice::getScan()->start(0);
  }

  delay(100);   //0.1s delay between reiterations
}
