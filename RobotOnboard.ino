#include <Motor.h>
#include <KalmanFilter.h>
#include <imu.h>
#include <SparkFunLSM9DS1.h>
#include <StateVector.h>
#include <ControlAlgorithm.h>
#include <BLEDevice.h>

BLEUUID serviceUUID("b221d5b4-a5d0-41a4-816f-28bff361aa96");    //service looking for - Handset
BLEUUID leftUUID("c40eac0e-d5a7-4133-a947-79577311f05b");       //UUID for left and right characteristics
BLEUUID rightUUID("c40eac0e-d5a7-4133-a947-79577311f060");
#define REFRESH_RATE 20
#define ADDR_M1 0x08
#define ADDR_M2 0x09
#define pi 3.1415926
#define BASE_MASS 2
#define PEND_MASS 4
#define PEND_LENGTH 0.3

Controller controller(PEND_MASS, BASE_MASS, PEND_LENGTH);
StateVector stateVector;
Motor motor1;
Motor motor2;
Kalman pitchFilter;
imu imu;
LSM9DS1 sensor;
bool doConnect = false;
bool connected = false;
bool doScan = false;
static BLERemoteCharacteristic* leftJoystick;
static BLERemoteCharacteristic* rightJoystick;
static BLEAdvertisedDevice* Handset;
float left_joy, right_joy = 0.0;

long loopTimer;
float pitch, roll = 0;

///////////////////////// MAIN FUNCTION //////////////////////////////

void setup() {
  pinSetup();
  initAll();
  loopTimer = micros();
}

void loop() {
 // runBLE();
  controller.algorithm(stateVector, motor1, motor2, pitchFilter, imu, sensor, 0, POSITION);

  //timing function to ensure correct loop timing
  while(micros() - loopTimer < 1000000/REFRESH_RATE);
  loopTimer = micros();
}

/////////////////////// MEMBER FUNCTIONS ////////////////////////////

void pinSetup(){
  pinMode(26,OUTPUT);           //define power pins for the LSM9DS1
  pinMode(25,OUTPUT);
  digitalWrite(26, HIGH);       //V+ pin for LSM9DS1
  digitalWrite(25, LOW);        //GND pin for LSM9DS1
}

void initAll(){
  Serial.begin(115200);                           //start serial port
  //BLESetup();                                     //Setup BLE and start advertising
  motor1.init(ADDR_M1);                           //set max speed = 255, set speed = 0, set encoder = 0
  motor2.init(ADDR_M2);
  pitchFilter.init();                             //initialise kalman filter for pitch angle
  imu.init();                                     //initialise imu object
  controller.init(REFRESH_RATE);                  //initialise control algorithm
  stateVector.init(REFRESH_RATE);                 //initialise state vector
  while(sensor.begin() == false){                 //initialise LSM9DS1 and break if unavailable
    Serial.println("Sensor unavailable!");
  }
  Serial.println("Sensor found");
  Serial.println("");
  
  imu.calcOffsets(sensor);
  delay(1000/REFRESH_RATE);
}

////////////////////////// BLE MEMBER FUNCTIONS ///////////////////////////
class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient){
  }
  void onDisconnect(BLEClient* pclient){
    connected = false;
  }
};

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

void BLESetup(){
  BLEDevice::init("Robot");
  BLEScan* Scan = BLEDevice::getScan();
  (*Scan).setAdvertisedDeviceCallbacks(new HandsetCallbacks());
  (*Scan).setInterval(1500);
  (*Scan).setWindow(500);
  (*Scan).setActiveScan(true);
  (*Scan).start(25, false);
}

void runBLE(){
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
}

void readCharacteristics(){
  left_joy = (*leftJoystick).readFloat();
  right_joy = (*rightJoystick).readFloat();
  Serial.print("Left Joystick value = ");
  Serial.println(left_joy);
  Serial.print("Right Joystick value = ");
  Serial.println(right_joy);
}

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
