#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define INPUT_UUID              "b221d5b4-a5d0-41a4-816f-28bff361aa96"
#define LEFT_JOYSTICK_UUID      "c40eac0e-d5a7-4133-a947-79577311f05b"
#define RIGHT_JOYSTICK_UUID     "c40eac0e-d5a7-4133-a947-79577311f060"
#define HORIZONTAL 33
#define VERTICAL 32

void pinSetup();
void getInput();

float leftRight, upDown = 0;
float verticalValue, horizontalValue = 0;
float latOffset, verOffset = 0;
float left[10] = {-1, -0.8, -0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6, 0.8};
float right[10] = {-0.9, -0.7, -0.5, -0.3, -0.1, 0.1, 0.3, 0.5, 0.7, 0.9};
float left_joy = 0;
float right_joy = 0;
bool left_flag = false;
bool right_flag = false;

BLECharacteristic *leftJoystick;
BLECharacteristic *rightJoystick;

void setup() {
  Serial.begin(115200);
  pinSetup();
  BLEDevice::init("HANDSET");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = (*pServer).createService(INPUT_UUID);
  leftJoystick = (*pService).createCharacteristic(
                                      LEFT_JOYSTICK_UUID,
                                      BLECharacteristic::PROPERTY_READ |
                                      BLECharacteristic::PROPERTY_WRITE);
  rightJoystick = (*pService).createCharacteristic(
                                      RIGHT_JOYSTICK_UUID,
                                      BLECharacteristic::PROPERTY_READ |
                                      BLECharacteristic::PROPERTY_WRITE);

  //(*leftJoystick).setValue("Hello");
  //(*rightJoystick).setValue("Hi");

  class LeftCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *leftJoystick){
      (*leftJoystick).setValue(left_joy);
    }
  };

  class RightCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *rightJoystick){
      (*rightJoystick).setValue(right_joy);
    }
  };

  (*leftJoystick).setCallbacks(new LeftCallbacks());
  (*rightJoystick).setCallbacks(new RightCallbacks());

  
  //start connection
  (*pService).start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  (*pAdvertising).addServiceUUID(INPUT_UUID);
  (*pAdvertising).setScanResponse(true);
  (*pAdvertising).setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  
}

void loop() {
  getInput();
  left_joy = horizontalValue;
  right_joy = verticalValue;
  delay(500);
}

void pinSetup(){
  pinMode(HORIZONTAL, INPUT);
  pinMode(VERTICAL, INPUT);
  pinMode(34, OUTPUT);
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);
  digitalWrite(34, LOW);
  for(int i = 0; i < 25; i++){
    latOffset += analogRead(HORIZONTAL);
    verOffset += analogRead(VERTICAL);
    delay(5);
  }
  latOffset = latOffset/25;
  verOffset = verOffset/25;
  Serial.println(latOffset); Serial.println(verOffset);
}

void getInput(){
  leftRight = analogRead(HORIZONTAL) - latOffset;
  upDown = analogRead(VERTICAL) - verOffset;
  Serial.print("LeftRight = "); Serial.println(leftRight);
  Serial.print("UpDown = "); Serial.println(upDown);
  verticalValue = upDown/2047;
  horizontalValue = leftRight/2047;
  if(verticalValue < -1){ verticalValue = -1; }
  else if(verticalValue > 1){ verticalValue = 1; }
  if(horizontalValue < -1){ horizontalValue = -1; }
  else if(horizontalValue > 1){ horizontalValue = 1; }
  Serial.print("Horizontal reading = "); Serial.println(horizontalValue);
  Serial.print("Vertical reading = "); Serial.println(verticalValue);
}
