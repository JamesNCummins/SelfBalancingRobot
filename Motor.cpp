#include <Motor.h>
#include <Wire.h>
#include <stdint.h>

Motor::Motor(){
}

Motor::~Motor(){
}

void Motor::init(){
	setMotorAddress(0x08);
	Wire.begin();
	setMaxSpeed(255);
	setVelocity(0);
	setEncoderPos(0);
}

void Motor::init(int address){
	setMotorAddress(address);
	Wire.begin();
	setMaxSpeed(255);
	setVelocity(0);
	setEncoderPos(0);
	//setDamping(0);
}

void Motor::setMotorAddress(int addr){
	_address = addr;
}

int Motor::getMotorAddress(){
	return _address;
}

void Motor::setMaxSpeed(int speed){
	Wire.beginTransmission(_address);
	Wire.write(0);
	Wire.write(speed);
	Wire.write(0);
	int error = Wire.endTransmission();
}

int Motor::getMaxSpeed(){
	int value = 0;
	Wire.beginTransmission(_address);
	Wire.write(0);
	Wire.endTransmission();
	Wire.requestFrom(_address, 2);
	value = Wire.read();
	value += Wire.read() << 8;
	return value;
}

void Motor::setVelocity(int velocity){
	if(velocity > 255){ velocity = 255; }
	if(velocity < -255){ velocity = -255; }
	Wire.beginTransmission(_address);
	Wire.write(1);
	if(velocity >= 0){
		Wire.write(velocity);
		Wire.write(0);
	}
	else if(velocity < 0){
		Wire.write(velocity + 256);
		Wire.write(255);
	}
	else{
		Wire.write(0);
		Wire.write(0);
	}
	int error = Wire.endTransmission();
}

void Motor::setSpeedAndDirection(int speed, direction dir){
	if(speed > 255){ speed = 255; }
	if(speed < 0){ speed = 0; }
	Wire.beginTransmission(_address);
	Wire.write(1);
	if(dir == REVERSE){ Wire.write(256 - speed); Wire.write(255); }
	else{ Wire.write(speed); Wire.write(0); }
	int error = Wire.endTransmission();
}

int Motor::getVelocity(){
	int value = 0;
	Wire.beginTransmission(_address);
	Wire.write(1);
	Wire.endTransmission();
	Wire.requestFrom(_address, 2);
	value = Wire.read();
	value += Wire.read() << 8;
	return value;
}

void Motor::setDamping(int damping){
	if(damping < 0){ damping = damping * -1; }
	if(damping > 255){ damping = 255; }
	Wire.beginTransmission(_address);
	Wire.write(2);
	Wire.write(damping);
	Wire.write(0);
	int error = Wire.endTransmission();
}

int Motor::getDamping(){
	int value = 0;
	Wire.beginTransmission(_address);
	Wire.write(2);
	Wire.endTransmission();
	Wire.requestFrom(_address, 2);
	value = Wire.read();
	value += Wire.read() << 8;
	return value;
}

void Motor::setEncoderPos(int pos){
	Wire.beginTransmission(_address);
	Wire.write(3);
	for(int i = 0; i < 4; i++){
		Wire.write((pos >> 8*i) & 0xFF);
	}
	int error = Wire.endTransmission();
}

int Motor::getEncoderPos(){
	int value = 0;
	Wire.beginTransmission(_address);
	Wire.write(3);
	Wire.endTransmission();
	Wire.requestFrom(_address, 4);
	value = Wire.read();
	value += Wire.read() << 8;
	value += Wire.read() << 16;
	value += Wire.read() << 24;
	return value;
}

void Motor::writeGoToPos(int goTo){
	Wire.beginTransmission(_address);
	Wire.write(4);
	for(int i = 0; i < 4; i++){
		Wire.write((goTo >> 8*i) & 0xFF);
	}
	int error = Wire.endTransmission();
}

int Motor::getGoToPos(){
	int value = 0;
	Wire.beginTransmission(_address);
	Wire.write(4);
	Wire.endTransmission();
	Wire.requestFrom(_address, 4);
	value = Wire.read();
	value += Wire.read() << 8;
	value += Wire.read() << 16;
	value += Wire.read() << 24;
	return value;
}

void Motor::writePGain(unsigned int gain){
	Wire.beginTransmission(_address);
	Wire.write(6);
	Wire.write(gain & 0xFF);
	Wire.write(gain >> 8);
	int error = Wire.endTransmission();
}

unsigned int Motor::readPGain(){
	unsigned int value = 0;
	Wire.beginTransmission(_address);
	Wire.write(6);
	Wire.endTransmission();
	Wire.requestFrom(_address, 2);
	value = Wire.read();
	value += Wire.read() << 8;
	return value;
}

void Motor::writeIGain(unsigned int gain){
	Wire.beginTransmission(_address);
	Wire.write(7);
	Wire.write(gain & 0xFF);
	Wire.write(gain >> 8);
	int error = Wire.endTransmission();
}

unsigned int Motor::readIGain(){
	unsigned int value = 0;
	Wire.beginTransmission(_address);
	Wire.write(7);
	Wire.endTransmission();
	Wire.requestFrom(_address, 2);
	value = Wire.read();
	value += Wire.read() << 8;
	return value;
}

void writeRelativeGoTo(int goTo){
	Wire.beginTransmission(0x08);
	Wire.write(8);
	for(int i = 0; i < 4; i++){
		Wire.write(goTo >> i*8);
	}
	int error = Wire.endTransmission();
}

