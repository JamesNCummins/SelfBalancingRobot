#include "imu.h"
#include <SparkFunLSM9DS1.h>
#include <cmath>
#include <Arduino.h>

#define pi 3.1415926

imu::imu(){
}

imu::~imu(){
}

void imu::init(){
	_gyroPitchIntegrator, _gyroRollIntegrator = 0;
	_values = {0,0,0,0,0,0};
	_gyroPitchOffset, _gyroRollOffset = 0;
}

void imu::readSensor(LSM9DS1 &sensor){
	if(sensor.accelAvailable()){
		_values.ax = sensor.calcAccel(sensor.readAccel(X_AXIS));
		_values.ay = sensor.calcAccel(sensor.readAccel(Y_AXIS));
		_values.az = sensor.calcAccel(sensor.readAccel(Z_AXIS));
	}
	if(sensor.gyroAvailable()){
		_values.gx = sensor.calcGyro(sensor.readGyro(X_AXIS));
		_values.gy = sensor.calcGyro(sensor.readGyro(Y_AXIS));
		_values.gz = sensor.calcGyro(sensor.readGyro(Z_AXIS));
	}
}

values imu::getValues(){
	return _values;
}

float imu::getAccelPitch(){
	float tanPitch = (-_values.ax)/(sqrt(_values.ay*_values.ay + _values.az*_values.az));
	float pitch = atan(tanPitch);
	pitch = pitch*180/pi;			//convert to degrees
	return -1*pitch;
}

float imu::getAccelRoll(){
	float tanRoll = _values.ay/_values.az;
	float roll = atan(tanRoll);
	roll = roll*180/pi;
	return -1*roll;
}

float imu::getGyroPitch(int refreshRate){		//pitch = rotation about y axis
	_gyroPitchIntegrator += (_values.gy - _gyroPitchOffset)/refreshRate;		//values.gy is change per sec so divide by refresh rate gives change per loop iteration
	if(getAccelPitch() > -0.25 and getAccelPitch() < 0.25){
		_gyroPitchIntegrator = 0;
	}
	return _gyroPitchIntegrator;
}

float imu::getGyroRoll(int refreshRate){		//roll = rotation about x axis
	_gyroRollIntegrator += (_values.gx - _gyroRollOffset)/refreshRate;
	if(getAccelRoll() > -0.25 and getAccelRoll() < 0.25){
		_gyroRollIntegrator = 0;
	}
	return _gyroRollIntegrator;
}

float imu::getRollRate(){
	float angle = _values.gx - _gyroRollOffset;
	return angle;
}

float imu::getPitchRate(){
	float angle = _values.gy - _gyroPitchOffset;
	return angle;
}

float imu::getPitchOffset(){
	return _gyroPitchOffset;
}

float imu::getRollOffset(){
	return _gyroRollOffset;
}

void imu::calcOffsets(LSM9DS1 &sensor){
	float gyroPitchOffset, gyroRollOffset;
	if(sensor.accelAvailable() and sensor.gyroAvailable()){
		for(int i = 0; i < 100; i++){
			readSensor(sensor);
			gyroPitchOffset += getPitchRate();
			gyroRollOffset += getRollRate();
			delay(0.05);
		}
		_gyroPitchOffset = gyroPitchOffset / 1000;
		_gyroRollOffset = gyroRollOffset / 1000;
	}
}