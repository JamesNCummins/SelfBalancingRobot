#include "StateVector.h"

#define pi 3.1415926

StateVector::StateVector(){
}

StateVector::~StateVector(){
}

void StateVector::init(int refreshRate){
	_stateSpace = {0,0,0,0};
	for(int i = 0; i < 10; i++){
		_M1Values[i], _M2Values[i] = 0;
	}
	_refreshRate = refreshRate;
}

state StateVector::getVector(){
	return _stateSpace;
}

void StateVector::updateVector(Motor &motor1, Motor &motor2, imu &imu, LSM9DS1 &sensor, Kalman &kalman){
	_stateSpace.pos = _readPos(motor1, motor2);
	_stateSpace.vel = _readVel();
	_stateSpace.angle = _readAngle(kalman, imu, sensor);
	_stateSpace.angular_rate = _readAngularRate(imu);
}

float StateVector::getValue(value val){
	if(val == POS)				{	return _stateSpace.pos;				}
	else if(val == VEL)			{	return _stateSpace.vel;				}
	else if(val == ANGLE)		{	return _stateSpace.angle;			}
	else if(val == ANGULAR_RATE){	return _stateSpace.angular_rate;	}
}

void StateVector::setParameter(float val, value param){
	if(param == POS){ _stateSpace.pos = val; }
	else if(param == VEL){ _stateSpace.vel = val; }
	else if(param == ANGLE){ _stateSpace.angle = val; }
	else if(param == ANGULAR_RATE){ _stateSpace.angular_rate = val; }
}

float StateVector::_readPos(Motor &motor1, Motor &motor2){
	for(int i = 0; i < 9; i++){
		_M1Values[9-i] = _M1Values[8-i];
		_M2Values[9-i] = _M2Values[8-i];
	}
	_M1Values[0] = motor1.getEncoderPos()*0.2*pi/1800;
	_M2Values[0] = motor2.getEncoderPos()*-0.2*pi/1800;
	float m1 = _M1Values[0];
	float m2 = _M2Values[0];
	float ave = (m1 + m2)/2;
	return ave;
}

float StateVector::_readVel(){			//readPos must be called first!
	float vel1 = (_M1Values[0] - _M1Values[9])*_refreshRate/10;
	Serial.print("Enc 1: "); Serial.print(_M1Values[0]); Serial.print(" , "); Serial.println(_M1Values[9]);
	float vel2 = (_M2Values[0] - _M2Values[9])*_refreshRate/10;
	Serial.print("Enc 2: "); Serial.print(_M2Values[0]); Serial.print(" , "); Serial.println(_M2Values[9]);
	float ave = (vel1 + vel2)/2;
	return ave;
}

float StateVector::_readAngle(Kalman &kalman, imu &imu, LSM9DS1 &sensor){
	imu.readSensor(sensor);
	kalman.filter(imu.getAccelPitch(), imu.getGyroPitch(_refreshRate));
	float pitch = kalman.getAngle();
	if(pitch < -89){ pitch = -89; }
	return pitch;
}

float StateVector::_readAngularRate(imu &imu){
	return imu.getPitchRate();
}