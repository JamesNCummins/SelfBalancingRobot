#include "ControlAlgorithm.h"

Controller::Controller(float mass, float Mass, float pendLength)
	: _m(mass), _M(Mass), _Length(pendLength){}

Controller::~Controller(){
}

void Controller::init(int refreshRate){
	_simulation = {0,0,0,0};
	setRefreshRate(refreshRate);
	_motorSignal, _desiredPos, _desiredVel = 0;
}

void Controller::setRefreshRate(int rate){
	_refreshRate = rate;
}

int Controller::getRefreshRate(){
	return _refreshRate;
}

void Controller::updateAx(){
	_Ax[0][0] = _A[0][0]*_simulation.pos + _A[0][1]*_simulation.vel + _A[0][2]*_simulation.angle; + _A[0][3]*_simulation.angular_rate;
	_Ax[1][0] = _A[1][0]*_simulation.pos + _A[1][1]*_simulation.vel + _A[1][2]*_simulation.angle; + _A[1][3]*_simulation.angular_rate;
	_Ax[2][0] = _A[2][0]*_simulation.pos + _A[2][1]*_simulation.vel + _A[2][2]*_simulation.angle; + _A[2][3]*_simulation.angular_rate;
	_Ax[3][0] = _A[3][0]*_simulation.pos + _A[3][1]*_simulation.vel + _A[3][2]*_simulation.angle; + _A[3][3]*_simulation.angular_rate;
}

void Controller::updateBu(){
	_Bu[0] = _BMatrix[0]*_u;
	_Bu[1] = _BMatrix[1]*_u;
	_Bu[2] = _BMatrix[2]*_u;
	_Bu[3] = _BMatrix[3]*_u;
}

void Controller::AxPlusBu(){
	_AxPlusBu[0][0] = _Ax[0][0]+_Bu[0];
	_AxPlusBu[1][0] = _Ax[1][0]+_Bu[1];
	_AxPlusBu[2][0] = _Ax[2][0]+_Bu[2];
	_AxPlusBu[3][0] = _Ax[3][0]+_Bu[3];
}

void Controller::integrator(){
	_simulation.pos += (_AxPlusBu[0][0]/_refreshRate);
	_simulation.vel += (_AxPlusBu[1][0]/_refreshRate);
	_simulation.angle += (_AxPlusBu[2][0]/_refreshRate);
	_simulation.angular_rate += (_AxPlusBu[3][0]/_refreshRate);
}

void Controller::setDesiredMotion(int target, motion select, state vector){
	if(select == HOLD){
		_desiredVel = 0;
		_desiredPos = vector.pos;
	}
	else if(select == VELOCITY){
		_desiredVel = target;
		_desiredPos += target/_refreshRate;
	}
	else if(select == POSITION){
		_desiredPos = target;
		_desiredVel = 0;
	}
}

void Controller::calcMotorSignal(state vector){
	float PosError = vector.pos - _desiredPos;
	float VelError = vector.vel - _desiredVel;
	_u = -1*_K[0]*PosError - _K[1]*VelError - _K[2]*vector.angle*pi/180 - _K[3]*vector.angular_rate*pi/180;		//pi/180 converts angle in degrees to radians for calculation's dimensional consistency
}

void Controller::writeMotorSignal(StateVector &vector, Motor &motor1, Motor &motor2){
	_motorSignal += (_u/_M)*255/(pi*_refreshRate);		//255/pi converts a speed setting to a register value. Max speed is pi m/s. If want to increase by 1ms^-2, that's an increase of 1/_refreshRate m/s/frame. The convert that to an increase in motor signal
	Serial.print("Unfiltered Motor Signal = "); Serial.println(_motorSignal);
	//_motorSignal += _motorSignal - vector.getValue(VEL);		//proportional control to get rid of the disparity between set speed and true speed
	Serial.print("Motor Signal = "); Serial.println(_motorSignal);
	motor1.setVelocity(_motorSignal);
	motor2.setVelocity(-1*_motorSignal);	//turns both motors the same way
}
	
void Controller::algorithm(StateVector &vector, Motor &motor1, Motor &motor2, Kalman &kalman, imu &imu, LSM9DS1 &sensor, int target, motion SELECT){
	vector.updateVector(motor1, motor2, imu, sensor, kalman);
	Serial.print("Pos = "); Serial.println(vector.getValue(POS));
	Serial.print("Vel = "); Serial.println(vector.getValue(VEL));
	Serial.print("Angle = "); Serial.println(vector.getValue(ANGLE));
	Serial.print("Angular Velocity = "); Serial.println(vector.getValue(ANGULAR_RATE));
	setDesiredMotion(target, SELECT, vector.getVector());
	calcMotorSignal(vector.getVector());
	Serial.print("Required Accel = "); Serial.println(_u/_M);
	writeMotorSignal(vector, motor1, motor2);
}

void Controller::simulate(int target, motion SELECT){
	updateAx(); updateBu();
	AxPlusBu();
	integrator();
	setDesiredMotion(target, SELECT, _simulation);
	calcMotorSignal(_simulation);
}
	