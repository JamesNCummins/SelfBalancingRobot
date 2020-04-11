#ifndef STATEVECTOR_H
#define STATEVECTOR_H

#include "SparkFunLSM9DS1.h"
#include "imu.h"
#include "KalmanFilter.h"
#include "Motor.h"

enum value{ POS, VEL, ANGLE, ANGULAR_RATE };

struct state {
	float pos;
	float vel;
	float angle;
	float angular_rate;
};

class StateVector{
	public:
	StateVector();
	~StateVector();
	void init(int refreshRate);
	state getVector();
	void updateVector(Motor &motor1, Motor &motor2, imu &imu, LSM9DS1 &sensor, Kalman &kalman);
	float getValue(value val);
	void setParameter(float val, value param);
	
	private:
	state _stateSpace;
	float _readPos(Motor &motor1, Motor &motor2);
	float _readVel();
	float _readAngle(Kalman &kalman, imu &imu, LSM9DS1 &sensor);
	float _readAngularRate(imu &imu);
	int _refreshRate;
	float _M1Values[10];
	float _M2Values[10];
	
};
#endif