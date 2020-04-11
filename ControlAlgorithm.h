#ifndef CONTROLALGORITHM_H
#define CONTROLALGORITHM_H

#include "StateVector.h"
#include "SparkFunLSM9DS1.h"
#include "imu.h"
#include "KalmanFilter.h"
#include "Motor.h"
#include <Arduino.h>

#define g -9.81
#define pi 3.1415926

enum motion{ HOLD, POSITION, VELOCITY };

class Controller{
	public:
	Controller(float mass, float Mass, float pendLength);
	~Controller();
	void init(int refreshRate);
	void algorithm(StateVector &vector, Motor &motor1, Motor &motor2, Kalman &kalman, imu &imu, LSM9DS1 &sensor, int target, motion SELECT);
	void simulate(int target, motion SELECT);
	void setDesiredMotion(int target, motion select, state vector);
	void calcMotorSignal(state vector);
	void writeMotorSignal(StateVector &vector, Motor &motor1, Motor &motor2);
	
	private:
	//state vector to hold the simulation of the process
	state _simulation;
	
	//members to define system refresh rate
	void setRefreshRate(int rate);
	int getRefreshRate();
	int _refreshRate;
	
	//constant parameters
	float _m;				//Pendulum Mass
	float _M;				//Cart Mass
	float _Length;				//Pendular Length (CoM to Axle)
	
	//matrices to represent equations of motion
	const float _A[4][4] = {						//contribution from natural response
		{0, 1, 0, 0},
		{0, 0, _m*g/_M, 0},
		{0, 0, 0, 1},
		{0, 0, (_M+_m)*g/(_M*_Length), 0}
	};
	const float _BMatrix[4] = {0, 1/_M, 0, 1/(_M*_Length) };			//contribution from applied force
	
	float _u;				//Required force from motor
	
	//internal processes to the control algorithm
	void updateAx();
	void updateBu();
	void AxPlusBu();
	void integrator();
	
	float _Ax[1][4];
	float _Bu[4];
	float _AxPlusBu[1][4];
	
	//controller vectors
	float _K[4] = {0.999999999999989, 4.152773550044069, 1.204315614723768, 1.447711834326177};
	
	//desired motion members
	float _desiredPos;
	float _desiredVel;
	
	int _motorSignal;
};
#endif