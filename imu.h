#ifndef IMU_H
#define IMU_H

#include <SparkFunLSM9DS1.h>

struct values{
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
};

class imu{
	public:
	imu();
	~imu();
	void init();
	void readSensor(LSM9DS1 &sensor);
	values getValues();
	float getAccelPitch();
	float getAccelRoll();
	float getGyroPitch(int refreshRate);
	float getGyroRoll(int refreshRate);
	float getPitchRate();
	float getRollRate();
	float getPitchOffset();
	float getRollOffset();
	void calcOffsets(LSM9DS1 &sensor);
	
	private:
	float _gyroPitchOffset;
	float _gyroRollOffset;
	values _values;
	float _gyroPitchIntegrator;
	float _gyroRollIntegrator;
};


#endif