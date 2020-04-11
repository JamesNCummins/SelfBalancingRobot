#ifndef KALMANFILTER_H
#define KALMANFILTER_H

enum sensor{
  ACCEL,
  GYRO,
  };

class Kalman {
	public:
	//constructor
	Kalman();
	~Kalman();
	
	//mutators
	void init(float *accel, float *gyro);
	void init();
	void filter(float accelDataPoint, float gyroDataPoint);
	float getAngle();

  private:
	void updateSample(float sampleDataPoint, sensor sensorChoice);
	float getSampleMean(float *sample);
	float getSampleVariance(float *sample);
	float _accelSample[10];
	float _gyroSample[10];
	float _accelAve[10];
	float _angle;
 
};
#endif
