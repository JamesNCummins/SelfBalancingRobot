#include "KalmanFilter.h"
//constructor
Kalman::Kalman(){
}

Kalman::~Kalman(){
}

//init all private variables
void Kalman::init(float *accel, float *gyro){
  for(int i = 0; i < 10; i++){
    _accelSample[i] = accel[i];
    _gyroSample[i] = gyro[i];
  }
}

void Kalman::init(){
	for(int i = 0; i < 10; i++){
		_accelSample[i], _gyroSample[i] = 0;
	}
}

void Kalman::filter(float accelDataPoint, float gyroDataPoint){
  //first produce a sample of each by quickly collecting 10 data points
  //find mean and variance for each sample too as these are used in Kalman filtering algorithm
  updateSample(accelDataPoint, ACCEL);
  updateSample(gyroDataPoint, GYRO);
  float accelMean = getSampleMean(_accelSample);
  float accelVariance = getSampleVariance(_accelSample);
  float gyroMean = getSampleMean(_gyroSample);
  float gyroVariance = getSampleVariance(_gyroSample);

  //first filter for angle

  //kalman filter uses an optimum ratio of the two angle readings according to the formula:
  //  x = w1.x1 + w2.x2 where w1 + w2 = 1 so that the output value x lies somewhere between x1 and x2
  //the optimum value of w2 = var2/(var1 + var2) and w1 = 1 - w2
  float w2 = (accelVariance)/(accelVariance + gyroVariance);
  float w1 = 1 - w2;

  float filteredOutput = w1*accelMean + w2*gyroMean;
  _angle = filteredOutput;
  }

float Kalman::getAngle(){
	return _angle;
}


void Kalman::updateSample(float sampleDataPoint, sensor sensorChoice){
  if(sensorChoice == ACCEL){
    for(int i = 0; i < 9; i++){
      _accelSample[i] = _accelSample[i+1];
    }
    _accelSample[9] = sampleDataPoint;
  }
  else if(sensorChoice == GYRO){
    for(int i = 0; i < 9; i++){
      _gyroSample[i] = _gyroSample[i+1];
    }
    _gyroSample[9] = sampleDataPoint;
  }
}

float Kalman::getSampleMean(float * sample){
  float sum = 0;
  for(int i = 0; i < 10; i++){
    sum += sample[i];
  }
  sum = sum/10;
  return sum;
}

float Kalman::getSampleVariance(float * sample){
  float mean = getSampleMean(sample);
  float sumOfDeviations;
  for(int i = 0; i < 10; i++){
    sumOfDeviations += (sample[i] - mean)*(sample[i] - mean);
  }
  sumOfDeviations = sumOfDeviations/9;
  return sumOfDeviations;
}
