#ifndef MOTOR_H
#define MOTOR_H

#include <Wire.h>

enum direction{ FORWARD, REVERSE };

class Motor{
	public:
		Motor();
		~Motor();
		void setMotorAddress(int addr);
		int getMotorAddress();
		void init();
		void init(int address);
		void setMaxSpeed(int speed);
		int getMaxSpeed();
		void setVelocity(int velocity);
		void setSpeedAndDirection(int speed, direction dir);
		int getVelocity();
		void setDamping(int damping);
		int getDamping();
		void setEncoderPos(int pos);
		int getEncoderPos();
		void writeGoToPos(int goTo);
		int getGoToPos();
		void writePGain(unsigned int gain);
		unsigned int readPGain();
		void writeIGain(unsigned int gain);
		unsigned int readIGain();
		void writeRelativeGoTo(int goTo);
	private:
		int _address;
};
#endif