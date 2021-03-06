#ifndef  _MOTOR_H
#define  _MOTOR_H

#include <stdio.h>
#include "digital.hpp"
#include "pwm.hpp"
#include "mcutime.h"
#include "math.h"
#include "encoder.hpp"
#include "pin.hpp"


#define FRONT 1
#define RIGHT 0
#define LEFT 2

#define MotorMax0 5400
#define MotorMax1 5300
#define MotorMax2 5300

#define PowToRev 5400

#define KF0 0.12
#define SF0 0.39

#define KF1 0.05
#define SF1 0.12

#define KF2 0.04
#define SF2 0.12

#define ARM_P 0
#define ARM_I 1
#define ARM_D 2

#define ARM_GAIN_P 0.1
#define ARM_GAIN_I 0.1
#define ARM_GAIN_D 0.1

#define MOTOR_GAIN 1.0



class Motor {
public:
	Pwm *pwm;
	Digital *ccw;
	Digital *cw;

	Motor(Pwm &pwmPin,Digital &ccwPin,Digital &cwPin){
		this-> pwm = &pwmPin;
		this-> cw = &cwPin;
		this-> ccw = &ccwPin;

		pwm->setupPwmOut(1000,0.50);
		cw->setupDigitalOut();
		ccw->setupDigitalOut();
	};

	void drive(float pwm,int a,int b);


private:

};


class Omni {
public:
	Motor *mt0;
	Motor *mt1;
	Motor *mt2;

	Enc0 *enc0;
	Enc1 *enc1;
	Enc2 *enc2;

	float motorPower[3];
	float powerLimit;
	float motorMax[3];
	float over[3];
	float margin[3];
	float marginTotal;
	float ratio[3];
	float ratioMax;

	int encCnt[3];
	int encTime;
	int motorRev[3];

	Omni(Motor &mt0,Motor &mt1,Motor &mt2,Enc0 &enc0,Enc1 &enc1,Enc2 &enc2){
		this->mt0 = &mt0;
		this->mt1 = &mt1;
		this->mt2 = &mt2;

		this->enc0 = &enc0;
		this->enc1 = &enc1;
		this->enc2 = &enc2;

		enc0.setup();
		enc1.setup();
		enc2.setup();
	};

	void request(float radian,float power,float spin);
	void drive();
	//float control(float targetX,float targetY,float targetRad);
	void stop();

};

float abs(float value);

class Arm{
public:

	Motor *mt3;
	A0 *a0;

	Arm(Motor &mt3,A0 &a0){
		this->mt3 = &mt3;
		this->a0 = &a0;
		a0.setupAnalogIn();
	};

	int move(float targetPosition);


};


#endif
