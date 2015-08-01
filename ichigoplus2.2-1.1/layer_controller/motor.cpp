#include "motor.h"

int Motor::setup(){
	if(setupFlag)return 1;
	setupFlag = 1;
	pwm->setupPwmOut(1000,0.50);
	cw->setupDigitalOut();
	ccw->setupDigitalOut();
	return 0;
};

void Motor::drive(float p,int a,int b){
	p = 1.0 - p;
	cw->digitalWrite(a&1);
	ccw->digitalWrite(b&1);
	pwm->pwmWrite(p);
};


int Omni::setup(){
	if(setupFlag)return 1;
	setupFlag = 1;
	mt0->setup();
	mt1->setup();
	mt2->setup();
	enc0->setup();
	enc1->setup();
	enc2->setup();
	return 0;
}

void Omni::request(float radian,float power,float spin){
	static int encTimeOld;
	power *= MOTOR_GAIN;

	encTime = millis() - encTimeOld;
	encTimeOld = millis();
	encTime /= 10;//�}�C�R���̕���\�ɂ��

	encData[0] = enc0->count();
	encData[1] = enc1->count();
	encData[2] = enc2->count();

	encCnt[0] = 10*(encData[0] - encCntOld[0]);	//encCnt>0�Ȃ烂�[�^�[������]
	encCnt[1] = 10*(encData[1] - encCntOld[1]);
	encCnt[2] = 10*(encData[2] - encCntOld[2]);

	motorRev[0] = encCnt[0]/encTime;
	motorRev[1] = encCnt[1]/encTime;
	motorRev[2] = encCnt[2]/encTime;

	radian-=M_PI/3;

	while(radian < -M_PI/6){
		radian += 2*M_PI;
	}
	while(radian > (11*M_PI)/6){
		radian -= 2*M_PI;
	}

	if(-M_PI/6 <= radian && radian <= M_PI/6){		//�ڕW��]���v�Z
		motorPower[FRONT] = 1.0;
		motorPower[RIGHT] =  2 - (4 / (tan(radian + M_PI/6) + 2));
		motorPower[LEFT]  = -2 - (4 / (tan(radian - M_PI/6) - 2));
	}else if(M_PI/6 <= radian && radian <= M_PI/2){
		motorPower[FRONT] =  2 + (4 / (tan(radian + M_PI/2) - 2));
		motorPower[RIGHT] = 1.0;
		motorPower[LEFT]  =  2 - (4 / (tan(radian - M_PI/6) + 2));
	}else if(M_PI/2 <= radian && radian <= (5*M_PI)/6){
		motorPower[FRONT] = -2 + (4 / (tan(radian + M_PI/2) + 2));
		motorPower[RIGHT] =  2 + (4 / (tan(radian + M_PI/6) - 2));
		motorPower[LEFT]  = 1.0;
	}else if((5*M_PI)/6 <= radian && radian <= (7*M_PI)/6){
		motorPower[FRONT] = -1.0;
		motorPower[RIGHT] = -2 + (4 / (tan(radian + M_PI/6) + 2));
		motorPower[LEFT]  =  2 + (4 / (tan(radian - M_PI/6) - 2));
	}else if((7*M_PI)/6 <= radian && radian <= (9*M_PI)/6){
		motorPower[FRONT] = -2 - (4 / (tan(radian + M_PI/2) - 2));
		motorPower[RIGHT] = -1.0;
		motorPower[LEFT]  = -2 + (4 / (tan(radian - M_PI/6) + 2));
	}else if((9*M_PI)/6 <= radian && radian <= (11*M_PI)/6){
		motorPower[FRONT] =  2 - (4 / (tan(radian + M_PI/2) + 2));
		motorPower[RIGHT] = -2 - (4 / (tan(radian + M_PI/6) - 2));
		motorPower[LEFT]  = -1.0;
	}
	motorPower[1] = -motorPower[1];
	motorPower[2] = -motorPower[2];

	motorPower[0] *= power;
	motorPower[1] *= power;
	motorPower[2] *= power;

	if(power != 0){		//���[�^�[�ő�o�͒���
		ratio[0] = abs((PowToRev*motorPower[0])/MotorMax0);
		ratio[1] = abs((PowToRev*motorPower[1])/MotorMax1);
		ratio[2] = abs((PowToRev*motorPower[2])/MotorMax2);

		ratioMax = ratio[0];
		if(ratio[1] > ratioMax){
			ratioMax = ratio[1];
		}
		if(ratio[2] > ratioMax){
			ratioMax = ratio[2];
		}

		if(ratioMax > 1.0){//�ア���[�^�[�����x�𒴂����ꍇ
			if(ratio[0] == ratioMax){
				ratio[0] = 1.0;
				ratio[1] = ratioMax;
				ratio[2] = ratioMax;
			}
			if(ratio[1] == ratioMax){
				ratio[0] = ratioMax;
				ratio[1] = 1.0;
				ratio[2] = ratioMax;
			}
			if(ratio[2] == ratioMax){
				ratio[0] = ratioMax;
				ratio[1] = ratioMax;
				ratio[2] = 1.0;
			}

			motorPower[0] /= ratio[0];
			motorPower[1] /= ratio[1];
			motorPower[2] /= ratio[2];
		}
	}


	if(spin > 3.0){
		spin = 3.0;
	}else if(spin < -3.0){
		spin = -3.0;
	}

	if(spin>0){
		margin[0] = 1.0 - motorPower[0];
		margin[1] = 1.0 - motorPower[1];
		margin[2] = 1.0 - motorPower[2];
	}else{
		margin[0] = 1.0 + motorPower[0];
		margin[1] = 1.0 + motorPower[1];
		margin[2] = 1.0 + motorPower[2];
	}
	marginTotal = margin[0] + margin[1] + margin[2];

	motorPower[0] = (margin[0]/marginTotal)*spin + motorPower[0];
	motorPower[1] = (margin[1]/marginTotal)*spin + motorPower[1];
	motorPower[2] = (margin[2]/marginTotal)*spin + motorPower[2];


	if(power != 0 || spin != 0){
		if(motorPower[2] > 0){	//���[�^�[�N���o�͕␳
			if(encCnt[2] > 0){
				motorPower[2]  = motorPower[2]*(1.0-KF2) + KF2;
			}else{
				motorPower[2]  = motorPower[2]*(1.0-SF2) + SF2;
			}
		}else{
			if(encCnt[2] < 0){
				motorPower[2]  = motorPower[2]*(1.0-KF2) - KF2;
			}else{
				motorPower[2]  = motorPower[2]*(1.0-SF2) - SF2;
			}
		}

		if(motorPower[0] > 0){
			if(encCnt[0] > 0){
				motorPower[0] = motorPower[0]*(1.0-KF0) + KF0;
			}else{
				motorPower[0] = motorPower[0]*(1.0-SF0) + SF0;
			}
		}else{
			if(encCnt[0] < 0){
				motorPower[0] = motorPower[0]*(1.0-KF0) - KF0;
			}else{
				motorPower[0] = motorPower[0]*(1.0-SF0) - SF0;
			}
		}
		if(motorPower[1] > 0){
			if(encCnt[1] > 0){
				motorPower[1]  = motorPower[1]*(1.0-KF1) + KF1;
			}else{
				motorPower[1]  = motorPower[1]*(1.0-SF1) + SF1;
			}
		}else{
			if(encCnt[1] < 0){
				motorPower[1]  = motorPower[1]*(1.0-KF1) - KF1;
			}else{
				motorPower[1]  = motorPower[1]*(1.0-SF1) - SF1;
			}
		}
	}


	//motorPower[0] += motorPower[0] / PowToRev;

	if(motorPower[0] > 1.0){//-1.0~1.0�ɂ܂Ƃ߂�
		motorPower[0] = 1.0;
	}else if(motorPower[0] < -1.0){
		motorPower[0] = -1.0;
	}
	if(motorPower[1] > 1.0){
		motorPower[1] = 1.0;
	}else if(motorPower[1] < -1.0){
		motorPower[1] = -1.0;
	}
	if(motorPower[2] > 1.0){
		motorPower[2] = 1.0;
	}else if(motorPower[2] < -1.0){
		motorPower[2] = -1.0;
	}
};

void Omni::drive(){
	encCntOld[0] = enc0->count();
	encCntOld[1] = enc1->count();
	encCntOld[2] = enc2->count();

	if(motorPower[0] >= 0){//RIGHT
		mt0->drive(motorPower[0],1,0);
	}else{
		mt0->drive(-motorPower[0],0,1);
	}

	if(motorPower[1] >= 0){//FRONT
		mt1->drive(motorPower[1],1,0);
	}else{
		mt1->drive(-motorPower[1],0,1);
	}

	if(motorPower[2] >= 0){//LEFT
		mt2->drive(motorPower[2],1,0);
	}else{
		mt2->drive(-motorPower[2],0,1);
	}
}

void Omni::stop(){
	mt0->drive(0,0,0);
	mt1->drive(0,0,0);
	mt2->drive(0,0,0);
};

int Arm::move(float targetPosition){
	//float position=0;
	float Pos[3];

	Pos[ARM_D] = Pos[ARM_P] - (targetPosition - a0->analogRead());
	Pos[ARM_P] = targetPosition - a0->analogRead();
	Pos[ARM_I] += Pos[ARM_P];
	Pos[ARM_D] = Pos[ARM_P]*ARM_GAIN_P + Pos[ARM_I]*ARM_GAIN_I + Pos[ARM_D]*ARM_GAIN_D;

	if(Pos[ARM_D] > 0){
		mt3->drive(Pos[ARM_D],1,0);
	}else{
		mt3->drive(-Pos[ARM_D],0,1);
	}
	return targetPosition - a0->analogRead();
};


float abs(float value){
	if(value < 0){
		value *= -1.0;
	}
	return value;
}
