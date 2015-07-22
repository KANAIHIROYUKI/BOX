#include "motor.h"


void Motor::drive(float p,int a,int b){
	p*=MOTOR_GAIN;
	p = 1.0 - p;
	cw->digitalWrite(a);
	ccw->digitalWrite(b);
	pwm->pwmWrite(p);
};



void Omni::request(float radian,float power,float spin){
	static int encCntOld[3];
	static int encTimeOld;

	encTime = millis() - encTimeOld;
	encTimeOld = millis();
	encTime /= 10;//マイコンの分解能による

	encCnt[0] = 10*(enc0->count() - encCntOld[0]);	//encCnt>0ならモーターが正回転
	encCnt[1] = 10*(enc1->count() - encCntOld[1]);
	encCnt[2] = 10*(enc2->count() - encCntOld[2]);
	encCntOld[0] = enc0->count();
	encCntOld[1] = enc1->count();
	encCntOld[2] = enc2->count();

	motorRev[0] = encCnt[0]/encTime;
	motorRev[1] = encCnt[1]/encTime;
	motorRev[2] = encCnt[2]/encTime;

	while(radian < -M_PI/6){
		radian += 2*M_PI;
	}
	while(radian > (11*M_PI)/6){
		radian -= 2*M_PI;
	}

	if(-M_PI/6 <= radian && radian <= M_PI/6){		//目標回転数計算
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

	if(power != 0){		//モーター最大出力調整
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

		if(ratioMax > 1.0){//弱いモーターが限度を超えた場合
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
		if(motorPower[2] > 0){	//モーター起動出力補正
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

	if(motorPower[0] > 1.0){//-1.0~1.0にまとめる
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
