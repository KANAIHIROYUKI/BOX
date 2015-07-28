//libraries

//application

//controller

//base
#include "system.h"
#include "mcutime.h"
#include "math.h"
#include "encoder.hpp"


//board
#include "pin.hpp"
#include "motor.h"
#include "odometry.h"

//circuit
#include "circuit/can_encoder.hpp"

#define GAIN_P 0.06
#define GAIN_I 0
#define GAIN_D 4.0

#define C 0
#define P 1
#define I 2
#define D 3

#define PAT_NUM 3
#define SOUND_ENTER 1
#define SOUND_OPERATION 0
#define SOUND_BREAK 3

#define ReferenceCircle 100

#define GAIN_SPIN_D 0

#define TargetQuantity 15
#define ControlCycle 10


int waitBreak(int waitTime);
void sound(int pattern);


int main(void)
{
	float spin=0,radianDif,distanceX,distanceY;
	float targetY[] = {0,350,350,310,260,200,200,200, 40,350,350,330,280,230,210,210, 40};
	float targetX[] = {0,  0, 80,160,160,160,100,  0,220,220,320,380,400,380,320,220,220};
	//float targetX[] = {0,500,500,  0,0,250,500};
	//float targetY[] = {0,  0,500,500,0,500,250};

	float targetRadian=0;
 	float radian = 0;
 	float power;
	float radianOld;

	float v_radian,v_slope,v_distance,c_radius = ReferenceCircle,d_radian,q_radian,d_r;

	int cnt=0,buzzerCnt = 0,spinCnt;
	int flag[3],testMode,swFlag;

	int_least32_t cycleTime;

	Serial0 serial;
	serial.setup(115200);

	A0 a0;
	a0.setupDigitalIn();

	A1 a1;
	a1.setupDigitalIn();

	A2 a2;
	a2.setupDigitalIn();

	A3 a3;
	a3.setupDigitalIn();

	Sw0 sw0;
	sw0.setupDigitalIn();

	Sw1 sw1;
	sw1.setupDigitalIn();

	Sw2 sw2;
	sw2.setupDigitalIn();

	Sw3 sw3;
	sw3.setupDigitalIn();

	Led0 led0;
	led0.setupDigitalOut();
	led0.digitalHigh();

	Led1 led1;
	led1.setupDigitalOut();
	led1.digitalHigh();

	Led2 led2;
	led2.setupDigitalOut();
	led2.digitalHigh();

	Led3 led3;
	led3.setupDigitalOut();
	led3.digitalHigh();

	Buzzer bz;
	bz.setupPwmOut(4000,0);

	Can0 can;
	can.setup();
	CanEncoder canEnc0(can,0,5);
	CanEncoder canEnc1(can,1,5);
	CanEncoder canEnc2(can,2,5);
	canEnc0.setup();
	canEnc1.setup();
	canEnc2.setup();

	/*while(1){
		serial.printf("ENC0: %d ENC1: %d ENC2: %d\n\r",canEnc0.count(),canEnc1.count(),canEnc2.count());
		wait(100);
	}*/
	/*sound(0);
	wait(300);
	sound(1);
	wait(300);
	sound(2);
	wait(300);
	sound(3);
	wait(500);*/

	Enc0 enc0;
	Enc1 enc1;
	Enc2 enc2;

	Pwm0 pwm0;
	CW0 cw0;
	CCW0 ccw0;

	Pwm1 pwm1;
	CW1 cw1;
	CCW1 ccw1;

	Pwm2 pwm2;
	CW2 cw2;
	CCW2 ccw2;

	Motor motor0(pwm0,cw0,ccw0);
	Motor motor1(pwm1,cw1,ccw1);
	Motor motor2(pwm2,cw2,ccw2);

	int encOld0 = enc0.count();//モーター起動･停止出力
	int encOld1 = enc1.count();
	int encOld2 = enc2.count();
	int encFlag0=0;
	int encFlag1=0;
	int encFlag2=0;

	Omni omni(motor0,motor1,motor2,enc0,enc1,enc2);
	OmniOdometry odm(143,canEnc0,canEnc1,canEnc2);

	while(1){
		while(sw0.digitalRead() == 0);
		sound(SOUND_BREAK);
		testMode = 0;
		cnt=1;
		swFlag = 1;
		serial.printf("MOTOR TEST \n\r SW1 and SW3 : select  SW2 : START\n\r");
		omni.request(0,0,0);
		omni.drive();
		bz.pwmWrite(0);
		while(1){

			if(sw1.digitalRead() == 0){
				while(sw1.digitalRead() == 0);
				if(testMode < 6)testMode++;
				swFlag = 1;
			}
			if(sw3.digitalRead() == 0){
				while(sw3.digitalRead() == 0);
				if(testMode > 0)testMode--;
				swFlag = 2;
			}
			if(sw2.digitalRead() == 0){
				while(sw2.digitalRead() == 0);
				break;
			}

			led0.digitalWrite(testMode & 1);
			led1.digitalWrite((testMode & 2) >> 1);
			led2.digitalWrite((testMode & 4) >> 2);
			led3.digitalWrite((testMode & 8) >> 3);
			if(swFlag != 0){
				serial.printf("%d\n\r",testMode);
				swFlag = 0;
				switch(testMode){
				case 0:
					serial.printf("COODINATE RUN\n\r");
					break;
				case 1:
					serial.printf("ENC TEST\n\r");
					break;
				case 2:
					serial.printf("ODOMETRY TEST\n\r");
					break;
				case 3:
					serial.printf("MOTOR REVOLUTION TEST\n\r");
					break;
				case 4:
					serial.printf("START/STOP PWM TEST\n\r");
					break;
				case 5:
					serial.printf("CIRCLE\n\r");
					break;
				case 6:
					serial.printf("MOTOR POSITION CHECK\n\r");
					break;
				case 7:
					serial.printf("DEGREE CONTROL\n\r");
				}
				sound(SOUND_OPERATION);
			}
		}
		sound(SOUND_ENTER);

		switch(testMode){

		case 0:
			odm.update();
			odm.reset();
			odm.update();
			cycleTime = millis() + ControlCycle;
			serial.printf("%f,%f,%f,%d,%d,%d\n\r",odm.integralX,odm.integralY,odm.radianAbs,odm.encTest[0],odm.encTest[1],odm.encTest[2]);
			while(sw0.digitalRead()){
				odm.update();

				/*targetRadian += M_PI/500;
				if(targetRadian > M_PI)targetRadian-=M_PI*2;*/

				distanceX = targetX[cnt] - odm.integralX;
				distanceY = targetY[cnt] - odm.integralY;

				if(sqrt(distanceX*distanceX + distanceY*distanceY) < c_radius){//目標地点が近ければ仮想軌道追尾はしない
			        d_radian = atan2(targetY[cnt] - odm.integralY,targetX[cnt] - odm.integralX);
			        //serial.printf(" CLOSE");
			    }else{
			        if(targetX[cnt-1] == targetX[cnt]){
			            v_distance = targetX[cnt] - odm.integralX;
			            if(v_distance <0 )v_distance=-v_distance;
			        }else{
			            if(targetY[cnt-1] != targetY[cnt]){
			                v_slope = (targetY[cnt] - targetY[cnt-1])/(targetX[cnt] + targetX[cnt-1]);
			                if(v_slope > 1 || v_slope < -1){//X-Y座標､直線との距離
			                    v_distance = (v_slope*odm.integralX) - v_slope*targetX[cnt-1] + targetY[cnt-1] - odm.integralY;
			                    if(v_distance < 0)v_distance = -v_distance;
			                    v_distance /= sqrt(1 + (v_slope*v_slope));
			                }else{//Y-X座標､直線との距離
			                    v_slope = (targetX[cnt] - targetX[cnt-1])/(targetY[cnt] - targetY[cnt-1]);
			                    v_distance = v_slope*odm.integralY - v_slope*targetY[cnt-1] + targetX[cnt-1] - odm.integralX;
			                    if(v_distance < 0)v_distance = -v_distance;
			                    v_distance /= sqrt(1 + (v_slope*v_slope));
			                }
			            }else{
			                v_distance = targetY[cnt] - odm.integralY;
			            }
			        }
			        //serial.printf("%f\n",v_distance);

			        if(v_distance < c_radius){
			            v_radian = atan2((targetY[cnt] - targetY[cnt-1]),(targetX[cnt] - targetX[cnt-1]));
			            q_radian = atan2((odm.integralY - targetY[cnt]),(odm.integralX - targetX[cnt])) - v_radian;

			            d_radian = asin(v_distance/c_radius);
			            if(q_radian < 0)q_radian+=2*M_PI;
			            if(q_radian > 2*M_PI)q_radian -= 2*M_PI;
			            //serial.printf("D= %f , VR= %f , QR= %f , DR= %f\n",v_distance,v_radian*180/M_PI,q_radian*180/M_PI,d_radian*180/M_PI);

			            if(d_radian < 0)d_radian = -d_radian;
			            d_r = d_radian;

			            if(q_radian <= M_PI/2){
			                d_radian = v_radian + d_radian;
			                d_radian += M_PI;
			                //serial.printf(" Q1");
			            }else if(q_radian <= M_PI){
			                d_radian = v_radian - d_radian;
			                //serial.printf(" Q2");
			            }else if(q_radian <= (3*M_PI)/2){
			                d_radian = v_radian + d_radian;
			                //serial.printf(" Q3");
			            }else{
			                d_radian = v_radian - d_radian;
			                d_radian += M_PI;
			                //serial.printf(" Q4");
			            }
			        }else{
			            //d_radian = atan2(odm.integralY - targetY[cnt],odm.integralX - targetX[cnt]);
			        	d_radian = atan2(targetY[cnt] - odm.integralY,targetX[cnt] - odm.integralX);
			            //serial.printf(" V_LINE FAR");
			        }
			    }
				if(d_radian < 0)d_radian+=2*M_PI;

				if(sqrt(distanceX*distanceX + distanceY*distanceY) < 10){
					cnt++;
					serial.printf("NEXT POINT");
					buzzerCnt = 10;
				}
				if(cnt>TargetQuantity){
					cnt--;
					//omni.drive(0,0,0);
					//wait(100);
					//break;
				}

				if(buzzerCnt > 0){
					buzzerCnt--;
					bz.pwmWrite(0.5);
				}else{
					bz.pwmWrite(0);
				}

				power = sqrt(distanceX*distanceX + distanceY*distanceY)*GAIN_P + (sqrt(distanceX*distanceX + distanceY*distanceY)*GAIN_P - power)*GAIN_D;
				if(power > 1.0)power = 1.0;


				if(odm.radianAbs > M_PI){
					radianDif = odm.radianAbs - 2*M_PI;
				}else{
					radianDif = odm.radianAbs;
				}
				radianDif = odm.radianAbs - targetRadian;
				radianOld = radian;
				if(radianDif < M_PI){
					radian = radianDif;
				}else{
					radian = radianDif - 2*M_PI;
				}
				spin = radian*radian*radian*radian*radian*radian*radian*10000 + (radian - radianOld)*GAIN_SPIN_D;

				if(spin<-3.0)spin=-3.0;
				if(spin>3.0)spin=3.0;


				if(cycleTime < millis()){
					cycleTime+=ControlCycle;
					omni.request(d_radian - odm.radianAbs,power,-spin);
					omni.drive();
					serial.printf("\n\r%f , %f , %f , %f , %f , %f , %f ,%d , %d , %d , %f ,",odm.integralX,odm.integralY,v_distance,v_radian,d_r,q_radian,d_radian,odm.encTest[0],odm.encTest[1],odm.encTest[2],radian);
					//serial.printf("\n\r %d,%f,%f,%d,%d,%d,%f,%f,",odm.radianOrg,spin,radian,odm.encTest[0],odm.encTest[1],odm.encTest[2],odm.integralX,odm.integralY);
				}

				//serial.printf("\n\r%f,%f,%f,",d_radian,radianDif,d_radian + radianDif);
				/*serial.printf("%d,%d",millis(),cycleTime);
				if(millis() >= cycleTime){
					cycleTime += ControlCycle;


				}*/
			}
			omni.request(0,0,0);
			omni.drive();
			break;

		case 1:
			while(sw0.digitalRead()){
				odm.update();
				//serial.printf("%d %d %d\n\r",odm.encData[0],odm.encData[1],odm.encData[2]);
				serial.printf("%d %d %d %d %d %d\n\r",enc0.count(),enc1.count(),enc2.count(),odm.encData[0],odm.encData[1],odm.encData[2]);
				waitBreak(100);
			}

			break;

		case 2:
			while(sw0.digitalRead()){
				odm.update();
				serial.printf("%f , %f , %f , %f , %f , %d\n\r",odm.integralX,odm.integralY,odm.radianAbs*180/M_PI,odm.X,odm.Y,odm.radianDelta - odm.radianOrg);
				waitBreak(100);
			}
			break;

		case 3:
			motor0.drive(1.0,1,0);
			motor1.drive(1.0,1,0);
			motor2.drive(1.0,1,0);
			waitBreak(1000);
			encOld0 = enc0.count();
			encOld1 = enc1.count();
			encOld2 = enc2.count();
			waitBreak(10000);
			serial.printf("ENC0 = %d ,ENC1 = %d ,ENC2 = %d\n\r",enc0.count() - encOld0,enc1.count() - encOld1,enc2.count() - encOld2);
			serial.printf("INVERSE");
			motor0.drive(1.0,0,1);
			motor1.drive(1.0,0,1);
			motor2.drive(1.0,0,1);
			waitBreak(1000);
			encOld0 = enc0.count();
			encOld1 = enc1.count();
			encOld2 = enc2.count();
			waitBreak(10000);
			serial.printf("ENC0 = %d ,ENC1 = %d ,ENC2 = %d\n\r",enc0.count() - encOld0,enc1.count() - encOld1,enc2.count() - encOld2);
			motor0.drive(0,1,0);
			motor1.drive(0,1,0);
			motor2.drive(0,1,0);
			break;

		case 4:
			encOld0 = enc0.count();
			encOld1 = enc1.count();
			encOld2 = enc2.count();
			for(float i = 0.0;i<1.0;i+=0.01){
				motor0.drive(i,1,0);
				motor1.drive(i,1,0);
				motor2.drive(i,1,0);
				if(encOld0 <= enc0.count() - 10 && encFlag0 == 0){
					serial.printf("ENC0 %f\n\r",i);
					encFlag0 = 1;
				}
				if(encOld1 <= enc1.count() - 10 && encFlag1 == 0){
					serial.printf("ENC1 %f\n\r",i);
					encFlag1 = 1;
				}
				if(encOld2 <= enc2.count() - 10 && encFlag2 == 0){
					serial.printf("ENC2 %f\n\r",i);
					encFlag2 = 1;
				}
				if(encFlag0 == 1 && encFlag1 == 1 && encFlag2 == 1){
					motor0.drive(0.5,1,0);
					motor1.drive(0.5,1,0);
					motor2.drive(0.5,1,0);
					waitBreak(1000);
					break;
				}
				encOld0 = enc0.count();
				encOld1 = enc1.count();
				encOld2 = enc2.count();
				waitBreak(500);
			}
			serial.printf("Deceleration\n\r");
			for(float i = 0.5;i>0.0;i-=0.01){
				motor0.drive(i,1,0);
				motor1.drive(i,1,0);
				motor2.drive(i,1,0);
				if(encOld0 == enc0.count() && encFlag0 == 1){
					serial.printf("ENC0 %f\n\r",i);
					encFlag0 = 0;
				}
				if(encOld1 == enc1.count() && encFlag1 == 1){
					serial.printf("ENC1 %f\n\r",i);
					encFlag1 = 0;
				}
				if(encOld2 == enc2.count() && encFlag2 == 1){
					serial.printf("ENC2 %f\n\r",i);
					encFlag2 = 0;
				}
				encOld0 = enc0.count();
				encOld1 = enc1.count();
				encOld2 = enc2.count();
				if(encFlag0 == 0 && encFlag1 == 0 && encFlag2 == 0){
					motor0.drive(0,1,0);
					motor1.drive(0,1,0);
					motor2.drive(0,1,0);
					break;
				}
				waitBreak(500);
			}
			break;

		case 5:
			for(float rad=0;rad<2*M_PI;rad+=(2*M_PI)/180){		//円形移動
				omni.request(rad,1.0,spin);
				omni.drive();
				odm.update();

				if(odm.radianAbs < M_PI){
					spin=-odm.radianAbs*odm.radianAbs*10;
				}
				if(odm.radianAbs > M_PI){
					spin = (2*M_PI - odm.radianAbs)*(2*M_PI - odm.radianAbs)*10;
				}
				if(spin<-3.0){
					spin=-3.0;
				}
				if(spin>3.0){
					spin=3.0;
				}
				if(odm.radianAbs < M_PI){
					radian = odm.radianAbs;
				}else{
					radian = -2*M_PI + odm.radianAbs;
				}
				//serial.printf("%f,%f,%f,%f,%d,%d,%d\r\n",rad,omni.motorPower[0],omni.motorPower[1],omni.motorPower[2],omni.encCnt[0],omni.encCnt[1],omni.encCnt[2]);
				serial.printf("%f,%f,%f,%f\n\r",spin,radian,odm.integralX,odm.integralY);
				waitBreak(50);
				if(sw0.digitalRead() == 0){
					while(sw0.digitalRead() == 0);

					break;
				}
			}
			break;

		case 6:
			motor0.drive(1.0,1,0);
			waitBreak(1000);
			motor1.drive(1.0,1,0);
			waitBreak(1000);
			motor2.drive(1.0,1,0);
			waitBreak(1000);
			break;

		case 7:
			spinCnt=0;
			targetRadian = 0;
			odm.reset();
			while(sw0.digitalRead()){
				odm.update();
				serial.printf("%f,%f\n\r",odm.radianAbs,spin);
				spinCnt++;

				if(spinCnt == 100){
					targetRadian = M_PI/2;
				}
				if(spinCnt == 200){
					targetRadian = M_PI;
				}
				if(spinCnt == 300){
					targetRadian = 3*M_PI/2;
				}
				if(spinCnt == 400){
					targetRadian = 0;
					spinCnt = 0;
				}

				radianDif = odm.radianAbs - targetRadian;
				radianOld = radian;
				if(radianDif < M_PI){
					radian = radianDif;
				}else{
					radian = radianDif - 2*M_PI;
				}
				spin = radian*radian*radian*radian*radian*radian*radian*10000 + (radian - radianOld)*GAIN_SPIN_D;

				if(spin<-3.0)spin=-3.0;
				if(spin>3.0)spin=3.0;

				omni.request(0,0,-spin);

				omni.drive();
				waitBreak(10);
			}
			break;

		}
	}
}


int waitBreak(int waitTime){
	Sw0 sw0;
	sw0.setupDigitalIn();
	waitTime += millis();
	while(waitTime > millis()){
		if(sw0.digitalRead() == 0){
			return 1;
		}
	}
	return 0;
}

void sound(int pattern){
	Buzzer bz;
	bz.setupPwmOut(1000,0);
	if(pattern > PAT_NUM)return;
	switch(pattern){
	case 0:
		bz.setupPwmOut(5000,0.5);
		waitBreak(80);
		break;
	case 1:
		bz.setupPwmOut(5000,0.5);
		waitBreak(50);
		bz.setupPwmOut(5000,0);
		waitBreak(30);
		bz.setupPwmOut(5000,0.5);
		waitBreak(50);
		break;
	case 2:
		bz.setupPwmOut(5000,0.5);
		waitBreak(80);
		bz.setupPwmOut(5000,0);
		waitBreak(100);
		bz.setupPwmOut(5000,0.5);
		waitBreak(80);
		break;
	case 3:
		bz.setupPwmOut(2500,0.5);
		waitBreak(200);
		bz.setupPwmOut(5000,0.5);
		waitBreak(50);
		break;
	}
	bz.setupPwmOut(5000,0);
}
