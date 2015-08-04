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

#define GAIN_SPIN_D 5

#define ControlCycle 10


int waitBreak(int waitTime);
void sound(int pattern);


int main(void)
{
	float spin=0,radianDif,distanceX,distanceY;
	unsigned char profile = 0;
	unsigned char targetQuantity[] = {4,6,4,5,6,16,0};//座標のある最後の配列の数
	float targetX[7][17] = {{0,    0,    0, -750, -750,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
						 	{0,    0,  750,  750,  750, -750, -750,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
						 	{0,    0, 1500, 1500, 1500,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
					 	 	{0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
					 	 	{0,  500,  500,    0,    0,  250,  500,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
						 	{0,    0,   80,  160,  160,  160,  100,    0,  220,  220,  320,  380,  400,  380,  320,  220,  220},
							{0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0}};
	float targetY[7][17] = {{0, 1750, 1000, 1000,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
							{0, 1000, 1000, 1750, 1000, 1000,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
							{0, 1000, 1000, 1750,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
							{0,  300,    0,  300,    0,  300,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
							{0,    0,  500,  500,    0,  500,  250,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
							{0,  350,  350,  310,  260,  200,  200,  200,   40,  350,  350,  330,  280,  230,  210,  210,   40},
							{0, 1500,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0}};

	unsigned char armPos[7][17] = {{0,5,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
								   {0,0,0,5,2,0,0,0,0,0,0,0,0,0,0,0,0},
								   {0,0,0,5,2,0,0,0,0,0,0,0,0,0,0,0,0},
								   {0,0,5,2,0,5,2,0,0,0,0,0,0,0,0,0,0},
							       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
								   {0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};//unsigned char armPos[] = {0,  0,  5,  2,  0,  5,  2};//1,2は動きながら､5,6は止まったまま

	float targetDegree[7][17] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
								 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
								 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
								 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
								 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
								 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
								 {0,180, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

	float percentage=0;

	unsigned char armOrder = 0;

	float targetRadian=0;

 	float radian = 0;
 	float power;
	float radianOld;

	float v_radian,v_slope,v_distance,c_radius = ReferenceCircle,d_radian,q_radian,d_r;

	int cnt=0,buzzerCnt = 0,spinCnt;
	int testMode,swFlag;

	int encOld[] = {0,0,0};
	int encFlag[] = {0,0,0};

	int_least32_t cycleTime;
	int armTime=0;

	Serial0 serial;
	serial.setup(115200);

	A0 a0;
	A1 a1;
	A2 a2;
	A3 a3;
	A4 a4;
	a0.setupDigitalIn();
	a1.setupDigitalIn();
	a2.setupDigitalIn();
	a3.setupDigitalIn();
	a4.setupDigitalIn();

	/*a0.setupAnalogIn();
	a1.setupAnalogIn();
	a2.setupAnalogIn();
	a3.setupAnalogIn();
	a4.setupAnalogIn();*/

	/*while(1){
		serial.printf("%d %d %d %d "
				"%d\n\r",a0.digitalRead(),a1.digitalRead(),a2.digitalRead(),a3.digitalRead(),a4.digitalRead());
		//serial.printf("%f %f %f %f %f\n\r",a0.analogRead(),a1.analogRead(),a2.analogRead(),a3.analogRead(),a4.analogRead());
		wait(100);
	}*/

	Sw0 sw0;
	Sw1 sw1;
	Sw2 sw2;
	Sw3 sw3;

	sw0.setupDigitalIn();
	sw1.setupDigitalIn();
	sw2.setupDigitalIn();
	sw3.setupDigitalIn();

	Led0 led0;
	Led1 led1;
	Led2 led2;
	Led3 led3;

	led0.setupDigitalOut();
	led1.setupDigitalOut();
	led2.setupDigitalOut();
	led3.setupDigitalOut();
	led0.digitalHigh();
	led1.digitalHigh();
	led2.digitalHigh();
	led3.digitalHigh();

	Buzzer bz;
	bz.setupPwmOut(4000,0);

	/*while(1){
		serial.printf("ENC0: %d ENC1: %d ENC2: %d\n\r",canEnc0.count(),canEnc1.count(),canEnc2.count());
		wait(100);
	}*/

	Enc0 enc0;
	Enc1 enc1;
	Enc2 enc2;

	Pwm0 pwm0;
	Pwm1 pwm1;
	Pwm2 pwm2;
	Pwm3 pwm3;
	CW0 cw0;
	CW1 cw1;
	CW2 cw2;
	CW3 cw3;
	CCW0 ccw0;
	CCW1 ccw1;
	CCW2 ccw2;
	CCW3 ccw3;

	Motor motor0(pwm0,cw0,ccw0);
	Motor motor1(pwm1,cw1,ccw1);
	Motor motor2(pwm2,cw2,ccw2);
	Motor motor3(pwm3,cw3,ccw3);
	motor0.setup();
	motor1.setup();
	motor2.setup();
	motor3.setup();


	/*while(1){
		motor3.drive(0.3,sw1.digitalRead(),sw2.digitalRead());
		//wait(200);
		//motor3.drive(0,0,0);
		//wait(50);
	}*/

	Can0 can;
	can.setup();
	CanEncoder canEnc0(can,0,5);
	CanEncoder canEnc1(can,1,5);
	CanEncoder canEnc2(can,2,5);
	canEnc0.setup();
	canEnc1.setup();
	canEnc2.setup();

	Omni omni(motor0,motor1,motor2,enc0,enc1,enc2);
	omni.setup();
	OmniOdometry odm(110,canEnc0,canEnc1,canEnc2);
	odm.setup();

	while(1){
		while(sw0.digitalRead() == 0);
		sound(SOUND_BREAK);
		testMode = 0;
		cnt=1;
		swFlag = 1;
		serial.printf("SW1 and SW3 : SELECT  SW2 : START\n\r");
		omni.request(0,0,0);
		omni.drive();
		bz.pwmWrite(0);
		while(sw2.digitalRead()){
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
		while(sw2.digitalRead() == 0);

		switch(testMode){

		case 0:
			wait(500);
			while(sw2.digitalRead()){
				if(sw1.digitalRead() == 0 && profile > 0){
					while(sw1.digitalRead() == 0);
					profile--;
					serial.printf("PROFILE : %d\n\r",profile);
					sound(SOUND_OPERATION);

				}
				if(sw3.digitalRead() == 0 && profile < 6){
					while(sw3.digitalRead() == 0);
					profile++;
					serial.printf("PROFILE : %d\n\r",profile);
					sound(SOUND_OPERATION);
				}
				led0.digitalWrite(profile & 1);
				led1.digitalWrite((profile & 2) >> 1);
				led2.digitalWrite((profile & 4) >> 2);
				led3.digitalWrite((profile & 8) >> 3);

			}
			sound(SOUND_ENTER);
			odm.update();
			odm.reset();
			odm.update();
			armTime = millis() - 1000;
			cycleTime = millis() + ControlCycle;
			serial.printf("%f,%f,%f,%d,%d,%d\n\r",odm.integralX,odm.integralY,odm.radianAbs,odm.encData[0],odm.encData[1],odm.encData[2]);
			while(sw0.digitalRead()){
				odm.update();

				distanceX = targetX[profile][cnt] - odm.integralX;
				distanceY = targetY[profile][cnt] - odm.integralY;

				if(sqrt(distanceX*distanceX + distanceY*distanceY) < c_radius){//目標地点が近ければ仮想軌道追尾はしない
			        d_radian = atan2(targetY[profile][cnt] - odm.integralY,targetX[profile][cnt] - odm.integralX);
			        //serial.printf(" CLOSE");
			    }else{
			        if(targetX[profile][cnt-1] == targetX[profile][cnt]){
			            v_distance = targetX[profile][cnt] - odm.integralX;
			            if(v_distance <0 )v_distance=-v_distance;
			        }else{
			            if(targetY[profile][cnt-1] != targetY[profile][cnt]){
			                v_slope = (targetY[profile][cnt] - targetY[profile][cnt-1])/(targetX[profile][cnt] + targetX[profile][cnt-1]);
			                if(v_slope > 1 || v_slope < -1){//X-Y座標､直線との距離
			                    v_distance = (v_slope*odm.integralX) - v_slope*targetX[profile][cnt-1] + targetY[profile][cnt-1] - odm.integralY;
			                    if(v_distance < 0)v_distance = -v_distance;
			                    v_distance /= sqrt(1 + (v_slope*v_slope));
			                }else{//Y-X座標､直線との距離
			                    v_slope = (targetX[profile][cnt] - targetX[profile][cnt-1])/(targetY[profile][cnt] - targetY[profile][cnt-1]);
			                    v_distance = v_slope*odm.integralY - v_slope*targetY[profile][cnt-1] + targetX[profile][cnt-1] - odm.integralX;
			                    if(v_distance < 0)v_distance = -v_distance;
			                    v_distance /= sqrt(1 + (v_slope*v_slope));
			                }
			            }else{
			                v_distance = targetY[profile][cnt] - odm.integralY;
			            }
			        }
			        //serial.printf("%f\n",v_distance);

			        if(v_distance < c_radius){
			            v_radian = atan2((targetY[profile][cnt] - targetY[profile][cnt-1]),(targetX[profile][cnt] - targetX[profile][cnt-1]));
			            q_radian = atan2((odm.integralY - targetY[profile][cnt]),(odm.integralX - targetX[profile][cnt])) - v_radian;

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
			        	d_radian = atan2(targetY[profile][cnt] - odm.integralY,targetX[profile][cnt] - odm.integralX);
			            //serial.printf(" V_LINE FAR");
			        }
			    }
				if(d_radian < 0)d_radian+=2*M_PI;


				power = sqrt(distanceX*distanceX + distanceY*distanceY)*GAIN_P + (sqrt(distanceX*distanceX + distanceY*distanceY)*GAIN_P - power)*GAIN_D;
				if(power > 1.0)power = 1.0;


				targetRadian = targetDegree[profile][cnt-1] * sqrt(distanceX*distanceX + distanceY*distanceY);
				percentage = sqrt(distanceX*distanceX + distanceY*distanceY);

				distanceX = targetX[profile][cnt-1] - odm.integralX;
				distanceY = targetY[profile][cnt-1] - odm.integralY;

				targetRadian += targetDegree[profile][cnt] * sqrt(distanceX*distanceX + distanceY*distanceY);
				percentage += sqrt(distanceX*distanceX + distanceY*distanceY);

				targetRadian = (targetRadian * M_PI) / (percentage*180);
				radianDif = odm.radianAbs - targetRadian;
				radianOld = radian;
				if(radianDif < M_PI){
					radian = radianDif;
				}else{
					radian = radianDif - 2*M_PI;
				}
				spin = radian*radian*radian*radian*radian*100000 + (radian - radianOld)*GAIN_SPIN_D;

				if(spin<-3.0)spin=-3.0;
				if(spin>3.0)spin=3.0;

				distanceX = targetX[profile][cnt] - odm.integralX;
				distanceY = targetY[profile][cnt] - odm.integralY;

				if(armTime + 1000 > millis() && armOrder != 0){
					motor3.drive(1.0,(armOrder&2)>>1,armOrder&1);
					serial.printf("MOVING %d\n\r",armOrder);
				}else{
					armOrder=0;
					motor3.drive(0,0,0);
				}

				if(sqrt(distanceX*distanceX + distanceY*distanceY) < 10 && cnt <= targetQuantity[profile]){//目標地点に接近した時の処理
					cnt++;
					if(armPos[profile][cnt] != 0){
						if(armPos[profile][cnt] > 2 && armOrder == 0 && armTime + 1000 <= millis()){
							armOrder = armPos[profile][cnt];
							armTime = millis();
							cnt--;
							serial.printf("ARM MOVE BEGIN %d\n\r",armOrder);
						}else if(armOrder != 0 && armTime + 1000 >= millis()){
							cnt--;
						}else /*if(armOrder == 0 && armTime + 1000 < millis())*/{
							if(armPos[profile][cnt+1] != 0){
								armOrder = armPos[profile][cnt+1];
								armTime = millis();
								serial.printf("NETX POINT WITH ARM MOVE %d\n\r",armOrder);
							}
						}/*else{
							serial.printf("END\n\r");
						}*/
					}else{//アーム操作要求なし
						armOrder = 0;
						armTime = millis() - 1000;
						serial.printf("NEXT POINT\n\r");
					}
					buzzerCnt = 10;
				}else if(sqrt(distanceX*distanceX + distanceY*distanceY) < 5 && cnt > targetQuantity[profile]){
					break;
				}

				if(buzzerCnt > 0){
					buzzerCnt--;
					bz.pwmWrite(0.5);
				}else{
					bz.pwmWrite(0);
				}

				if(cycleTime < millis()){
					cycleTime+=ControlCycle;
					omni.request(d_radian - odm.radianAbs,power,-spin);
					omni.drive();
					//serial.printf("\n\r%f , %f , %f , %f , %f , %f , %f ,%d , %d , %d , %f , %f , %f , %d",odm.integralX,odm.integralY,v_distance,v_radian,d_r,q_radian,d_radian,odm.encData[0],odm.encData[1],odm.encData[2],radian,sqrt(distanceX*distanceX + distanceY*distanceY),spin,millis());
					serial.printf("%d %d %d %f\n\r",cnt,armTime,armOrder,sqrt(distanceX*distanceX + distanceY*distanceY));
					//serial.printf("\n\r %d,%f,%f,%d,%d,%d,%f,%f,",odm.radianOrg,spin,radian,odm.encTest[0],odm.encTest[1],odm.encTest[2],odm.integralX,odm.integralY);
				}

				//serial.printf("\n\r%f,%f,%f,",d_radian,radianDif,d_radian + radianDif);

			}
			omni.request(0,0,0);
			omni.drive();
			motor3.drive(0,0,0);
			break;


		case 1:
			while(sw0.digitalRead()){
				odm.update();
				//serial.printf("%d %d %d\n\r",odm.encData[0],odm.encData[1],odm.encData[2]);
				serial.printf("%d %d %d %d %d %d %d\n\r",enc0.count(),enc1.count(),enc2.count(),odm.encData[0],odm.encData[1],odm.encData[2],odm.radianOrg);
				waitBreak(100);
			}

			break;

		case 2:
			odm.reset();
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
			encOld[0] = enc0.count();
			encOld[1] = enc1.count();
			encOld[2] = enc2.count();
			waitBreak(10000);
			serial.printf("ENC0 = %d ,ENC1 = %d ,ENC2 = %d\n\r",enc0.count() - encOld[0],enc1.count() - encOld[1],enc2.count() - encOld[2]);
			serial.printf("INVERSE");
			motor0.drive(1.0,0,1);
			motor1.drive(1.0,0,1);
			motor2.drive(1.0,0,1);
			waitBreak(1000);
			encOld[0] = enc0.count();
			encOld[1] = enc1.count();
			encOld[2] = enc2.count();
			waitBreak(10000);
			serial.printf("ENC0 = %d ,ENC1 = %d ,ENC2 = %d\n\r",enc0.count() - encOld[0],enc1.count() - encOld[1],enc2.count() - encOld[2]);
			motor0.drive(0,1,0);
			motor1.drive(0,1,0);
			motor2.drive(0,1,0);
			break;

		case 4:
			encOld[0] = enc0.count();
			encOld[1] = enc1.count();
			encOld[2] = enc2.count();
			for(float i = 0.0;i<1.0;i+=0.01){
				motor0.drive(i,1,0);
				motor1.drive(i,1,0);
				motor2.drive(i,1,0);
				if(encOld[0] <= enc0.count() - 10 && encFlag[0] == 0){
					serial.printf("ENC0 %f\n\r",i);
					encFlag[0] = 1;
				}
				if(encOld[1] <= enc1.count() - 10 && encFlag[1] == 0){
					serial.printf("ENC1 %f\n\r",i);
					encFlag[1] = 1;
				}
				if(encOld[2] <= enc2.count() - 10 && encFlag[2] == 0){
					serial.printf("ENC2 %f\n\r",i);
					encFlag[2] = 1;
				}
				if(encFlag[0] == 1 && encFlag[1] == 1 && encFlag[2] == 1){
					motor0.drive(0.5,1,0);
					motor1.drive(0.5,1,0);
					motor2.drive(0.5,1,0);
					waitBreak(1000);
					break;
				}
				encOld[0] = enc0.count();
				encOld[1] = enc1.count();
				encOld[2] = enc2.count();
				waitBreak(500);
			}
			serial.printf("Deceleration\n\r");
			for(float i = 0.5;i>0.0;i-=0.01){
				motor0.drive(i,1,0);
				motor1.drive(i,1,0);
				motor2.drive(i,1,0);
				if(encOld[0] == enc0.count() && encFlag[0] == 1){
					serial.printf("ENC0 %f\n\r",i);
					encFlag[0] = 0;
				}
				if(encOld[1] == enc1.count() && encFlag[1] == 1){
					serial.printf("ENC1 %f\n\r",i);
					encFlag[1] = 0;
				}
				if(encOld[2] == enc2.count() && encFlag[2] == 1){
					serial.printf("ENC2 %f\n\r",i);
					encFlag[2] = 0;
				}
				encOld[0] = enc0.count();
				encOld[1] = enc1.count();
				encOld[2] = enc2.count();
				if(encFlag[0] == 0 && encFlag[1] == 0 && encFlag[2] == 0){
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
