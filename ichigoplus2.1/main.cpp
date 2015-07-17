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

void control(float targetX,float targetY,float targetRad);

#define GAIN_P 0.5
#define GAIN_I 0
#define GAIN_D 0.1

#define C 0
#define P 1
#define I 2
#define D 3

#define GAIN_SPIN_D 1

#define TargetQuantity 7

int main(void)
{
	float spin=0,radianDif,distanceX,distanceY;
	float targetY[] = {0,   0,250,   0,-250,  0,0,0};
	float targetX[] = {0,-250,   0,250,   0,-250,0,0};
	float targetRadian=0;
 	float radian = 0;
 	float power;
	float radianOld;

	float v_radian,v_slope,v_distance,c_radius = 50,d_radian,q_radian,d_r;

	int cnt=0;
	int flag[3],testMode,swFlag;



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

	wait(500);
	Enc0 enc0;
	Enc1 enc1;
	Enc2 enc2;

	enc0.setup();
	enc1.setup();
	enc2.setup();

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
	OmniOdometry odm(82.5,enc0,enc1,enc2);



	while(1){
		swFlag = 1;
		testMode = 0;
		cnt=1;
		serial.printf("MOTOR TEST \n\r SW1 and SW3 : select  SW2 : START\n\r");
		wait(100);
		while(1){

			if(sw1.digitalRead() == 0){
				while(sw1.digitalRead() == 0);
				if(testMode < 5)testMode++;
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
			/*if(sw0.digitalRead() == 0){
				testMode = 10;
				break;
			}*/
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
					serial.printf("ODOMETRY TEST\n\r");
					break;
				case 2:
					serial.printf("MOTOR REVOLUTION TEST\n\r");
					break;
				case 3:
					serial.printf("START/STOP PWM TEST\n\r");
					break;
				case 4:
					serial.printf("CIRCLE\n\r");
					break;
				case 5:
					serial.printf("MOTOR POSITION CHECK\n\r");
					break;
				}
				wait(200);
			}
		}


		switch(testMode){
		case 0:
			odm.update();
			for(int i=0;i<=TargetQuantity;i++){
				targetX[i]+=odm.integralX;
				targetY[i]+=odm.integralY;
				serial.printf("X= %f,Y= %f\n\r",targetX[i],targetY[i]);
			}
			targetRadian += odm.radianAbs;

			while(1){
				odm.update();
				distanceX = targetX[cnt] - odm.integralX;
				distanceY = targetY[cnt] - odm.integralY;

				if(sqrt(distanceX*distanceX +distanceY*distanceY) < c_radius){//目標地点が近ければ仮想軌道追尾はしない
			        d_radian = atan2(targetY[cnt] - odm.integralY,targetX[cnt] - odm.integralX);
			        serial.printf(" CLOSE");
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
			                serial.printf(" Q1");
			            }else if(q_radian <= M_PI){
			                d_radian = v_radian - d_radian;
			                serial.printf(" Q2");
			            }else if(q_radian <= (3*M_PI)/2){
			                d_radian = v_radian + d_radian;
			                serial.printf(" Q3");
			            }else{
			                d_radian = v_radian - d_radian;
			                d_radian += M_PI;
			                serial.printf(" Q4");
			            }
			        }else{
			            //d_radian = atan2(odm.integralY - targetY[cnt],odm.integralX - targetX[cnt]);
			        	d_radian = atan2(targetY[cnt] - odm.integralY,targetX[cnt] - odm.integralX);
			            serial.printf(" V_LINE FAR");
			        }
			    }
				if(d_radian < 0)d_radian+=2*M_PI;
				serial.printf("\n\r%f , %f , %f , %f , %f , %f , %f ",odm.integralX,odm.integralY,v_distance,v_radian,d_r,q_radian,d_radian);


				if(sqrt(distanceX*distanceX + distanceY*distanceY) < 10){
					cnt++;
					serial.printf("NEXT POINT");
				}
				if(cnt>TargetQuantity){
					cnt--;
					omni.drive(0,0,0);
					wait(100);
					break;
				}


				power = sqrt(distanceX*distanceX + distanceY*distanceY)*0.01;
				if(power > 1.0)power = 1.0;

				radianDif = odm.radianAbs - targetRadian;
				radianOld = radian;
				if(radianDif < M_PI){
					radian = radianDif;
				}else{
					radian = radianDif - 2*M_PI;
				}
				spin = radian*radian*radian*radian*radian*2000 + (radianOld - radian)*GAIN_SPIN_D;


				if(spin<-3.0)spin=-3.0;
				if(spin>3.0)spin=3.0;

				omni.drive(d_radian - radianDif,power,-spin);
				wait(10);
				if(sw0.digitalRead() == 0){
					while(sw0.digitalRead() == 0);
					omni.drive(0,0,0);
					wait(100);
					break;
				}
			}
			break;


		case 1:
			while(flag[0]){
				serial.printf("ODOMETRY TEST\n\r");
				odm.update();
				if(odm.radianAbs > 0){
					radian = odm.radianAbs;
				}else{
					radian = odm.radianAbs - 2*M_PI;
				}
				//serial.printf("%f , %f , %f , %f , %f , %d\n\r",odm.integralX,odm.integralY,radian*180/M_PI,odm.X,odm.Y,odm.radianDelta - odm.radianOrg);
				//serial.printf("%d %d %d\n\r",odm.encTest[0],odm.encTest[1],odm.encTest[2]);
				serial.printf("%d %d %d\n\r",enc0.count(),enc1.count(),enc2.count());
				if(sw0.digitalRead() == 0){
					while(sw0.digitalRead() == 0);
					break;
				}
				wait(100);
			}

			break;


		case 2:
			motor0.drive(1.0,1,0);
			motor1.drive(1.0,1,0);
			motor2.drive(1.0,1,0);
			wait(1000);
			encOld0 = enc0.count();
			encOld1 = enc1.count();
			encOld2 = enc2.count();
			wait(10000);
			serial.printf("ENC0 = %d ,ENC1 = %d ,ENC2 = %d\n\r",enc0.count() - encOld0,enc1.count() - encOld1,enc2.count() - encOld2);
			serial.printf("INVERSE");
			motor0.drive(1.0,0,1);
			motor1.drive(1.0,0,1);
			motor2.drive(1.0,0,1);
			wait(1000);
			encOld0 = enc0.count();
			encOld1 = enc1.count();
			encOld2 = enc2.count();
			wait(10000);
			serial.printf("ENC0 = %d ,ENC1 = %d ,ENC2 = %d\n\r",enc0.count() - encOld0,enc1.count() - encOld1,enc2.count() - encOld2);
			motor0.drive(0,1,0);
			motor1.drive(0,1,0);
			motor2.drive(0,1,0);
			break;


		case 3:
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
					wait(1000);
					break;
				}
				encOld0 = enc0.count();
				encOld1 = enc1.count();
				encOld2 = enc2.count();
				wait(500);
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
				wait(500);
			}
			break;

		case 4:
			for(float rad=0;rad<2*M_PI;rad+=(2*M_PI)/180){		//円形移動
				omni.drive(rad,1.0,spin);
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
				wait(50);
				if(sw0.digitalRead() == 0){
					while(sw0.digitalRead() == 0);
					break;
				}
			}
			break;

		case 5:
			motor0.drive(1.0,1,0);
			wait(1000);
			motor1.drive(1.0,1,0);
			wait(1000);
			motor2.drive(1.0,1,0);
			wait(1000);
			break;


		}

	}
}
