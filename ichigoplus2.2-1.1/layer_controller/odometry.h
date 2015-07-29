#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdio.h>
#include "digital.hpp"
#include "pwm.hpp"
#include "mcutime.h"
#include "math.h"
#include "encoder.hpp"
#include "pin.hpp"
#include "circuit/can_encoder.hpp"
#include "can.hpp"

int abs(int a);

#define ENC_FRONT 2
#define ENC_RIGHT 1
#define ENC_LEFT  0



class OmniOdometry{
private:
    float cumulativeX;
    float cumulativeY;	//累積座標
    int radianOld;
    float length;    //中心からエンコーダまでの距離
    int encTime;
    int setupFlag;

public:

    float integralX;
    float integralY;  //現在座標

    float X;
    float Y;            //移動量
    float radian;
    int radianDelta;
    float degree;       //移動角度
    float radianAbs;    //ロボットの角度

    int encCntDif;   //エンコーダの前回の値
    long radianOrg;   //機体角度､エンコーダ生の値

    Can0 can;
    CanEncoder *enc0;
    CanEncoder *enc1;
    CanEncoder *enc2;

    int enc[3];
    int encOld[3];
    int encOffSet[3];
	int encData[3];

    OmniOdometry(float wheelLength,CanEncoder &enc0,CanEncoder &enc1,CanEncoder &enc2){

    	setupFlag = 0;
        cumulativeX = 0;
        cumulativeY = 0;	//累積座標
        radianOld = 0;
        length = 0;    //中心からエンコーダまでの距離
        encTime = 0;

        encOffSet[0] = 0;
        encOffSet[1] = 0;
        encOffSet[2] = 0;
    	encData[0] = 0;
    	encData[1] = 0;
    	encData[2] = 0;

        integralX = 0;
        integralY = 0;  //現在座標

        X = 0;
        Y = 0;            //移動量
        radian = 0;
        radianDelta = 0;
        degree = 0;       //移動角度
        radianAbs = 0;    //ロボットの角度

        encCntDif = 0;   //エンコーダの前回の値
        radianOrg = 0;   //機体角度､エンコーダ生の値

    	length = wheelLength;

    	this->enc0 = &enc0;
    	this->enc1 = &enc1;
    	this->enc2 = &enc2;
    };

    int setup();
    void update();
    void reset();


};




#endif
