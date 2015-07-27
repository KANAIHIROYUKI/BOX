#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdio.h>
#include "digital.hpp"
#include "pwm.hpp"
#include "mcutime.h"
#include "math.h"
#include "encoder.hpp"
#include "pin.hpp"

int abs(int a);

#define FRONT 1
#define RIGHT 0
#define LEFT  2



class OmniOdometry{
public:

    float integralX;
    float integralY;  //現在座標
    float cumulativeX;
    float cumulativeY;	//累積座標

    float X;
    float Y;            //移動量
    float radian;
    float degree;       //移動角度
    float radianAbs;    //ロボットの角度
    int radianDelta;
    int radianOld;
    //int radianCorrection;
    float length;    //中心からエンコーダまでの距離

    int encCntDif;   //エンコーダの前回の値
    int radianOrg;   //機体角度､エンコーダ生の値

    int encTime;

    Enc0 *enc0;
    Enc1 *enc1;
    Enc2 *enc2;

    int encTest[3];
    int enc[3];
    int encOld[3];
    int encOffSet[3];


    OmniOdometry(float wheelLength,Enc0 &enc0,Enc1 &enc1,Enc2 &enc2){

    	length = wheelLength;
    	this->enc0 = &enc0;
    	this->enc1 = &enc1;
    	this->enc2 = &enc2;

    	enc0.setup();
    	enc1.setup();
    	enc2.setup();

    	encOld[0] = enc0.count();
    	encOld[1] = enc1.count();
    	encOld[2] = enc2.count();

    	encTime = micros();
    };

    void update();
    void reset();


};




#endif
