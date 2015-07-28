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

#define FRONT 2
#define RIGHT 1
#define LEFT  0



class OmniOdometry{
public:

    float integralX = 0;
    float integralY = 0;  //���ݍ��W
    float cumulativeX = 0;
    float cumulativeY = 0;	//�ݐύ��W

    float X = 0;
    float Y = 0;            //�ړ���
    float radian = 0;
    float degree = 0;       //�ړ��p�x
    float radianAbs = 0;    //���{�b�g�̊p�x
    int radianDelta = 0;
    int radianOld = 0;
    //int radianCorrection;
    float length = 0;    //���S����G���R�[�_�܂ł̋���

    int encCntDif = 0;   //�G���R�[�_�̑O��̒l
    long radianOrg = 0;   //�@�̊p�x��G���R�[�_���̒l

    int encTime = 0;

    /*Enc0 *enc0;
    Enc1 *enc1;
    Enc2 *enc2;*/

    Can0 can;
    CanEncoder *enc0;
    CanEncoder *enc1;
    CanEncoder *enc2;

    int encTest[3] = {0,0,0};
    int enc[3] = {0,0,0};
    int encOld[3] = {0,0,0};
    int encOffSet[3] = {0,0,0};
	int encData[3] = {0,0,0};

    OmniOdometry(float wheelLength,CanEncoder &enc0,CanEncoder &enc1,CanEncoder &enc2){

    	length = wheelLength;
    	this->enc0 = &enc0;
    	this->enc1 = &enc1;
    	this->enc2 = &enc2;

    	enc0.setup();
    	enc1.setup();
    	enc2.setup();

    	encOld[0] = enc0.count();
    	encOld[1] = enc1.count()*5;
    	encOld[2] = enc2.count()*5;

    	encTime = micros();
    };

    void update();
    void reset();


};




#endif
