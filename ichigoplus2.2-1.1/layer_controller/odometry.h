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
    float cumulativeY;	//�ݐύ��W
    int radianOld;
    float length;    //���S����G���R�[�_�܂ł̋���
    int encTime;
    int setupFlag;

public:

    float integralX;
    float integralY;  //���ݍ��W

    float X;
    float Y;            //�ړ���
    float radian;
    int radianDelta;
    float degree;       //�ړ��p�x
    float radianAbs;    //���{�b�g�̊p�x

    int encCntDif;   //�G���R�[�_�̑O��̒l
    long radianOrg;   //�@�̊p�x��G���R�[�_���̒l

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
        cumulativeY = 0;	//�ݐύ��W
        radianOld = 0;
        length = 0;    //���S����G���R�[�_�܂ł̋���
        encTime = 0;

        encOffSet[0] = 0;
        encOffSet[1] = 0;
        encOffSet[2] = 0;
    	encData[0] = 0;
    	encData[1] = 0;
    	encData[2] = 0;

        integralX = 0;
        integralY = 0;  //���ݍ��W

        X = 0;
        Y = 0;            //�ړ���
        radian = 0;
        radianDelta = 0;
        degree = 0;       //�ړ��p�x
        radianAbs = 0;    //���{�b�g�̊p�x

        encCntDif = 0;   //�G���R�[�_�̑O��̒l
        radianOrg = 0;   //�@�̊p�x��G���R�[�_���̒l

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
