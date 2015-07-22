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


class OmniOdometry
{
    public:
    float integralX=0;
    float integralY=0;  //���ݍ��W
    float cumulativeX=0;
    float cumulativeY=0;	//�ݐύ��W

    float X;
    float Y;            //�ړ���
    float radian;
    float degree;       //�ړ��p�x
    float radianAbs;    //���{�b�g�̊p�x
    int radianDelta;
    int radianOld;
    float length=30;    //���S����G���R�[�_�܂ł̋���


    int encCntDif = 0;   //�G���R�[�_�̑O��̒l
    int radianOrg = 0;   //�@�̊p�x��G���R�[�_���̒l

    int encTime = 0;

    Enc0 *enc0;
    Enc1 *enc1;
    Enc2 *enc2;


    int encTest[3];
    int enc[3];
    int encOld[3];


    OmniOdometry(float wheelLength,Enc0 &enc0,Enc1 &enc1,Enc2 &enc2){
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

        length = wheelLength;
        radianAbs = 0;

        printf("%f\n",length);
    };

    void update();


};




#endif
