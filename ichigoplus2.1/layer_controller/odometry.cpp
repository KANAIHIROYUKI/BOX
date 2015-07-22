#include "odometry.h"


void OmniOdometry::update(){
    radianOrg = enc0->count() + enc1->count() + enc2->count();	//�p�x���̒l(�ݐςɂ��덷���������Ȃ�)����Ȃ̂͐ݒ�ƃG���R�[�_�̉�]���t�Ȃ���
    radianAbs = (radianOrg*M_PI)/(15*length);	//�ʓx�@�ɒ���

    while(radianAbs >= 2*M_PI){//0 =< rad =< 2pi�ɒ���
        radianAbs -= 2*M_PI;
    }
    while(radianAbs < 0){
        radianAbs += 2*M_PI;
    }

    enc[0] = enc0->count() - encOld[0];
    enc[1] = enc1->count() - encOld[1];
    enc[2] = enc2->count() - encOld[2];

    encTest[0] = enc0->count();
    encTest[1] = enc1->count();
    encTest[2] = enc2->count();

    encCntDif = abs(enc[0]) + abs(enc[1]) + abs(enc[2]);

    X = -enc[FRONT] + enc[LEFT]/2 + enc[RIGHT]/2;
    Y = sqrtf(3)*enc[RIGHT]/2 - sqrtf(3)*enc[LEFT]/2;//���Ȉʒu�v�Z
    X = (X*M_PI)/7.3;
    Y = (Y*M_PI)/7.3;

    integralX = cumulativeX + X*cos(radianAbs) - Y*sin(radianAbs);
    integralY = cumulativeY + X*sin(radianAbs) + Y*cos(radianAbs);//��΍��W�ɕϊ�

    if(encCntDif > 30 && encCntDif <= 100|| radianDelta - radianOrg > 10 || radianDelta - radianOrg < -10){						//�O��v�Z���Ƃ̍������ȏ�ɒB������v�Z
    	radianDelta = radianOrg;
        radian = atan2(Y,X);
        cumulativeX += X*cos(radianAbs) - Y*sin(radianAbs);
        cumulativeY += X*sin(radianAbs) + Y*cos(radianAbs);//��΍��W�ɕϊ�
        degree = radian*180/M_PI;

        encOld[0] = enc0->count();
        encOld[1] = enc1->count();
        encOld[2] = enc2->count();
    }else if(encCntDif >= 100){
    	radianOrg =  - enc0->count() + encOld[0] - enc1->count() + encOld[1] - enc2->count() + encOld[2];
    	encOld[0] = enc0->count();
        encOld[1] = enc1->count();
    	encOld[2] = enc2->count();
    }
};


int abs(int value){
	if(value < 0){
		value = -value;
	}
	return value;
}
