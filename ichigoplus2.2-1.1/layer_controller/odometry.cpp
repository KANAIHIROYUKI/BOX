#include "odometry.h"




void OmniOdometry::update(){
	encData[0] = enc0->count();
	encData[1] = enc1->count()*5;
	encData[2] = enc2->count()*5;
    radianOrg = encData[0] + encData[1] + encData[2] - encOffSet[0]  - encOffSet[1]  - encOffSet[2];	//角度生の値(累積による誤差が発生しない)
    radianAbs = (radianOrg*M_PI)/(150*length);	//弧度法に直す

    while(radianAbs > 2*M_PI){//0 =< rad =< 2piに直す
        radianAbs -= 2*M_PI;
    }
    while(radianAbs < 0){
        radianAbs += 2*M_PI;
    }

    enc[0] = encData[0] - encOld[0];// - encOffSet[0];
    enc[1] = encData[1] - encOld[1];// - encOffSet[1];
    enc[2] = encData[2] - encOld[2];// - encOffSet[2];

    encTest[0] = encData[0];
    encTest[1] = encData[1];
    encTest[2] = encData[2];

    encCntDif = abs(enc[0]) + abs(enc[1]) + abs(enc[2]);

    X = -enc[FRONT] + enc[LEFT]/2 + enc[RIGHT]/2;
    Y = sqrtf(3)*enc[RIGHT]/2 - sqrtf(3)*enc[LEFT]/2;//自己位置計算
    X = (X*M_PI)/(7.3*10);
    Y = (Y*M_PI)/(7.3*10);

    integralX = cumulativeX + X*cos(radianAbs) - Y*sin(radianAbs);
    integralY = cumulativeY + X*sin(radianAbs) + Y*cos(radianAbs);//絶対座標に変換

    if((encCntDif > 300 && encCntDif <= 1000) || radianDelta - radianOrg > 50 || radianDelta - radianOrg < -50){						//前回計算時との差が一定以上に達したら計算
    	radianDelta = radianOrg;
        radian = atan2(Y,X);
        cumulativeX += X*cos(radianAbs) - Y*sin(radianAbs);
        cumulativeY += X*sin(radianAbs) + Y*cos(radianAbs);//絶対座標に変換
        degree = radian*180/M_PI;

        encOld[0] = encData[0];// - encOffSet[0];
        encOld[1] = encData[1];// - encOffSet[1];
        encOld[2] = encData[2];// - encOffSet[2];
    }else if(encCntDif >= 1000){
    	//radianCorrection -= encCntDif;
    	encOld[0] = encData[0];// - encOffSet[0];
        encOld[1] = encData[1];// - encOffSet[1];
    	encOld[2] = encData[2];// - encOffSet[2];
    }

};

void OmniOdometry::reset(){
	encOffSet[0] = enc0->count();
	encOffSet[1] = enc1->count()*5;
	encOffSet[2] = enc2->count()*5;
	encOld[0] = enc0->count();
	encOld[1] = enc1->count()*5;
	encOld[2] = enc2->count()*5;
	cumulativeX = 0;
	cumulativeY = 0;
};

int abs(int value){
	if(value < 0){
		value = -value;
	}
	return value;
}
