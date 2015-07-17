#pragma once
#include "util.h"
/*******************************************
 * utilplus ver0.2 2015/1/14
 * This is a program made for mathematically calculation.
 * This is the first kitrp's microcomputer program that uses class!
 * [Dependency]
 * util 1.1+
 *
 * [Note]
 *
 * [Author]
 * Tomoki Nagatani
 *
 * [Change history]
 * ver1.0 2015/ 2/ 6 Add revolution. Add operator overload for Coord. Add sign.Dependency update to util 1.1+.
 * ver0.2 2015/ 1/16 Addition of add functions.
 * ver0.1 2015/ 1/11 The first version.Constuctor ,read and write members available.
 *
 ******************************************/

class Coord{
private:
	float cartesianXData;
	float cartesianYData;
	float cartesianZData;
	float eularXData;
	float eularYData;
	float eularZData;

public:
	Coord(){
		cartesianXData=0;
		cartesianYData=0;
		cartesianZData=0;
		eularXData=0;
		eularYData=0;
		eularZData=0;
	};
	Coord(Coord const &coord);
	Coord(float cx,float cy,float cz);
	Coord(float cx,float cy,float cz,float ex,float ey,float ez);

	void cartesian(float cx,float cy,float cz);
	float cartesianX();
	float cartesianY();
	float cartesianZ();
	void cartesianX(float x);
	void cartesianY(float y);
	void cartesianZ(float z);
	void cartesianXAdd(float x);
	void cartesianYAdd(float y);
	void cartesianZAdd(float z);

	void polar(float radius,float theta,float phi);
	float polarRadius();
	float polarTheta();
	float polarPhi();
	void polarRadius(float radius);
	void polarTheta(float theta);
	void polarPhi(float phi);
	void polarRadiusAdd(float radius);
	void polarThetaAdd(float theta);
	void polarPhiAdd(float phi);

	void eular(float x,float y,float z);
	float eularX();
	float eularY();
	float eularZ();
	void eularX(float x);
	void eularY(float y);
	void eularZ(float z);
	void eularXAdd(float x);
	void eularYAdd(float y);
	void eularZAdd(float z);

	float x();
	float y();
	float z();
	float radius();
	float theta();
	float phi();

	float ex();
	float ey();
	float ez();

	float Reradius(float deg);

	void revolutionXY(float angle);
	void revolutionYZ(float angle);
	void revolutionZX(float angle);

	void revolutionX(float angle);
	void revolutionY(float angle);
	void revolutionZ(float angle);

	void revolutionEuler(float a, float b, float c);
	void revolutionEulerinverse(float a, float b, float c);

};

Coord operator+(Coord coord1,Coord coord2);
Coord operator-(Coord coord1,Coord coord2);
Coord operator*(float value,Coord coord2);
Coord operator*(Coord coord1,float value);
Coord operator/(Coord coord1,float value);
Coord operator/(float value,Coord coord2);

int sign(int val);
int sign(float val);

template<typename T=char,int bufSize=256> class RingBuffer{
private:
    T data[bufSize+2];
    int readPointer;
    int writePointer;
public:
    RingBuffer(){
        readPointer=0;
        writePointer=0;
    };
    int write(T value){
        int next=(writePointer + 1) % (bufSize+1);
        if(readPointer==next)return 1;
        data[writePointer]=value;
        writePointer = next;
        return 0;
    }
    T read(){
        if(readPointer==writePointer) return 0;
        T buf=data[readPointer];
        readPointer = (readPointer + 1) % (bufSize+1);
        return buf;
    }
    void clear(){ readPointer = 0; writePointer = 0; };
    int isEmpty(){return  (readPointer==writePointer)?(1):(0);}
    int isFull(){return (readPointer==((writePointer + 1) % (bufSize+1)))?(1):(0);}
    int size(){return bufSize;};
};
