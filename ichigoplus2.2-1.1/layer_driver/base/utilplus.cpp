#include "util.hpp"
#define _USE_MATH_DEFINES //Å@ÉŒÇóòópâ¬î\Ç…Ç∑ÇÈ
#include <math.h>

Coord::Coord(Coord const &coord){
	cartesianXData=coord.cartesianXData;
	cartesianYData=coord.cartesianYData;
	cartesianZData=coord.cartesianZData;
	eularXData=coord.eularXData;
	eularYData=coord.eularYData;
	eularZData=coord.eularZData;
}

Coord::Coord(float cx,float cy,float cz){
	cartesianXData=cx;
	cartesianYData=cy;
	cartesianZData=cz;
	eularXData=0;
	eularYData=0;
	eularZData=0;
}

Coord::Coord(float cx,float cy,float cz,float ex,float ey,float ez){
	cartesianXData=cx;
	cartesianYData=cy;
	cartesianZData=cz;
	eularXData=ex;
	eularYData=ey;
	eularZData=ez;
}

void Coord::cartesian(float cx,float cy,float cz){
	cartesianXData=cx;
	cartesianYData=cy;
	cartesianZData=cz;
}

void Coord::cartesianX(float x){
	cartesianXData=x;
}

void Coord::cartesianY(float y){
	cartesianYData=y;
}


void Coord::cartesianZ(float z){
	cartesianZData=z;
}

float Coord::cartesianX(){
	return cartesianXData;
}

float Coord::cartesianY(){
	return cartesianYData;
}

float Coord::cartesianZ(){
	return cartesianZData;
}

void Coord::cartesianXAdd(float x){
	cartesianX(cartesianX()+x);
}

void Coord::cartesianYAdd(float y){
	cartesianY(cartesianY()+y);
}

void Coord::cartesianZAdd(float z){
	cartesianZ(cartesianZ()+z);
}

void Coord::polar(float radius,float theta,float phi){
	float xy=0;
	if(theta==0){
		cartesianXData=0;
		cartesianYData=0;
		cartesianZData=radius;
		return;
	}
	cartesianZData=radius*cosf(theta);
	xy=radius*sinf(theta);
	cartesianXData=xy*cosf(phi);
	cartesianYData=xy*sinf(phi);
}

float Coord::polarRadius(){
	float radius;
	//    float theta;
	//    float phi;
	radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
	//    phi=atan2f(cartesianYData,cartesianXData);
	//    if(radius==0) theta=0;
	//    else theta=acosf(cartesianZData/radius);
	return radius;
}

float Coord::polarTheta(){
	float radius;
	float theta;
	//float phi;
	radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
	//phi=atan2f(cartesianYData,cartesianXData);
	if(radius==0) theta=0;
	else theta=acosf(cartesianZData/radius);
	return theta;
}

float Coord::polarPhi(){
//	float radius;
//	float theta;
	float phi;
	//    float radius;
	//float theta;
	//radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
	phi=atan2f(cartesianYData,cartesianXData);
	//if(radius==0) theta=0;
	//else theta=acosf(cartesianZData/radius);
	return phi;
}

void Coord::polarRadius(float radiusData){
	float radius;
	float theta;
	float phi;
	radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
	phi=atan2f(cartesianYData,cartesianXData);
	if(radius==0) theta=0;
	else theta=acosf(cartesianZData/radius);
	radius=radiusData;
	Coord::polar(radius,theta,phi);
}

void Coord::polarTheta(float thetaData){
	float radius;
	float theta;
	float phi;
	radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
	phi=atan2f(cartesianYData,cartesianXData);
	if(radius==0) theta=0;
	else theta=acosf(cartesianZData/radius);
	theta=thetaData;
	Coord::polar(radius,theta,phi);
}

void Coord::polarPhi(float phiData){
	float radius;
	float theta;
	float phi;
	radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
	phi=atan2f(cartesianYData,cartesianXData);
	if(radius==0) theta=0;
	else theta=acosf(cartesianZData/radius);
	phi=phiData;
	Coord::polar(radius,theta,phi);
}

void Coord::polarRadiusAdd(float radius){
	polarRadius(polarRadius()+radius);
}

void Coord::polarThetaAdd(float theta){
	polarTheta(polarTheta()+theta);
}

void Coord::polarPhiAdd(float phiArg){
	polarPhi(polarPhi()+phiArg);
}

void Coord::eular(float x,float y,float z){
	eularXData=x;
	eularYData=y;
	eularZData=z;
}

float Coord::eularX(){
	return eularXData;
}

float Coord::eularY(){
	return eularYData;
}

float Coord::eularZ(){
	return eularZData;
}

void Coord::eularX(float x){
	eularXData=x;
}

void Coord::eularY(float y){
	eularYData=y;
}

void Coord::eularZ(float z){
	eularZData=z;
}

void Coord::eularZAdd(float z){
	eularZData+=z;
}

float Coord::x(){
	return Coord::cartesianX();
}

float Coord::y(){
	return Coord::cartesianY();
}

float Coord::z(){
	return Coord::cartesianZ();
}

float Coord::radius(){
	return Coord::polarRadius();
}

float Coord::theta(){
	return Coord::polarTheta();
}

float Coord::phi(){
	return Coord::polarPhi();
}

float Coord::ex(){
	return Coord::eularX();
}

float Coord::ey(){
	return Coord::eularY();
}

float Coord::ez(){
	return Coord::eularZ();
}


float Coord::Reradius(float deg){
	float rad = deg * M_PI / 180.0;
	return rad;
}

void Coord::revolutionXY(float angle){
	float cartesianXSub = cartesianXData;
	float cartesianYSub = cartesianYData;
	eularZData = eularZData + angle;
	//	int a = cos(angle);
	//	std::cout << std::endl << " angle : " << a << std::endl;
	cartesianXData = cartesianXSub * cos(angle) + (-1 * cartesianYSub * sin(angle));
	cartesianYData = cartesianXSub * sin(angle) + cartesianYSub * cos(angle);
}

void Coord::revolutionYZ(float angle){
	float cartesianYSub = cartesianYData;
	float cartesianZSub = cartesianZData;
	eularXData = eularXData + angle;
	cartesianYData = cartesianYSub * cos(angle) - cartesianZSub * sin(angle);
	cartesianZData = cartesianYSub * sin(angle) + cartesianZSub * cos(angle);
}

void Coord::revolutionZX(float angle){
	float cartesianXSub = cartesianXData;
	float cartesianZSub = cartesianZData;
	eularYData = eularYData + angle;
	cartesianZData = cartesianZSub * cos(angle) - cartesianXSub * sin(angle);
	cartesianXData = cartesianZSub * sin(angle) + cartesianXSub * cos(angle);
}

void Coord::revolutionX(float angle){
	float cartesianXSub = cartesianXData;
	float cartesianYSub = cartesianYData;
	float cartesianZSub = cartesianZData;
	eularXData = eularXData + angle;
	cartesianXData = cartesianXSub;
	cartesianYData = cartesianYSub * cos(angle) - cartesianZSub * sin(angle);
	cartesianZData = cartesianYSub * sin(angle) + cartesianZSub * cos(angle);
}

void Coord::revolutionY(float angle){
	float cartesianXSub = cartesianXData;
	float cartesianYSub = cartesianYData;
	float cartesianZSub = cartesianZData;
	eularYData = eularYData + angle;
	cartesianXData = cartesianXSub * cos(angle) + cartesianZSub * sin(angle);
	cartesianYData = cartesianYSub;
	cartesianZData = (-1 * cartesianYSub * sin(angle)) + cartesianZSub * cos(angle);
}

void Coord::revolutionZ(float angle){
	float cartesianXSub = cartesianXData;
	float cartesianYSub = cartesianYData;
	float cartesianZSub = cartesianZData;
	eularZData = eularZData + angle;
	cartesianXData = cartesianXSub * cos(angle) - cartesianYSub * sin(angle);
	cartesianYData = cartesianXSub * sin(angle) + cartesianYSub * cos(angle);
	cartesianZData = cartesianZSub;
}

void Coord::revolutionEuler(float a, float b, float c){
	float cartesianXSub = cartesianXData;
	float cartesianYSub = cartesianYData;
	float cartesianZSub = cartesianZData;
	cartesianXData = cartesianXSub * (cos(a) * cos(c) - sin(a) * cos(b) * sin(c)) + cartesianYSub * (cos(a) * sin(c) + sin(a) * cos(b) * cos(c)) + cartesianZSub * (sin(a) * sin(b));
	cartesianYData = cartesianXSub * (-1 * sin(a) * cos(c) - cos(a) * cos(b) * sin(c)) + cartesianYData * (-1 * sin(a) * sin(c) + cos(a) * cos(b) * cos(c)) + cartesianZData * (cos(a) * sin(b));
	cartesianZData = cartesianXSub * (sin(b) * sin(c)) + cartesianYSub * (-1 * sin(b) * cos(c)) + cartesianZSub * cos(b);

}

void Coord::revolutionEulerinverse(float a, float b, float c){
	float cartesianXSub = cartesianXData;
	float cartesianYSub = cartesianYData;
	float cartesianZSub = cartesianZData;
	cartesianXData = cartesianXSub * (cos(a) * cos(c) - sin(a) * cos(b) * sin(c)) + cartesianYSub * (-1 * sin(a) * cos(c) + -1 * cos(a) * cos(b) * sin(c)) + cartesianZSub * (sin(b) * sin(c));
	cartesianYData = cartesianXSub * (cos(a) * sin(c) + sin(a) * cos(b) * cos(c)) + cartesianYSub * (-1 * sin(a) * sin(c) + cos(a) * cos(b) * cos(c)) + cartesianZSub * (-1 * sin(b) * cos(c));
	cartesianZData = cartesianXSub * (sin(a) * sin(b)) + cartesianYSub * (cos(a) * sin(b)) + cartesianZSub * cos(b);

}

Coord operator+(Coord coord1,Coord coord2){
	return Coord(coord1.x()+coord2.x(),coord1.y()+coord2.y(),coord1.z()+coord2.z(),coord1.ex()+coord2.ex(),coord1.ey()+coord2.ey(),coord1.ez()+coord2.ez());
}

Coord operator-(Coord coord1,Coord coord2){
	return Coord(coord1.x()-coord2.x(),coord1.y()-coord2.y(),coord1.z()-coord2.z(),coord1.ex()-coord2.ex(),coord1.ey()-coord2.ey(),coord1.ez()-coord2.ez());
}

Coord operator*(float value,Coord coord2){
	return Coord(coord2.x()*value,coord2.y()*value,coord2.z()*value,coord2.ex()*value,coord2.ey()*value,coord2.ez()*value);
}

Coord operator*(Coord coord1,float value){
	return Coord(coord1.x()*value,coord1.y()*value,coord1.z()*value,coord1.ex()*value,coord1.ey()*value,coord1.ez()*value);
}

Coord operator/(Coord coord1,float value){
	return Coord(coord1.x()/value,coord1.y()/value,coord1.z()/value,coord1.ex()/value,coord1.ey()/value,coord1.ez()/value);
}

Coord operator/(float value,Coord coord2){
	return Coord(value/coord2.x(),value/coord2.y(),value/coord2.z(),value/coord2.ex(),value/coord2.ey(),value/coord2.ez());
}


int sign(int val)
{
	if(val>=0) return 1;
	else return -1;
}

int sign(float val)
{
	if(val>=0.0) return 1;
	else return -1;
}
