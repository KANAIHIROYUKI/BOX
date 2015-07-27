#include "can.hpp"
#include <stdio.h>
int CanInterface::canInterfaceSetup(Can *can){
	this->can=can;
	return 0;
}

int CanInterface::canSetup(){
	return can->setup();
}

int CanInterface::canWrite(int idArg,int numberArg,unsigned char dataArg[8]){
	return can->write(idArg,numberArg,dataArg);
}

int CanInterface::canSetId(int id){
	return can->setId(id);
};

int CanInterface::canSetIdAll(){
	return can->setIdAll();
}
