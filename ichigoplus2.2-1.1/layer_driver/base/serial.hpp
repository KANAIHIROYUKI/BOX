#ifndef USARTPLUS_HPP
#define USARTPLUS_HPP
/*******************************************
 * usartplus ver2.0 2015/3/10
 * Classes of usart for stm32f4 discovery 2-layer board.
 * Serial Interface declaration.
 *
 * [Dependency]
 * usart
 * if(STM32F4DISCOVERY_2LB_3_6) encoderplus
 *
 * [Note]
 *
 * [Change history]
 * ver2.1 2015/ 3/12 At STM32F4D_2LB ver.3.6 enc2 can use as serial communication port.
 * ver2.0 2015/ 3/10 Rename usart to serial.Multiple boards available.
 * ver1.0 2015/ 3/ 3 The first version.
 ******************************************/
#ifdef __cplusplus

#include "util.hpp"
#include <stdarg.h>
#define DEFAULT_LINEFEED '\r'
#define SERIAL_BUFFER_LENGTH 256

class SerialInterface{
private:
    int stringWriterFlag;
	void (*serialWriter)(char);
	void (*stringWriter)(char*);
protected:
    RingBuffer<char,SERIAL_BUFFER_LENGTH> rxBuffer;
public:
	int serialInterfaceSetup(void (*writerArg)(char)){serialWriter=writerArg;stringWriterFlag=0;return 0;};
	int serialInterfaceSetup(void (*writerArg)(char),void (*stringWriterArg)(char*)){serialWriter=writerArg;stringWriter=stringWriterArg;stringWriterFlag=1;return 0;};
	void serialCharWrite(char value){serialWriter(value);};
	void serialStringWrite(char *value){if(stringWriterFlag==0)for(int i=0;value[i]!='\0';i++) serialCharWrite(value[i]);else stringWriter(value);};
	void serialPrintf(const char *pFormat, ...);
	virtual void serialReadChar(char value);
	virtual void serialReadString(char *data){};
};

class Serial{
private:
    class Receiver:public SerialInterface{
    public:
//        RingBuffer<char,SERIAL_BUFFER_LENGTH> rxBuffer;
        int stringAvailavleData;
        Serial *master;
        Receiver(Serial *serial){
            master=serial;
            stringAvailavleData=0;
        };
        void serialReadChar(char value){
        	if(!rxBuffer.write(value)){
        		if(value==DEFAULT_LINEFEED) stringAvailavleData++;
        	}else{
        	    if(stringAvailavleData==0&&value==DEFAULT_LINEFEED)stringAvailavleData=1;
        	}
        };
        int charAvailable(){
        	return (!rxBuffer.isEmpty());
        };
        int stringAvailable(){
        	return stringAvailavleData;
        };
        char readChar(){
        	return rxBuffer.read();
        };
        int readString(char *value){
            if(rxBuffer.isEmpty()){
                value[0]=0;
                stringAvailavleData=0;
                return 0;
            }
        	for(int i=0;i<256;i++){
        		value[i]=rxBuffer.read();
        		if(value[i]==DEFAULT_LINEFEED||i>=SERIAL_BUFFER_LENGTH-1){
        		    value[i]=0;
                    if(rxBuffer.isEmpty()) stringAvailavleData=0;
                    else stringAvailavleData--;
                    return i;
        		}
        		if(rxBuffer.isEmpty()){
                    value[i]=0;
                    stringAvailavleData=0;
                    return i;
                }
        	}
        	return 0;
        };
    };
    Receiver *defaultInterface;
    int _serial_mode;
    enum{
        SERIAL_MODE_NULL,
        SERIAL_MODE_LOCAL,
        SERIAL_MODE_INTERFACE
    };
protected:
    virtual int _setup(int baudrate,SerialInterface &interfaceArg,int parity,int wordLength)=0;
public:
    enum{
    	SERIAL_PARITY_NONE,
    	SERIAL_PARITY_ODD,
    	SERIAL_PARITY_EVEN
    };
	virtual void charWrite(char value)=0;
	virtual int stringAvailable(){
		if(_serial_mode==SERIAL_MODE_LOCAL) return defaultInterface->stringAvailable();
		else return 0;
	};
	virtual int readString(char *value){
		value[0]=0;
		if(_serial_mode==SERIAL_MODE_LOCAL)return defaultInterface->readString(value);
		else return 0;
	};
	virtual int charAvailable(){
		if(_serial_mode==SERIAL_MODE_LOCAL)return defaultInterface->charAvailable();
		else return 0;
	};
	virtual char readChar(){
		if(_serial_mode==SERIAL_MODE_LOCAL)return defaultInterface->readChar();
		else return 0;
	};

	virtual void stringWrite(char *value){for(int i=0;value[i]!='\0';i++) charWrite(value[i]);};
	virtual void printf(const char *pFormat, ...);
	virtual int setup(int baudrate,SerialInterface &interfaceArg,int parity=SERIAL_PARITY_NONE,int wordLength=8){_serial_mode=SERIAL_MODE_INTERFACE;return _setup(baudrate,interfaceArg,wordLength,parity);};
	virtual int setup(int baudrate,int parity=SERIAL_PARITY_NONE,int wordLength=8){_serial_mode=SERIAL_MODE_LOCAL;defaultInterface=new Receiver(this);return _setup(baudrate,*defaultInterface,wordLength,parity);};
};
#endif//__cplusplus

#ifdef __cplusplus
extern "C" void std_char_out(char data);
extern "C" int std_char_out_setup();
#else
void std_char_out(char data);
int std_char_out_setup();
#endif
#endif//USARTPLUS_HPP

