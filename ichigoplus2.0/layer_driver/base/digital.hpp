#ifndef DIGITAL_H_INCLUDED
#define DIGITAL_H_INCLUDED


#define DIGITAL_IN 0
#define DIGITAL_OUT 1
#define DIGITAL_IN_PLUP 2
#define DIGITAL_IN_PLDN 3

#define DIGITAL_HIGH    1
#define DIGITAL_LOW     0
#define DIGITAL_TOGGLE -1
#include <stdio.h>

class Digital{
protected:
//***************override******************
    virtual int _digitalWrite()=0;
    virtual void _digitalWrite(int)=0;
    virtual int _digitalRead()=0;
    virtual int _setupDigitalIn()=0;
	virtual int _setupDigitalOut()=0;
	virtual int _setupDigitalInPullUp()=0;
	virtual int _setupDigitalInPullDown()=0;
//***************override******************

    enum{
        DIGITAL_MODE_NULL,
        DIGITAL_MODE_IN,
        DIGITAL_MODE_OUT,
        DIGITAL_MODE_PULLUP,
        DIGITAL_MODE_PULLDOWN,
    };
    int _digitalMode;
public:
	virtual int setupDigitalIn(){
	    int i=_setupDigitalIn();
	    if(i)return i;
	    _digitalMode=DIGITAL_MODE_IN;
	    return 0;
	};
	virtual int setupDigitalOut(){
        int i=_setupDigitalOut();
	    if(i)return i;
	    _digitalMode=DIGITAL_MODE_OUT;
	    return 0;
    };
	virtual int setupDigitalInPullUp(){
	    int i=_setupDigitalInPullUp();
	    if(i)return i;
	    _digitalMode=DIGITAL_MODE_PULLUP;
	    return 0;
    };
	virtual int setupDigitalInPullDown(){
	    int i=_setupDigitalInPullDown();
	    if(i)return i;
	    _digitalMode=DIGITAL_MODE_PULLDOWN;
	    return 0;
	};

	virtual void digitalWrite(int value){
	    if(_digitalMode==DIGITAL_MODE_NULL) return;
	    if(_digitalMode==DIGITAL_MODE_OUT)_digitalWrite(value);
	    else{
            if(value==1)setupDigitalInPullUp();
            else if(value==0)setupDigitalInPullDown();
            else setupDigitalIn();
	    }
	};
	virtual int digitalRead(){
	    if(_digitalMode==DIGITAL_MODE_NULL) return 0;
	    if(_digitalMode==DIGITAL_MODE_IN)return _digitalRead();
	    else return _digitalWrite();
	};
	virtual void digitalHigh(){digitalWrite(1);};
	virtual void digitalLow(){digitalWrite(0);};
	virtual void digitalToggle(){digitalWrite(!digitalRead());};

    Digital& operator= (int value) {
        digitalWrite(value);
        return *this;
    }

    Digital& operator= (Digital& dig) {
        digitalWrite(dig.digitalRead());
        return *this;
    }

    operator int() {
        return digitalRead();
    }

};




#endif // DIGITAL_H_INCLUDED
