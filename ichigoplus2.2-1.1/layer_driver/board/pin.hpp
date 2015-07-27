#ifndef PIN_H_INCLUDED
#define PIN_H_INCLUDED

#include "serial.hpp"
#include "digital.hpp"
#include "Analog.hpp"
#include "encoder.hpp"
#include "pwm.hpp"

/**********IO****************************/
class Led0:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class Led1:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class Led2:public Digital,public Pwm{
private:
	int mode;
	enum{
		MODE_NULL,
		MODE_PWM,
		MODE_DIGITAL
	};
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
	int _setupPwmOut(float frequency, float duty);
	void _pwmWrite(float duty);
	float _pwmWrite();
public:

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

class Led3:public Digital,public Pwm{
private:
	int mode;
	enum{
		MODE_NULL,
		MODE_PWM,
		MODE_DIGITAL
	};
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
	int _setupPwmOut(float frequency, float duty);
	void _pwmWrite(float duty);
	float _pwmWrite();
public:

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

class Sw0:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class Sw1:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class Sw2:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class Sw3:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class Buzzer:public Digital,public Pwm{
private:
	int mode;
	enum{
		MODE_NULL,
		MODE_PWM,
		MODE_DIGITAL
	};
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
	int _setupPwmOut(float frequency, float duty);
	void _pwmWrite(float duty);
	float _pwmWrite();
public:

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

class CW0:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CCW0:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CW1:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CCW1:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CW2:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CCW2:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CW3:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CCW3:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CW4:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CCW4:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CW5:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

class CCW5:public Digital{
private:
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();
public:

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

/***********AD**************************/
class A0:public Digital,public Analog{
private:
	enum{
		MODE_NULL,
		MODE_ANALOG,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupAnalogIn();
	float _analogRead();
public:
	A0(){mode=0;};

	int analogResolution(){return 4095;};
	float analogVoltage(){return 3.3;};
	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		return analogRead();
	}
};

class A1:public Digital,public Analog{
private:
	enum{
		MODE_NULL,
		MODE_ANALOG,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupAnalogIn();
	float _analogRead();
public:
	A1(){mode=0;};

	int analogResolution(){return 4095;};
	float analogVoltage(){return 3.3;};

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		return analogRead();
	}
};

class A2:public Digital,public Analog{
private:
	enum{
		MODE_NULL,
		MODE_ANALOG,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupAnalogIn();
	float _analogRead();
public:
	A2(){mode=0;};

	int analogResolution(){return 4095;};
	float analogVoltage(){return 3.3;};

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		return analogRead();
	}
};

class A3:public Digital,public Analog{
private:
	enum{
		MODE_NULL,
		MODE_ANALOG,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupAnalogIn();
	float _analogRead();
public:
	A3(){mode=0;};

	int analogResolution(){return 4095;};
	float analogVoltage(){return 3.3;};

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		return analogRead();
	}
};

class A4:public Digital,public Analog{
private:
	enum{
		MODE_NULL,
		MODE_ANALOG,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupAnalogIn();
	float _analogRead();
public:
	A4(){mode=0;};

	int analogResolution(){return 4095;};
	float analogVoltage(){return 3.3;};

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		return analogRead();
	}
};


/********pwm****************/
class Pwm0:public Digital,public Pwm{
private:
	enum{
		MODE_NULL,
		MODE_PWM,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupPwmOut(float frequency,float duty);
	void _pwmWrite(float duty);
	float _pwmWrite();
public:
	Pwm0(){mode=0;}

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	Pwm& operator= (float value) {
		pwmWrite(value);
		return *this;
	}

	Pwm& operator= (Pwm& pwm) {
		pwm.pwmWrite(pwm.pwmWrite());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		if (mode==MODE_PWM)return pwmWrite();
		else return 0.0;
	}
};

class Pwm1:public Digital,public Pwm{
private:
	enum{
		MODE_NULL,
		MODE_PWM,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupPwmOut(float frequency,float duty);
	void _pwmWrite(float duty);
	float _pwmWrite();
public:
	Pwm1(){mode=0;}

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	Pwm& operator= (float value) {
		pwmWrite(value);
		return *this;
	}

	Pwm& operator= (Pwm& pwm) {
		pwm.pwmWrite();
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		if (mode==MODE_PWM)return pwmWrite();
		else return 0.0;
	}
};

class Pwm2:public Digital,public Pwm{
private:
	enum{
		MODE_NULL,
		MODE_PWM,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupPwmOut(float frequency,float duty);
	void _pwmWrite(float duty);
	float _pwmWrite();
public:
	Pwm2(){mode=0;}

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	Pwm& operator= (float value) {
		pwmWrite(value);
		return *this;
	}

	Pwm& operator= (Pwm& pwm) {
		pwm.pwmWrite(pwm.pwmWrite());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		if (mode==MODE_PWM)return pwmWrite();
		else return 0.0;
	}
};

class Pwm3:public Digital,public Pwm{
private:
	enum{
		MODE_NULL,
		MODE_PWM,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupPwmOut(float frequency,float duty);
	void _pwmWrite(float duty);
	float _pwmWrite();
public:
	Pwm3(){mode=0;}

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	Pwm& operator= (float value) {
		pwmWrite(value);
		return *this;
	}

	Pwm& operator= (Pwm& pwm) {
		pwm.pwmWrite(pwm.pwmWrite());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		if (mode==MODE_PWM)return pwmWrite();
		else return 0.0;
	}
};

class Pwm4:public Digital,public Pwm{
private:
	enum{
		MODE_NULL,
		MODE_PWM,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupPwmOut(float frequency,float duty);
	void _pwmWrite(float duty);
	float _pwmWrite();
public:
	Pwm4(){mode=0;}

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	Pwm& operator= (float value) {
		pwmWrite(value);
		return *this;
	}

	Pwm& operator= (Pwm& pwm) {
		pwm.pwmWrite(pwm.pwmWrite());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		if (mode==MODE_PWM)return pwmWrite();
		else return 0.0;
	}
};

class Pwm5:public Digital,public Pwm{
private:
	enum{
		MODE_NULL,
		MODE_PWM,
		MODE_DIGITAL
	};
	int mode;
	int _digitalWrite();
	void _digitalWrite(int value);
	int _digitalRead();
	int _setupDigitalIn();
	int _setupDigitalOut();
	int _setupDigitalInPullUp();
	int _setupDigitalInPullDown();

	int _setupPwmOut(float frequency,float duty);
	void _pwmWrite(float duty);
	float _pwmWrite();
public:
	Pwm5(){mode=0;}

	Digital& operator= (int value) {
		digitalWrite(value);
		return *this;
	}

	Digital& operator= (Digital& dig) {
		digitalWrite(dig.digitalRead());
		return *this;
	}

	Pwm& operator= (float value) {
		pwmWrite(value);
		return *this;
	}

	Pwm& operator= (Pwm& pwm) {
		pwm.pwmWrite(pwm.pwmWrite());
		return *this;
	}

	operator bool(){
		return digitalRead();
	}

	operator int() {
		return digitalRead();
	}

	operator float() {
		if (mode==MODE_PWM)return pwmWrite();
		else return 0.0;
	}
};

/********encoder*************/
class Enc0:public Encoder{
public:
	int setup();
	int count();

	operator int() {
		return count();
	}
};

class Enc1:public Encoder{
public:
	int setup();
	int count();
	operator int() {
		return count();
	}
};

class Enc2:public Encoder{
public:
	int setup();
	int count();
	operator int() {
		return count();
	}
};

/*********Serial*************/
class Serial0:public Serial{
private:
	static SerialInterface *interface;
	int _setup(int baudrate, SerialInterface &interfaceArg, int parity, int wordLength);
	friend int std_char_out_setup();
	friend void serial0_interrupt();
public:
	static void transmit(char);
	void charWrite(char value);
};

class Serial1:public Serial{
private:
	static SerialInterface *interface;
	int _setup(int baudrate, SerialInterface &interfaceArg, int parity, int wordLength);
	friend void serial1_interrupt();
public:
	static void transmit(char);
	void charWrite(char value);
};

#include "can.hpp"
#define NUMBER_OF_CAN_INTERFACE 30
class Can0:public Can{
public:
	Can0();
	int setup();
	int setupLoopBack();
	int addInterface(CanInterface &interfaceArg);
	int setId(int id);
	int setIdAll();
	int write(int id,int number,unsigned char data[8]);
private:
	int filter_number;
	short filter[14*4];
	bool setuped;
	static CanInterface *canInterface[30];
	static int canInterfaceCursor;
	static int read(int id,int number,unsigned char data[8]);
	friend void Can0_Interrupt();
};
#endif // PIN_H_INCLUDED
