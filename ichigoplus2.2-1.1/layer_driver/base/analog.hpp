#ifndef ANALOG_HPP_INCLUDED
#define ANALOG_HPP_INCLUDED

class Analog{
//***************override******************
protected:
    virtual int _setupAnalogIn()=0;
    virtual float _analogRead()=0;
public:
    virtual int analogResolution()=0;
    virtual float analogVoltage()=0;
//***************override******************
    virtual int setupAnalogIn(){return _setupAnalogIn();}
    virtual float analogRead(){return _analogRead();};

    operator float() {
        return analogRead();
    }
};


#endif // ANALOG_HPP_INCLUDED
