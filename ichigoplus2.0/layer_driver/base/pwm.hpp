#ifndef PWM_H_INCLUDED
#define PWM_H_INCLUDED

class Pwm{
protected:
//***************override******************
    virtual int _setupPwmOut(float frequency,float duty)=0;
    virtual void _pwmWrite(float duty)=0;
    virtual float _pwmWrite()=0;
//***************override******************
public:
    virtual int setupPwmOut(float frequency,float duty){return _setupPwmOut(frequency,duty);};
    virtual void pwmWrite(float duty){_pwmWrite(duty);};
    virtual float pwmWrite(){return _pwmWrite();};
};

#endif // PWM_H_INCLUDED
