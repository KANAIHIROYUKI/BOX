#ifndef ENCODER_H_INCLUDED
#define ENCODER_H_INCLUDED

class Encoder{
protected:
	int cprData;
	bool revFlag;
	float mltData;
public:
	Encoder(){cprData=1;revFlag=false;mltData=1;};

	//depends on hardware
	virtual int setup()=0;
	virtual int count()=0;

	void rev(bool value);
	void mlt(float mltArg);
	void cpr(int cprArg);
	int cpr();
	float radian();
	float value();
};

#endif // ENCODER_H_INCLUDED
