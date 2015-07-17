#ifndef CONSOLEPLUS_H
#define CONSOLEPLUS_H
/*******************************************
 * consoleplus ver1.1 2015/1/14
 * This is a program made for mathematically calculation.
 * This is the first kitrp's microcomputer program that uses class!
 * [Dependency]
 *
 * [Note]
 *
 * [Change history]
 * ver1.2 2015/ 4/ 4 multiple console available (constructing now...)
 * ver1.1 2015/ 3/10 receive improvement.
 * ver1.0 2015/ 3/ 3 The first version.
 *
 ******************************************/
#define NUMBER_OF_COMMANDS 100
#define LENGTH_OF_COMMAND 20
#define NUL '\0'
#define SPACE ' '
#define LF '\r'
#define SENTENCE_LIMIT 64

#include "serial.hpp"
#include <stdarg.h>

class ConsoleCommand{
private:
	SerialInterface *masterConsole;
protected:
	char comsent[LENGTH_OF_COMMAND];
public:
	ConsoleCommand(){};
	ConsoleCommand(const char *command);
	void commandSetup(SerialInterface *console){masterConsole=console;};
	virtual int commandCompare(char *command)=0;
	virtual int commandRequest(int argc,char argv[NUMBER_OF_COMMANDS][LENGTH_OF_COMMAND])=0;//1:sustain 0:unsustain
	virtual int commandCycle(){return 0;};
	virtual void consoleCharReceive(char data){};
	virtual void consolePrintf(const char *pFormat, ...);
};

class Console:public SerialInterface{
private:
	ConsoleCommand* command[30];
	int commandCursor;
	char readingBuffer[256];
	int  readingCursor;
	char readData[256];
	int availableFlag;
	char LFData;
	int commandRunning;
	bool echoSetting;

	int relativeCursorX;
	int relativeCursorY;
	char prompt[10];
public:
	Console();
	int setup();
	int addCommand(ConsoleCommand &commandClass);
	void serialCharReceive(char data);
	void linefeed(char LFArg);
	int  available();
	void getString(char *output);
	void cycle();
	void echo(bool value);
	void setPrompt(char *value);

	void clear();
	void startRelative();
	void relative(int x,int y);
	void absolute(int x,int y);
};

class CommandReady : public ConsoleCommand{//reference
private:
	int endFlag;
public:
	CommandReady(const char *command):ConsoleCommand(command){endFlag=0;};
	int commandRequest(int argc,char argv[NUMBER_OF_COMMANDS][LENGTH_OF_COMMAND]);
	int commandCycle();
	int commandCompare(char *command);
	void consoleCharReceive(char data);
};

#endif//CONSOLEPLUS_H
