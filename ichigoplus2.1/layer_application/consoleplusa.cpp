#include "consoleplus.hpp"
#include <stdio.h>
#include <string.h>
static int setcommand(char com[][LENGTH_OF_COMMAND],char *sent);

void ConsoleCommand::consolePrintf(const char *pFormat, ...){
	char buffer[256];
	va_list ap;
	va_start(ap, pFormat);
	vsprintf(buffer,pFormat,ap);
	va_end(ap);
	masterConsole->serialStringTransmit(buffer);
}

Console::Console(){
	LFData='\r';
	readingCursor=0;
	availableFlag=0;
	commandCursor=0;
	readingBuffer[0]='\0';
	readData[0]='\0';
	commandRunning=-1;
	echoSetting=true;

	relativeCursorX=0;
	relativeCursorY=0;

	strcpy(prompt,">>>");
}

void Console::echo(bool value){
	echoSetting=value;
}

void Console::setPrompt(char *value){
	strcpy(prompt,value);
}

void Console::serialCharReceive(char data){
	if(commandRunning!=-1){
		command[commandRunning]->consoleCharReceive(data);
		return;
	}
	if(data=='\b'){
		int length=strlen(readingBuffer);
		if(length>0){
//			readingCursor--;
//			readingBuffer[readingCursor]=NUL;
			readingBuffer[strlen(readingBuffer)-1]=NUL;
			if(echoSetting){putchar('\b');putchar(' ');putchar('\b');}
		}
	}else{
		char cbuf[2]={0};
		cbuf[0]=data;cbuf[1]=NUL;
		strncat(readingBuffer,cbuf,256);
		if(echoSetting) putchar(data);
	}
	if (strchr(readingBuffer, LF) != NULL) availableFlag = 1;
	else availableFlag = 0;
}

int Console::available(){
	return availableFlag;
}

void Console::getString(char *output){
	int i=0;
	for(i=0;readingBuffer[i]!=LF;i++) output[i]=readingBuffer[i];
	output[i++]=NUL;
	char buffer[256];
	int j=0;
	for(j=0;readingBuffer[i+j-1]!=NUL;j++)buffer[j]=readingBuffer[i+j];
	strcpy(readingBuffer,buffer);
	if (strchr(readingBuffer, LF) != NULL) availableFlag = 1;
	else availableFlag = 0;
}

int Console::addCommand(ConsoleCommand &commandClass){
	if(commandCursor>30) return 1;
	command[commandCursor++]=&commandClass;
	commandClass.commandSetup(this);
	return 0;
}

void Console::linefeed(char LFArg){
	LFData=LFArg;
}

int Console::setup(){
	printf(prompt);
	return 0;
}

void Console::cycle(){

	char str[256];

	while(available()){
		getString(str);
		//		int i=0;
		//		int returnbuf=0;
		char charCommand[NUMBER_OF_COMMANDS][LENGTH_OF_COMMAND];
		int argc=0;
		int exeFlag=0;
		argc=setcommand(charCommand,str);
		for(int i=0;i<commandCursor;i++){
			if( command[i]->commandCompare(charCommand[0]) ==1 ){
				if(command[i]->commandRequest(argc,charCommand)){
					commandRunning=i;
				}else{
					printf(prompt);
				}
				exeFlag=1;
				break;
			}
		}
		if(exeFlag==0){
			if(charCommand[0][0]!='\0')printf("%s: command not found.\r",charCommand[0]);
			printf(prompt);
		}
	}
	if(commandRunning!=-1){
		if(command[commandRunning]->commandCycle()==0){
			commandRunning=-1;
			printf(prompt);
		}
	}
}

void Console::startRelative(){
	clear();
	printf("\033[1;1H");
	relativeCursorX=0;
	relativeCursorY=0;
	printf("\0337");
}

void Console::relative(int x,int y){
	printf("\0338");
	if(x-relativeCursorX>0) printf("\033[%dC",x-relativeCursorX);
	else if(x-relativeCursorX<0) printf("\033[%dD",relativeCursorX-x);
	if(y-relativeCursorY>0) printf("\033[%dB",y-relativeCursorY);
	else if(y-relativeCursorY<0) printf("\033[%dA",relativeCursorY-y);
	relativeCursorX=x;
	relativeCursorY=y;
	printf("\0337");
}

void Console::clear(){
	printf("\033[2J");
}

void Console::absolute(int x,int y){
	printf("\033[%d;%dH",y,x);
}

ConsoleCommand::ConsoleCommand(const char *command){
	strcpy(comsent,command);
}

int CommandReady::commandRequest(int argc,char argv[NUMBER_OF_COMMANDS][LENGTH_OF_COMMAND]){
	if(argc>1&&!strcmp(argv[1],"i")){
		endFlag=0;
		return 1;
	}
	printf("ready\r");
	endFlag=1;
	return 0;
}

void CommandReady::consoleCharReceive(char data){
	if(data=='\r') endFlag=1;
}

int CommandReady::commandCycle(){
	if(endFlag==1) return 0;
	printf("ready\r");
	return 1;
}

int CommandReady::commandCompare(char *command){
	return (!strcmp(command,"ready"));
}


static int setcommand(char com[][LENGTH_OF_COMMAND],char *sent)//–ß‚è’l‚Í’PŒê‚Ì”
{
	int senti = 0;
	int comi = 0;
	int count = 0;

	int nulcount=0;

	while(sent[senti] == SPACE)
	{
		if(senti > SENTENCE_LIMIT) return -1;
		senti++;
	}
	while(sent[senti] != LF&&sent[senti] != NUL)
	{
		comi = 0;
		while(sent[senti] != LF && sent[senti] != NUL && sent[senti] != SPACE)
		{
			if(senti > SENTENCE_LIMIT) return -1;
			com[count][comi] = sent[senti];
			senti++;
			comi++;
		}
		com[count][comi] = NUL;
		count++;
		while(sent[senti] == SPACE)
		{
			if(senti > SENTENCE_LIMIT) return -1;
			senti++;
		}
	}
	for(nulcount=count;nulcount<NUMBER_OF_COMMANDS;nulcount++)
	{
		com[nulcount][0]=NUL;
	}
	return count;
}

