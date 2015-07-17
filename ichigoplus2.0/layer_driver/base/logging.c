#include "logging.h"
#include "serial.hpp"
#include <stdio.h>
#define line_feed "\0"

void default_logging_out(char *value);
void (*char_out_func)(char)=std_char_out;
void (*string_out_func)(char *)=default_logging_out;
static char buffer[256];
static char output[256];

void default_logging_out(char *value){
//	string_out(char_out_func,value);
	char_out_func('\r');
}

static int logging_level_value=LOGGING_INFO;

int logging_level(int level){
	logging_level_value=level;
	return level;
}

int logging_setup(void (*char_out_arg)(char)){
	char_out_func=char_out_arg;
	return 0;
}

int logging_setup_string(void (*string_out_arg)(char *)){
	string_out_func=string_out_arg;
	return 0;
}

void fatal(char *file,const char *func,int line,int time,const char *pFormat, ...){
	if(LOGGING_FATAL<logging_level_value){
		return;
	}
	va_list ap;
	va_start(ap, pFormat);
	vsprintf(buffer,pFormat,ap);
	va_end(ap);
	sprintf(output,"fatal,%s,%s,%d,%d,%s%s",file,func,line,time,buffer,line_feed);
	string_out_func(output);
}

void warn(char *file,const char *func,int line,int time,const char *pFormat, ...){
	if(LOGGING_WARN<logging_level_value){
		return;
	}
	va_list ap;
	va_start(ap, pFormat);
	vsprintf(buffer,pFormat,ap);
	va_end(ap);
	sprintf(output,"warn,%s,%s,%d,%d,%s%s",file,func,line,time,buffer,line_feed);
	string_out_func(output);
}

void info(char *file,const char *func,int line,int time,const char *pFormat, ...){
	if(LOGGING_INFO<logging_level_value){
		return;
	}
	va_list ap;
	va_start(ap, pFormat);
	vsprintf(buffer,pFormat,ap);
	va_end(ap);
	sprintf(output,"info,%s,%s,%d,%d,%s%s",file,func,line,time,buffer,line_feed);
	string_out_func(output);
}

void debug(char *file,const char *func,int line,int time,const char *pFormat, ...){
	if(LOGGING_DEBUG<logging_level_value){
		return;
	}
	va_list ap;
	va_start(ap, pFormat);
	vsprintf(buffer,pFormat,ap);
	va_end(ap);
	sprintf(output,"debug,%s,%s,%d,%d,%s%s",file,func,line,time,buffer,line_feed);
	string_out_func(output);
}
