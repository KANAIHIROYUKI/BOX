#ifndef LOGGING_H
#define LOGGING_H

#include <stdarg.h>

#define LOGGING_FATAL 1000
#define LOGGING_WARN 800
#define LOGGING_INFO 300
#define LOGGING_DEBUG 200

#ifdef __cplusplus
 extern "C" {
#endif

#include <string.h>
#include "mcutime.h"

#define FILE_NAME (strrchr(__FILE__,'\\' ) + 1 )  //for windows
//#define FILE_NAME __FILE__ //for common os

#define FATAL(...)  info(FILE_NAME,__func__,__LINE__,millis(),__VA_ARGS__)
#define WARN(...)   warn(FILE_NAME,__func__,__LINE__,millis(),__VA_ARGS__)
#define INFO(...)   info(FILE_NAME,__func__,__LINE__,millis(),__VA_ARGS__)
#define DEBUG(...) debug(FILE_NAME,__func__,__LINE__,millis(),__VA_ARGS__)


int logging_setup(void (*char_out_arg)(char));
int logging_setup_string(void (*string_out_arg)(char *));
int logging_level(int level);

void fatal(char *file,const char *func,int line,int time,const char *pFormat, ...);
void warn(char *file,const char *func,int line,int time,const char *pFormat, ...);
void info(char *file,const char *func,int line,int time,const char *pFormat, ...);
void debug(char *file,const char *func,int line,int time,const char *pFormat, ...);


#ifdef __cplusplus
}
#endif


#endif//LOGGING_H
