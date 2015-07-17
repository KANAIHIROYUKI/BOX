#include "serial.hpp"
#include <stdio.h>
void defaultRxIntFunc(char)
{
    return;
};
void defaultTxIntFunc()
{
    return;
};
void SerialInterface::serialReadChar(char value)
{
    rxBuffer.write(value);
    if(value==DEFAULT_LINEFEED){
        char data[SERIAL_BUFFER_LENGTH];
        for(int i=0;i<SERIAL_BUFFER_LENGTH;i++){
            if(rxBuffer.isEmpty()){
                data[i]=0;
                break;
            }
            data[i]=rxBuffer.read();
            if(data[i]==DEFAULT_LINEFEED||i>=SERIAL_BUFFER_LENGTH-1){
                data[i]=0;
                serialReadString(data);
                break;
            }
        }
    }
}
void SerialInterface::serialPrintf(const char *pFormat, ...)
{
    char buffer[SERIAL_BUFFER_LENGTH];
    va_list ap;
    va_start(ap, pFormat);
    vsprintf(buffer,pFormat,ap);
    va_end(ap);
    serialStringWrite(buffer);
}

void Serial::printf(const char *pFormat, ...)
{
    char buffer[SERIAL_BUFFER_LENGTH];
    va_list ap;
    va_start(ap, pFormat);
    vsprintf(buffer,pFormat,ap);
    va_end(ap);
    stringWrite(buffer);
}

