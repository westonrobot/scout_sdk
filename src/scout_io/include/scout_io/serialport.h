#ifndef __SERIALPORT_h__
#define __SERIALPORT_h__

#include <string>

namespace scout_serial
{
int Open_Serial(std::string port_name, int baud_rate);
unsigned int Write(unsigned char *data, unsigned short len);
unsigned int Read(unsigned char *data, unsigned short len);
unsigned int GetDataCount(void);
} // namespace scout_serial

#endif
