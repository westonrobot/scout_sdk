/* 
 * scout_serial_protocol.hpp
 * 
 * Created on: Jun 05, 2019 02:34
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_SERIAL_PROTOCOL_HPP
#define SCOUT_SERIAL_PROTOCOL_HPP

namespace wescore
{
struct ScoutSerialProtocol
{
    struct Frame
    {
        unsigned short Header;
        unsigned char Len;
        unsigned char Typedef;
        unsigned char Count;
        unsigned char Time_Out;
        short Linear;
        short Angular;
        unsigned short CheckSum;
    };
};
} // namespace wescore

#endif /* SCOUT_SERIAL_PROTOCOL_HPP */
