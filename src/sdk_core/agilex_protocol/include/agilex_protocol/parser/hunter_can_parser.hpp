/* 
 * hunter_can_parser.hpp
 * 
 * Created on: Aug 07, 2019 12:25
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_CAN_PARSER_HPP
#define HUNTER_CAN_PARSER_HPP

#include <cstdint>
#include <cstring>

#include <linux/can.h>

#include "agilex_protocol/hunter_protocol.h"

namespace wescore
{
namespace HunterCANParser
{
// Hunter CAN Frame IDs
constexpr int32_t CAN_MSG_MOTION_CONTROL_CMD_ID = 0x130;
constexpr int32_t CAN_MSG_MOTION_CONTROL_STATUS_ID = 0x131;
constexpr int32_t CAN_MSG_LIGHT_CONTROL_CMD_ID = 0x140;
constexpr int32_t CAN_MSG_LIGHT_CONTROL_STATUS_ID = 0x141;
constexpr int32_t CAN_MSG_SYSTEM_STATUS_STATUS_ID = 0x151;
constexpr int32_t CAN_MSG_MOTOR1_DRIVER_STATUS_ID = 0x200;
constexpr int32_t CAN_MSG_MOTOR2_DRIVER_STATUS_ID = 0x201;
constexpr int32_t CAN_MSG_MOTOR3_DRIVER_STATUS_ID = 0x202;
constexpr int32_t CAN_MSG_MOTOR4_DRIVER_STATUS_ID = 0x203;

uint8_t Agilex_CANMsgChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

HunterStatusMessage UnpackHunterCANFrameToMsg(can_frame *rx_frame);

template <typename MsgType>
can_frame PackMsgToHunterCANFrame(const MsgType &msg)
{
    can_frame frame;
    frame.can_id = msg.id;
    frame.can_dlc = msg.len;
    std::memcpy(frame.data, msg.msg.raw, msg.len);
    return frame;
}
} // namespace HunterCANParser
} // namespace wescore

#endif /* HUNTER_CAN_PARSER_HPP */
