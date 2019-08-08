/* 
 * scout_can_parser.cpp
 * 
 * Created on: Jul 24, 2019 23:54
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "agilex_protocol/parser/scout_can_parser.hpp"

namespace wescore
{
uint8_t ScoutCANParser::Agilex_CANMsgChecksum(uint16_t id, uint8_t *data, uint8_t dlc)
{
    uint8_t checksum = 0x00;
    checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
    for (int i = 0; i < (dlc - 1); ++i)
        checksum += data[i];
    return checksum;
};

ScoutStatusMessage ScoutCANParser::UnpackScoutCANFrameToMsg(can_frame *rx_frame)
{
    ScoutStatusMessage msgs;
    msgs.updated_msg_type = ScoutStatusNone;

    // ScoutMessage msg;
    switch (rx_frame->can_id)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CONTROL_STATUS_ID:
    {
        msgs.updated_msg_type = ScoutMotionStatusMsg;
        msgs.motion_status_msg.id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        std::memcpy(msgs.motion_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_CONTROL_STATUS_ID:
    {
        msgs.updated_msg_type = ScoutLightStatusMsg;
        msgs.light_status_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        std::memcpy(msgs.light_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_SYSTEM_STATUS_STATUS_ID:
    {
        msgs.updated_msg_type = ScoutSystemStatusMsg;
        msgs.system_status_msg.id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        std::memcpy(msgs.system_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR1_DRIVER_STATUS_ID:
    {
        msgs.updated_msg_type = ScoutMotor1DriverStatusMsg;
        msgs.motor_driver_status_msg.id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        msgs.motor_driver_status_msg.motor_id = SCOUT_MOTOR1_ID;
        std::memcpy(msgs.motor_driver_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR2_DRIVER_STATUS_ID:
    {
        msgs.updated_msg_type = ScoutMotor2DriverStatusMsg;
        msgs.motor_driver_status_msg.id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        msgs.motor_driver_status_msg.motor_id = SCOUT_MOTOR2_ID;
        std::memcpy(msgs.motor_driver_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR3_DRIVER_STATUS_ID:
    {
        msgs.updated_msg_type = ScoutMotor3DriverStatusMsg;
        msgs.motor_driver_status_msg.id = CAN_MSG_MOTOR3_DRIVER_STATUS_ID;
        msgs.motor_driver_status_msg.motor_id = SCOUT_MOTOR3_ID;
        std::memcpy(msgs.motor_driver_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR4_DRIVER_STATUS_ID:
    {
        msgs.updated_msg_type = ScoutMotor4DriverStatusMsg;
        msgs.motor_driver_status_msg.id = CAN_MSG_MOTOR4_DRIVER_STATUS_ID;
        msgs.motor_driver_status_msg.motor_id = SCOUT_MOTOR4_ID;
        std::memcpy(msgs.motor_driver_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    default:
        break;
    }

    return msgs;
}
} // namespace wescore