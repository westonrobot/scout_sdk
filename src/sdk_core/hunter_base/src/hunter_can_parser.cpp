/* 
 * hunter_can_parser.cpp
 * 
 * Created on: Aug 07, 2019 12:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#include "hunter_base/details/hunter_can_parser.hpp"

namespace wescore
{
uint8_t HunterCANParser::Agilex_CANMsgChecksum(uint16_t id, uint8_t *data, uint8_t dlc)
{
    uint8_t checksum = 0x00;
    checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
    for (int i = 0; i < (dlc - 1); ++i)
        checksum += data[i];
    return checksum;
};

HunterStatusMessage HunterCANParser::UnpackHunterCANFrameToMsg(can_frame *rx_frame)
{
    HunterStatusMessage msgs;
    msgs.updated_msg_type = HunterStatusNone;

    // HunterMessage msg;
    switch (rx_frame->can_id)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CONTROL_STATUS_ID:
    {
        msgs.updated_msg_type = HunterMotionStatusMsg;
        msgs.motion_status_msg.id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        std::memcpy(msgs.motion_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_CONTROL_STATUS_ID:
    {
        msgs.updated_msg_type = HunterLightStatusMsg;
        msgs.light_status_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        std::memcpy(msgs.light_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_SYSTEM_STATUS_STATUS_ID:
    {
        msgs.updated_msg_type = HunterSystemStatusMsg;
        msgs.system_status_msg.id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        std::memcpy(msgs.system_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR1_DRIVER_STATUS_ID:
    {
        msgs.updated_msg_type = HunterMotor1DriverStatusMsg;
        msgs.motor_driver_status_msg.id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        msgs.motor_driver_status_msg.motor_id = SCOUT_MOTOR1_ID;
        std::memcpy(msgs.motor_driver_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR2_DRIVER_STATUS_ID:
    {
        msgs.updated_msg_type = HunterMotor2DriverStatusMsg;
        msgs.motor_driver_status_msg.id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        msgs.motor_driver_status_msg.motor_id = SCOUT_MOTOR2_ID;
        std::memcpy(msgs.motor_driver_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR3_DRIVER_STATUS_ID:
    {
        msgs.updated_msg_type = HunterMotor3DriverStatusMsg;
        msgs.motor_driver_status_msg.id = CAN_MSG_MOTOR3_DRIVER_STATUS_ID;
        msgs.motor_driver_status_msg.motor_id = SCOUT_MOTOR3_ID;
        std::memcpy(msgs.motor_driver_status_msg.msg.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR4_DRIVER_STATUS_ID:
    {
        msgs.updated_msg_type = HunterMotor4DriverStatusMsg;
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