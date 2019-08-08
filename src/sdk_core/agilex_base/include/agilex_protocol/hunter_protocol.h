/* 
 * hunter_protocol.h
 * 
 * Created on: Aug 07, 2019 21:30
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef HUNTER_PROTOCOL_H
#define HUNTER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "agilex_common/mobile_base_protocol.h"

#define HUNTER_MOTOR_STEER_ID           ((uint8_t)0x01)
#define HUNTER_MOTOR_DRIVE_ID           ((uint8_t)0x02)

#pragma pack(push, 1)

typedef enum
{
    HunterStatusNone = 0x00,
    HunterMotionStatusMsg = 0x01,
    HunterLightStatusMsg = 0x02,
    HunterSystemStatusMsg = 0x03,
    HunterMotor1DriverStatusMsg = 0x04,
    HunterMotor2DriverStatusMsg = 0x05,
    HunterMotor3DriverStatusMsg = 0x06,
    HunterMotor4DriverStatusMsg = 0x07
} HunterStatusMsgType;

typedef struct 
{
    HunterStatusMsgType updated_msg_type;

    // only one of the following fields is updated, as specified by updated_msg_type
    MotionStatusMessage motion_status_msg;
    LightStatusMessage light_status_msg;
    SystemStatusMessage system_status_msg;
    MotorDriverStatusMessage motor_driver_status_msg;
} HunterStatusMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* HUNTER_PROTOCOL_H */
