/* 
 * scout_messages.h
 * 
 * Created on: Aug 07, 2019 21:27
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef SCOUT_MESSAGES_H
#define SCOUT_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "agilex_common/mobile_base_protocol.h"

#define SCOUT_CMD_BUF_LEN               32

#define SCOUT_MOTOR1_ID                 ((uint8_t)0x01)
#define SCOUT_MOTOR2_ID                 ((uint8_t)0x02)
#define SCOUT_MOTOR3_ID                 ((uint8_t)0x03)
#define SCOUT_MOTOR4_ID                 ((uint8_t)0x04)

#pragma pack(push, 1)

// For convenience to access status message
typedef enum
{
    ScoutStatusNone = 0x00,
    ScoutMotionStatusMsg = 0x01,
    ScoutLightStatusMsg = 0x02,
    ScoutSystemStatusMsg = 0x03,
    ScoutMotor1DriverStatusMsg = 0x04,
    ScoutMotor2DriverStatusMsg = 0x05,
    ScoutMotor3DriverStatusMsg = 0x06,
    ScoutMotor4DriverStatusMsg = 0x07
} ScoutStatusMsgType;

typedef struct 
{
    ScoutStatusMsgType updated_msg_type;

    // only one of the following fields is updated, as specified by updated_msg_type
    MotionStatusMessage motion_status_msg;
    LightStatusMessage light_status_msg;
    SystemStatusMessage system_status_msg;
    MotorDriverStatusMessage motor_driver_status_msg;
} ScoutStatusMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* SCOUT_MESSAGES_H */
