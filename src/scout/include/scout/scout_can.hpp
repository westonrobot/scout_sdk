/* 
 * scout_can.hpp
 * 
 * Created on: Jun 04, 2019 03:30
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_CAN_HPP
#define SCOUT_CAN_HPP

namespace wescore
{
enum ScoutCANMessages
{
    SCOUT_CAN_MOTION_CONTROL = 1,           // 0x130
    SCOUT_CAN_MOTION_CONTROL_FEEDBACK = 2,  // 0x131
    SCOUT_CAN_LIGHT_CONTROL = 3,            // 0x140
    SCOUT_CAN_LIGHT_CONTROL_FEEDBACK = 4,   // 0x141
    SCOUT_CAN_SYSTEM_STATUS_FEEDBACK = 5,   // 0x151
    SCOUT_CAN_MOTOR1_DRIVER_FEEDBACK = 6,   // 0x201
    SCOUT_CAN_MOTOR2_DRIVER_FEEDBACK = 7,   // 0x202
    SCOUT_CAN_MOTOR3_DRIVER_FEEDBACK = 8,   // 0x203
    SCOUT_CAN_MOTOR4_DRIVER_FEEDBACK = 9,   // 0x204
    SCOUT_CAN_LAST
};
}

#endif /* SCOUT_CAN_HPP */
