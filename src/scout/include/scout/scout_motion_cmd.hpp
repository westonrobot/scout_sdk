/* 
 * scout_motion_cmd.hpp
 * 
 * Created on: Jun 11, 2019 08:49
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MOTION_CMD_HPP
#define SCOUT_MOTION_CMD_HPP

#include <cstdint>

namespace wescore
{
struct ScoutMotionCmd
{
    enum class FaultClearFlag
    {
        NO_FAULT = 0x00,
        BAT_UNDER_VOL = 0x01,
        BAT_OVER_VOL = 0x02,
        MOTOR1_COMM = 0x03,
        MOTOR2_COMM = 0x04,
        MOTOR3_COMM = 0x05,
        MOTOR4_COMM = 0x06,
        MOTOR_DRV_OVERHEAT = 0x07,
        MOTOR_OVERCURRENT = 0x08
    };

    ScoutMotionCmd(int8_t linear = 0, int8_t angular = 0,
                   FaultClearFlag fault_clr_flag = FaultClearFlag::NO_FAULT)
        : linear_velocity(linear), angular_velocity(angular),
          fault_clear_flag(fault_clr_flag) {}

    int8_t linear_velocity;
    int8_t angular_velocity;
    FaultClearFlag fault_clear_flag;

    static constexpr double max_linear_velocity = 1.5;      // 1.5m/s
    static constexpr double min_linear_velocity = -1.5;     // -1.5m/s
    static constexpr double max_angular_velocity = 0.7853;  // 0.7853rad/s
    static constexpr double min_angular_velocity = -0.7853; // -0.7853rad/s
};
} // namespace wescore

#endif /* SCOUT_MOTION_CMD_HPP */
