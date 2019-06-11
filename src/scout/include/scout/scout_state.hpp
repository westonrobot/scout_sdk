/* 
 * scout_state.hpp
 * 
 * Created on: Jun 11, 2019 08:48
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_STATE_HPP
#define SCOUT_STATE_HPP

#include <cstdint>

namespace wescore
{
struct ScoutState
{
    enum class BaseState
    {
        NORMAL,
        ESTOP,
        EXCEPTION
    };

    enum class ControlMode
    {
        NORMAL,
        ESTOP,
        EXCEPTION
    };

    struct FaultCode
    {
        bool can_checksum_error;
        bool motor_driver_overheat;
        bool motor_overcurrent;
        bool battery_low_voltage;
        bool battery_low_volrage_fault;
        bool batter_over_voltage;
        bool motor1_comm_error;
        bool motor2_comm_error;
        bool motor3_comm_error;
        bool motor4_comm_error;
        bool motor_driver_overheat_protection;
        bool motor_overcurrent_protection;
    };

    enum MotorID
    {
        FRONT_LEFT = 0,
        FRONT_RIGHT = 1,
        REAR_LEFT = 2,
        REAR_RIGHT = 3
    };

    struct MotorState
    {
        double current; // in A
        double rpm;
        double temperature;
    };

    enum class LightMode
    {
        CONST_ON,
        CONST_OFF,
        BREATH,
        CUSTOM
    };

    struct LightState
    {
        LightMode mode;
        uint8_t custom_value;
    };

    // base state
    BaseState base_state;
    ControlMode control_mode;
    FaultCode fault_code;
    double battery_voltage;

    // motor state
    MotorState motor_states[4];

    // light state
    bool light_control_enabled;
    LightState front_light_state;
    LightState rear_light_state;

    // motion state
    double linear_velocity;
    double angular_velocity;
};
} // namespace wescore

#endif /* SCOUT_STATE_HPP */
