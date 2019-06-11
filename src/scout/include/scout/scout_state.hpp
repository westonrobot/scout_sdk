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
#include <iostream>

namespace wescore
{
struct ScoutState
{
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

    struct LightState
    {
        uint8_t mode;
        uint8_t custom_value;
    };

    // base state
    uint8_t base_state;
    uint8_t control_mode;
    uint16_t fault_code;
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

    friend std::ostream &operator<<(std::ostream &os, const ScoutState &state)
    {
        std::cout << "-------------------------------" << std::endl;
        std::cout << "control mode: " << state.control_mode << " , base state: " << state.base_state << std::endl;
        std::cout << "battery voltage: " << state.battery_voltage << std::endl;
        std::cout << "velocity (linear, angular): " << state.linear_velocity << ", " << state.angular_velocity;
    }
};
} // namespace wescore

#endif /* SCOUT_STATE_HPP */
