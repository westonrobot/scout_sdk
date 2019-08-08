/* 
 * hunter_state.hpp
 * 
 * Created on: Aug 07, 2019 12:23
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef HUNTER_STATE_HPP
#define HUNTER_STATE_HPP

#include <cstdint>
#include <iostream>

namespace wescore
{
struct HunterState
{
    enum MotorID
    {
        STEER = 0,
        DRIVE = 1,
    };

    struct MotorState
    {
        double current = 0; // in A
        double rpm = 0;
        double temperature = 0;
    };

    // base state
    uint8_t base_state = 0;
    uint8_t control_mode = 0;
    uint16_t fault_code = 0;
    double battery_voltage = 0.0;

    // motion state
    double linear_velocity;
    double angular_velocity;
};
} // namespace wescore

#endif /* HUNTER_STATE_HPP */
