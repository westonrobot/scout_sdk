/* 
 * scout_base.hpp
 * 
 * Created on: Jun 04, 2019 01:22
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_BASE_HPP
#define SCOUT_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "stopwatch/stopwatch.h"
#include "async_io/async_can.hpp"

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

class ScoutBase
{
public:
    ScoutBase() = default;
    ~ScoutBase();

    // do not allow copy
    ScoutBase(const ScoutBase &scout) = delete;
    ScoutBase &operator=(const ScoutBase &scout) = delete;

public:
    void ConnectSerialPort(const std::string &port_name = "/dev/ttyUSB0", int32_t baud_rate = 115200);
    void ConnectCANBus(const std::string &can_if_name = "can1");

    bool IsConnectionActive() const { return serial_connected_; }

    void StartCmdThread(int32_t period_ms);

    void SetMotionCommand(double linear_vel, double angular_vel,
                          ScoutMotionCmd::FaultClearFlag fault_clr_flag = ScoutMotionCmd::FaultClearFlag::NO_FAULT);
    void SetLightCommand();

private:
    bool serial_connected_ = false;

    std::shared_ptr<ASyncCAN> can_if_;

    std::thread cmd_thread_;
    std::mutex motion_cmd_mutex_;

    // stopwatch::StopWatch ctrl_loop_stopwatch_;
    // TimePoint current_time_;
    // TimePoint last_time_;

    ScoutState scout_state_;
    ScoutMotionCmd current_motion_cmd_;

    void ControlLoop(int32_t period_ms);
    void ParseCANFrame(can_frame *rx_frame);
};
} // namespace wescore

#endif /* SCOUT_BASE_HPP */
