/* 
 * hunter_base.hpp
 * 
 * Created on: Aug 07, 2019 12:22
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef HUNTER_BASE_HPP
#define HUNTER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "async_io/async_can.hpp"
#include "async_io/async_serial.hpp"

#include "hunter_base/hunter_protocol.h"
#include "hunter_base/hunter_state.hpp"
#include "hunter_base/hunter_command.hpp"

#include "hunter_base/details/hunter_serial_parser.hpp"
#include "hunter_base/details/hunter_can_parser.hpp"

namespace wescore
{
class AgilexMobileBase
{
public:
    AgilexMobileBase() = default;
    ~AgilexMobileBase();

    // do not allow copy
    AgilexMobileBase(const AgilexMobileBase &base) = delete;
    AgilexMobileBase &operator=(const AgilexMobileBase &base) = delete;

public:
    // connect to roboot from CAN or serial
    void Connect(std::string dev_name, int32_t baud_rate = 0);

    // disconnect from roboot, only valid for serial port
    void Disconnect();

    // cmd thread runs at 100Hz (10ms) by default
    void SetCmdThreadPeriodMs(int32_t period_ms) { cmd_thread_period_ms_ = period_ms; };

    // motion control
    void SetMotionCommand(double linear_vel, double angular_vel,
                          HunterMotionCmd::FaultClearFlag fault_clr_flag = HunterMotionCmd::FaultClearFlag::NO_FAULT);

    // get robot state
    HunterState GetHunterState();

protected:
    // hardware communication interface
    std::shared_ptr<ASyncCAN> can_if_;
    std::shared_ptr<ASyncSerial> serial_if_;

    // CAN priority higher than serial if both connected
    bool can_connected_ = false;
    bool serial_connected_ = false;

    // serial port related variables
    uint8_t tx_cmd_len_;
    uint8_t tx_buffer_[SCOUT_CMD_BUF_LEN];
    HunterSerialParser serial_parser_;

    // cmd/status update related variables
    std::thread cmd_thread_;
    std::mutex hunter_state_mutex_;
    std::mutex motion_cmd_mutex_;
    std::mutex light_cmd_mutex_;

    HunterState hunter_state_;
    HunterMotionCmd current_motion_cmd_;
    HunterLightCmd current_light_cmd_;

    int32_t cmd_thread_period_ms_ = 10;
    bool cmd_thread_started_ = false;

    bool light_ctrl_enabled_ = false;
    bool light_ctrl_requested_ = false;

    // internal functions
    void ConfigureCANBus(const std::string &can_if_name = "can1");
    void ConfigureSerial(const std::string uart_name = "/dev/ttyUSB0", int32_t baud_rate = 115200);

    void StartCmdThread();
    void ControlLoop(int32_t period_ms);

    void SendMotionCmd(uint8_t count);
    void SendLightCmd(uint8_t count);

    void ParseCANFrame(can_frame *rx_frame);
    void ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received);

    void NewStatusMsgReceivedCallback(const HunterStatusMessage &msg);

public:
    static void UpdateHunterState(const HunterStatusMessage &status_msg, HunterState &state);
};
} // namespace wescore

#endif /* HUNTER_BASE_HPP */
