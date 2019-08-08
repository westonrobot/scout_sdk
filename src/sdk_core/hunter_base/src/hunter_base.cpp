/* 
 * hunter_base.cpp
 * 
 * Created on: Aug 07, 2019 12:23
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#include "hunter_base/hunter_base.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

namespace
{
// source: https://github.com/rxdu/stopwatch
struct StopWatch
{
    using Clock = std::chrono::high_resolution_clock;
    using time_point = typename Clock::time_point;
    using duration = typename Clock::duration;

    StopWatch() { tic_point = Clock::now(); };

    time_point tic_point;

    void tic()
    {
        tic_point = Clock::now();
    };

    double toc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count() / 1000000.0;
    };

    // for different precisions
    double stoc()
    {
        return std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - tic_point).count();
    };

    double mtoc()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();
    };

    double utoc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();
    };

    double ntoc()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - tic_point).count();
    };

    // you have to call tic() before calling this function
    void sleep_until_ms(int64_t period_ms)
    {
        int64_t duration = period_ms - std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(duration));
    };

    void sleep_until_us(int64_t period_us)
    {
        int64_t duration = period_us - std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(duration));
    };
};
} // namespace

namespace wescore
{
HunterBase::~HunterBase()
{
    if (serial_connected_)
        serial_if_->close();

    if (cmd_thread_.joinable())
        cmd_thread_.join();
}

void HunterBase::Connect(std::string dev_name, int32_t baud_rate)
{
    if (baud_rate == 0)
    {
        ConfigureCANBus(dev_name);
    }
    else
    {
        ConfigureSerial(dev_name, baud_rate);

        if (!serial_connected_)
            std::cerr << "ERROR: Failed to connect to serial port" << std::endl;
    }
}

void HunterBase::Disconnect()
{
    if (serial_connected_)
    {
        if (serial_if_->is_open())
            serial_if_->close();
    }
}

void HunterBase::ConfigureCANBus(const std::string &can_if_name)
{
    can_if_ = std::make_shared<ASyncCAN>(can_if_name);

    can_if_->set_receive_callback(std::bind(&HunterBase::ParseCANFrame, this, std::placeholders::_1));

    can_connected_ = true;
}

void HunterBase::ConfigureSerial(const std::string uart_name, int32_t baud_rate)
{
    serial_if_ = std::make_shared<ASyncSerial>(uart_name, baud_rate);
    serial_if_->open();

    if (serial_if_->is_open())
        serial_connected_ = true;

    serial_if_->set_receive_callback(std::bind(&HunterBase::ParseUARTBuffer, this,
                                               std::placeholders::_1,
                                               std::placeholders::_2,
                                               std::placeholders::_3));

    serial_parser_.SetReceiveCallback(std::bind(&HunterBase::NewStatusMsgReceivedCallback, this, std::placeholders::_1));
}

void HunterBase::StartCmdThread()
{
    cmd_thread_ = std::thread(std::bind(&HunterBase::ControlLoop, this, cmd_thread_period_ms_));
    cmd_thread_started_ = true;
}

void HunterBase::SendMotionCmd(uint8_t count)
{
    // motion control message
    MotionControlMessage m_msg;

    if (can_connected_)
    {
        m_msg.id = HunterCANParser::CAN_MSG_MOTION_CONTROL_CMD_ID;
        m_msg.msg.cmd.control_mode = CTRL_MODE_CMD_CAN;
    }
    else if (serial_connected_)
    {
        m_msg.id = HunterSerialParser::FRAME_MOTION_CONTROL_CMD_ID;
        m_msg.msg.cmd.control_mode = CTRL_MODE_CMD_UART;
    }

    motion_cmd_mutex_.lock();
    m_msg.msg.cmd.fault_clear_flag = static_cast<uint8_t>(current_motion_cmd_.fault_clear_flag);
    m_msg.msg.cmd.linear_velocity_cmd = current_motion_cmd_.linear_velocity;
    m_msg.msg.cmd.angular_velocity_cmd = current_motion_cmd_.angular_velocity;
    motion_cmd_mutex_.unlock();

    m_msg.msg.cmd.reserved0 = 0;
    m_msg.msg.cmd.reserved1 = 0;
    m_msg.msg.cmd.count = count;

    if (can_connected_)
        m_msg.msg.cmd.checksum = HunterCANParser::Agilex_CANMsgChecksum(m_msg.id, m_msg.msg.raw, m_msg.len);
    // serial_connected_: checksum will be calculated later when packed into a complete serial frame

    if (can_connected_)
    {
        // send to can bus
        can_frame m_frame = HunterCANParser::PackMsgToHunterCANFrame(m_msg);
        can_if_->send_frame(m_frame);
    }
    else
    {
        // send to serial port
        HunterSerialParser::PackMotionControlMsgToBuffer(m_msg, tx_buffer_, tx_cmd_len_);
        serial_if_->send_bytes(tx_buffer_, tx_cmd_len_);
    }
}

void HunterBase::ControlLoop(int32_t period_ms)
{
    StopWatch ctrl_sw;
    uint8_t cmd_count = 0;
    uint8_t light_cmd_count = 0;
    while (true)
    {
        ctrl_sw.tic();

        // motion control message
        SendMotionCmd(cmd_count++);

        // check if there is request for light control
        if (light_ctrl_requested_)
            SendLightCmd(light_cmd_count++);

        ctrl_sw.sleep_until_ms(period_ms);
        // std::cout << "control loop update frequency: " << 1.0 / ctrl_sw.toc() << std::endl;
    }
}

HunterState HunterBase::GetHunterState()
{
    std::lock_guard<std::mutex> guard(hunter_state_mutex_);
    return hunter_state_;
}

void HunterBase::SetMotionCommand(double linear_vel, double angular_vel, HunterMotionCmd::FaultClearFlag fault_clr_flag)
{
    // make sure cmd thread is started before attempting to send commands
    if (!cmd_thread_started_)
        StartCmdThread();

    if (linear_vel < HunterMotionCmd::min_linear_velocity)
        linear_vel = HunterMotionCmd::min_linear_velocity;
    if (linear_vel > HunterMotionCmd::max_linear_velocity)
        linear_vel = HunterMotionCmd::max_linear_velocity;
    if (angular_vel < HunterMotionCmd::min_angular_velocity)
        angular_vel = HunterMotionCmd::min_angular_velocity;
    if (angular_vel > HunterMotionCmd::max_angular_velocity)
        angular_vel = HunterMotionCmd::max_angular_velocity;

    std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
    current_motion_cmd_.linear_velocity = static_cast<uint8_t>(linear_vel / HunterMotionCmd::max_linear_velocity * 100.0);
    current_motion_cmd_.angular_velocity = static_cast<uint8_t>(angular_vel / HunterMotionCmd::max_angular_velocity * 100.0);
    current_motion_cmd_.fault_clear_flag = fault_clr_flag;
}

void HunterBase::ParseCANFrame(can_frame *rx_frame)
{
    // validate checksum, discard frame if fails
    if (!rx_frame->data[7] == HunterCANParser::Agilex_CANMsgChecksum(rx_frame->can_id, rx_frame->data, rx_frame->can_dlc))
    {
        std::cerr << "ERROR: checksum mismatch, discard frame with id " << rx_frame->can_id << std::endl;
        return;
    }

    // otherwise, update robot state with new frame
    auto status_msg = HunterCANParser::UnpackHunterCANFrameToMsg(rx_frame);
    NewStatusMsgReceivedCallback(status_msg);
}

void HunterBase::ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    // std::cout << "bytes received from serial: " << bytes_received << std::endl;
    // serial_parser_.PrintStatistics();
    serial_parser_.ParseBuffer(buf, bytes_received);
}

void HunterBase::NewStatusMsgReceivedCallback(const HunterStatusMessage &msg)
{
    // std::cout << "new status msg received" << std::endl;
    std::lock_guard<std::mutex> guard(hunter_state_mutex_);
    UpdateHunterState(msg, hunter_state_);
}

void HunterBase::UpdateHunterState(const HunterStatusMessage &status_msg, HunterState &state)
{
    switch (status_msg.updated_msg_type)
    {
    case HunterMotionStatusMsg:
    {
        // std::cout << "motion control feedback received" << std::endl;
        const MotionStatusMessage &msg = status_msg.motion_status_msg;
        state.linear_velocity = static_cast<int16_t>(static_cast<uint16_t>(msg.msg.status.linear_velocity.low_byte) | static_cast<uint16_t>(msg.msg.status.linear_velocity.high_byte) << 8) / 1000.0;
        state.angular_velocity = static_cast<int16_t>(static_cast<uint16_t>(msg.msg.status.angular_velocity.low_byte) | static_cast<uint16_t>(msg.msg.status.angular_velocity.high_byte) << 8) / 1000.0;
        break;
    }
    case HunterSystemStatusMsg:
    {
        // std::cout << "system status feedback received" << std::endl;
        const SystemStatusMessage &msg = status_msg.system_status_msg;
        state.control_mode = msg.msg.status.control_mode;
        state.base_state = msg.msg.status.base_state;
        state.battery_voltage = (static_cast<uint16_t>(msg.msg.status.battery_voltage.low_byte) | static_cast<uint16_t>(msg.msg.status.battery_voltage.high_byte) << 8) / 10.0;
        state.fault_code = (static_cast<uint16_t>(msg.msg.status.fault_code.low_byte) | static_cast<uint16_t>(msg.msg.status.fault_code.high_byte) << 8);
        break;
    }
    }
}
} // namespace wescore
