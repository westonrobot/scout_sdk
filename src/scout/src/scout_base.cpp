#include "scout/scout_base.hpp"

#include <string>
#include <cstring>
#include <iostream>

#include "scout/scout_can_protocol.h"

namespace wescore
{
ScoutBase::~ScoutBase()
{
    if (cmd_thread_.joinable())
        cmd_thread_.join();
}

void ScoutBase::ConnectSerialPort(const std::string &port_name, int32_t baud_rate)
{
    // serial_connected_ = (scout_serial::Open_Serial(port_name, baud_rate) > 0) ? true : false;
}

void ScoutBase::ConnectCANBus(const std::string &can_if_name)
{
    can_if_ = std::make_shared<ASyncCAN>(can_if_name);

    can_if_->set_receive_callback(std::bind(&ScoutBase::ParseCANFrame, this, std::placeholders::_1));
}

void ScoutBase::StartCmdThread(int32_t period_ms)
{
    cmd_thread_ = std::thread(std::bind(&ScoutBase::ControlLoop, this, period_ms));
}

void ScoutBase::ControlLoop(int32_t period_ms)
{
    stopwatch::StopWatch ctrl_sw;
    uint8_t cmd_count = 0;
    while (true)
    {
        ctrl_sw.tic();

        MotionControlMessage msg;
        msg.data.cmd.control_mode = CMD_MODE;

        motion_cmd_mutex_.lock();
        msg.data.cmd.fault_clear_flag = static_cast<uint8_t>(current_motion_cmd_.fault_clear_flag);
        msg.data.cmd.linear_velocity_cmd = current_motion_cmd_.linear_velocity;
        msg.data.cmd.angular_velocity_cmd = current_motion_cmd_.angular_velocity;
        motion_cmd_mutex_.unlock();

        msg.data.cmd.reserved0 = 0;
        msg.data.cmd.reserved1 = 0;
        msg.data.cmd.count = cmd_count++;
        msg.data.cmd.checksum = Agilex_CANMsgChecksum(msg.id, msg.data.raw, msg.dlc);

        // send to can bus
        can_frame frame;
        frame.can_id = msg.id;
        frame.can_dlc = msg.dlc;
        std::memcpy(frame.data, msg.data.raw, msg.dlc * sizeof(uint8_t));
        can_if_->send_frame(frame);
        // ------------------

        ctrl_sw.sleep_until_ms(period_ms);
        // std::cout << "control loop update frequency: " << 1.0 / ctrl_sw.toc() << std::endl;
    }
}

void ScoutBase::SetMotionCommand(double linear_vel, double angular_vel, ScoutMotionCmd::FaultClearFlag fault_clr_flag)
{
    if (linear_vel < ScoutMotionCmd::min_linear_velocity)
        linear_vel = ScoutMotionCmd::min_linear_velocity;
    if (linear_vel > ScoutMotionCmd::max_linear_velocity)
        linear_vel = ScoutMotionCmd::max_linear_velocity;
    if (angular_vel < ScoutMotionCmd::min_angular_velocity)
        angular_vel = ScoutMotionCmd::min_angular_velocity;
    if (angular_vel > ScoutMotionCmd::max_angular_velocity)
        angular_vel = ScoutMotionCmd::max_angular_velocity;

    std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
    current_motion_cmd_.linear_velocity = static_cast<uint8_t>(linear_vel / ScoutMotionCmd::max_linear_velocity * 100.0);
    current_motion_cmd_.angular_velocity = static_cast<uint8_t>(angular_vel / ScoutMotionCmd::max_angular_velocity * 100.0);
    current_motion_cmd_.fault_clear_flag = fault_clr_flag;
}

void ScoutBase::ParseCANFrame(can_frame *rx_frame)
{
    // validate checksum, discard frame if fails
    if (!rx_frame->data[7] == Agilex_CANMsgChecksum(rx_frame->can_id, rx_frame->data, rx_frame->can_dlc))
    {
        std::cerr << "ERROR: checksum mismatch, discard frame with id " << rx_frame->can_id << std::endl;
        return;
    }

    // otherwise, update robot state with new frame
    std::lock_guard<std::mutex> guard(scout_state_mutex_);
    UpdateScoutState(scout_state_, rx_frame);

    std::cout << "-------------------------------" << std::endl;
    std::cout << "control mode: " << static_cast<int>(scout_state_.control_mode) << " , base state: " << static_cast<int>(scout_state_.base_state) << std::endl;
    std::cout << "battery voltage: " << scout_state_.battery_voltage << std::endl;
    std::cout << "velocity (linear, angular): " << scout_state_.linear_velocity << ", " << scout_state_.angular_velocity << std::endl;
    std::cout << "-------------------------------" << std::endl;
}

void ScoutBase::UpdateScoutState(ScoutState &state, can_frame *rx_frame)
{
    switch (rx_frame->can_id)
    {
    case MSG_MOTION_CONTROL_FEEDBACK_ID:
    {
        // std::cout << "motion control feedback received" << std::endl;
        MotionStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.linear_velocity = (static_cast<uint16_t>(msg.data.status.linear_velocity.low_byte) | static_cast<uint16_t>(msg.data.status.linear_velocity.high_byte) << 8) / 1000.0;
        state.angular_velocity = (static_cast<uint16_t>(msg.data.status.angular_velocity.low_byte) | static_cast<uint16_t>(msg.data.status.angular_velocity.high_byte) << 8) / 1000.0;
        break;
    }
    case MSG_LIGHT_CONTROL_FEEDBACK_ID:
    {
        // std::cout << "light control feedback received" << std::endl;
        LightStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        if (msg.data.status.light_ctrl_enable == DISABLE_LIGHT_CTRL)
            state.light_control_enabled = false;
        else
            state.light_control_enabled = true;
        state.front_light_state.mode = msg.data.status.front_light_mode;
        state.front_light_state.custom_value = msg.data.status.front_light_custom;
        state.rear_light_state.mode = msg.data.status.rear_light_mode;
        state.rear_light_state.custom_value = msg.data.status.rear_light_custom;
        break;
    }
    case MSG_SYSTEM_STATUS_FEEDBACK_ID:
    {
        // std::cout << "system status feedback received" << std::endl;
        SystemStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.control_mode = msg.data.status.control_mode;
        state.base_state = msg.data.status.base_state;
        state.battery_voltage = (static_cast<uint16_t>(msg.data.status.battery_voltage.low_byte) | static_cast<uint16_t>(msg.data.status.battery_voltage.high_byte) << 8) / 10.0;
        state.fault_code = (static_cast<uint16_t>(msg.data.status.fault_code.low_byte) | static_cast<uint16_t>(msg.data.status.fault_code.high_byte) << 8);
        // uint16_t battery_value = (static_cast<uint16_t>(msg.data.status.battery_voltage.low_byte) | static_cast<uint16_t>(msg.data.status.battery_voltage.high_byte) << 8);
        // std::cout << "battery: " << std::hex << static_cast<int>(msg.data.status.battery_voltage.high_byte) << " , "
        //         << static_cast<int>(msg.data.status.battery_voltage.low_byte) << " => " << battery_value << " = " << state.battery_voltage << std::endl;
        break;
    }
    case MSG_MOTOR1_DRIVER_FEEDBACK_ID:
    {
        // std::cout << "motor 1 driver feedback received" << std::endl;
        Motor1DriverStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.motor_states[0].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
        state.motor_states[0].rpm = (static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        ;
        state.motor_states[0].temperature = msg.data.status.temperature;
        break;
    }
    case MSG_MOTOR2_DRIVER_FEEDBACK_ID:
    {
        // std::cout << "motor 2 driver feedback received" << std::endl;
        Motor2DriverStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.motor_states[1].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
        state.motor_states[1].rpm = (static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        ;
        state.motor_states[1].temperature = msg.data.status.temperature;
        break;
    }
    case MSG_MOTOR3_DRIVER_FEEDBACK_ID:
    {
        // std::cout << "motor 3 driver feedback received" << std::endl;
        Motor3DriverStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.motor_states[2].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
        state.motor_states[2].rpm = (static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        ;
        state.motor_states[2].temperature = msg.data.status.temperature;
        break;
    }
    case MSG_MOTOR4_DRIVER_FEEDBACK_ID:
    {
        // std::cout << "motor 4 driver feedback received" << std::endl;
        Motor4DriverStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.motor_states[3].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
        state.motor_states[3].rpm = (static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        state.motor_states[3].temperature = msg.data.status.temperature;
        break;
    }
    }
}
} // namespace wescore
