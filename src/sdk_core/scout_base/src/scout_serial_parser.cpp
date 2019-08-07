/* 
 * scout_serial_parser.cpp
 * 
 * Created on: Jul 25, 2019 00:09
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_base/details/scout_serial_parser.hpp"

#include <iostream>

// #define PRINT_DEBUG_INFO
// #define USE_XOR_CHECKSUM

namespace wescore
{
void ScoutSerialParser::PackMotionControlMsgToBuffer(const MotionControlMessage &msg, uint8_t *buf, uint8_t &len)
{
    // SOF
    buf[0] = FRAME_SOF1;
    buf[1] = FRAME_SOF2;

    // frame len, type, ID
    buf[2] = 0x0a;
    buf[3] = FRAME_TYPE_CMD;
    buf[4] = msg.id;

    // frame payload
    buf[5] = msg.msg.cmd.control_mode;
    buf[6] = msg.msg.cmd.fault_clear_flag;
    buf[7] = msg.msg.cmd.linear_velocity_cmd;
    buf[8] = msg.msg.cmd.angular_velocity_cmd;
    buf[9] = 0x00;
    buf[10] = 0x00;

    // frame count, checksum
    buf[11] = msg.msg.cmd.count;
    buf[12] = ScoutSerialParser::CalcChecksum(buf, buf[2] + FRAME_SOF_LEN);

    // length: SOF + Frame + Checksum
    len = buf[2] + FRAME_SOF_LEN + 1;
}

void ScoutSerialParser::PackLightControlMsgToBuffer(const LightControlMessage &msg, uint8_t *buf, uint8_t &len)
{
    // SOF
    buf[0] = FRAME_SOF1;
    buf[1] = FRAME_SOF2;

    // frame len, type, ID
    buf[2] = 0x0a;
    buf[3] = FRAME_TYPE_CMD;
    buf[4] = msg.id;

    // frame payload
    buf[5] = msg.msg.cmd.light_ctrl_enable;
    buf[6] = msg.msg.cmd.front_light_mode;
    buf[7] = msg.msg.cmd.front_light_custom;
    buf[8] = msg.msg.cmd.rear_light_mode;
    buf[9] = msg.msg.cmd.rear_light_custom;
    buf[10] = 0x00;

    // frame count, checksum
    buf[11] = msg.msg.cmd.count;
    buf[12] = ScoutSerialParser::CalcChecksum(buf, buf[2] + FRAME_SOF_LEN);

    // length: SOF + Frame + Checksum
    len = buf[2] + FRAME_SOF_LEN + 1;
}

uint8_t ScoutSerialParser::CalcChecksum(uint8_t *buf, uint8_t len)
{
    uint8_t checksum = 0;

#ifdef USE_XOR_CHECKSUM
    for (int i = 0; i < len; ++i)
        checksum ^= buf[i];
#else
    for (int i = 0; i < len; ++i)
        checksum += buf[i];
#endif

    return checksum;
}

uint8_t ScoutSerialParser::CalcBufferedFrameChecksum()
{
    uint8_t checksum = 0x00;

#ifdef USE_XOR_CHECKSUM
    checksum ^= FRAME_SOF1;
    checksum ^= FRAME_SOF2;
    checksum ^= frame_len_;
    checksum ^= FRAME_TYPE_STATUS;
    checksum ^= frame_id_;
    for (size_t i = 0; i < payload_buffer_.size(); ++i)
        checksum ^= payload_buffer_[i];
    checksum ^= frame_cnt_;
#else
    checksum += FRAME_SOF1;
    checksum += FRAME_SOF2;
    checksum += frame_len_;
    checksum += FRAME_TYPE_STATUS;
    checksum += frame_id_;
    for (size_t i = 0; i < payload_buffer_.size(); ++i)
        checksum += payload_buffer_[i];
    checksum += frame_cnt_;
#endif

    return checksum;
}

void ScoutSerialParser::PrintStatistics()
{
    std::cout << "total bytes received: " << total_byte_received_ << " , parsed frames: " << frame_parsed_
              << " , wrong frames: " << frame_with_wrong_checksum_ << " , total frames: " << frame_with_wrong_checksum_ + frame_parsed_ << std::endl;
}

void ScoutSerialParser::ParseChar(uint8_t c)
{
    bool new_frame_parsed = false;
    switch (decode_state_)
    {
    case DecodeState::WAIT_FOR_SOF1:
    {
        if (c == FRAME_SOF1)
        {
            frame_id_ = FRAME_NONE_ID;
            frame_len_ = 0;
            frame_cnt_ = 0;
            frame_checksum_ = 0;
            internal_checksum_ = 0;
            payload_buffer_.clear();

            decode_state_ = DecodeState::WAIT_FOR_SOF2;
#ifdef PRINT_DEBUG_INFO
            std::cout << "found sof1" << std::endl;
#endif
        }
        break;
    }
    case DecodeState::WAIT_FOR_SOF2:
    {
        if (c == FRAME_SOF2)
        {
            decode_state_ = DecodeState::WAIT_FOR_FRAME_LEN;
#ifdef PRINT_DEBUG_INFO
            std::cout << "found sof2" << std::endl;
#endif
        }
        else
        {
            decode_state_ = DecodeState::WAIT_FOR_SOF1;
#ifdef PRINT_DEBUG_INFO
            std::cout << "failed to find sof2" << std::endl;
#endif
        }
        break;
    }
    case DecodeState::WAIT_FOR_FRAME_LEN:
    {
        frame_len_ = c;
        decode_state_ = DecodeState::WAIT_FOR_FRAME_TYPE;
#ifdef PRINT_DEBUG_INFO
        std::cout << "frame len: " << std::hex << static_cast<int>(frame_len_) << std::dec << std::endl;
#endif
        break;
    }
    case DecodeState::WAIT_FOR_FRAME_TYPE:
    {
        switch (c)
        {
        case FRAME_TYPE_STATUS:
        {
            decode_state_ = DecodeState::WAIT_FOR_FRAME_ID;
#ifdef PRINT_DEBUG_INFO
            std::cout << "status type frame received" << std::endl;
#endif
            break;
        }
        default:
        {
#ifdef PRINT_DEBUG_INFO
            std::cerr << "ERROR: Not expecting frame of a type other than FRAME_TYPE_STATUS" << std::endl;
#endif
            decode_state_ = DecodeState::WAIT_FOR_SOF1;
        }
        }
        break;
    }
    case DecodeState::WAIT_FOR_FRAME_ID:
    {
        switch (c)
        {
        case FRAME_SYSTEM_STATUS_ID:
        case FRAME_MOTION_STATUS_ID:
        case FRAME_MOTOR1_DRIVER_STATUS_ID:
        case FRAME_MOTOR2_DRIVER_STATUS_ID:
        case FRAME_MOTOR3_DRIVER_STATUS_ID:
        case FRAME_MOTOR4_DRIVER_STATUS_ID:
        case FRAME_LIGHT_STATUS_ID:
        {
            frame_id_ = c;
            decode_state_ = DecodeState::WAIT_FOR_PAYLOAD;
#ifdef PRINT_DEBUG_INFO
            std::cout << "frame id: " << std::hex << static_cast<int>(frame_id_) << std::dec << std::endl;
#endif
            break;
        }
        default:
        {
#ifdef PRINT_DEBUG_INFO
            std::cerr << "ERROR: Unknown frame id" << std::endl;
#endif
            decode_state_ = DecodeState::WAIT_FOR_SOF1;
        }
        }
        break;
    }
    case DecodeState::WAIT_FOR_PAYLOAD:
    {
        payload_buffer_.push_back(c);
#ifdef PRINT_DEBUG_INFO
        std::cout << "1 byte added: " << std::hex << static_cast<int>(c) << std::dec << std::endl;
#endif
        if (payload_buffer_.size() == (frame_len_ - FRAME_FIXED_FIELD_LEN))
            decode_state_ = DecodeState::WAIT_FOR_FRAME_COUNT;
        break;
    }
    case DecodeState::WAIT_FOR_FRAME_COUNT:
    {
        frame_cnt_ = c;
        decode_state_ = DecodeState::WAIT_FOR_CHECKSUM;
#ifdef PRINT_DEBUG_INFO
        std::cout << "frame count: " << std::hex << static_cast<int>(frame_cnt_) << std::dec << std::endl;
#endif
        break;
    }
    case DecodeState::WAIT_FOR_CHECKSUM:
    {
        frame_checksum_ = c;
        internal_checksum_ = CalcBufferedFrameChecksum();
        new_frame_parsed = true;
        decode_state_ = DecodeState::WAIT_FOR_SOF1;
#ifdef PRINT_DEBUG_INFO
        std::cout << "--- frame checksum: " << std::hex << static_cast<int>(frame_checksum_) << std::dec << std::endl;
        std::cout << "--- internal frame checksum: " << std::hex << static_cast<int>(internal_checksum_) << std::dec << std::endl;
#endif
        break;
    }
    default:
        break;
    }

    if (new_frame_parsed)
    {
        if (frame_checksum_ == internal_checksum_)
        {
#ifdef PRINT_DEBUG_INFO
            std::cout << "checksum correct" << std::endl;
#endif
            auto msg = ConstructMessage();
            ++frame_parsed_;
            ProcessFrame(msg);
        }
        else
        {
            ++frame_with_wrong_checksum_;
#ifdef PRINT_DEBUG_INFO
            std::cout << "checksum is NOT correct" << std::endl;
            std::cout << std::hex << static_cast<int>(frame_id_) << " , " << static_cast<int>(frame_len_) << " , " << static_cast<int>(frame_cnt_) << " , " << static_cast<int>(frame_checksum_) << " : " << std::dec << std::endl;
            std::cout << "payload: ";
            for (int i = 0; i < payload_buffer_.size(); ++i)
                std::cout << std::hex << static_cast<int>(payload_buffer_[i]) << std::dec << " ";
            std::cout << std::endl;
            std::cout << "--- frame checksum: " << std::hex << static_cast<int>(frame_checksum_) << std::dec << std::endl;
            std::cout << "--- internal frame checksum: " << std::hex << static_cast<int>(internal_checksum_) << std::dec << std::endl;
#endif
        }
    }
}

ScoutStatusMessage ScoutSerialParser::ConstructMessage()
{
    ScoutStatusMessage msg;
    switch (frame_id_)
    {
    case FRAME_SYSTEM_STATUS_ID:
    {
        msg.updated_msg_type = ScoutSystemStatusMsg;
        msg.system_status_msg.msg.status.base_state = payload_buffer_[0];
        msg.system_status_msg.msg.status.control_mode = payload_buffer_[1];
        msg.system_status_msg.msg.status.battery_voltage.high_byte = payload_buffer_[2];
        msg.system_status_msg.msg.status.battery_voltage.low_byte = payload_buffer_[3];
        msg.system_status_msg.msg.status.fault_code.high_byte = payload_buffer_[4];
        msg.system_status_msg.msg.status.fault_code.low_byte = payload_buffer_[5];
        msg.system_status_msg.msg.status.count = frame_cnt_;
        msg.system_status_msg.msg.status.checksum = frame_checksum_;
        break;
    }
    case FRAME_MOTION_STATUS_ID:
    {
        msg.updated_msg_type = ScoutMotionStatusMsg;
        msg.motion_status_msg.msg.status.linear_velocity.high_byte = payload_buffer_[0];
        msg.motion_status_msg.msg.status.linear_velocity.low_byte = payload_buffer_[1];
        msg.motion_status_msg.msg.status.angular_velocity.high_byte = payload_buffer_[2];
        msg.motion_status_msg.msg.status.angular_velocity.low_byte = payload_buffer_[3];
        msg.motion_status_msg.msg.status.reserved0 = 0x00;
        msg.motion_status_msg.msg.status.reserved0 = 0x00;
        msg.motion_status_msg.msg.status.count = frame_cnt_;
        msg.motion_status_msg.msg.status.checksum = frame_checksum_;
        break;
    }
    case FRAME_MOTOR1_DRIVER_STATUS_ID:
    {
        msg.updated_msg_type = ScoutMotor1DriverStatusMsg;
        msg.motor_driver_status_msg.msg.status.current.high_byte = payload_buffer_[0];
        msg.motor_driver_status_msg.msg.status.current.low_byte = payload_buffer_[1];
        msg.motor_driver_status_msg.msg.status.rpm.high_byte = payload_buffer_[2];
        msg.motor_driver_status_msg.msg.status.rpm.low_byte = payload_buffer_[3];
        msg.motor_driver_status_msg.msg.status.temperature = payload_buffer_[4];
        ;
        msg.motor_driver_status_msg.msg.status.reserved0 = 0x00;
        msg.motor_driver_status_msg.msg.status.count = frame_cnt_;
        msg.motor_driver_status_msg.msg.status.checksum = frame_checksum_;
        break;
    }
    case FRAME_MOTOR2_DRIVER_STATUS_ID:
    {
        msg.updated_msg_type = ScoutMotor2DriverStatusMsg;
        msg.motor_driver_status_msg.msg.status.current.high_byte = payload_buffer_[0];
        msg.motor_driver_status_msg.msg.status.current.low_byte = payload_buffer_[1];
        msg.motor_driver_status_msg.msg.status.rpm.high_byte = payload_buffer_[2];
        msg.motor_driver_status_msg.msg.status.rpm.low_byte = payload_buffer_[3];
        msg.motor_driver_status_msg.msg.status.temperature = payload_buffer_[4];
        ;
        msg.motor_driver_status_msg.msg.status.reserved0 = 0x00;
        msg.motor_driver_status_msg.msg.status.count = frame_cnt_;
        msg.motor_driver_status_msg.msg.status.checksum = frame_checksum_;
        break;
    }
    case FRAME_MOTOR3_DRIVER_STATUS_ID:
    {
        msg.updated_msg_type = ScoutMotor3DriverStatusMsg;
        msg.motor_driver_status_msg.msg.status.current.high_byte = payload_buffer_[0];
        msg.motor_driver_status_msg.msg.status.current.low_byte = payload_buffer_[1];
        msg.motor_driver_status_msg.msg.status.rpm.high_byte = payload_buffer_[2];
        msg.motor_driver_status_msg.msg.status.rpm.low_byte = payload_buffer_[3];
        msg.motor_driver_status_msg.msg.status.temperature = payload_buffer_[4];
        ;
        msg.motor_driver_status_msg.msg.status.reserved0 = 0x00;
        msg.motor_driver_status_msg.msg.status.count = frame_cnt_;
        msg.motor_driver_status_msg.msg.status.checksum = frame_checksum_;
        break;
    }
    case FRAME_MOTOR4_DRIVER_STATUS_ID:
    {
        msg.updated_msg_type = ScoutMotor4DriverStatusMsg;
        msg.motor_driver_status_msg.msg.status.current.high_byte = payload_buffer_[0];
        msg.motor_driver_status_msg.msg.status.current.low_byte = payload_buffer_[1];
        msg.motor_driver_status_msg.msg.status.rpm.high_byte = payload_buffer_[2];
        msg.motor_driver_status_msg.msg.status.rpm.low_byte = payload_buffer_[3];
        msg.motor_driver_status_msg.msg.status.temperature = payload_buffer_[4];
        ;
        msg.motor_driver_status_msg.msg.status.reserved0 = 0x00;
        msg.motor_driver_status_msg.msg.status.count = frame_cnt_;
        msg.motor_driver_status_msg.msg.status.checksum = frame_checksum_;
        break;
    }
    case FRAME_LIGHT_STATUS_ID:
    {
        msg.updated_msg_type = ScoutLightStatusMsg;
        msg.light_status_msg.msg.status.light_ctrl_enable = payload_buffer_[0];
        msg.light_status_msg.msg.status.front_light_mode = payload_buffer_[1];
        msg.light_status_msg.msg.status.front_light_custom = payload_buffer_[2];
        msg.light_status_msg.msg.status.rear_light_mode = payload_buffer_[3];
        msg.light_status_msg.msg.status.rear_light_custom = payload_buffer_[4];
        ;
        msg.light_status_msg.msg.status.reserved0 = 0x00;
        msg.light_status_msg.msg.status.count = frame_cnt_;
        msg.light_status_msg.msg.status.checksum = frame_checksum_;
        break;
    }
    }
    return msg;
}

void ScoutSerialParser::ParseBuffer(uint8_t *buf, size_t len)
{
    for (int i = 0; i < len; ++i)
        ParseChar(buf[i]);
    total_byte_received_ += len;
}

void ScoutSerialParser::DefaultReceiveCallback(const ScoutStatusMessage &msg)
{
    switch (msg.updated_msg_type)
    {
    case ScoutMotionStatusMsg:
    {
        std::cout << "motion control feedback received" << std::endl;
        break;
    }
    case ScoutLightStatusMsg:
    {
        std::cout << "light control feedback received" << std::endl;
        break;
    }
    case ScoutSystemStatusMsg:
    {
        std::cout << "system status feedback received" << std::endl;
        break;
    }
    case ScoutMotor1DriverStatusMsg:
    {
        std::cout << "motor 1 driver feedback received" << std::endl;
        break;
    }
    case ScoutMotor2DriverStatusMsg:
    {
        std::cout << "motor 2 driver feedback received" << std::endl;
        break;
    }
    case ScoutMotor3DriverStatusMsg:
    {
        std::cout << "motor 3 driver feedback received" << std::endl;
        break;
    }
    case ScoutMotor4DriverStatusMsg:
    {
        std::cout << "motor 4 driver feedback received" << std::endl;
        break;
    }
    }
}
} // namespace wescore