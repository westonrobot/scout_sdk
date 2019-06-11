/* 
 * scout_can_protocol.hpp
 * 
 * Created on: Jun 05, 2019 02:36
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_CAN_PROTOCOL_HPP
#define SCOUT_CAN_PROTOCOL_HPP

#include <linux/can.h>

#include <atomic>
#include <cstring>
#include <cstdint>
#include <cassert>
#include <iostream>

namespace wescore
{
enum ScoutCANMsgIDs
{
    MSG_MOTION_CONTROL_ID = 0x130,
    MSG_MOTION_CONTROL_FEEDBACK_ID = 0x131,
    MSG_LIGHT_CONTROL_ID = 0x140,
    MSG_LIGHT_CONTROL_FEEDBACK_ID = 0x141,
    MSG_SYSTEM_STATUS_FEEDBACK_ID = 0x151,
    MSG_MOTOR1_DRIVER_FEEDBACK_ID = 0x201,
    MSG_MOTOR2_DRIVER_FEEDBACK_ID = 0x202,
    MSG_MOTOR3_DRIVER_FEEDBACK_ID = 0x203,
    MSG_MOTOR4_DRIVER_FEEDBACK_ID = 0x204,
    MSG_LAST_ID
};

class ScoutCANFrame
{
protected:
    ScoutCANFrame() : id(0), dlc(0){};
    ScoutCANFrame(uint32_t _id, uint8_t _dlc) : id(_id), dlc(_dlc) {}
    ScoutCANFrame(uint32_t _id, uint8_t _dlc, uint8_t _data[8]) : id(_id), dlc(_dlc)
    {
        std::memcpy(data, _data, _dlc * sizeof(uint8_t));
    }

    const uint32_t id;
    const int dlc;
    uint8_t data[8] = {0};

    uint8_t GenCheckSum() const
    {
        uint8_t checksum = 0x00;
        checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
        for (int i = 0; i < (dlc - 1); ++i)
            checksum += data[i];
        return checksum;
    }

    friend std::ostream &operator<<(std::ostream &os, const ScoutCANFrame &frame)
    {
        std::cout << "ID: " << std::hex << frame.id << ", DLC: " << static_cast<int>(frame.dlc) << ", Data: ";
        for (int i = 0; i < 8; ++i)
            std::cout << static_cast<int>(frame.data[i]) << " ";
        std::cout << ", Checksum: " << std::dec << static_cast<int>(frame.GenCheckSum());
    }
};

/*--------------------------------------------------------------------------*/
/*---------------------------- Control Messages ----------------------------*/
/*--------------------------------------------------------------------------*/

/*------------------------- Motion Control Message -------------------------*/
enum class ControlMode
{
    REMOTE_MODE = 0x00,
    CMD_MODE = 0x01
};

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

struct MotionControlMessage : ScoutCANFrame
{
    MotionControlMessage() : ScoutCANFrame(MSG_MOTION_CONTROL_ID, 0x08) {}

    MotionControlMessage(double linear, double angular,
                         FaultClearFlag fault_clr_flag = FaultClearFlag::NO_FAULT)
        : ScoutCANFrame(MSG_MOTION_CONTROL_ID, 0x08), linear_velocity(linear),
          angular_velocity(angular), fault_clear_flag(fault_clr_flag)
    {
        gen();
    }

    ControlMode control_mode = ControlMode::CMD_MODE;
    FaultClearFlag fault_clear_flag = FaultClearFlag::NO_FAULT;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    static std::atomic<uint8_t> count;
    static constexpr double max_linear_velocity = 1.5;      // 1.5m/s
    static constexpr double min_linear_velocity = -1.5;     // -1.5m/s
    static constexpr double max_angular_velocity = 0.7853;  // 0.7853rad/s
    static constexpr double min_angular_velocity = -0.7853; // -0.7853rad/s

    void gen()
    {
        if (linear_velocity < min_linear_velocity)
            linear_velocity = min_linear_velocity;
        if (linear_velocity > max_linear_velocity)
            linear_velocity = max_linear_velocity;
        if (angular_velocity < min_angular_velocity)
            angular_velocity = min_angular_velocity;
        if (angular_velocity > max_angular_velocity)
            angular_velocity = max_angular_velocity;

        data[0] = static_cast<uint8_t>(control_mode);
        data[1] = static_cast<uint8_t>(fault_clear_flag);
        data[2] = static_cast<uint8_t>(linear_velocity / max_linear_velocity * 100.0);
        data[3] = static_cast<uint8_t>(angular_velocity / max_angular_velocity * 100.0);
        data[4] = 0;
        data[5] = 0;
        data[6] = MotionControlMessage::count;
        data[7] = GenCheckSum();

        count.fetch_add(1);
    }

    can_frame to_frame() const
    {
        can_frame frame;
        frame.can_id = id;
        frame.can_dlc = dlc;
        std::memcpy(frame.data, data, dlc * sizeof(uint8_t));
        return frame;
    }
};

/*-------------------------- Light Control Message -------------------------*/
enum LightControlFlag
{
    DISABLE = 0x00,
    ENABLE_FRONT = 0x01
};

enum LightModes
{
    CONST_OFF = 0x00,
    CONST_ON = 0x01,
    BREATH = 0x02,
    CUSTOM = 0x03
};

struct LightControlMessage : ScoutCANFrame
{
    union MsgDef {

        uint8_t _data[8];
    };

    LightControlMessage() : ScoutCANFrame(MSG_LIGHT_CONTROL_ID, 0x08)
    {
        count.fetch_add(1);
    }

    LightControlMessage(uint8_t _data[8]) : ScoutCANFrame(MSG_LIGHT_CONTROL_ID, 0x08, _data)
    {
        count.fetch_add(1);
    }

    static std::atomic<uint8_t> count;
    static constexpr uint8_t max_light_custom_value = 100;
    static constexpr uint8_t min_light_custom_value = 0;
};

/*--------------------------------------------------------------------------*/
/*---------------------------- Feedback Messages ---------------------------*/
/*--------------------------------------------------------------------------*/

struct MotionControlFeedbackMessage : ScoutCANFrame
{
    // static constexpr uint32_t id = 0x131;
    // static constexpr uint8_t dlc = 0x08;
};

// struct LightControlFeedbackMessage : ScoutCANFrame<LightControlFeedbackMessage>
// {
//     // static constexpr uint32_t id = 0x141;
//     // static constexpr uint8_t dlc = 0x08;
// };

// struct SystemStatusFeedbackMessage : ScoutCANFrame<SystemStatusFeedbackMessage>
// {
//     // static constexpr uint32_t id = 0x151;
//     // static constexpr uint8_t dlc = 0x08;
// };

// struct Motor1FeedbackMessage : ScoutCANFrame<Motor1FeedbackMessage>
// {
//     // static constexpr uint32_t id = 0x201;
//     // static constexpr uint8_t dlc = 0x08;
// };

// struct Motor2FeedbackMessage : ScoutCANFrame<Motor1FeedbackMessage>
// {
//     //     static constexpr uint32_t id = 0x202;
//     //     static constexpr uint8_t dlc = 0x08;
// };

// struct Motor3FeedbackMessage : ScoutCANFrame<Motor1FeedbackMessage>
// {
//     // static constexpr uint32_t id = 0x203;
//     // static constexpr uint8_t dlc = 0x08;
// };

// struct Motor4FeedbackMessage : ScoutCANFrame<Motor1FeedbackMessage>
// {
//     // static constexpr uint32_t id = 0x204;
//     // static constexpr uint8_t dlc = 0x08;
// };
} // namespace wescore

#endif /* SCOUT_CAN_PROTOCOL_HPP */
