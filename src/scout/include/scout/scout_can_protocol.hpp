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

#include <atomic>
#include <cstring>
#include <cstdint>
#include <iostream>

namespace wescore
{
namespace ScoutCANProtocol
{
enum CANMessageIDs
{
    MSG_ID_MOTION_CONTROL = 0x130,
    MSG_ID_MOTION_CONTROL_FEEDBACK = 0x131,
    MSG_ID_LIGHT_CONTROL = 0x140,
    MSG_ID_LIGHT_CONTROL_FEEDBACK = 0x141,
    MSG_ID_SYSTEM_STATUS_FEEDBACK = 0x151,
    MSG_ID_MOTOR1_DRIVER_FEEDBACK = 0x201,
    MSG_ID_MOTOR2_DRIVER_FEEDBACK = 0x202,
    MSG_ID_MOTOR3_DRIVER_FEEDBACK = 0x203,
    MSG_ID_MOTOR4_DRIVER_FEEDBACK = 0x204,
    MSG_ID_LAST
};

struct MotionControlDef
{
    enum ControlModes
    {
        REMOTE_MODE = 0x00,
        CMD_MODE = 0x01
    };

    enum FaultClearFlags
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

    static constexpr double max_linear_speed = 1.5;  // 1.5m/s
    static constexpr double min_linear_speed = -1.5; // -1.5m/s

    static constexpr double max_angular_speed = 0.7853;  // 0.7853rad/s
    static constexpr double min_angular_speed = -0.7853; // -0.7853rad/s
};

struct LightControlDef
{
    enum ControlFlags
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

    static constexpr uint8_t max_light_custom_value = 100;
    static constexpr uint8_t min_light_custom_value = 0;

    static constexpr int8_t max_linear_speed = 100;  // 1.5m/s
    static constexpr int8_t min_linear_speed = -100; // -1.5m/s

    static constexpr int8_t max_angular_speed = 100;  // 0.7853rad/s
    static constexpr int8_t min_angular_speed = -100; // -0.7853rad/s
};

struct ScoutCANFrame
{
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

struct MotionControlMessage : ScoutCANFrame
{
    MotionControlMessage(double linear = 0, double angular = 0,
                         uint8_t fault_clr_flag = MotionControlDef::NO_FAULT)
        : ScoutCANFrame(MSG_ID_MOTION_CONTROL, 0x08)
    {
        count.fetch_add(1);

        if (linear > MotionControlDef::max_linear_speed)
            linear = MotionControlDef::max_linear_speed;
        if (linear < MotionControlDef::min_linear_speed)
            linear = MotionControlDef::min_linear_speed;

        if (angular > MotionControlDef::max_angular_speed)
            angular = MotionControlDef::max_angular_speed;
        if (angular < MotionControlDef::min_angular_speed)
            angular = MotionControlDef::min_angular_speed;

        data[0] = MotionControlDef::CMD_MODE;
        data[1] = fault_clr_flag;
        data[2] = static_cast<uint8_t>(linear / MotionControlDef::max_linear_speed * 100.0);
        data[3] = static_cast<uint8_t>(angular / MotionControlDef::max_angular_speed * 100.0);
        data[6] = MotionControlMessage::count;
        data[7] = GenCheckSum();
    }

    MotionControlMessage(uint8_t _data[8]) : ScoutCANFrame(MSG_ID_MOTION_CONTROL, 0x08, _data)
    {
        count.fetch_add(1);

        data[7] = GenCheckSum();
    }

    static std::atomic<uint8_t> count;
};

struct LightControlMessage : ScoutCANFrame
{
    LightControlMessage() : ScoutCANFrame(MSG_ID_LIGHT_CONTROL, 0x08)
    {
        count.fetch_add(1);
    }

    LightControlMessage(uint8_t _data[8]) : ScoutCANFrame(MSG_ID_LIGHT_CONTROL, 0x08, _data)
    {
        count.fetch_add(1);
    }

    static std::atomic<uint8_t> count;
};

// /*--------------------------------------------------------------------------*/
// /*---------------------------- Feedback Messages ---------------------------*/
// /*--------------------------------------------------------------------------*/

// struct MotionControlFeedbackMessage : ScoutCANFrame<MotionControlFeedbackMessage>
// {
//     // static constexpr uint32_t id = 0x131;
//     // static constexpr uint8_t dlc = 0x08;
// };

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
}; // namespace ScoutCANProtocol
} // namespace wescore

#endif /* SCOUT_CAN_PROTOCOL_HPP */
