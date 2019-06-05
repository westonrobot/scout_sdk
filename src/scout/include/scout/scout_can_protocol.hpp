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

#include <cstring>
#include <cstdint>
#include <iostream>

namespace wescore
{
namespace ScoutCAN
{
enum ScoutCANMessages
{
    SCOUT_CAN_MOTION_CONTROL = 0x130,
    SCOUT_CAN_MOTION_CONTROL_FEEDBACK = 0x131,
    SCOUT_CAN_LIGHT_CONTROL = 0x140,
    SCOUT_CAN_LIGHT_CONTROL_FEEDBACK = 0x141,
    SCOUT_CAN_SYSTEM_STATUS_FEEDBACK = 0x151,
    SCOUT_CAN_MOTOR1_DRIVER_FEEDBACK = 0x201,
    SCOUT_CAN_MOTOR2_DRIVER_FEEDBACK = 0x202,
    SCOUT_CAN_MOTOR3_DRIVER_FEEDBACK = 0x203,
    SCOUT_CAN_MOTOR4_DRIVER_FEEDBACK = 0x204,
    SCOUT_CAN_LAST
};

template <typename T>
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

    friend std::ostream &operator<<(std::ostream &os, const T &frame)
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

struct MotionControlMessage : ScoutCANFrame<MotionControlMessage>
{
    MotionControlMessage() : ScoutCANFrame<MotionControlMessage>(SCOUT_CAN_MOTION_CONTROL, 0x08) {}
    MotionControlMessage(uint8_t _data[8]) : ScoutCANFrame<MotionControlMessage>(SCOUT_CAN_MOTION_CONTROL, 0x08, _data) {}
};

struct LightControlMessage : ScoutCANFrame<LightControlMessage>
{
    LightControlMessage() : ScoutCANFrame<LightControlMessage>(SCOUT_CAN_LIGHT_CONTROL, 0x08) {}
    LightControlMessage(uint8_t _data[8]) : ScoutCANFrame<LightControlMessage>(SCOUT_CAN_LIGHT_CONTROL, 0x08, _data) {}
};

/*--------------------------------------------------------------------------*/
/*---------------------------- Feedback Messages ---------------------------*/
/*--------------------------------------------------------------------------*/

struct MotionControlFeedbackMessage : ScoutCANFrame<MotionControlFeedbackMessage>
{
    // static constexpr uint32_t id = 0x131;
    // static constexpr uint8_t dlc = 0x08;
};

struct LightControlFeedbackMessage : ScoutCANFrame<LightControlFeedbackMessage>
{
    // static constexpr uint32_t id = 0x141;
    // static constexpr uint8_t dlc = 0x08;
};

struct SystemStatusFeedbackMessage : ScoutCANFrame<SystemStatusFeedbackMessage>
{
    // static constexpr uint32_t id = 0x151;
    // static constexpr uint8_t dlc = 0x08;
};

struct Motor1FeedbackMessage : ScoutCANFrame<Motor1FeedbackMessage>
{
    // static constexpr uint32_t id = 0x201;
    // static constexpr uint8_t dlc = 0x08;
};

struct Motor2FeedbackMessage : ScoutCANFrame<Motor1FeedbackMessage>
{
    //     static constexpr uint32_t id = 0x202;
    //     static constexpr uint8_t dlc = 0x08;
};

struct Motor3FeedbackMessage : ScoutCANFrame<Motor1FeedbackMessage>
{
    // static constexpr uint32_t id = 0x203;
    // static constexpr uint8_t dlc = 0x08;
};

struct Motor4FeedbackMessage : ScoutCANFrame<Motor1FeedbackMessage>
{
    // static constexpr uint32_t id = 0x204;
    // static constexpr uint8_t dlc = 0x08;
};
}; // namespace ScoutCAN
} // namespace wescore

#endif /* SCOUT_CAN_PROTOCOL_HPP */
