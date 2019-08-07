/* 
 * scout_serial_parser.hpp
 * 
 * Created on: Jul 24, 2019 23:47
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_SERIAL_PARSER_HPP
#define SCOUT_SERIAL_PARSER_HPP

#include <cstdint>
#include <string>
#include <functional>

#include "scout_base/scout_protocol.h"

namespace wescore
{
class ScoutSerialParser
{
    enum class DecodeState
    {
        WAIT_FOR_SOF1 = 0,
        WAIT_FOR_SOF2,
        WAIT_FOR_FRAME_LEN,
        WAIT_FOR_FRAME_TYPE,
        WAIT_FOR_FRAME_ID,
        WAIT_FOR_PAYLOAD,
        WAIT_FOR_FRAME_COUNT,
        WAIT_FOR_CHECKSUM
    };

    static constexpr uint8_t FRAME_SOF_LEN = 2;
    static constexpr uint8_t FRAME_FIXED_FIELD_LEN = 4;

    static constexpr uint8_t FRAME_SOF1 = 0x5a;
    static constexpr uint8_t FRAME_SOF2 = 0xa5;

    static constexpr uint8_t FRAME_TYPE_CMD = 0x55;
    static constexpr uint8_t FRAME_TYPE_STATUS = 0xaa;

    static constexpr uint8_t FRAME_NONE_ID = 0x00;

    static constexpr uint8_t FRAME_SYSTEM_STATUS_ID = 0x01;
    static constexpr uint8_t FRAME_MOTION_STATUS_ID = 0x02;
    static constexpr uint8_t FRAME_MOTOR1_DRIVER_STATUS_ID = 0x03;
    static constexpr uint8_t FRAME_MOTOR2_DRIVER_STATUS_ID = 0x04;
    static constexpr uint8_t FRAME_MOTOR3_DRIVER_STATUS_ID = 0x05;
    static constexpr uint8_t FRAME_MOTOR4_DRIVER_STATUS_ID = 0x06;
    static constexpr uint8_t FRAME_LIGHT_STATUS_ID = 0x07;

public:
    static constexpr uint8_t FRAME_MOTION_CONTROL_CMD_ID = 0x01;
    static constexpr uint8_t FRAME_LIGHT_CONTROL_CMD_ID = 0x02;

    using ReceiveCallback = std::function<void(const ScoutStatusMessage &msg)>;

public:
    void ParseBuffer(uint8_t *buf, size_t len);
    void SetReceiveCallback(ReceiveCallback cb) { ProcessFrame = cb; }

    static uint8_t CalcChecksum(uint8_t *buf, uint8_t len);
    static void PackMotionControlMsgToBuffer(const MotionControlMessage &msg, uint8_t *buf, uint8_t& len);
    static void PackLightControlMsgToBuffer(const LightControlMessage &msg, uint8_t *buf, uint8_t& len);

    void PrintStatistics();

private:
    // frame buffer
    uint8_t frame_id_ = 0;
    uint8_t frame_len_ = 0;
    uint8_t frame_cnt_ = 0;
    uint8_t frame_checksum_ = 0;
    uint8_t internal_checksum_ = 0;
    std::string payload_buffer_;

    // statisctics
    uint32_t total_byte_received_ = 0;
    uint32_t frame_parsed_ = 0;
    uint32_t frame_with_wrong_checksum_ = 0;

    // decoding control variables
    DecodeState decode_state_ = DecodeState::WAIT_FOR_SOF1;
    ReceiveCallback ProcessFrame = std::bind(&ScoutSerialParser::DefaultReceiveCallback, this, std::placeholders::_1);

    void ParseChar(uint8_t c);
    uint8_t CalcBufferedFrameChecksum();
    ScoutStatusMessage ConstructMessage();

    void DefaultReceiveCallback(const ScoutStatusMessage &msg);
};
} // namespace wescore

#endif /* SCOUT_SERIAL_PARSER_HPP */
