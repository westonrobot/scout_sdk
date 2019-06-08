/* 
 * scout_can_protocol.cpp
 * 
 * Created on: Jun 05, 2019 03:11
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout/scout_can_protocol.hpp"

namespace wescore
{
namespace ScoutCANProtocol
{
std::atomic<uint8_t> MotionControlMessage::count{0};
std::atomic<uint8_t> LightControlMessage::count{0};
} // namespace ScoutCANProtocol
} // namespace wescore