#include "scout/scout_can_protocol.hpp"

using namespace wescore;

int main()
{
    uint8_t data1[8] = {1, 2, 3};
    uint8_t data2[8] = {0, 1, 2, 3, 4, 5, 6, 7};

    std::cout << "control modes: " << ScoutCANProtocol::MotionControlDef::REMOTE_MODE
              << " max speed " << static_cast<int>(ScoutCANProtocol::MotionControlDef::max_angular_speed) << std::endl;

    std::cout << "motion ctrl msg count: " << static_cast<int>(ScoutCANProtocol::MotionControlMessage::count) << std::endl;
    std::cout << "motion ctrl msg count: " << static_cast<int>(ScoutCANProtocol::MotionControlMessage::count) << std::endl;

    std::cout << "light ctrl msg count: " << static_cast<int>(ScoutCANProtocol::LightControlMessage::count) << std::endl;
    std::cout << "light ctrl msg count: " << static_cast<int>(ScoutCANProtocol::LightControlMessage::count) << std::endl;

    ScoutCANProtocol::MotionControlMessage msg1(data1);
    ScoutCANProtocol::LightControlMessage msg2(data2);

    ScoutCANProtocol::MotionControlMessage msg3(data1);
    ScoutCANProtocol::LightControlMessage msg4(data2);

    ScoutCANProtocol::LightControlMessage msg5 = msg4;
    ScoutCANProtocol::LightControlMessage msg6 = ScoutCANProtocol::LightControlMessage(data2);

    std::cout << "motion ctrl msg count: " << static_cast<int>(ScoutCANProtocol::MotionControlMessage::count) << std::endl;
    std::cout << "light ctrl msg count: " << static_cast<int>(ScoutCANProtocol::LightControlMessage::count) << std::endl;

    ScoutCANProtocol::MotionControlMessage msg7(0, 0);
    ScoutCANProtocol::MotionControlMessage msg8(0.15, 0);
    ScoutCANProtocol::MotionControlMessage msg9(0, 0.07853);

    std::cout << msg7 << std::endl;
    std::cout << msg8 << std::endl;
    std::cout << msg9 << std::endl;

    // MotionControlMessage::

    return 0;
}