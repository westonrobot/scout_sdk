#include "scout/scout_can_protocol.hpp"

using namespace wescore;

int main()
{
    uint8_t data1[8] = {1, 2, 3};
    uint8_t data2[8] = {0, 1, 2, 3, 4, 5, 6, 7};

    ScoutCAN::MotionControlMessage msg1(data1);
    ScoutCAN::LightControlMessage msg2(data2);

    std::cout << msg1 << std::endl;
    std::cout << msg2 << std::endl;

    return 0;
}