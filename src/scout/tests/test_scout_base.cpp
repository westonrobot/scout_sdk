#include <unistd.h>

#include <thread>
#include <mutex>
#include <functional>
#include <string>
#include <iostream>
#include <chrono>
#include <cmath>

#include "stopwatch/stopwatch.h"

#include "scout/scout_base.hpp"

#define TEST_WITHOUT_SERIAL_HARDWARE

using namespace wescore;

int main(int argc, char **argv)
{
    ScoutBase scout;
    scout.ConnectCANBus("can1");
    scout.StartCmdThread(10);

    bool light_on = false;
    while (true)
    {
        scout.SetMotionCommand(0.15, 0);

        if (!light_on)
            scout.SetLightCommand({ScoutLightCmd::LightMode::CONST_ON, 0, ScoutLightCmd::LightMode::CONST_ON, 0});
        sleep(1);
    }

    return 0;
}