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

    scout.SetLightCommand({ScoutLightCmd::LightMode::CONST_ON, 0, ScoutLightCmd::LightMode::CONST_ON, 0});

    int count = 0;
    while (true)
    {
        scout.SetMotionCommand(0.5, 0.2);

        if(count == 10)
        {
            // scout.SetLightCommand({ScoutLightCmd::LightMode::CONST_OFF, 0, ScoutLightCmd::LightMode::CONST_OFF, 0});
            scout.DisableLightCmdControl();
        }

        sleep(1);
        ++count;
    }

    return 0;
}