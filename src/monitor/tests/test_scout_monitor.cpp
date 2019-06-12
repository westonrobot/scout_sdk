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

// struct ScoutMessenger
// {
//     void UpdateState(ScoutState *robot_state)
//     {
//         std::cout << "state updated" << std::endl;
//     }

//     void UpdateCommandCallback()
//     {
//         std::cout << "do nothing" << std::endl;
//     }
// };

int main(int argc, char **argv)
{
    ScoutBase scout;

#ifndef TEST_WITHOUT_SERIAL_HARDWARE
    scout.ConnectSerialPort("/dev/ttyUSB0", 115200);

    if (!scout.IsConnectionActive())
    {
        std::cerr << "Failed to connect to robot" << std::endl;
        return -1;
    }
#endif

    scout.ConnectCANBus("can1");

    // ScoutMessenger msg;

    // scout.SetStateUpdateCallback(std::bind(&ScoutMessenger::UpdateState, &msg, std::placeholders::_1));
    // scout.SetControlUpdateFunction(std::bind(&ScoutMessenger::UpdateCommandCallback, &msg));

    // scout.Run();

    // const int32_t ctrl_period_ms = 10;
    // stopwatch::StopWatch main_sw;

    // while (true)
    // {
    //     main_sw.tic();

    //     MotionControlMessage msg;
    //     msg.linear_velocity = 0;
    //     msg.angular_velocity = 0.5;
    //     msg.fault_clear_flag = FaultClearFlag::NO_FAULT;
    //     msg.gen();
    //     // std::cout << msg << std::endl;
    //     scout.SendMotionCommand(msg);

    //     if (main_sw.toc() * 1000 > ctrl_period_ms)
    //         std::cerr << "UpdateControl() took longer than allowable time for an update iteration" << std::endl;
    //     else
    //         main_sw.sleep_until_ms(ctrl_period_ms);
    //     std::cout << "update freq: " << 1.0 / main_sw.toc() << std::endl;
    // }

    scout.StartCmdThread(10);

    while (true)
    {
        scout.SetMotionCommand(0.15, 0);
        sleep(1);
    }

    return 0;
}