/* 
 * scout_base.hpp
 * 
 * Created on: Jun 04, 2019 01:22
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_BASE_HPP
#define SCOUT_BASE_HPP

#include <string>
#include <cstdint>
#include <functional>

#include "stopwatch/stopwatch.h"
#include "async_io/async_can.hpp"
#include "scout/scout_can_protocol.hpp"

namespace wescore
{
struct ScoutState
{
    ScoutState() : linear(0), angular(0) {}
    ScoutState(short _linear, short _angular) : linear(_linear), angular(_angular) {}

    short linear;
    short angular;
};

struct ScoutCmd
{
    ScoutCmd() : linear(0), angular(0) {}
    ScoutCmd(double _linear, double _angular, uint32_t cnt)
        : linear(_linear), angular(_angular), count(0) {}

    double linear;
    double angular;
    uint32_t count;
};

class ScoutBase
{
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = typename Clock::time_point;

    using StateUpdateCallbackFunc = std::function<void(ScoutState *robot_state)>;
    using ControlUpdateFunc = std::function<void(void)>;

public:
    void ConnectSerialPort(const std::string &port_name = "/dev/ttyUSB0", int32_t baud_rate = 115200);
    void ConnectCANBus(const std::string &can_if_name = "can1");

    bool IsConnectionActive() const { return serial_connected_; }

    void SetStateUpdateCallback(StateUpdateCallbackFunc cb) { UpdateState = cb; }
    void SetControlUpdateFunction(ControlUpdateFunc ctrl_func) { UpdateControl = ctrl_func; }

    // void Run(int32_t loop_period_ms = 10);

    void DisableLightControl();

    void SendMotionCommand(const MotionControlMessage &msg);
    void SendLightCommand();

private:
    bool serial_connected_ = false;

    std::shared_ptr<ASyncCAN> can_if_;

    stopwatch::StopWatch ctrl_loop_stopwatch_;
    TimePoint current_time_;
    TimePoint last_time_;

    StateUpdateCallbackFunc UpdateState = nullptr;
    ControlUpdateFunc UpdateControl = nullptr;

    void SendCommand(const ScoutCmd &cmd);
    bool QueryRobotState(ScoutState *data);
};
} // namespace wescore

#endif /* SCOUT_BASE_HPP */
