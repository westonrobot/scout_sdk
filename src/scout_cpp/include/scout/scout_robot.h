#ifndef SCOUT_ROBOT_H
#define SCOUT_ROBOT_H

#include <string>
#include <cstdint>

namespace scout
{
struct RobotState
{
    RobotState() : linear(0), angular(0) {}
    RobotState(short _linear, short _angular) : linear(linear), angular(_angular) {}

    short linear;
    short angular;
};

class ScoutRobot
{
public:
    void ConnectSerialPort(const std::string &port_name = "/dev/ttyUSB0", int32_t baud_rate = 115200);
    void ConnectCanBus(const std::string &port_name);

    bool IsConnectionActive() const { return serial_connected_ | can_connected_; }

    void SendCommand(double angular, double linear, uint32_t count);
    bool QueryRobotState(RobotState *data);

private:
    bool serial_connected_ = false;
    bool can_connected_ = false;
};
} // namespace scout

#endif /* SCOUT_ROBOT_H */
