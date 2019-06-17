/* 
 * scout_monitor.hpp
 * 
 * Created on: Jun 12, 2019 01:19
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MONITOR_HPP
#define SCOUT_MONITOR_HPP

#include <ncurses.h>

namespace wescore
{
class ScoutMonitor
{
public:
    ScoutMonitor();
    ~ScoutMonitor();

    void Run();
    void Terminate() { keep_running_ = false; }

private:
    bool keep_running_ = true;
    void Update();
};
} // namespace wescore

#endif /* SCOUT_MONITOR_HPP */
