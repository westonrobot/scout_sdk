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

    int term_sx_ = -1;
    int term_sy_ = -1;

    WINDOW *body_info_win_;
    int bi_win_sx_;
    int bi_win_sy_;
    int bi_origin_x_;
    int bi_origin_y_;

    WINDOW *system_info_win_;
    int si_win_sx_;
    int si_win_sy_;
    int si_origin_x_;
    int si_origin_y_;

    WINDOW *scout_cmd_win_;

    bool resizing_detected_;

    void UpdateAll();

    void CheckWindowDimensions();
    void HandleResizing();

    void UpdateScoutBodyInfo();
    void UpdateScoutSystemInfo();
    void UpdateScoutCmdWindow();
};
} // namespace wescore

#endif /* SCOUT_MONITOR_HPP */
