/* 
 * scout_monitor.cpp
 * 
 * Created on: Jun 12, 2019 01:19
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

/*
 * Coordinate System:
 *
 *		o --------------------> x
 *		|
 *		|
 *		|
 *		|
 *		|
 *		|
 *		v
 *		y
 */

#include "monitor/scout_monitor.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "stopwatch/stopwatch.h"
#include "monitor/nshapes.hpp"

namespace
{
// reference: https://thispointer.com/c-convert-double-to-string-and-manage-precision-scientific-notation/
std::string ConvertVelocityToString(double vel)
{
    std::ostringstream streamObj;
    streamObj << std::fixed;
    streamObj << std::setprecision(3);
    streamObj << vel;
    return streamObj.str();
}
} // namespace

namespace wescore
{
ScoutMonitor::ScoutMonitor()
{
    // init ncurses
    initscr();
    // raw();
    cbreak();
    noecho();
    nonl();
    curs_set(FALSE);
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);

    CalcDimensions();

    // setup sub-windows
    body_info_win_ = newwin(bi_win_sy_, bi_win_sx_, bi_origin_y_, bi_origin_x_);
    system_info_win_ = newwin(si_win_sy_, si_win_sx_, si_origin_y_, si_origin_x_);
}

ScoutMonitor::~ScoutMonitor()
{
    delwin(body_info_win_);
    delwin(system_info_win_);
    endwin();
}

void ScoutMonitor::UpdateAll()
{
    ClearAll();

    CalcDimensions();
    if (resizing_detected_)
        HandleResizing();

    UpdateScoutBodyInfo();
    UpdateScoutSystemInfo();
}

void ScoutMonitor::Run()
{
    stopwatch::StopWatch sw;
    while (keep_running_)
    {
        // label starting point of iteration
        sw.tic();

        // update window contents
        UpdateAll();

        // manage window refresh rate
        sw.sleep_until_ms(500);
    }
}

void ScoutMonitor::CalcDimensions()
{
    int sy, sx;
    getmaxyx(stdscr, sy, sx);

    if (sy != term_sy_ || sx != term_sx_)
    {
        resizing_detected_ = true;

        term_sy_ = sy;
        term_sx_ = sx;

        bi_win_sy_ = term_sy_ - 3;
        bi_win_sx_ = term_sx_ * 15 / 24;
        bi_origin_y_ = 0;
        bi_origin_x_ = 0;

        si_win_sy_ = term_sy_;
        si_win_sx_ = term_sx_ * 9 / 24;
        si_origin_y_ = 0;
        si_origin_x_ = bi_win_sx_;
    }
}

void ScoutMonitor::HandleResizing()
{
    delwin(body_info_win_);
    delwin(system_info_win_);

    body_info_win_ = newwin(bi_win_sy_, bi_win_sx_, bi_origin_y_, bi_origin_x_);
    system_info_win_ = newwin(si_win_sy_, si_win_sx_, si_origin_y_, si_origin_x_);

    resizing_detected_ = false;
}

void ScoutMonitor::ClearAll()
{
    wclear(body_info_win_);
    wclear(system_info_win_);
}

void ScoutMonitor::DrawVehicle(int y, int x)
{
    // draw linear velocity
    const int linear_axis_x = x + vehicle_fp_offset_x_;
    const int linear_axis_tip_y = y + 2;
    const int linear_axis_origin_y = linear_axis_tip_y + linear_axis_length_;
    const int linear_axis_negative_y = linear_axis_origin_y + linear_axis_length_ + 1;
    mvwprintw(body_info_win_, linear_axis_tip_y - 1, linear_axis_x, "^");
    for (int i = linear_axis_tip_y; i < linear_axis_origin_y; ++i)
        mvwprintw(body_info_win_, i, linear_axis_x, "-");
    mvwprintw(body_info_win_, linear_axis_origin_y, linear_axis_x, "x");
    for (int i = linear_axis_origin_y + 1; i < linear_axis_negative_y; ++i)
        mvwprintw(body_info_win_, i, linear_axis_x, "-");
    mvwprintw(body_info_win_, linear_axis_negative_y, linear_axis_x, "v");

    // draw angular velocity
    const int angular_axis_y = linear_axis_origin_y;
    const int angular_axis_origin_x = linear_axis_x;
    const int angular_axis_positive_x = angular_axis_origin_x + angular_axis_length_ + 1;
    const int angular_axis_negative_x = angular_axis_origin_x - angular_axis_length_;
    mvwprintw(body_info_win_, angular_axis_y, angular_axis_negative_x - 1, "<");
    for (int i = angular_axis_negative_x; i < angular_axis_origin_x; ++i)
        mvwprintw(body_info_win_, angular_axis_y, i, "-");
    mvwprintw(body_info_win_, linear_axis_origin_y, linear_axis_x, "x");
    for (int i = angular_axis_origin_x + 1; i < angular_axis_positive_x; ++i)
        mvwprintw(body_info_win_, angular_axis_y, i, "-");
    mvwprintw(body_info_win_, angular_axis_y, angular_axis_positive_x, ">");

    // draw velocity
    double linear_vel = 1.234;
    std::string linear_vel_str = "linear : " + ConvertVelocityToString(linear_vel);
    mvwprintw(body_info_win_, linear_axis_negative_y + 2, angular_axis_negative_x - 2, linear_vel_str.c_str());

    double angular_vel = 0.123;
    std::string angular_vel_str = "angular: " + ConvertVelocityToString(angular_vel);
    mvwprintw(body_info_win_, linear_axis_negative_y + 3, angular_axis_negative_x - 2, angular_vel_str.c_str());

    // draw vehicle base
    NShapes::DrawRectangle(body_info_win_, linear_axis_tip_y - 2, angular_axis_negative_x - 4,
                           linear_axis_negative_y + 4, angular_axis_positive_x + 3);

    // draw vehicle wheels
    NShapes::DrawRectangle(body_info_win_, linear_axis_tip_y - 1, angular_axis_negative_x - 9,
                           linear_axis_tip_y + 4, angular_axis_negative_x - 5);
    NShapes::DrawRectangle(body_info_win_, linear_axis_negative_y - 2, angular_axis_negative_x - 9,
                           linear_axis_negative_y + 3, angular_axis_negative_x - 5);
    NShapes::DrawRectangle(body_info_win_, linear_axis_tip_y - 1, angular_axis_positive_x + 4,
                           linear_axis_tip_y + 4, angular_axis_positive_x + 8);
    NShapes::DrawRectangle(body_info_win_, linear_axis_negative_y - 2, angular_axis_positive_x + 4,
                           linear_axis_negative_y + 3, angular_axis_positive_x + 8);

    // draw vehicle lights
    for (int i = angular_axis_origin_x - 5; i < angular_axis_origin_x - 1; ++i)
        mvwprintw(body_info_win_, linear_axis_tip_y - 3, i, "\\");
    mvwprintw(body_info_win_, linear_axis_tip_y - 3, angular_axis_origin_x, "|");
    for (int i = angular_axis_origin_x + 2; i <= angular_axis_origin_x + 5; ++i)
        mvwprintw(body_info_win_, linear_axis_tip_y - 3, i, "/");

    for (int i = angular_axis_origin_x - 5; i < angular_axis_origin_x - 1; ++i)
        mvwprintw(body_info_win_, linear_axis_negative_y + 5, i, "/");
    mvwprintw(body_info_win_, linear_axis_negative_y + 5, angular_axis_origin_x, "|");
    for (int i = angular_axis_origin_x + 2; i <= angular_axis_origin_x + 5; ++i)
        mvwprintw(body_info_win_, linear_axis_negative_y + 5, i, "\\");
}

void ScoutMonitor::UpdateScoutBodyInfo()
{
    for (int i = 0; i < bi_win_sx_; i++)
        mvwprintw(body_info_win_, bi_win_sy_ - 1, i, "-");

    DrawVehicle(bi_win_sy_ / 2 - vehicle_fp_offset_y_, bi_win_sx_ / 2 - vehicle_fp_offset_x_);

    wrefresh(body_info_win_);
}

void ScoutMonitor::UpdateScoutSystemInfo()
{
    for (int i = 0; i < si_win_sy_; i++)
        mvwprintw(system_info_win_, i, 0, "|");

    const int start_col = (si_win_sx_ - 24) / 2;
    const int sec1 = static_cast<int>(std::round((si_win_sy_ - 20) / 2.0));
    mvwprintw(system_info_win_, sec1, start_col, "System state     : NORMAL");
    mvwprintw(system_info_win_, sec1 + 1, start_col, "Control mode     : CAN");
    mvwprintw(system_info_win_, sec1 + 2, start_col, "Battery voltage  : 29v");

    const int sec2 = sec1 + 4;
    mvwprintw(system_info_win_, sec2, start_col, "System faults");
    mvwprintw(system_info_win_, sec2 + 1, start_col, "-Drv over-current: N W P");
    mvwprintw(system_info_win_, sec2 + 2, start_col, "-Mt over-heat    : N W P");
    mvwprintw(system_info_win_, sec2 + 3, start_col, "-Bat under volt  : N W F");
    mvwprintw(system_info_win_, sec2 + 4, start_col, "-Bat over volt   : N   F");

    const int sec3 = sec2 + 6;
    mvwprintw(system_info_win_, sec3, start_col, "Comm states");
    mvwprintw(system_info_win_, sec3 + 1, start_col, "-CAN cmd error   : N   F");
    mvwprintw(system_info_win_, sec3 + 2, start_col, "-Motor 1 comm    : N   F");
    mvwprintw(system_info_win_, sec3 + 3, start_col, "-Motor 2 comm    : N   F");
    mvwprintw(system_info_win_, sec3 + 4, start_col, "-Motor 3 comm    : N   F");
    mvwprintw(system_info_win_, sec3 + 5, start_col, "-Motor 4 comm    : N   F");

    const int sec4 = sec3 + 7;
    mvwprintw(system_info_win_, sec4, start_col, " N: normal  W: warning");
    mvwprintw(system_info_win_, sec4 + 1, start_col, " F: fault   P: protection");

    wrefresh(system_info_win_);
}
} // namespace wescore
