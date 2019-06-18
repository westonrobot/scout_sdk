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

#include <iostream>
#include <sstream>
#include <iomanip>

#include "stopwatch/stopwatch.h"

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

        bi_win_sy_ = term_sy_ * 3 / 4;
        bi_win_sx_ = term_sx_ * 2 / 3;
        bi_origin_y_ = 0;
        bi_origin_x_ = 0;

        si_win_sy_ = term_sy_ * 3 / 4;
        si_win_sx_ = term_sx_ * 1 / 3;
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
    std::string linear_vel_str = " linear: " + ConvertVelocityToString(linear_vel);
    mvwprintw(body_info_win_, linear_axis_negative_y + 2, angular_axis_negative_x - 1, linear_vel_str.c_str());

    double angular_vel = 0.123;
    std::string angular_vel_str = "angular: " + ConvertVelocityToString(angular_vel);
    mvwprintw(body_info_win_, linear_axis_negative_y + 3, angular_axis_negative_x - 1, angular_vel_str.c_str());

    // draw vehicle footprint
    for (int i = linear_axis_tip_y - 2; i < linear_axis_negative_y + 5; ++i)
    {
        mvwprintw(body_info_win_, i, angular_axis_negative_x - 4, "|");
        mvwprintw(body_info_win_, i, angular_axis_positive_x + 3, "|");
    }
    for (int i = angular_axis_negative_x - 4; i < angular_axis_positive_x + 4; ++i)
    {
        mvwprintw(body_info_win_, linear_axis_tip_y - 2, i, "-");
        mvwprintw(body_info_win_, linear_axis_negative_y + 4, i, "-");
    }
}

void ScoutMonitor::UpdateScoutBodyInfo()
{
    // static int32_t count = 0;
    // std::string display_str = "scout body info win (y,x): " + std::to_string(bi_win_sy_) + " " + std::to_string(bi_win_sx_);
    // std::string display_str = "bi (y,x): " + std::to_string(bi_win_sy_) + " " + std::to_string(bi_win_sx_) + " ; si (y,x): " + std::to_string(si_win_sy_) + " " + std::to_string(si_win_sx_);
    // mvwprintw(body_info_win_, 0, 0, display_str.c_str());

    // for (int i = 0; i < 10; ++i)
    //     mvwprintw(body_info_win_, 0, i, std::to_string(i).c_str());
    // for (int i = 0; i < 10; ++i)
    //     mvwprintw(body_info_win_, 0, i + 10, std::to_string(i).c_str());
    // for (int i = 0; i < 10; ++i)
    //     mvwprintw(body_info_win_, 0, i + 20, std::to_string(i).c_str());
    // for (int i = 0; i < 10; ++i)
    //     mvwprintw(body_info_win_, 0, i + 30, std::to_string(i).c_str());

    // for (int i = 0; i < 10; ++i)
    //     mvwprintw(body_info_win_, i, 0, std::to_string(i).c_str());
    // for (int i = 0; i < 10; ++i)
    //     mvwprintw(body_info_win_, i + 10, 0, std::to_string(i).c_str());

    // std::string display_str2 = "bio (y,x): " + std::to_string(bi_origin_y_) + " " + std::to_string(bi_origin_x_) + " ; sio (y,x): " + std::to_string(si_origin_y_) + " " + std::to_string(si_origin_x_);
    // mvwprintw(body_info_win_, 1, 0, display_str2.c_str());
    // mvwprintw(body_info_win_, 1, 1, std::to_string(count++).c_str());

    // std::string display_str3 = "term (y,x): " + std::to_string(term_sy_) + " " + std::to_string(term_sx_);
    // mvwprintw(body_info_win_, 2, 1, display_str3.c_str());

    for (int i = 0; i < bi_win_sy_; i++)
        mvwprintw(body_info_win_, i, bi_win_sx_ - 1, "|");

    DrawVehicle(bi_win_sy_ / 2 - vehicle_fp_offset_y_, bi_win_sx_ / 2 - vehicle_fp_offset_x_);
    // DrawVehicle(0, 0);

    wrefresh(body_info_win_);
}

void ScoutMonitor::UpdateScoutSystemInfo()
{
    // mvwprintw(system_info_win_, 0, 0, "12345678901234567890 - system info");
    std::string display_str = "scout system info win (y,x): " + std::to_string(si_win_sy_) + " " + std::to_string(si_win_sx_);
    mvwprintw(system_info_win_, 0, 0, display_str.c_str());

    wrefresh(system_info_win_);
}
} // namespace wescore
