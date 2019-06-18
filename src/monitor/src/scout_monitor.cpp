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

#include "stopwatch/stopwatch.h"

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

    CheckWindowDimensions();

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
    // int ch = 'x';

    // printw("Type any character to see it in bold\n");
    // // ch = getch();                 /* If raw() hadn't been called
    // // 				 * we have to press enter before it
    // // 				 * gets to the program 		*/
    // // if (ch == KEY_F(1))           /* Without keypad enabled this will */
    // //     printw("F1 Key pressed"); /*  not get to us either	*/
    // /* Without noecho() some ugly escape
    // 				 * charachters might have been printed
    // 				 * on screen			*/
    // // else
    // // {
    // printw("The pressed key is ");
    // attron(A_BOLD);
    // printw("%c\n", ch);
    // attroff(A_BOLD);
    // // }

    // refresh(); /* Print it on to the real screen */
    // getch();   /* Wait for user input */
    CheckWindowDimensions();
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
        sw.sleep_until_ms(20);
    }
}

void ScoutMonitor::CheckWindowDimensions()
{
    int sy, sx;
    getmaxyx(stdscr, sy, sx);

    if (sy != term_sy_ || sx != term_sx_)
    {
        resizing_detected_ = true;

        term_sy_ = sy;
        term_sx_ = sx;

        bi_win_sy_ = term_sy_ * 2 / 3;
        bi_win_sx_ = term_sx_ * 2 / 3;
        bi_origin_y_ = 0;
        bi_origin_x_ = 0;

        si_win_sy_ = term_sy_ * 2 / 3;
        si_win_sx_ = term_sx_ * 1 / 3;
        si_origin_y_ = 0;
        si_origin_x_ = bi_win_sx_;
    }
}

void ScoutMonitor::HandleResizing()
{
    mvwin(body_info_win_, bi_origin_y_, bi_origin_x_);
    mvwin(system_info_win_, si_origin_y_, si_origin_x_);
    resizing_detected_ = false;
}

void ScoutMonitor::UpdateScoutBodyInfo()
{
    wclear(body_info_win_);

    // std::string display_str = "scout body info win (y,x): " + std::to_string(bi_win_sy_) + " " + std::to_string(bi_win_sx_);
    std::string display_str = "bi (y,x): " + std::to_string(bi_win_sy_) + " " + std::to_string(bi_win_sx_) + " ; si (y,x): " + std::to_string(si_win_sy_) + " " + std::to_string(si_win_sx_);
    mvwprintw(body_info_win_, 0, 0, display_str.c_str());

    std::string display_str2 = "bio (y,x): " + std::to_string(bi_origin_y_) + " " + std::to_string(bi_origin_x_) + " ; sio (y,x): " + std::to_string(si_origin_y_) + " " + std::to_string(si_origin_x_);
    mvwprintw(body_info_win_, 1, 0, display_str2.c_str());

    std::string display_str3 = "term (y,x): " + std::to_string(term_sy_) + " " + std::to_string(term_sx_);
    mvwprintw(body_info_win_, 2, 0, display_str3.c_str());

    int x, y, i;

    // 4 corners
    // mvwprintw(body_info_win_, 0, 0, "+");
    // mvwprintw(body_info_win_, y - 1, 0, "+");
    // mvwprintw(body_info_win_, 0, x - 1, "+");
    // mvwprintw(body_info_win_, y - 1, x - 1, "+");

    // sides for (i = 1; i < (y - 1); i++) { mvwprintw(screen, i, 0, "|"); mvwprintw(screen, i, x - 1, "|"); } // top and bottom for (i = 1; i < (x - 1); i++) { mvwprintw(screen, 0, i, "-"); mvwprintw(screen, y - 1, i, "-"); }

    wrefresh(body_info_win_);
}

void ScoutMonitor::UpdateScoutSystemInfo()
{
    wclear(system_info_win_);

    // mvwprintw(system_info_win_, 0, 0, "12345678901234567890 - system info");

    std::string display_str = "scout system info win (y,x): " + std::to_string(si_win_sy_) + " " + std::to_string(si_win_sx_);
    mvwprintw(system_info_win_, 0, 0, display_str.c_str());

    wrefresh(system_info_win_);
}
} // namespace wescore
