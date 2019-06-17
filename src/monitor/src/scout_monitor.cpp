/* 
 * scout_monitor.cpp
 * 
 * Created on: Jun 12, 2019 01:19
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "monitor/scout_monitor.hpp"
#include "stopwatch/stopwatch.h"

namespace wescore
{
ScoutMonitor::ScoutMonitor()
{
    initscr();            /* Start curses mode 		*/
    raw();                /* Line buffering disabled	*/
    keypad(stdscr, TRUE); /* We get F1, F2 etc..		*/
    noecho();             /* Don't echo() while we do getch */
}

ScoutMonitor::~ScoutMonitor()
{
    endwin(); /* End curses mode		  */
}

void ScoutMonitor::Update()
{
    int ch;

    printw("Type any character to see it in bold\n");
    ch = getch();                 /* If raw() hadn't been called
					 * we have to press enter before it
					 * gets to the program 		*/
    if (ch == KEY_F(1))           /* Without keypad enabled this will */
        printw("F1 Key pressed"); /*  not get to us either	*/
    /* Without noecho() some ugly escape
					 * charachters might have been printed
					 * on screen			*/
    else
    {
        printw("The pressed key is ");
        attron(A_BOLD);
        printw("%c\n", ch);
        attroff(A_BOLD);
    }
    refresh(); /* Print it on to the real screen */
    getch();   /* Wait for user input */
}

void ScoutMonitor::Run()
{
    stopwatch::StopWatch sw;
    while (keep_running_)
    {
        sw.tic();
        Update();
        
        refresh(); /* Print it on to the real screen */
        getch();   /* Wait for user input */

        sw.sleep_until_ms(20);
    }
}
} // namespace wescore
