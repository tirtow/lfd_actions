#ifndef GUARD_utils_h
#define GUARD_utils_h

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <termios.h>

/**
 * Reads a character without blocking execution
 * Returns the read character
 */
int getch() {
    static struct termios oldt, newt;

    // Saving old settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable buffering
    newt.c_lflag &= ~(ICANON);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;

    // Apply new settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read character without blocking
    int c = getchar();

    // Restore old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return c;
}
