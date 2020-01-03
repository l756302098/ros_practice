#pragma once

/* This class is designed to provide easy serial port communication. To that end
 *  I like to think it performs well and is a -relatively- concise solution.
 */

/* This function initializes and opens a serial port for reading/writing. It
 *  uses a comically archaic system for doing so, but the means fit the medium :P
 *  It takes in a device location (/dev/ttyUSBX of /dev/ttySX) and baudrate, and
 *  spits out an INT file descriptor number. From that point it can be read from
 *  and written to like any file.
 *
 *  WARNING: Changing the flags on the init section is particulary... risky.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions, THE BIG ONE
#include <sys/ioctl.h>
#include <getopt.h>

#include <iostream>

class Serial
{
public:
    Serial(std::string serial_port = "/dev/ttyACM0", int baud = 9600);
    ~Serial();
    void clear();
    // void clearBuffer();
    ssize_t Write(const char *msg, size_t bytes);
    ssize_t Read(char *buffer, size_t bytes);
    size_t ReadBlocking(char *buffer, size_t bytes);

private:
    int file_descriptor;
};