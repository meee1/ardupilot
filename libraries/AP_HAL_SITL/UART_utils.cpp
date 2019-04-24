/*
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "UARTDriver.h"

#if defined(__CYGWIN__) || defined(__CYGWIN64__) || defined(__APPLE__)
#define USE_TERMIOS
#endif

#ifdef USE_TERMIOS
#include <termios.h>
#else
//#include <asm/termbits.h>
#endif

#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

bool HALSITL::UARTDriver::set_speed(int speed)
{
    if (_fd < 0) {
        return false;
    }
#ifdef USE_TERMIOS
    struct termios t;
    tcgetattr(_fd, &t);
    cfsetspeed(&t, speed);
    tcsetattr(_fd, TCSANOW, &t);
#else

#endif

    return true;
}

void HALSITL::UARTDriver::configure_parity(uint8_t v)
{
    if (_fd < 0) {
        return;
    }
#ifdef USE_TERMIOS
    struct termios t;

    tcgetattr(_fd, &t);
#else

#endif
    if (v != 0) {
        // enable parity
 //       t.c_cflag |= PARENB;
        if (v == 1) {
    //        t.c_cflag |= PARODD;
        } else {
    //        t.c_cflag &= ~PARODD;
        }
    }
    else {
        // disable parity
    //    t.c_cflag &= ~PARENB;
    }

#ifdef USE_TERMIOS
    tcsetattr(_fd, TCSANOW, &t);
#else
  //  ioctl(_fd, TCSETS2, &t);
#endif
}

void HALSITL::UARTDriver::set_stop_bits(int n)
{
    if (_fd < 0) {
        return;
    }
#ifdef USE_TERMIOS
    struct termios t;

    tcgetattr(_fd, &t);
#else

#endif

    if (n > 1) {
  //      t.c_cflag |= CSTOPB;
    } else {
    //    t.c_cflag &= ~CSTOPB;
    }

#ifdef USE_TERMIOS
    tcsetattr(_fd, TCSANOW, &t);
#else
 
#endif
}

#endif
