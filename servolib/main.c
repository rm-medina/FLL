/**
 * @file servolib/main.c
 * @brief demo application to exercise servolib
 *
 * @author:
 *     Raquel M. Medina Rodriguez <raquel.medina.rodriguez@gmail.com>
 *
 * servolib provides POSIX functions to control servos connected to a 
 * 6 channel USB Maestro servo controller. The controller serial mode is expected to be "USB Dual Port", in this mode the baud rate is 9600bps.
 *
 * => the servo channel id range is 0 to 5
 * => the servo's range is 942 to 2000
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <getopt.h>
#include <errno.h>

#define SERVO_CHANNEL_MAX 6

extern int verbose;

#define printerr(s, ...)                                \
        fptrintf(stderr, "%s:%d %s: " s "\n", __FILE__, \
		 
