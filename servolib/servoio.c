/**
 * @file servolib/servoio.c
 * @brief API to control servos connected to micro maestro 6channel USB servo 
 * controller.
 *
 * @author:
 *     Raquel M. Medina Rodriguez <raquel.medina.rodriguez@gmail.com>
 *
 * servolib provides POSIX functions to control servos connected to a 
 * 6 channel micro Maestro USB servo controller. The controller serial mode is 
 * expected to be "USB Dual Port", in this mode the baud rate is 9600bps.
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
#include <fcntl.h>
#include "servolib.h"
#define SERVO_CHANNEL_MAX 6

int verbose = 1;

#define printerr(s, ...)				\
	fprintf(stderr, "%s:%d %s" s "\n", __FILE__,	\
		__LINE__, __func__, ##__VA_ARGS__);	\

#define printdbg(s, ...)				\
        do {						\
	       if (verbose)				\
		 printerr(s, ##__VA_ARGS__);		\
	}while (0);

static int __servoio_open(int id, int mode)
{
	int fd, ret;
	char *ctrlpath;
	struct termios term_attribs;

	ret = asprintf(&ctrlpath, "/dev/ttyACM%d", id);
	if (ret < 0)
		return -ENOMEM;

	fd = open(ctrlpath, mode);
	free(ctrlpath);
	if (fd < 0)
		return -errno;

	tcgetattr(fd, &term_attribs);
	term_attribs.c_iflag &=  ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
	term_attribs.c_oflag &= ~(ONLCR | OCRNL);
	term_attribs.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	if (cfsetispeed(&term_attribs, B9600) < 0)
		printerr("invalid baud rate");

	tcsetattr(fd, TCSANOW, &term_attribs);
	
	return fd;
}

/* Get Position: get the position of a servo.
 *         ===================
 * command:| 0x90| channel_id|
 *         -------------------
 *         |1byte|   1byte   |
 *         ===================
 *          ========================================
 * response:| position bits 0-7| position bits 8-15|
 *          ----------------------------------------
 *          |       1byte      |         1byte     |
 *          ========================================
 */
static int __servoio_read_pos(int fd, int target)
{
	char cmd[2];
	char buf[2];
	int ret;

	cmd[0] = 0x90;
	cmd[1] = target;
	ret = write(fd, cmd, sizeof(cmd));
	if (ret <= 0)
		return ret ? -errno : -EINVAL;
	
	ret = read(fd, buf, sizeof(buf));
	if (ret <= 0)
		return ret ? -errno : -EINVAL;
	return buf[0] + (buf[1] << 8);  /* maybe atoi(buf) instead? */
}

/* Set Target: it configures output channel pulse.
 *         ======================================
 * command:| 0x84| channel_id|low bits|high bits|
 *         --------------------------------------
 *         |1byte|   1byte   | 1byte  |  1byte  |
 *         ======================================
 * low bits and high bits represent the pulse width to transmit in units of
 * quarter microseconds. A value of 0 tells the maestro controller to stop
 * sending pulses to the servo.
 */
static int __servoio_write_pulse(int fd, int target, int pulse)
{
	char cmd[4];
	int ret;

	cmd[0] = 0x84;
	cmd[1] = target;
	cmd[2] = (char)(pulse & 0x7f);
	cmd[3] = (char)((pulse >> 7) & 0x7f);
	ret = write(fd, cmd, sizeof(cmd));
	if (ret <= 0)
		return ret ? -errno : -EINVAL;
	
	return 0;
}

/* Set Speed: configure channel speed limit.
 *         ==================================================
 * command:| 0x87| channel_id|speed low bits|speed high bits|
 *         --------------------------------------------------
 *         |1byte|   1byte   |      1byte   |     1byte     |
 *         ==================================================
 * This command limits the speed at which a servo channel’s output value changes.
 * The speed limit is given in units of (0.25 μs)/(10 ms).
 * What this means is that if you send a "Set Target" command to adjust the target
 * from, say, 1000 μs to 1350 μs, it will take 100 ms to make that adjustment. 
 * A speed of 0 makes the speed unlimited, so that setting the target will 
 * immediately affect the position. 
 */
static int __servoio_write_speed(int fd, int target, int speed_limit)
{
	char cmd[4];
	int ret;

	cmd[0] = 0x84;
	cmd[1] = target;
	cmd[2] = (char)(speed_limit & 0x7f);
	cmd[3] = (char)((speed_limit >> 7) & 0x7f);
	ret = write(fd, cmd, sizeof(cmd));
	if (ret <= 0)
		return ret ? -errno : -EINVAL;
	
	return 0;
}

/* Set Acceleration: configure channel acceleration limit.
 *         ======================================
 * command:| 0x89| channel_id|low bits|high bits|
 *         --------------------------------------
 *         |1byte|   1byte   | 1byte  |  1byte  |
 *         ======================================
 * This command limits the acceleration of a servo channel’s output. 
 * The acceleration limit is a value from 0 to 255 in units of 
 * (0.25 μs)/(10 ms)/(80 ms).
 * A value of 0 tells the maestro controller to stop
 * sending pulses to the servo.
 * An acceleration limit causes the speed of a servo to slowly ramp up until 
 * it reaches the maximum speed, then to ramp down again as position approaches
 * target, resulting in a relatively smooth motion from one point to another. 
 * With acceleration and speed limits, only a few target settings are required 
 * to make natural-looking motions that would otherwise be quite complicated to 
 * produce.
 */
static int __servoio_write_accel(int fd, int target, int acc_limit)
{
	char cmd[4];
	int ret;

	cmd[0] = 0x84;
	cmd[1] = target;
	cmd[2] = (char)(acc_limit & 0x7f);
	cmd[3] = (char)((acc_limit >> 7) & 0x7f);
	ret = write(fd, cmd, sizeof(cmd));
	if (ret <= 0)
		return ret ? -errno : -EINVAL;
	
	return 0;
}

/* Get Error: get any errors flagged by any channel.
 *         ===================
 * command:| 0xa1| channel_id|
 *         -------------------
 *         |1byte|   1byte   |
 *         ===================
 *          ==================================
 * response:| error bits 0-7| error bits 8-15|
 *          ----------------------------------
 *          |       1byte   |      1byte     |
 *          ==================================
 * After the response is sent the errors are cleared.
 */
static int __servoio_read_error(int fd)
{
	char cmd = 0xa1;
	char buf[2];
	int ret;

	ret = write(fd, &cmd, 1);
	if (ret <= 0)
		return ret ? -errno : -EINVAL;
	
	ret = read(fd, buf, sizeof(buf));
	if (ret <= 0)
		return ret ? -errno : -EINVAL;
	return buf[0] + (buf[1] << 8);  /* maybe atoi(buf) instead? */
}

/* "Go home": this command sends all servos and outputs to their 'home' 
 * positions.
 *         ===================
 * command:| 0xa2| channel_id|
 *         -------------------
 *         |1byte|   1byte   |
 *         ===================
 */
static int __servoio_reset_all(int fd)
{
	char cmd = 0xa2;
	int ret;
	
	ret = write(fd, &cmd, 1);
	if (ret <= 0)
		return ret ? -errno : -EINVAL;

	return 0;
}

int servoio_set_accel(int id, int channel, int value)
{
	int fd, ret;
	fd = __servoio_open(id, O_RDWR | O_NOCTTY);
	if (fd < 0)
		return fd;
	ret = __servoio_write_accel(fd, channel, value);
	close(fd);
	return ret;
}

int servoio_set_speed(int id, int channel, int value)
{
	int fd, ret;
	fd = __servoio_open(id, O_RDWR | O_NOCTTY);
	if (fd < 0)
		return fd;
	ret = __servoio_write_speed(fd, channel, value);
	close(fd);
	return ret;
}

int servoio_set_pulse(int id, int channel, int value)
{
	int fd, ret;
	fd = __servoio_open(id, O_RDWR | O_NOCTTY);
	if (fd < 0)
		return fd;
	ret = __servoio_write_pulse(fd, channel, value);
	close(fd);
	return ret;
}

int servoio_configure(int id, int channel, int pulse, int speed, int accel)
{
	int fd, ret;
	fd = __servoio_open(id, O_RDWR | O_NOCTTY);
	if (fd < 0)
		return fd;

	printdbg("Stopping channel %d on target %d.\n",
		 channel, id);
	ret = __servoio_write_pulse(fd, channel, 0); /*stop*/
	if (ret < 0)
		printdbg("write pulse 0 error.");
	
	printdbg("Setting speed limit for channel %d on target %d" \
		 "to %d (0.25 μs)/(10 ms).\n",
		 channel, id, speed);
	ret = __servoio_write_speed(fd, channel, speed);
	if (ret < 0)
		printdbg("write speed limit error.");

	printdbg("Setting accel limit for channel %d on target %d" \
		 "to %d (0.25 μs)/(10 ms)/(80 ms).\n",
		 channel, id, accel);
	ret = __servoio_write_accel(fd, channel,  accel);
	if (ret < 0)
		printdbg("write accel limit error.");

	printdbg("Setting pulse for channel %d on target %d" \
		 "to %d (0.25 μs) = %d μs.\n",
		 channel, id, pulse, pulse>>2);
	ret = __servoio_write_pulse(fd, channel, pulse);
	if (ret < 0)
		printdbg("write pulse value error.");
		 
	close (fd);
	return ret;
}

int servoio_get_position(int id, int channel)
{
	int fd, ret;
	fd = __servoio_open(id, O_RDWR | O_NOCTTY);
	if (fd < 0)
		return fd;
	ret = __servoio_read_pos(fd, channel);
	close(fd);

	return ret;
}

int servoio_get_any_error(int id)
{
	int fd, ret;
	fd = __servoio_open(id, O_RDWR | O_NOCTTY);
	if (fd < 0)
		return fd;
	ret = __servoio_read_error(fd);
	close(fd);

	return ret;
}

int servoio_all_go_home(int id)
{
	int fd, ret;
	fd = __servoio_open(id, O_RDWR | O_NOCTTY);
	if (fd < 0)
		return fd;
	ret = __servoio_reset_all(fd);
	close(fd);

	return ret;
}
