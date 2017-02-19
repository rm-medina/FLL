/**
 * @file servolib/test-servo.c
 * test program to use servos connected to micro maestro USB 6 channel
 *
 */

#include <fcntl.h>
#include <getopt.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "servolib.h"

static int cmd_enum;
static const struct option options[] = {
	{
#define help_opt 0
		.name = "help",
		.has_arg = 0,
		.flag = NULL,
	},
	{
#define device_opt 1
		.name = "dev",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define channel_opt 2
		.name = "channel",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define position_opt 3
		.name = "pos",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define speed_limit_opt 4
		.name = "speed",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define accel_limit_opt 5
		.name = "accel",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define reset_opt 6
		.name = "reset",
		.has_arg = 0,
		.flag = &cmd_enum,
		.val = SERVOIO_RESET,
	},
	{
#define check_opt 7
		.name = "check",
		.has_arg = 0,
		.flag = &cmd_enum,
		.val = SERVOIO_CHECK,
	},
	{ 
#define read_opt 8		
		.name = "read",
		.has_arg = 0,
		.flag = &cmd_enum,
		.val = SERVOIO_READ,
	},
	{
#define configure_opt 9
		.name = "init",
		.has_arg = 0,
		.flag = &cmd_enum,
		.val = SERVOIO_CONFIG,
	},
	{
#define write_opt 10
		.name = "write",
		.has_arg = 0,
		.flag = &cmd_enum,
		.val = SERVOIO_WRITE,
	},
#define newc_opt 11
	{
		.name = "coord",
		.has_arg = 1,
		.flag = NULL,

	},
#define minc_opt 12
	{
		.name = "min_coord",
		.has_arg = 1,
		.flag = NULL,
		
	},
#define maxc_opt 13
	{
		.name = "max_coord",
		.has_arg = 1,
		.flag = NULL,
	},
};

static void usage(void)
{
	fprintf(stderr, "usage: test-servo <options>, with: \n");
	fprintf(stderr, "--dev <n> - n: ttyACM device number (default 0).\n");
	fprintf(stderr, "--channel <m> - m: controller channel id (0 to 5)" \
		"(default 5).\n");
	fprintf(stderr, "--pos <p> - p:position for dev,channel.\n");
	fprintf(stderr, "--speed <s> - s: speed limit for dev,channel.\n");
	fprintf(stderr, "--accel <a> - a: accel limit for dev,channel.\n");
	fprintf(stderr, "--read - get position from dev,channel.\n");
	fprintf(stderr, "--init - configure dev, channel with position, speed" \
		"and accel values.\n");
	fprintf(stderr, "--write <p|s|a> (p: position, s:speed, a:accel).\n");
	fprintf(stderr, "--coord <c> - c:coordinate to map to PWM pulse duration.\n");
	fprintf(stderr, "--min_coord <c> - c:min coordinate to map to PWM pulse duraLtion.\n");
	fprintf(stderr, "--nax_coord <c> - c:max coordinate to map to PWM pulse duration.\n");
	fprintf(stderr, "--help this help.\n");
}

int main(int argc, char *argv[])
{
	int opt,readerr, newc;
	int dev, pos, acclim, speelim;
	int opt_index, do_reset, check_err, get_pos, do_conf, do_set,ret;
	int chan = 5;
	int minc = 0;
	int maxc = 0;
	
	do_reset = check_err = get_pos = do_conf = do_set = 0;
	dev = pos = acclim = speelim=-1;
	for (;;) {
		opt_index = -1;
		opt = getopt_long_only(argc, argv, "", options, &opt_index);
		if (opt == EOF)
			break;
		fprintf(stderr, "--help this help: %d.\n", opt_index);
		switch (opt_index) {
		case help_opt:
			usage();
			exit(0);
		case device_opt:
			dev = atoi(optarg);
			break;
		case channel_opt:
			chan = atoi(optarg);
			break;
		case position_opt:
			pos = atoi(optarg);
			fprintf(stderr, "user entered pos:%d.\n", pos);
			break;
		case speed_limit_opt:
			speelim = atoi(optarg);
			fprintf(stderr, "user entered speed limit:%d.\n", pos);
			break;
		case accel_limit_opt:
			acclim = atoi(optarg);
			fprintf(stderr, "user entered accel limit:%d.\n", pos);
			break;
		case reset_opt:
			do_reset = 1;
			break;
		case check_opt:
			check_err = 1;
			break;
		case read_opt:
			get_pos = 1;
			break;
		case configure_opt:
			do_conf = 1;
			break;
		case newc_opt:
			newc = atoi(optarg);
			break;
		case minc_opt:
			minc = atoi(optarg);
			break;
		case maxc_opt:
			maxc = atoi(optarg);
			break;
		case write_opt:
			do_set = 1;
			break;

		default:
			usage();
			exit(1);
		}
	}

	if (maxc) {
		ret = servoio_map_coordinate(newc, minc, maxc);
		fprintf(stderr, "test-servo map coordinates "	\
		"coord: %d in (%d, %d) pulse: %d 0.25us.\n",
		newc, minc, maxc, ret);
		ret = servoio_set_pulse(dev, chan, ret);
		pos = ret;
		get_pos = 1;
	}
	if ((do_set) && (get_pos != -1)) {
		ret = servoio_map_pulse(dev, chan, pos);
		fprintf(stderr, "test-servo set_pulse " \
			"ttyACM%d channel %d pulse:%d ret 0x%x.\n",
			dev, chan, pos, ret);
	}
	if ((do_set) && (acclim != -1)) {
		ret = servoio_set_accel(dev,chan, acclim);
		fprintf(stderr, "test-servo set_accel "			\
			"ttyACM%d channel %d accel:%d ret 0x%x.\n",
			dev, chan, acclim, ret);
	}
	if ((do_set) && (speelim != -1)) {
		ret = servoio_set_speed(dev,chan, speelim);
		fprintf(stderr, "test-servo set_speed " \
			"ttyACM%d channel %d speed:%d ret 0x%x.\n",
			dev, chan, speelim, ret);
	}
	if (get_pos) {
		pos = servoio_get_position(dev, chan);
		fprintf(stderr, "test-servo ttyACM%d channel %d pos:%d.\n",
			dev, chan, pos);
	}
	if ((do_conf) && (pos != -1) && (acclim != -1) &&
	    (speelim != -1)) {
		ret = servoio_configure(dev, chan, pos, speelim, acclim);
		fprintf(stderr, "test-servo ttyACM%d channel %d returned " \
			"0x%x.\n", dev, chan, ret);
	}
	if (do_reset)
		servoio_all_go_home(dev);
	if (check_err) {
		readerr = servoio_get_any_error(dev);
		fprintf(stderr, "servoio err %d on dev ttyACM%d.\n",
			readerr, dev);
	}
	return 0;
}
