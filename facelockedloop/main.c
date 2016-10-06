/**
 * @file facelockedloop/main.c
 * @brief Application that allows face tracking within a given video stream.
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <sys/types.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>

extern const char *fll_version_name;

static void init_skeleton(void)
{
	int ret;
	ret = 0;
        /* initialize servolib */
	/* initialize opencv */
	/* any other start configuration */
}

static void teardown_skeleton(void)
{
	int ret;
	/* initialize servolib to 0 or home? */
	/* any opencv specifics? */
	/* any other finalizing action? */
}

int main(int argc, char *const argv[])
{
	int pos[FLL_MAX_SERVO_COUNT] = { [0 ... FLL_MAX_SERVO_COUNT -1] = -1};
	int speed[FLL_MAX_SERVO_COUNT] = { [0 ... FLL_MAX_SERVO_COUNT -1] = -1};
	int accel[FLL_MAX_SERVO_COUNT] = { [0 ... FLL_MAX_SERVO_COUNT -1] = -1};
	struct servo_params servo_params[FLL_SERVO_COUNT];
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	return 0;
	
}
