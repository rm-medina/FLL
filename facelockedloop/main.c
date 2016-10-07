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
#include "servolib.h"

extern const char *fll_version_name;
#define FLL_MAX_SERVO_COUNT SERVOLIB_MAX_SERVO_COUNT
#define FLL_SERVO_COUNT 2

/*helper function*/
static int timeval_subtract (struct timeval *result,
			     struct timeval *x,
			     struct timeval *y) __attribute__((unused));

static int timeval_subtract (struct timeval *result,
			     struct timeval *x, struct timeval *y)
{
	/* Perform the carry for the later subtraction by updating y. */
	if (x->tv_usec < y->tv_usec) {
		int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
		y->tv_usec -= 1000000 * nsec;
		y->tv_sec += nsec;
	}
	if (x->tv_usec - y->tv_usec > 1000000) {
		int nsec = (x->tv_usec - y->tv_usec) / 1000000;
		y->tv_usec += 1000000 * nsec;
		y->tv_sec -= nsec;
	}

	/* Compute the time remaining to wait.
	   tv_usec is certainly positive. */
	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_usec = x->tv_usec - y->tv_usec;

	/* Return 1 if result is negative. */
	return x->tv_sec < y->tv_sec;
}

static void timespec_substract(struct timespec *const diff,
			       const struct timespec *const start,
			       const struct timespec *const end)
{
	if ((end->tv_nsec-start->tv_nsec)<0) {
		diff->tv_sec = end->tv_sec-start->tv_sec-1;
		diff->tv_nsec = 1000000000+end->tv_nsec-start->tv_nsec;
	} else {
		diff->tv_sec = end->tv_sec-start->tv_sec;
		diff->tv_nsec = end->tv_nsec-start->tv_nsec;
	}
}

static void init_skeleton(void)
{
	int ret __attribute__((unused));
	/* initialize servolib */
	/* initialize opencv */
	/* any other start configuration */
}

static void teardown_skeleton(void)
{
	int ret __attribute__((unused));
	/* initialize servolib to 0 or home? */
	/* any opencv specifics? */
	/* any other finalizing action? */
}

int main(int argc, char *const argv[])
{
	struct timespec start_time, stop_time, duration;
	int pos[FLL_MAX_SERVO_COUNT] =
		{ [0 ... FLL_MAX_SERVO_COUNT -1] = -1};
	int speed[FLL_MAX_SERVO_COUNT] =
		{ [0 ... FLL_MAX_SERVO_COUNT -1] = -1};
	int accel[FLL_MAX_SERVO_COUNT] =
		{ [0 ... FLL_MAX_SERVO_COUNT -1] = -1};
	struct servo_params servo_params[FLL_SERVO_COUNT] __attribute__((unused));
	int i = 0;
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	init_skeleton();
	for (;i < FLL_MAX_SERVO_COUNT; i++)
		printf("servo channel %d, pos:%d, speedLim:%d, accelLim:%d.\n",
		       i, pos[i], speed[i], accel[i]);

	teardown_skeleton();
	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	timespec_substract(&duration, &start_time, &stop_time);
	printf("duration->  %lds %ldns .\n", duration.tv_sec , duration.tv_nsec );

	return 0;
	
}
