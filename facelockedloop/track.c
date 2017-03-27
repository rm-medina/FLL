/**
 * @file facelockedloop/track.c
 * @brief Face tracking stage, based on fll's servolib library/
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <errno.h>
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "track.h"
#include "servolib.h"
#include "kernel_utils.h"
#include "time_utils.h"
#include "debug.h"

static const int tilt_change_rate = 64;
static const int pan_change_rate = 64;

#define MAX_FRAME_WIDTH	640
#define MAX_FRAME_HEIGHT 480

enum servo_id { pan = 0, tilt = 1};
static int pan_channel = -1;
static int tilt_channel = -1;
static sem_t acc_lock;

static const char* const tname = "tracker";

static int track_get_max_abse(struct tracker *t);
static int track_update_stats(struct tracker *t);
static int override_setup(pthread_attr_t *attr, int prio);
static void *override_ctrl(void *cookie);

static void track_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe)
{
	stage_up(stg, p, o, pipe);
	pipeline_register(pipe, stg);
}

static void track_stage_down(struct stage *stg)
{
	struct tracker *tracer = container_of(stg, struct tracker, step);

	if (!tracer)
		return;

	track_teardown(tracer);
	stage_down(stg);
	pipeline_deregister(stg->pipeline, stg);
}

static int track_stage_run(struct stage *stg)
{
	struct tracker *tracer = container_of(stg, struct tracker, step);
	int ret;
	static long last = 0;
	struct timespec spec;
	long current;

	if (!tracer)
		return -EINVAL;

	/* artificial delay */
	clock_gettime(CLOCK_REALTIME, &spec);
	current = timespec_msecs(&spec);
	if (current < last)
		return 0;

	last = timespec_msecs(&spec) + 350;

	ret = track_run(tracer);
	stg->stats.ofinterest = track_get_max_abse(tracer);

	return ret;
}

static void track_stage_wait(struct stage *stg)
{
	stage_wait(stg);
}

static void track_stage_go(struct stage *stg)
{
	stage_go(stg);
}

static int track_stage_input(struct stage *stg, void **it)
{
	void *itin = NULL;
	struct tracker *tracer;

	tracer = container_of(stg, struct tracker, step);
	if (!tracer)
		return -EINVAL;

	stage_input(stg, &itin);
	tracer->params.bbox  = *(struct store_box*) itin;
        free((struct store_box*)itin);

	return 0;
}

static struct stage_ops track_ops = {
	.up = track_stage_up,
	.down = track_stage_down,
	.run = track_stage_run,
	.wait = track_stage_wait,
	.go = track_stage_go,
	.input = track_stage_input,
};

int track_initialize(struct tracker *t, struct tracker_params *p,
		     struct pipeline *pipe)
{
  	struct stage_params stgparams;
	int ret;
	pthread_attr_t ov_attr;
	pthread_t override;
	
	pan_channel = p->pan_params.channel;
	tilt_channel = p->tilt_params.channel;
	
	stgparams.nth_stage = TRACKING_STAGE;
	stgparams.data_out = NULL;
	stgparams.data_in = NULL;
	stgparams.name = "TRA_STG";
	p->pan_params.home_position = HOME_POSITION_QUARTER_US;
	p->tilt_params.home_position = HOME_POSITION_QUARTER_US;

	t->params = *p;

	ret = sem_init(&acc_lock, 0, 1);
	if (ret < 0) {
		printf("%s failed to init acc_lock.\n", __func__);
		return -EIO;
	}

	ret = override_setup(&ov_attr, 0);
	if (ret) {
		printf("%s failed to setup override thread.\n", __func__);
		return -EIO;
	}

	ret = pthread_create(&override, &ov_attr, override_ctrl, &t->params.dev);
	if (ret)
		printf("%s failed to create override thread.\n", __func__);
#if 0
	ret = servoio_all_go_home(t->params.dev);
	if (ret < 0) {
		printf("%s home error %d.\n", __func__, ret);
		return -EIO;
	}
#endif
	ret = servoio_set_pulse(t->params.dev, t->params.pan_params.channel,
				HOME_POSITION_QUARTER_US);
	if (ret < 0) {
		debug(t, "%s set pulse error %d.\n", __func__, ret);
		return -EIO;
	}

	ret = servoio_set_pulse(t->params.dev, t->params.tilt_params.channel,
				HOME_POSITION_QUARTER_US);
	if (ret < 0) {
		debug(t, "%s set pulse error %d.\n", __func__, ret);
		return -EIO;
	}

	track_stage_up(&t->step, &stgparams, &track_ops, pipe);

	return ret;
	
}

void track_teardown(struct tracker *t)
{
	int ret;

#if 0	
	ret = servoio_all_go_home(t->params.dev);
	if (ret < 0) {
		debug(t, "%s home error %d.\n", __func__, ret);
		return;
	}
#endif

	ret = servoio_configure(t->params.dev, t->params.pan_params.channel,
				HOME_POSITION_QUARTER_US, 0, 0);
	if (ret < 0) {
		debug(t, "%s stopping pan servo failed %d.\n", __func__, ret);
		return;
	}

	ret = servoio_configure(t->params.dev, t->params.tilt_params.channel,
				HOME_POSITION_QUARTER_US, 0, 0);
	if (ret < 0) {
		debug(t, "%s stopping tilt servo failed %d.\n", __func__, ret);
		return;
	}
}

/*
 * 4000 0.25us => all the way left/up
 * 6000 0.25us => servo span middle/middle
 * 8000 0.25us => all the way right/down
 */
static int map_pixels2servoio_pos(enum servo_id sid, int pixels, int cpos)
{
	int servo_tgt;
	
	printf( "%s pan_rate:%d, tilt_rate:%d.\n", __func__, pan_change_rate,
	       tilt_change_rate);
	
	if (sid == pan) {

		servo_tgt = cpos + pixels/pan_change_rate;

		/* 
		 * errata : to avoid pan servo stalling on extreme positions of 
		 * range, reduced boundaries are required.
		 * (correction literals heuristically obtained).
		 * 
		 */
		
		if (pixels < 0)
			servo_tgt -= (servo_tgt % 256);
		else
			servo_tgt += (servo_tgt % 256) ? 256 - (servo_tgt % 256) : 0;
		
		
		if (servo_tgt < SERVOLIB_MIN_PULSE_QUARTER_US)
			servo_tgt = SERVOLIB_MIN_PULSE_QUARTER_US + 256;
	
		if (servo_tgt > SERVOLIB_MAX_PULSE_QUARTER_US)
			servo_tgt = SERVOLIB_MAX_PULSE_QUARTER_US - 256;
		
	}
	else  {
		servo_tgt = cpos - pixels/tilt_change_rate;

		/* 
		 * errata : to avoid tilt servo stalling on  extreme postions of
		 * range, reduced  boundaries are required.
		 * (correction literals heuristically obtained).
		 * 
		 */
		
		if (pixels < 0)
			servo_tgt += (servo_tgt % 256) ? 256 - (servo_tgt % 256) : 0;
		else
			servo_tgt -= (servo_tgt % 256);
		
		
		if (servo_tgt < SERVOLIB_MIN_PULSE_QUARTER_US + 756)
			servo_tgt = SERVOLIB_MIN_PULSE_QUARTER_US + 756;
	
		if (servo_tgt > SERVOLIB_MAX_PULSE_QUARTER_US - 756)
			servo_tgt = SERVOLIB_MAX_PULSE_QUARTER_US - 756;
		
	}
	
	return servo_tgt;
}

static int get_bbox_center(int ptB, int ptA)
{
	if (ptA > ptB)
		return -EINVAL;
	
	return ((ptB - ptA) >> 1) + ptA;
}

static int get_pixels_shift(int ptM, int ptC)
{
	return ptM - ptC;
}

static int move_servo(int id, int channel, int target)
{
	int pos = 0, ret;
	int i;

	for (i = 0; i < 10 && !!(target - pos); i++) {

		ret = servoio_set_pulse(id, channel, target);
		if (ret < 0) {
			return -EIO;
		}

		pos = servoio_get_position(id, channel);
		if (pos < 0) {
			return -EIO;
		}
	}

	/* we did our best ... not much we can do */
	return 0;
}

static int start_scan_seq(struct tracker_params *p)
{
	static struct {
		enum {fwd, bck} direction;
		int skip;
		int step;
	} value = {
		.direction = fwd,
		.step = 256,
		.skip = 0,
		};
	char ch;
	volatile int v;
	/* skip every other frame */
	value.skip = ~value.skip;
	if (value.skip)
		return 0;

	v = servoio_get_position(p->dev, pan_channel);


	if (value.direction == bck) {
		ch = '-';
		if ((v - value.step) > SERVOLIB_MAX_PULSE_QUARTER_US) 
			v = SERVOLIB_MAX_PULSE_QUARTER_US;
		
		else if ((v - value.step) > SERVOLIB_MIN_PULSE_QUARTER_US)
				v -= value.step;
		else {
			value.direction = fwd;
			v = SERVOLIB_MIN_PULSE_QUARTER_US + 256;
			goto done;
		}
	}

	if (value.direction == fwd) {
		ch = '+';
		if ((v + value.step) < SERVOLIB_MIN_PULSE_QUARTER_US) 
			v = SERVOLIB_MIN_PULSE_QUARTER_US + 256;
		
		else if ((v + value.step) < SERVOLIB_MAX_PULSE_QUARTER_US)
			v += value.step;
		else {
			value.direction = bck;
			v = SERVOLIB_MAX_PULSE_QUARTER_US;
		}
	}
done:
	printf("search%c\t[%d, %d]\n", ch, v, servoio_get_position(p->dev, tilt_channel));

	return servoio_set_pulse(p->dev, pan_channel, v);

}

int track_run(struct tracker *t)
{
	int id;
	int box_ptC_x, box_ptC_y;
	int cpos; /* current motor position */
	int tpos; /* target motor position  */
	int ret;

	ret = sem_trywait(&acc_lock);
	if (ret < 0)
		return 0;
	
	id = t->params.dev;
	if (t->params.bbox.scan) {
		/* no detection, initialize scan sequence */
		ret = start_scan_seq(&t->params);
		goto done;
	}
	
	/* handle PAN */
	box_ptC_x = get_bbox_center(t->params.bbox.ptB_x, t->params.bbox.ptA_x);
	if (box_ptC_x < 0) {
		debug(t, "%s: %d error %d.\n", __func__, __LINE__, box_ptC_x);
		return -EINVAL;
	}
	cpos = servoio_get_position(id, pan_channel);
	if (cpos < 0) {
		return -EINVAL;
	}
	t->params.pan_params.position = cpos;
	
	tpos = map_pixels2servoio_pos(pan, get_pixels_shift(MAX_FRAME_WIDTH >> 1, box_ptC_x), cpos);
	ret = move_servo(id, pan_channel, tpos);
	if (ret) {
		debug(t, "%s: %d error %d.\n", __func__, __LINE__, ret);
		sleep(1000);
		return -EINVAL;
	}
	t->params.pan_tgt = ret;
	t->params.pan_params.poserr = ret - cpos;
	
	/* handle TILT */
	box_ptC_y = get_bbox_center(t->params.bbox.ptB_y, t->params.bbox.ptA_y);
	if (box_ptC_y < 0) {
		debug(t, "%s: %d error %d.\n", __func__, __LINE__, box_ptC_y);
		return -EINVAL;
	}
	
	cpos = servoio_get_position(id, tilt_channel);
	if (cpos < 0) {
		return -EINVAL;
	}
	t->params.tilt_params.position = cpos;
	
	tpos = map_pixels2servoio_pos(tilt, get_pixels_shift(MAX_FRAME_HEIGHT >> 1, box_ptC_y), cpos);
	ret = move_servo(id, tilt_channel, tpos);
	if (ret) {
		debug(t, "%s: %d error %d.\n", __func__, __LINE__, ret);

		sleep(1000);
		return -EINVAL;
	}
	t->params.tilt_tgt = ret;
	t->params.tilt_params.poserr = ret - cpos;

	track_update_stats(t);

done:
	sem_post(&acc_lock);
	return ret;
}

int track_get_max_abse(struct tracker *t)
{
	int pabse, tabse;

	pabse = abs(t->stats.pan_stats.rt_err);
	tabse = abs(t->stats.tilt_stats.rt_err);

	return ((abs(pabse) > abs(tabse))? pabse : tabse);
}


static int override_setup(pthread_attr_t *attr, int prio)
{
 	struct sched_param p;
	int ret;

	ret = pthread_attr_init(attr);
	if (ret)
		return -1;

	ret = pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
	if (ret)
		return -1;

	ret = pthread_attr_setschedpolicy(attr, prio ? SCHED_FIFO : SCHED_OTHER);
	if (ret)
		return -1;

	p.sched_priority = prio;
	ret = pthread_attr_setschedparam(attr, &p);
	if (ret)
		return -1;

	return 0;
}

static void clear_screen(void)
{
	const char* cmd = "\e[1;1H\e[2J";

	write(2, cmd, strlen(cmd));
}

static void print_config(int dev)
{
	printf("Use the UP/DOWN cursor keys to calibrate the servos\n");
	printf("any other key to exit\n");
	printf("\tpan  :\t\t%3d\n", servoio_get_position(dev, pan_channel));
	printf("\ttilt :\t\t%3d\n", servoio_get_position(dev, tilt_channel));
}

static void *override_ctrl(void *cookie)
{
	int id, ch, pos, locked = 0;
	char c;
	id = *(int*)cookie;
	
	for (;;) {
		clear_screen();
		print_config(id);

		c = kbhit_irq();
		if (!locked && (c == 'A' || c == 'B' || c == 'C' || c == 'D')) {
			locked = 1;
			sem_wait(&acc_lock);
		}
		else if (locked && (c == 'X')) {
			sem_post(&acc_lock);
			locked = 0;
			continue;
		}

		if (c == 'C' || c == 'D') {
			ch = pan_channel;
		} else {
			ch = tilt_channel;
		}

		pos = servoio_get_position(id, ch);

		switch(c) {
		case 'B':
		case 'C':
			if ((pos-512) < SERVOLIB_MIN_PULSE_QUARTER_US)
				servoio_set_pulse(id, ch, SERVOLIB_MIN_PULSE_QUARTER_US + 512);
		
			else if ((pos+256) > SERVOLIB_MAX_PULSE_QUARTER_US)
			        servoio_set_pulse(id, ch, SERVOLIB_MAX_PULSE_QUARTER_US - 256);
			else 

				servoio_set_pulse(id, ch, pos + 256);
			break;
		case 'A':
		case 'D':
			if ((pos-512) < SERVOLIB_MIN_PULSE_QUARTER_US)
				servoio_set_pulse(id, ch, SERVOLIB_MIN_PULSE_QUARTER_US + 512);
		
			else if ((pos+256) > SERVOLIB_MAX_PULSE_QUARTER_US)
			        servoio_set_pulse(id, ch, SERVOLIB_MAX_PULSE_QUARTER_US - 256);
			else 
				servoio_set_pulse(id, ch, pos - 256);
			break;
		}
	}

	return NULL;
}

#if !defined(ENABLE_STATS)

int track_update_stats(struct tracker *t)
{
	return 0;
}

int track_print_stats(struct tracker *t)
{
	return 0;
}

#else

int track_update_stats(struct tracker *t)
{
	if (!t)
		return -EINVAL;
	
	t->stats.pan_stats.rt_err = t->params.pan_params.poserr;
	if (t->params.pan_params.poserr < t->stats.pan_stats.min_poserr)
		t->stats.pan_stats.min_poserr = t->params.pan_params.poserr;

	if (t->params.pan_params.poserr > t->stats.pan_stats.max_poserr)
		t->stats.pan_stats.max_poserr = t->params.pan_params.poserr;

	if (t->params.pan_params.position < t->stats.pan_stats.min_pos)
		t->stats.pan_stats.min_pos = t->params.pan_params.position;

	if (t->params.pan_params.position > t->stats.pan_stats.max_pos)
		t->stats.pan_stats.max_pos = t->params.pan_params.position;

	t->stats.tilt_stats.rt_err = t->params.tilt_params.poserr;
	if (t->params.tilt_params.poserr < t->stats.tilt_stats.min_poserr)
		t->stats.tilt_stats.min_poserr = t->params.tilt_params.poserr;

	if (t->params.tilt_params.poserr > t->stats.tilt_stats.max_poserr)
		t->stats.tilt_stats.max_poserr = t->params.tilt_params.poserr;

	if (t->params.pan_params.position < t->stats.tilt_stats.min_pos)
		t->stats.tilt_stats.min_pos = t->params.tilt_params.position;

	if (t->params.tilt_params.position > t->stats.tilt_stats.max_pos)
		t->stats.tilt_stats.max_pos = t->params.tilt_params.position;

	return 0;
}

int track_print_stats(struct tracker *t)
{
	if (!t)
		return -EINVAL;

	debug(t, "servo    min_pos      max_pos      min_err     max_err     \n");
	debug(t, "-----------------------------------------------------------\n");
	debug(t, "PAN      %d           %d           %d          %d          \n",
	       t->stats.pan_stats.min_pos,
	       t->stats.pan_stats.max_pos,
	       t->stats.pan_stats.min_poserr,
	       t->stats.pan_stats.max_poserr);
	debug(t, "TILT     %d           %d           %d          %d          \n",
	       t->stats.tilt_stats.min_pos,
	       t->stats.tilt_stats.max_pos,
	       t->stats.tilt_stats.min_poserr,
	       t->stats.tilt_stats.max_poserr);

	return 0;
}

#endif
