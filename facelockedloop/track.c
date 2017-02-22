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

#include "track.h"
#include "servolib.h"
#include "kernel_utils.h"

static const int tilt_change_rate = 64;
static const int pan_change_rate = 64;

#define MAX_FRAME_WIDTH	640
#define MAX_FRAME_HEIGHT 480

enum servo_id { pan = 0, tilt = 1};


static int track_get_max_abse(struct tracker *t);
static int track_update_stats(struct tracker *t);

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

	if (!tracer)
		return -EINVAL;
	
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

	tracer->params.bbox.ptA_x = ((struct store_box*)itin)->ptA_x;
	tracer->params.bbox.ptA_y = ((struct store_box*)itin)->ptA_y;
	tracer->params.bbox.ptB_x = ((struct store_box*)itin)->ptB_x;
	tracer->params.bbox.ptB_y = ((struct store_box*)itin)->ptB_y;

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

	stgparams.nth_stage = TRACKING_STAGE;
	stgparams.data_out = NULL;
	stgparams.data_in = NULL;
	p->pan_params.home_position = HOME_POSITION_QUARTER_US;
	p->tilt_params.home_position = HOME_POSITION_QUARTER_US;

	t->params = *p;

	ret = servoio_all_go_home(t->params.dev);
	if (ret < 0) {
		printf("%s home error %d.\n", __func__, ret);
		return -EIO;
	}
	ret = servoio_set_pulse(t->params.dev, t->params.pan_params.channel,
				HOME_POSITION_QUARTER_US);
	if (ret < 0) {
		printf("%s set pulse error %d.\n", __func__, ret);
		return -EIO;
	}

	ret = servoio_set_pulse(t->params.dev, t->params.tilt_params.channel,
				HOME_POSITION_QUARTER_US);
	if (ret < 0) {
		printf("%s set pulse error %d.\n", __func__, ret);
		return -EIO;
	}

	track_stage_up(&t->step, &stgparams, &track_ops, pipe);

	return ret;
	
}

void track_teardown(struct tracker *t)
{
	int ret;
	
	ret = servoio_all_go_home(t->params.dev);
	if (ret < 0) {
		printf("%s home error %d.\n", __func__, ret);
		return;
	}

	ret = servoio_configure(t->params.dev, t->params.pan_params.channel,
				HOME_POSITION_QUARTER_US, 0, 0);
	if (ret < 0) {
		printf("%s stopping pan servo failed %d.\n", __func__, ret);
		return;
	}

	ret = servoio_configure(t->params.dev, t->params.tilt_params.channel,
				HOME_POSITION_QUARTER_US, 0, 0);
	if (ret < 0) {
		printf("%s stopping tilt servo failed %d.\n", __func__, ret);
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
	
	printf("%s pan_rate:%d, tilt_rate:%d.\n", __func__, pan_change_rate,
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

int track_run(struct tracker *t)
{
	int id, pan_channel, tilt_channel;
	int box_ptC_x, box_ptC_y;
	int cpos; /* current motor position */
	int tpos; /* target motor position  */
	int ret;

	tilt_channel = t->params.tilt_params.channel;
	pan_channel = t->params.pan_params.channel;
	id = t->params.dev;

	/* handle PAN */
	box_ptC_x = get_bbox_center(t->params.bbox.ptB_x, t->params.bbox.ptA_x);
	if (box_ptC_x < 0) {
		printf("%s: %d error %d.\n", __func__, __LINE__, box_ptC_x);
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
		printf("%s: %d error %d.\n", __func__, __LINE__, ret);
		sleep(1000);
		return -EINVAL;
	}
	t->params.pan_tgt = ret;
	t->params.pan_params.poserr = ret - cpos;
	
	/* handle TILT */
	box_ptC_y = get_bbox_center(t->params.bbox.ptB_y, t->params.bbox.ptA_y);
	if (box_ptC_y < 0) {
		printf("%s: %d error %d.\n", __func__, __LINE__, box_ptC_y);
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
		printf("%s: %d error %d.\n", __func__, __LINE__, ret);

		sleep(1000);
		return -EINVAL;
	}
	t->params.tilt_tgt = ret;
	t->params.tilt_params.poserr = ret - cpos;

	track_update_stats(t);
	return 0;
}

int track_get_max_abse(struct tracker *t)
{
	int pabse, tabse;

	pabse = abs(t->stats.pan_stats.rt_err);
	tabse = abs(t->stats.tilt_stats.rt_err);

	return ((abs(pabse) > abs(tabse))? pabse : tabse);
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

	printf("servo    min_pos      max_pos      min_err     max_err     \n");
	printf("-----------------------------------------------------------\n");
	printf("PAN      %d           %d           %d          %d          \n",
	       t->stats.pan_stats.min_pos,
	       t->stats.pan_stats.max_pos,
	       t->stats.pan_stats.min_poserr,
	       t->stats.pan_stats.max_poserr);
	printf("TILT     %d           %d           %d          %d          \n",
	       t->stats.tilt_stats.min_pos,
	       t->stats.tilt_stats.max_pos,
	       t->stats.tilt_stats.min_poserr,
	       t->stats.tilt_stats.max_poserr);

	return 0;
}

#endif
