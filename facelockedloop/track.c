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

#define ZONES_N 10
#define PtA_MIDDLE_ZONE 4
#define PtB_MIDDLE_ZONE 5
#define Pt_x_MIDDLE 320
#define Pt_y_MIDDLE 240
#define PAN_STEP_PIXELS 64
#define TILT_STEP_PIXELS 48
const int pan_zones[] = {0, 64, 128, 192, 256, 320, 384, 448, 512, 576};
const int tilt_zones[] = {0, 48, 96, 144, 192, 240, 288, 336, 384, 432};  

static void track_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe);
static void track_stage_down(struct stage *stg);
static int track_stage_run(struct stage *stg);
static void track_stage_wait(struct stage *stg);
static void track_stage_go(struct stage *stg);
static int track_stage_input(struct stage *stg, void** it);
static int track_map_inframe_shift_to_servo_pos(int ispanservo, int ifshift,
						int servo_cpos);

static struct stage_ops track_ops = {
	.up = track_stage_up, 
	.down = track_stage_down,
	.run = track_stage_run,
	.wait = track_stage_wait,
	.go = track_stage_go,
	.input = track_stage_input,
};

static void track_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe)
{
	stage_up(stg, p, o, pipe);
	pipeline_register(pipe, stg);
	
}

static void track_stage_down(struct stage *stg)
{
	struct tracker *tracer;

	tracer = container_of(stg, struct tracker, step);
	track_teardown(tracer);
	stage_down(stg);
	pipeline_deregister(stg->pipeline, stg);
}

static int track_stage_run(struct stage *stg)
{
	struct tracker *tracer;
	int ret;
	
	tracer = container_of(stg, struct tracker, step);
	if (!tracer)
		return -EINVAL;
	
	ret =track_run(tracer);
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
	stage_input(stg, &itin);

	tracer->params.bbox.ptA_x = ((struct store_box*)itin)->ptA_x;
	tracer->params.bbox.ptA_y = ((struct store_box*)itin)->ptA_y;
	tracer->params.bbox.ptB_x = ((struct store_box*)itin)->ptB_x;
	tracer->params.bbox.ptB_y = ((struct store_box*)itin)->ptB_y;
	
	return 0;
}

int track_initialize(struct tracker *t, struct tracker_params *p,
		     struct pipeline *pipe)
{
  	struct stage_params stgparams;
	int ret;

	stgparams.nth_stage = TRACKING_STAGE;
	stgparams.data_in = NULL;
	stgparams.data_out = NULL;
	p->pan_params.home_position = HOME_POSITION_QUARTER_US;
	p->tilt_params.home_position = HOME_POSITION_QUARTER_US;

	t->params = *p;
	ret = servoio_all_go_home(t->params.dev);
	if (ret < 0)
		printf("%s home error %d.\n", __func__, ret);

	t->params.pan_tgt = HOME_POSITION_QUARTER_US;
	ret = servoio_set_pulse(t->params.dev,
				t->params.pan_params.channel,
				t->params.pan_tgt);

	p->pan_params.position =
		servoio_get_position(p->dev,
				     p->pan_params.channel);
	printf("%s pan pos %d home %d.\n", __func__, p->pan_params.position,
		p->pan_params.home_position);
	t->stats.pan_stats.cmdstally[SERVOIO_READ] = 1;

	t->params.tilt_tgt = HOME_POSITION_QUARTER_US;
	ret = servoio_set_pulse(t->params.dev,
				t->params.tilt_params.channel,
				t->params.tilt_tgt);

	p->tilt_params.position = servoio_get_position(p->dev,
						       p->tilt_params.channel);
	printf("%s tilt pos %d home %d.\n", __func__, p->tilt_params.position,
		p->tilt_params.home_position);
	t->stats.pan_stats.cmdstally[SERVOIO_WRITE] = 1;

	track_stage_up(&t->step, &stgparams, &track_ops, pipe);
	return ret;
	
}

void track_teardown(struct tracker *t)
{
	int ret;
	
	ret = servoio_all_go_home(t->params.dev);
	if (ret < 0)
		printf("%s home error %d.\n", __func__, ret);

	ret = servoio_configure(t->params.dev,
				t->params.pan_params.channel,
				HOME_POSITION_QUARTER_US, 0, 0);
	if (ret < 0)
		printf("%s stopping pan servo failed %d.\n", __func__, ret);

	ret = servoio_configure(t->params.dev,
				t->params.tilt_params.channel,
				HOME_POSITION_QUARTER_US, 0, 0);
	if (ret < 0)
		printf("%s stopping tilt servo failed %d.\n", __func__, ret);

	t->params.pan_params.position =
		servoio_get_position(t->params.dev,
				     t->params.pan_params.channel);
	printf("%s pan pos %d home %d.\n", __func__,
	       t->params.pan_params.position,
	       t->params.pan_params.home_position);

	t->params.tilt_params.position =
		servoio_get_position(t->params.dev,
				     t->params.tilt_params.channel);
	printf("%s tilt pos %d home %d.\n", __func__,
	       t->params.tilt_params.position,
	       t->params.tilt_params.home_position);
	
}

/*
 * 4000 0.25us => all the way left/up
 * 6000 0.25us => servo span middle/middle
 * 8000 0.25us => all the way right/down
 */
static int track_map_inframe_shift_to_servo_pos(int is_panservo, int ifshift,
						int servo_cpos)
{
	/* positive pan_pskip => Pt < Center 
	 *                    => face on left half of frame, 
	 *		      => shift camera left to move Pt to the right
	 *                    => reduce servo pulse value (take closer to 
	 *                       lower pulse limit (4000 0.25us).
	 * negative pan_pskip => Pt > Center
	 *                    => face on right half of frame,
	 *                    => shift camera right to move Pt to the left
	 *                    => increment servo pulse value (take closer to
	 *                       upper pulse limit (6000 0.25us).
	 *
	 * span of 4000 0.25us => 1000 camera positions from furthest left 
	 *                        to furthest right.
	 * positive tilt_pskip => Pt < Center 
	 *                     => face on top half of frame, 
	 *		       => shift camera up to move Pt down.
	 *                     => reduce servo pulse value (take closer to 
	 *                       lower pulse limit (4000 0.25us).
	 * negative pan_pskip => Pt > Center
	 *                    => face on bottom half of frame,
	 *                    => shift camera down to move Pt to the top
	 *                    => increment servo pulse value (take closer to
	 *                       upper pulse limit (6000 0.25us).
	 *
	 * span of 4000 0.25us => 1000 camera positions from furthest top
	 *                        to furthest bottom.
	 *
	 * 1 unit camera position move => new frame shifted 1 pixel.
	 * 
	 * servo => pulse span = 4000 0.25us pulse span; 
	 *          min step: 4 0.25us
	 *	    ==> position span = pulse span/min step = 
	 *	                      = 4000 0.25us / 4 0.25us = 
	 *			      = 1000 different positions.
	 * 1 unit move in servo => at least 1 pixel difference in frame.
	 * pan case:
	 * 640 * 3 = 1920 pixels covered by pan servo: 
	 *         => 1920 pixels/1000 positions 1.92 pixels change per pos.
	 * tilt case:
	 * 480 * 5 = 2400 pixels covered by tilt servo =>
	 *         => 2400 pixels/1000 positions = 2.4 pixels change per pos.
	 */
	const int tilt_change_rate = 64;
	const int pan_change_rate = 64;
	int servo_tgt;
	
	printf("%s pan_rate:%d, tilt_rate:%d.\n", __func__, pan_change_rate, tilt_change_rate);
	
	if (is_panservo) {
	  	servo_tgt = servo_cpos + ifshift/pan_change_rate;

		if (ifshift < 0)
			servo_tgt -= (servo_tgt % 256) ? (servo_tgt % 256) : 0;
		else
			servo_tgt += (servo_tgt % 256) ? 256 - (servo_tgt % 256) : 0;
		
		if (servo_tgt < SERVOLIB_MIN_PULSE_QUARTER_US)
			servo_tgt = SERVOLIB_MIN_PULSE_QUARTER_US + 256;
	
		if (servo_tgt > SERVOLIB_MAX_PULSE_QUARTER_US)
			servo_tgt = SERVOLIB_MAX_PULSE_QUARTER_US - 256;		
	}
	else  {
		servo_tgt = servo_cpos - ifshift/tilt_change_rate;

		if (ifshift < 0)
			servo_tgt += (servo_tgt % 256) ? 256 - (servo_tgt % 256) : 0;
		else
			servo_tgt -= (servo_tgt % 256) ? (servo_tgt % 256) : 0;

		if (servo_tgt < SERVOLIB_MIN_PULSE_QUARTER_US + 512)
			servo_tgt = SERVOLIB_MIN_PULSE_QUARTER_US + 512;
	
		if (servo_tgt > SERVOLIB_MAX_PULSE_QUARTER_US - 512)
			servo_tgt = SERVOLIB_MAX_PULSE_QUARTER_US - 512;
	}
	
	return servo_tgt;
}

static int get_bbox_center(int ptB, int ptA)
{
	if (ptA > ptB)
		return -EINVAL;
	
	return ((ptB - ptA) >> 1) + ptA;
}

static int get_frame_pixel_shift(int ptM, int ptC)
{
	return ptM - ptC;
}

int track_run(struct tracker *t)
{
	int  d, e;
	int ret = 0;
	int box_ptC_x, box_ptC_y;
	int d_ptCptM = 0;
	const int frame_ptM_x = 320;
	const int frame_ptM_y = 240;
	int count = 10;
	
	/* ptC: boundingbox center in current frame, 
	 * frame points range: (0,0):(639,479)
	 */
	box_ptC_x = get_bbox_center(t->params.bbox.ptB_x, t->params.bbox.ptA_x);
	if (box_ptC_x < 0) {
		printf("%s: %d error %d.\n",
		       __func__, __LINE__, box_ptC_x);
		sleep(1000);
		return -EINVAL;
	}
	
	t->params.pan_params.position = servoio_get_position(t->params.dev, t->params.pan_params.channel);
	++(t->stats.pan_stats.cmdstally[SERVOIO_READ]);
	d_ptCptM = get_frame_pixel_shift(frame_ptM_x, box_ptC_x);
	
	/*same distance to be applied to all points in frame,
	 * thus resulting in some pixels disappearing/moving forward  in
	 *  new frame, or moving 
	 * that distance  backwards / disappearing in new frame.
	 */
	t->params.pan_tgt = track_map_inframe_shift_to_servo_pos(1, d_ptCptM, t->params.pan_params.position);
	if (t->params.pan_tgt < 0) {
		printf("%s: %d error .\n",  __func__, __LINE__);
		sleep(1000);
		return -EINVAL;
	}

	printf("box_ptC_x=%d.\n", box_ptC_x);
	printf("current pan pos: %d.\n", t->params.pan_params.position);
	printf("pan_tgt:  %d .\n", t->params.pan_tgt);

retry_pan:
	d = servoio_get_position(t->params.dev, t->params.pan_params.channel);
	if (d < 0) {
		printf("%s: %d error %d.\n", __func__, __LINE__, d);
		return -EIO;
	}
	else
		printf("%s: %d pan = %d.\n", __func__, __LINE__, d);

	if (t->params.pan_params.position != t->params.pan_tgt) {
		ret = servoio_set_pulse(t->params.dev, t->params.pan_params.channel, t->params.pan_tgt);
		if (ret < 0) {
			printf("%s: %d error %d.\n",  __func__, __LINE__,ret);
			return -EIO;
		}
		++(t->stats.pan_stats.cmdstally[SERVOIO_WRITE]);
	}
	
	d = servoio_get_position(t->params.dev, t->params.pan_params.channel);
	if (d < 0) {
		printf("%s: %d error %d.\n", __func__, __LINE__, d);
		sleep(1000);
		return -EIO;
	}
	
	++(t->stats.pan_stats.cmdstally[SERVOIO_READ]);	
	e = t->params.pan_tgt - d;
	if (e) {
		printf("%s pan target: %d current %d err %d.\n", __func__,
		       t->params.pan_tgt, d, e);

			/* hack: remove this */
			if (count--)
				goto retry_pan;
	}

	if (e < t->stats.pan_stats.min_poserr)
		t->stats.pan_stats.min_poserr = e;

	if (e > t->stats.pan_stats.max_poserr)
		t->stats.pan_stats.max_poserr = e;
	t->stats.pan_stats.rt_err = e;
	t->params.pan_params.poserr = e;
	
	t->params.pan_params.position = d;
		
	if (d < t->stats.pan_stats.min_pos)
		t->stats.pan_stats.min_pos = d;

	/* repeat same algorithm for tilt, make a function! */
	box_ptC_y = (t->params.bbox.ptB_y - t->params.bbox.ptA_y) >> 1;
	box_ptC_y += t->params.bbox.ptA_y;
	printf("box_ptC_y=%d.\n", box_ptC_y);

	count = 10;
retry_tilt:
	
	t->params.tilt_params.position = servoio_get_position(t->params.dev, t->params.tilt_params.channel);
	++(t->stats.pan_stats.cmdstally[SERVOIO_READ]);

	d_ptCptM = frame_ptM_y - box_ptC_y;

	t->params.tilt_tgt = track_map_inframe_shift_to_servo_pos(0, d_ptCptM, t->params.tilt_params.position); 

	printf("tilt_tgt:  %d .\n", t->params.tilt_tgt);
	if (t->params.tilt_params.position != t->params.tilt_tgt) {
		ret = servoio_set_pulse(t->params.dev,
					t->params.tilt_params.channel,
					t->params.tilt_tgt);
		if (ret < 0) {
			printf("%s error %d.\n", __func__, ret);
			sleep(1000);
		}
		++(t->stats.tilt_stats.cmdstally[SERVOIO_WRITE]);
	}

	d = servoio_get_position(t->params.dev, t->params.tilt_params.channel);
	++(t->stats.pan_stats.cmdstally[SERVOIO_READ]);
	e = t->params.tilt_tgt - d;

	if (e) {
		printf("%s tilt target: %d current %d err %d.\n", __func__, t->params.tilt_tgt, d, e);

			/* hack: remove this */
			if (count--)
				goto retry_tilt;
	}

	printf("%s tilt target: %d current %d err %d.\n", __func__,
	       t->params.tilt_tgt, d, e);
	
	if (e < t->stats.tilt_stats.min_poserr)
		t->stats.tilt_stats.min_poserr = e;
	
	if (e > t->stats.tilt_stats.max_poserr)
		t->stats.tilt_stats.max_poserr = e;
	
	t->stats.tilt_stats.rt_err = e;
	t->params.tilt_params.poserr = e;
	t->params.tilt_params.position = d;
	if (d < t->stats.tilt_stats.min_pos)
		t->stats.tilt_stats.min_pos = d;

	return 0;
}


int track_get_max_abse(struct tracker *t)
{
	int pabse, tabse;
	pabse = abs(t->stats.pan_stats.rt_err);
	tabse = abs(t->stats.tilt_stats.rt_err);

	return ((abs(pabse) > abs(tabse))? pabse : tabse);
}

int track_print_stats(struct tracker *t)
{
	return 0;
}
