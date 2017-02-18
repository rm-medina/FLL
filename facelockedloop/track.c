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
 * 4000 0.25us => izqda, arriba
 * 6000 0.25us => centro, centro
 * 8000 0.25us => derecha, abajo
 */
int track_run(struct tracker *t)
{
	int  d, e, i, pan_pskip, tilt_pskip;
	int ret = 0;

	int pan_zskip = 0;
	int tilt_zskip = 0;
	int box_ptC_x, box_ptC_y;

	box_ptC_x = (t->params.bbox.ptB_x - t->params.bbox.ptA_x) >> 1;
	box_ptC_x += t->params.bbox.ptA_x;
	printf("box_ptC_x=%d.\n", box_ptC_x);
	
	pan_pskip = box_ptC_x - Pt_x_MIDDLE;
	if (!pan_pskip) {
		d = servoio_get_position(t->params.dev,
					 t->params.pan_params.channel);
	        if ((d < 4000) || (d > 8000))
			printf("get pan position error %d.\n", d);
		else
			t->params.pan_tgt = t->params.pan_params.position = d;
		goto pan_over;
	}	
	for (i = 1; i < ZONES_N; i++)
		if (box_ptC_x > pan_zones[i])
			continue;
		else
			break;
	--i;
	pan_zskip = ((i != PtA_MIDDLE_ZONE) && (i!= PtB_MIDDLE_ZONE))?
	  (PtB_MIDDLE_ZONE - i) : 0;

	printf("pan ptC: pan_pskip:%d i=%d, pan_zskip:%d.\n", pan_pskip,
	       i, pan_zskip);

	/* positive pan_pskip => Pt < Center 
	 *                    => face on left half, move it right 
	 *		      => shift camera left
	 * negative pan_pskip => Pt > Center
	 *                    => face on right half, move it left
	 *                    => shift camera right
	 *
	 * 4000 0.25us / 640 pixels => 25/4 = 6.25 0.25us/pixel
	 */
	if (pan_pskip)
		t->params.pan_tgt = HOME_POSITION_QUARTER_US -
			((25 * pan_pskip)>>4);
	
	printf("pan_tgt:  %d .\n",
	       t->params.pan_tgt);

pan_over:	
	/* repeat same algorithm for tilt, make a function! */
	box_ptC_y = (t->params.bbox.ptB_y - t->params.bbox.ptA_y) >> 1;
	box_ptC_y += t->params.bbox.ptA_y;
	printf("box_ptC_y=%d.\n", box_ptC_y);
	
	tilt_pskip = box_ptC_y - Pt_y_MIDDLE;
	if (!tilt_pskip) {
		d = servoio_get_position(t->params.dev,
					 t->params.tilt_params.channel);
	        if (d < 0)
			printf("get tilt position error %d.\n", d);
		else
			t->params.tilt_tgt = t->params.tilt_params.position = d;

		goto tilt_over;
	}
	for (i = 1; i < ZONES_N; i++)
		if (box_ptC_y > tilt_zones[i])
			continue;
		else
			break;
	--i;
	tilt_zskip = ((i != PtA_MIDDLE_ZONE) && (i != PtB_MIDDLE_ZONE))? 
		(i - PtB_MIDDLE_ZONE) : 0;

	printf("tilt ptc: tilt_pskip:%d i=%d, tilt_zskip:%d.\n", tilt_pskip,
	       i, tilt_zskip);

	/* positive tilt_pskip => Pt < Center 
	 *                     => face on top half, move it down 
	 *		       => shift camera up
	 * negative tilt_pskip => Pt > Center
	 *                     => face on bottom half, move it up
	 *                     => shift camera down
	 *
	 * 4000 0.25us / 480 pixels => 25/3 = 8.33 0.25us/pixel
	 */

	if (tilt_zskip)
		/* maybe even diminish tilt rate to avoid loosing  track,
		   do >>4 instead of >>2?
		 */
		t->params.tilt_tgt = HOME_POSITION_QUARTER_US -
			(((50 * pan_pskip)/3)>>2);

	printf("tilt_tgt:  %d .\n",
	       t->params.tilt_tgt);

tilt_over:
	
	if (t->params.pan_params.position != t->params.pan_tgt) {
		ret = servoio_set_pulse(t->params.dev,
					t->params.pan_params.channel,
					t->params.pan_tgt);
		++(t->stats.pan_stats.cmdstally[SERVOIO_WRITE]);

		if (ret < 0)
			printf("%s error %d.\n", __func__, ret);

		d = servoio_get_position(t->params.dev,
					 t->params.pan_params.channel);

		++(t->stats.pan_stats.cmdstally[SERVOIO_READ]);
		e = t->params.pan_tgt - d;

		printf("%s pan target: %d current %d err %d.\n", __func__,
		       t->params.pan_tgt, d, e);

		if (e < t->stats.pan_stats.min_poserr)
			t->stats.pan_stats.min_poserr = e;

		if (e > t->stats.pan_stats.max_poserr)
			t->stats.pan_stats.max_poserr = e;
		t->stats.pan_stats.rt_err = e;
		t->params.pan_params.poserr = e;

		t->params.pan_params.position = d;
		
		if (d < t->stats.pan_stats.min_pos)
			t->stats.pan_stats.min_pos = d;
	
	}

	if (t->params.tilt_params.position != t->params.tilt_tgt) {
		ret = servoio_set_pulse(t->params.dev,
					t->params.tilt_params.channel,
					t->params.tilt_tgt);
		++(t->stats.tilt_stats.cmdstally[SERVOIO_WRITE]);
	
		if (ret < 0)
			printf("%s error %d.\n", __func__, ret);

		d = servoio_get_position(t->params.dev, t->params.tilt_params.channel);
		++(t->stats.pan_stats.cmdstally[SERVOIO_READ]);
		e = t->params.tilt_tgt - d;
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
	}
	
	return ret;
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
