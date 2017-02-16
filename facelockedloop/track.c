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

int track_run(struct tracker *t)
{
	int  d, e;
	int ret = 0;
	int i, pan_zskip, tilt_zskip;
	/* calculate new target */

	for (i = 1; i < ZONES_N; i++)
		if (t->params.bbox.ptA_x > pan_zones[i])
			continue;
		else
			break;
	pan_zskip = (i != PtA_MIDDLE_ZONE)?
		(PtA_MIDDLE_ZONE - i) : 0;

	/* if pan_zskip=0, check PtB */
	if (!pan_zskip)
		for (i =PtA_MIDDLE_ZONE; i < ZONES_N; i++)
			if (t->params.bbox.ptB_x > pan_zones[i])
				continue;
			else
				break;
	pan_zskip = (i != PtB_MIDDLE_ZONE)?
		(PtB_MIDDLE_ZONE -i) : 0;
	
	/* positive pan_zskip => left
	 * negative pan_zskip => right
	 */
	if (pan_zskip)
		t->params.pan_tgt = HOME_POSITION_QUARTER_US +
			SERVOLIB_DEF_STEP_QUARTER_US * pan_zskip;
		
	for (i = 1; i < ZONES_N; i++)
		if (t->params.bbox.ptA_y > tilt_zones[i])
			continue;
		else
			break;
	tilt_zskip = (i != PtA_MIDDLE_ZONE)? 
		(PtA_MIDDLE_ZONE - i) : 0;

	/* if pan_zskip=0, check PtB */
	if (!tilt_zskip)
		for (i =PtA_MIDDLE_ZONE; i < ZONES_N; i++)
			if (t->params.bbox.ptB_x > tilt_zones[i])
				continue;
			else
				break;
	tilt_zskip = (i != PtB_MIDDLE_ZONE)?
		(PtB_MIDDLE_ZONE -i) : 0;

	/* positive tilt_zskip => up
	 * negative tilt_zskip => down
	 */
	if (tilt_zskip)
		t->params.tilt_tgt = HOME_POSITION_QUARTER_US +
			SERVOLIB_DEF_STEP_QUARTER_US * tilt_zskip;

	printf("tilt_tgt: %d, tilt_zskip: %d pan_tgt: %d, pan_zskip: %d .\n",
	       t->params.tilt_tgt, tilt_zskip, t->params.pan_tgt, pan_zskip);
	
	if (t->params.pan_params.position != t->params.pan_tgt) {
		ret = servoio_set_pulse(t->params.dev,
					t->params.pan_params.channel,
					t->params.pan_tgt);
		++(t->stats.pan_stats.cmdstally[SERVOIO_WRITE]);

		if (ret < 0)
			printf("%s error %d.\n", __func__, ret);
		else
			t->params.pan_params.position = t->params.pan_tgt;

		if (t->params.pan_params.position < t->stats.pan_stats.min_pos)
			t->stats.pan_stats.min_pos = t->params.pan_params.position;
	
		d = servoio_get_position(t->params.dev, t->params.pan_params.channel);
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
	}

	if (t->params.tilt_params.position != t->params.tilt_tgt) {
		ret = servoio_set_pulse(t->params.dev,
					t->params.tilt_params.channel,
					t->params.pan_tgt);
		++(t->stats.tilt_stats.cmdstally[SERVOIO_WRITE]);
	
		if (ret < 0)
			printf("%s error %d.\n", __func__, ret);
		else
			t->params.tilt_params.position = t->params.tilt_tgt;

		d = servoio_get_position(t->params.dev, t->params.tilt_params.channel);
		++(t->stats.pan_stats.cmdstally[SERVOIO_READ]);
		e = t->params.tilt_tgt - d;
		printf("%s tilt target: %d current %d err %d.\n", __func__,
		       t->params.tilt_tgt, d, e);

		if (e < t->stats.tilt_stats.min_poserr)
			t->stats.tilt_stats.min_poserr = e;

		if (e > t->stats.tilt_stats.max_poserr)
			t->stats.tilt_stats.max_poserr = e;

		t->stats.pan_stats.rt_err = e;
		t->params.tilt_params.poserr = e;
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
