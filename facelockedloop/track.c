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

static void track_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe);
static void track_stage_down(struct stage *stg);
static int track_stage_run(struct stage *stg);
static void track_stage_wait(struct stage *stg);
static void track_stage_go(struct stage *stg);

static struct stage_ops track_ops = {
	.up = track_stage_up, 
	.down = track_stage_down,
	.run = track_stage_run,
	.wait = track_stage_wait,
	.go = track_stage_go,
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

	tracer = container_of(stg, struct tracker, step);
	return track_run(tracer);
}

static void track_stage_wait(struct stage *stg)
{
	stage_wait(stg);
}

static void track_stage_go(struct stage *stg)
{
	stage_go(stg);
}


int track_initialize(struct tracker *t, struct tracker_params *p,
		     struct pipeline *pipe)
{
  	struct stage_params stgparams;
	int ret;

	stgparams.nth_stage = TRACKING_STAGE;
	stgparams.data_in = NULL;
	stgparams.data_out = NULL;

	t->params = *p;
	ret = servoio_all_go_home(t->params.dev);
	if (ret < 0)
		printf("%s home error %d.\n", __func__, ret);

	p->pan_params.position =
		servoio_get_position(p->dev,
				     p->pan_params.channel);
	printf("%s pan pos %d home %d.\n", __func__, p->pan_params.position,
		p->pan_params.home_position);
	p->tilt_params.position = servoio_get_position(p->dev,
						       p->tilt_params.channel);
	printf("%s tilt pos %d home %d.\n", __func__, p->tilt_params.position,
		p->tilt_params.home_position);
	
	stage_up(&t->step, &stgparams, &track_ops, pipe);
	return ret;
	
}

void track_teardown(struct tracker *t)
{
	int ret;
	
	ret = servoio_all_go_home(t->params.dev);
	if (ret < 0)
		printf("%s home error %d.\n", __func__, ret);

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

	if (ret < 0)
		printf("%s go home failed.\n", __func__);

	ret = servoio_configure(t->params.dev,
				t->params.pan_params.channel,
				0, 0, 0);
	if (ret < 0)
		printf("%s stopping pan servo failed %d.\n", __func__, ret);

	ret = servoio_configure(t->params.dev,
				t->params.tilt_params.channel,
				0, 0, 0);
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
	
	t->params.pan_params.home_position = t->params.pan_params.position;
	t->params.tilt_params.home_position =
		t->params.tilt_params.home_position;
}

int track_run(struct tracker *t)
{
	int  d, e;
	int ret = 0;
	
	if (t->params.pan_params.position != t->params.pan_tgt)
		ret = servoio_set_pulse(t->params.dev,
					t->params.pan_params.channel,
					t->params.pan_tgt);
	else
		goto tilt;
	
	if (ret < 0)
		printf("%s error %d.\n", __func__, ret);
	else
		t->params.pan_params.position = t->params.pan_tgt;

	if (t->params.pan_params.position < t->stats.pan_stats.min_pos)
		t->stats.pan_stats.min_pos = t->params.pan_params.position;
	
	d = servoio_get_position(t->params.dev, t->params.pan_params.channel);
	e = t->params.pan_tgt - d;
	printf("%s pan target: %d current %d err %d./n", __func__,
	       t->params.pan_tgt, d, e);

	if (e < t->stats.pan_stats.min_poserr)
		t->stats.pan_stats.min_poserr = e;

	if (e > t->stats.pan_stats.max_poserr)
		t->stats.pan_stats.max_poserr = e;

	t->params.pan_params.poserr = e;

tilt:
	if (t->params.tilt_params.position != t->params.tilt_tgt)
		ret = servoio_set_pulse(t->params.dev,
					t->params.tilt_params.channel,
					t->params.pan_tgt);
	else
		goto done;
	
	if (ret < 0)
		printf("%s error %d.\n", __func__, ret);
	else
		t->params.tilt_params.position = t->params.tilt_tgt;

	d = servoio_get_position(t->params.dev, t->params.tilt_params.channel);
	e = t->params.tilt_tgt - d;
	printf("%s tilt target: %d current %d err %d./n", __func__,
	       t->params.tilt_tgt, d, e);

	if (e < t->stats.tilt_stats.min_poserr)
		t->stats.tilt_stats.min_poserr = e;

	if (e > t->stats.tilt_stats.max_poserr)
		t->stats.tilt_stats.max_poserr = e;

	t->params.tilt_params.poserr = e;
done:
	
	return ret;
}

int track_get_max_abse(struct tracker *t)
{
	int pabse, tabse;
	pabse = (abs(t->stats.pan_stats.min_poserr) >
		 abs(t->stats.pan_stats.max_poserr))?
		t->stats.pan_stats.min_poserr : t->stats.pan_stats.max_poserr;

	
	tabse = (abs(t->stats.tilt_stats.min_poserr) >
		 abs(t->stats.tilt_stats.max_poserr))?
		t->stats.tilt_stats.min_poserr : t->stats.tilt_stats.max_poserr;

	return ((abs(pabse) > abs(tabse))? pabse : tabse);
}

int track_print_stats(struct tracker *t)
{
	return 0;
}
