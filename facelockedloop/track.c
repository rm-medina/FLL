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

#include "track.h"


int track_initialize(struct tracker *t, struct tracker_params *p)
{
	int ret;

	ret = servoio_all_go_home(tracker->dev);
	if (ret < 0)
		printf("%s home error %d.\n", __func__, ret);

	p->pan_params.position = servoio_get_position(tracker->dev,
						      p->pan_params.channel);
	printf("%s pan pos %d home %d.\n", __func__, p->pan_params.position,
		p->pan_params.home_position);
	p->tilt_params.position = servoio_get_position(tracker->dev,
						       p->tilt_params.channel);
	printf("%s tilt pos %d home %d.\n", __func__, p->tilt_params.position,
		p->tilt_params.home_position);
	return ret;
}

int track_teardown(struct tracker *t)
{
	int ret;
	
	ret = track_initialize(t, &t->params);
	if (ret < 0)
		printf("%s go home failed.\n", __func__);
	ret = servoio_configure(t->dev,
				t->params.pan_params.channel,
				0, 0, 0);
	if (ret < 0)
		printf("%s stopping pan servo failed %d.\n", __func__, ret);

	ret = servoio_configure(t->dev,
				t->params.tilt_params.channel,
				0, 0, 0);
	if (ret < 0)
		printf("%s stopping tilt servo failed %d.\n", __func__, ret);

	p->pan_params.position = servoio_get_position(tracker->dev,
						      p->pan_params.channel);
	printf("%s pan pos %d home %d.\n", __func__, p->pan_params.position,
		p->pan_params.home_position);
	p->tilt_params.position = servoio_get_position(tracker->dev,
						       p->tilt_params.channel);
	printf("%s tilt pos %d home %d.\n", __func__, p->tilt_params.position,
		p->tilt_params.home_position);

	p->pan_params.home_position = p->pan_params.position;
	p->tilt_params.home_position = p->tilt_params.home_position;
	return 0;
}

int track_run(struct tracker *t)
{
	int  d, e;
	int ret = 0;
	
	if (t->params.pan_params.position != t->params.pan_tgt)
		ret = servoio_set_pulse(t->params.pan_params.channel,
					t->params.pan_tgt);
	else
		goto tilt;
	
	if (ret < 0)
		printf("%s error %d.\n", __func__, ret);
	else
		t->params.pan_params.position = t->params.pan_tgt;

	if (t->params.pan_params.position < t->params.pan_stats.min_position)
		t->params.pan_stats.min_position = t->params.pan_params.position;
	
	d = servoio_get_position(t->dev, t->params.pan_params.channel);
	e = t->params.pan_tgt - d;
	printf("%s pan target: %d current %d err %d./n",
	       t->params.pan_tgt, d, e);

	if (e < t->params.pan_stats.min_poserr)
		t->params.pan_stats.min_poserr = e;

	if (e > t->params.pan_stats.max_poserr)
		t->params.pan_stats.max_poserr = e;

	t->params.pan_params.poserr = e;

tilt:
	if (t->params.tilt_params.position != t->params.tilt_tgt)
		ret = servoio_set_pulse(t->params.tilt_params.channel,
					t->tilt.pan_tgt);
	else
		goto done;
	
	if (ret < 0)
		printf("%s error %d.\n", __func__, ret);
	else
		t->params.tilt_params.position = t->params.tilt_tgt;

	d = servoio_get_position(t->dev, t->params.tilt_params.channel);
	e = t->params.tilt_tgt - d;
	printf("%s tilt target: %d current %d err %d./n",
	       t->params.tilt_tgt, d, e);

	if (e < t->params.tilt_stats.min_poserr)
		t->params.tilt_stats.min_poserr = e;

	if (e > t->params.tilt_stats.max_poserr)
		t->params.tilt_stats.max_poserr = e;

	t->params.tilt_params.poserr = e;
done:
	
	return ret;
}

int track_get_max_abse(struct tracker *t)
{
	int pabse, tabse;
	pabse = max(t->params.pan_stats.min_poserr,
			t->params.pan_stats.max_poserr);

	tabse = max(t->params.tilt_stats.min_poserr,
			t->params.tilt_stats.max_poserr);

	return max(panabserr, tiltabserr);
}

int track_print_stats(struct tracker *t)
{
	return 0;
}
