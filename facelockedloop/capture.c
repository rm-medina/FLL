/**
 * @file facelockedloop/capture.c
 * @brief Frame capture stage, based on OpenCV libraries.
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include "capture.h"
#include "kernel_utils.h"

static void capture_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe);
static void capture_stage_down(struct stage *stg);
static int capture_stage_run(struct stage *stg);
static void capture_stage_wait(struct stage *stg);
static void capture_stage_go(struct stage *stg);

static struct stage_ops capture_ops = {
	.up = capture_stage_up, 
	.down = capture_stage_down,
	.run = capture_stage_run,
	.wait = capture_stage_wait,
	.go = capture_stage_go,
};

static void capture_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe)
{
	stage_up(stg, p, o, pipe);
	pipeline_register(pipe, stg);
	
}

static void capture_stage_down(struct stage *stg)
{
	struct imager *imgr;

	imgr = container_of(stg, struct imager, step);
	capture_teardown(imgr);
	stage_down(stg);
	pipeline_deregister(stg->pipeline, stg);
}

static int capture_stage_run(struct stage *stg)
{
	struct imager *imgr;

	imgr = container_of(stg, struct imager, step);
	return capture_run(imgr);
}

static void capture_stage_wait(struct stage *stg)
{
	stage_wait(stg);
}

static void capture_stage_go(struct stage *stg)
{
	stage_go(stg);
}


#if HAVE_OPENCV2

int capture_initialize(struct imager *i, struct imager_params *p,
		       struct pipeline *pipe)
{
	struct stage_params stgparams;
	char *wname;
	int ret;
	
	stgparams.nth_stage = CAPTURE_STAGE;
	stgparams.data_in = NULL;
	stgparams.data_out = NULL;

	i->params = *p;
	i->params.videocam = cvCreateCameraCapture(CV_CAP_ANY +
						   i->params.vididx); 
	if (!(i->params.videocam))
		return -ENODEV;

	ret = asprintf(&wname, "FLL cam%d", i->params.vididx);
	if (ret < 0)
		return -ENOMEM;
	
	cvNamedWindow(wname, CV_WINDOW_AUTOSIZE);

	capture_stage_up(&i->step, &stgparams, &capture_ops, pipe);
	return ret;
}

void capture_teardown(struct imager *i)
{
	char *wname;
	int ret;
	ret = asprintf(&wname, "FLL cam%d", i->params.vididx);
	if (!ret)
		cvDestroyWindow(wname);
}

int capture_run(struct imager *i)
{
	IplImage *srcframe;
	char* wname;
	int ret;
	
	if (i->params.vididx < 0)
		return -EINVAL;
	
	if (!(i->params.videocam))
		return -ENODEV;

	if (cvGrabFrame(i->params.videocam)) {
		srcframe = cvRetrieveFrame(i->params.videocam,
					   i->params.frameidx);
		if (!srcframe)
			return -EIO;

		i->params.frame = srcframe;
		printf("cam%d captured %dth image(%p): %dx%d with [%d channels,"
		       "%d step, %p data.\n",
 		       i->params.vididx , i->params.frameidx, srcframe,
		       srcframe->height, srcframe->width, srcframe->nChannels,
		       srcframe->widthStep, srcframe->imageData);
		ret = asprintf(&wname, "FLL cam%d", i->params.vididx);
		if (!ret)
			cvShowImage(wname, (CvArr*)srcframe);
		++(i->stats.tally);
	}
	return 0;
}

int capture_get_imgcount(struct imager *i)
{
	return i->stats.tally;
}

#else

int capture_initialize(struct imager *i, struct imager_params *p,
		       struct pipeline *pipe)
{
	return -ENODEV;
}
	
int capture_run(struct imager *i)
{
	return -EINVAL;
}
	
void capture_teardown(struct imager *i)
{
	return;
}

int capture_get_imgcount(struct imager *i)
{
	return 0;
}

#endif /*HAVE_OPENCV2*/

int capture_print_stats(struct imager *i)
{
	return 0;
}
