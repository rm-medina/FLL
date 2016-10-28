/**
 * @file facelockedloop/capture.c
 * @brief Frame capture stage, based on OpenCV libraries.
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <errno.h>
#include <stdio.h>

#include "capture.h"

static struct stage_ops capture_ops = {
	.up = capture_stage_up, 
	.down = capture_stage_down,
	.run = capture_stage_run,
	.wait = capture_stage_wait,
	.go = capture_stage_go,
};

static void capture_stage_up(struct stage *stg, struct pipeline *pipe)
{
	stage_up();
	pipeline_register(stg, pipe);
	
}
static void capture_stage_down(struct stage *stg, struct pipeline *pipe)
{
	stage_down(stg);
	pipeline_deregister(stg, pipe);
}

static void capture_stage_run(struct stage *stg)
{
	stage_run(stg);
}

static void capture_stage_wait(struct stage *stg)
{
	stage_wait(stg);
}

static void capture_stage_go(struct stage *stg)
{
	stage_go(stg);
}


#if defined(HAVE_OPENCV2)

int capture_initialize(struct imager *i, struct imager_params *p,
		       struct pipeline *pipe)
{
	struct stage_params stgparams;
	int ret;
	
	stgparams.nth_stage = CAPTURE_STAGE;
	stgparams.data_in = NULL;
	stgparams.data_out = NULL;

	i->params = *p;
	i->params.videocam = cvCreateCameraCapture(CV_CAP_ANY +
						   i->params.vindex); 
	if (!(i->params.videocam))
		return -ENODEV;

	
	cvNamedWindow(sscanf("FLL cam%d", i->params.vindex),
		      CV_WINDOW_AUTOSIZE);

	ret = stage_up(i->step, &stgparams, &capture_ops, pipe);
	return ret;
}

void capture_teardown(struct imager *i)
{
	cvDestroyWindow(sscanf("FLL cam%d",i->params.vindex));
}

int capture_run(struct imager *i)
{
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
		cvShowImage(sscanf("FLL cam%d", i->params.vididx), (CvArr*)srcframe);
		++(i->stats.tally);
	}
	return 0;
}

int capture_get_imgcount(struct imager *i)
{
	return i->stats.tally;
}

#else

int capture_initialize(struct imager *i, struct imager_params *p)
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
