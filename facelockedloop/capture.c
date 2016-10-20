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

#if defined(HAVE_OPENCV2)

#include "highgui/highgui_c.h"
#define CAMERA_MAX_NUMBER 1

static CvCapture* videocam[CAMERA_MAX_NUMBER];
static int ncam;

struct imager_stats {
	int tally;
	int fps;
};

struct imager_params {
	int camidx;
	IplImage* frame;
};

struct imager {
	struct stage *step;
	struct capture_params params;
	struct capture_stats;
	int status;
};

int capture_initialize(struct imager* i, struct imager_params *p)
{
	if (ncam >= CAMERA_MAX_NUMBER)
		return -EINVAL;
	videocam[ncam] = cvCreateCameraCapture(CV_CAP_ANY + vindex); 
	if (!videocam[ncam])
		return -ENODEV;
	++ncam;
	cvNamedWindow(sscanf("FLL cam%d",ncam), CV_WINDOW_AUTOSIZE);
	return ncam;
}

void capture_teardown(void)
{
	int i;
	
	for (i=0; i<ncam; i++)
		cvDestroyWindow(sscanf("FLL cam%d",i));
}

int capture_run(int camid, IplImage* srcframe, int frame_idx)
{
	if (camid > CAMERA_MAX_NUMBER)
		return -EINVAL;
	
	if (!videocam[camid-1])
		return -ENODEV;

	if (cvGrabFrame(videocam[camid-1])) {
		srcframe = cvRetrieveFrame(videocam[camid-1], frame_idx);
		if (!srcframe)
			return -EIO;
		
		printf("cam%d captured %dth image(%p): %dx%d with [%d channels,"
		       "%d step, %p data.\n",
		       camid, frame_idx, srcframe, srcframe->height,
		       srcframe->width, srcframe->nChannels,
		       srcframe->widthStep, srcframe->imageData);
		cvShowImage(sscanf("FLL cam%d",camid-1), (CvArr*)srcframe);
	}
	return 0;
}

int capture_get_camcount(void)
{
	return ncam;
}

#else

int capture_initialize(int vindex)
{
	return -ENODEV;
}
	
int capture_run(void* srcframe, int frame_idx)
{
	return -EINVAL;
}
	
void capture_teardown(void)
{
	return;
}

int capture_get_camcount(void)
{
	return 0;
}

#endif /*HAVE_OPENCV2*/
