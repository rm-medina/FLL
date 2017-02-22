/**
 * @file facelockedloop/test-display.c
 * test program to check camera and display.
 *
 */

#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
//#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui_c.h"

/*
 * Substract the 'struct timespec' values end and start, storing the result in
 * diff.
 */
static void timespec_substract(struct timespec *const diff,
			       const struct timespec *const start,
			       const struct timespec *const end)
{
	if ((end->tv_nsec-start->tv_nsec)<0) {
		diff->tv_sec = end->tv_sec-start->tv_sec-1;
		diff->tv_nsec = 1000000000+end->tv_nsec-start->tv_nsec;
	} else {
		diff->tv_sec = end->tv_sec-start->tv_sec;
		diff->tv_nsec = end->tv_nsec-start->tv_nsec;
	}
}

int main(int argc, char *const argv[])
{
	CvCapture* videocam;
	IplImage* srcframe;
	struct timespec start_time, stop_time, duration;
	int nframe = 0;
	char c;

	if (!argc || !argv)
		printf("No input args to test-display.\n");
	
	videocam = cvCreateCameraCapture(CV_CAP_ANY+1); 
	if (!videocam)
		return -ENODEV;

	cvNamedWindow("FLL display test", CV_WINDOW_AUTOSIZE);
	clock_gettime(CLOCK_MONOTONIC, &start_time);

	for (;;) {
		if (!cvGrabFrame(videocam))
			goto loopback;
		srcframe = cvRetrieveFrame(videocam, nframe);
		if (!srcframe)
			goto loopback;
		++nframe;
		printf("Captured %dth image(%p): %dx%d with [%d channels,"
		       "%d step, %p data.\n",
		       nframe, srcframe, srcframe->height,
		       srcframe->width, srcframe->nChannels,
		       srcframe->widthStep, srcframe->imageData);
		cvShowImage("FLL display test", (CvArr*)srcframe);
loopback:
		c = cvWaitKey(10);
		if (c == 'q')
			break;
	}

	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	timespec_substract(&duration, &start_time, &stop_time);
	printf("duration->  %lds %ldns .\n", duration.tv_sec , duration.tv_nsec );
	cvDestroyWindow("FLL display test");
	return 0;
}
