/**
 * @file facelockedloop/test-haar.c
 * test program to evaluate haar cascade detection.
 * The results depend on the training data supplied via xml files.
 */

#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include "highgui/highgui_c.h"
#include "imgproc/imgproc_c.h"
#include "objdetect/objdetect.hpp"

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
	CvCapture *videocam;
	IplImage *srcframe, *dstframe;
	CvMemStorage* scratchbuf;
	CvSeq* faces;

	struct timespec start_time, stop_time, duration;
	int nframe = 0;
	int i;
	char c;
	const char* cascade_train = "haarcascade_frontalface_default.xml";
	CvHaarClassifierCascade* cdtHaard;
  
	if (!argc || !argv)
		printf("No input args to test-display.\n");

	scratchbuf = cvCreateMemStorage(0); /*block_size: 0->64K*/
	if (!scratchbuf)
		return -ENOMEM;

	videocam = cvCreateCameraCapture(CV_CAP_ANY); 
	if (!videocam)
		return -ENODEV;

	printf("Loading %s.\n", cascade_train);
	cdtHaard = (CvHaarClassifierCascade*)cvLoad(cascade_train, 0, 0, 0 );
	if (!cdtHaard) {
		printf("Failed to load classifier training file.\n");
		return -EBADF;
	}
	
	cvNamedWindow("FLL display test", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("FLL detection test", CV_WINDOW_AUTOSIZE);
	
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
		dstframe = cvCreateImage(cvSize(srcframe->width,
						srcframe->height),
					 srcframe->depth, 1);
		if (!dstframe)
			goto loopback;

		printf("Grey %dth image(%p): %dx%d with [%d channels,"
		       "%d step, %p data.\n",
		       nframe, dstframe, dstframe->height,
		       dstframe->width, dstframe->nChannels,
		       dstframe->widthStep, dstframe->imageData);
				
		cvCvtColor(srcframe, dstframe, CV_BGR2GRAY);
		cvClearMemStorage(scratchbuf);	
		faces = cvHaarDetectObjects(dstframe, cdtHaard, scratchbuf,
				    1.2, /*default scale factor: 1.1*/
				    2,   /*default min neighbors: 3*/
				    CV_HAAR_DO_CANNY_PRUNING,
				    cvSize(60, 60),  /*min size*/
				    cvSize(180, 180) /*max size*/
			); 
		if (!faces)
			printf("No faces.\n");
		printf("%d faces.\n", faces->total);
		for (i = 0; i < faces->total; i++) {
			CvRect* rAB = (CvRect*)cvGetSeqElem(faces, i);
			CvPoint ptA, ptB;
			int scale = 1;
			ptA.x = rAB->x * scale;
			ptB.x = (rAB->x + rAB->width)*scale;
			ptA.y = rAB->y*scale;
			ptB.y = (rAB->y+rAB->height)*scale;
			cvRectangle(dstframe, ptA, ptB, CV_RGB(255,0,0),
				    3, 8, 0 );
			printf("(%d,%d) and (%d,%d).\n", ptA.x, ptA.y,
			       ptB.x, ptB.y);

		}
		cvShowImage("FLL detection test", (CvArr*)dstframe);


loopback:
		c = cvWaitKey(20);
		if (c == 'q')
			break;
	}

	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	timespec_substract(&duration, &start_time, &stop_time);
	printf("duration->  %lds %ldns .\n", duration.tv_sec , duration.tv_nsec );
	cvDestroyWindow("FLL display test");
	cvDestroyWindow("FLL detection test");
	if (scratchbuf)
		cvReleaseMemStorage(&scratchbuf);
	return 0;
}
