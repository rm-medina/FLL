
#include "capture.h"
#include <errno.h>
#include <stdio.h>
#if defined(HAVE_OPENCV2)

#include "highgui/highgui_c.h"
#include "imgproc/imgproc_c.h"
#include "objdetect/objdetect.hpp"

/*
 * detector filter definition:
 * haarcascade_frontalface_default.xml
 * We locad the trained detector from a file.
 */
const char* cascade_xml = "../haarcascade_frontalfrace_default.xml";
static CvLatentSvmDetector* cdtSVM_detector = 0;
static CvHaarClassifierCascade* cdtHaar_detector = 0;
static CvCapture* videocam;
static CvMemStorage* scratchbuf = 0;


/*function prototypes: */
static int capture_initialize(enum capture_detector_t cdt);
static int capture_get_frame(IplImage* srcframe, int frame_idx);
static int capture_tear_down(IplImage* dstframe);
static int capture_configure_detector(const IplImage* srcframe,
				      IplImage* dstframe,
				      enum capture_detector_t cdt);

static CvSeq* capture_run_Haar_detector(IplImage* frame,
					CvMemStorage* const buffer);
static CvSeq* capture_run_latentSVM_detector(IplImage* frame,
					     CvMemStorage* const buffer);
static int capture_overlay_bboxes(CvSeq* faces, IplImage* img, int scale);
/*
 *
 */
static int capture_initialize(enum capture_detector_t cdt) {

	if (cdt == CDT_HAAR)
		cdtHaar_detector = (CvHaarClassifierCascade*)cvLoad(
			cascade_xml, 0, 0, 0 );
	else if (cdt == CDT_LSVM)
		cdtSVM_detector = cvLoadLatentSvmDetector(cascade_xml);
	else
		return -EINVAL;
	
	if ((cdtHaar_detector == 0) && (cdtSVM_detector == 0))
		/* no filter data */
		return -EINVAL;
	
	videocam = cvCreateCameraCapture(CV_CAP_ANY); /*autodetect*/
	if (!videocam)
		return -ENODEV;

	scratchbuf = cvCreateMemStorage(0); /*block_size: 0->64K*/
	if (!scratchbuf)
		return -ENOMEM;

	/* create display window */
	cvNamedWindow("FLL", CV_WINDOW_AUTOSIZE);
	return 0;
}

static int capture_tear_down(IplImage* dstframe) {
	cvDestroyWindow("FLL");
	cvDestroyWindow("FLL Grey");
	cvReleaseImage(&dstframe);
	if (scratchbuf)
		cvReleaseMemStorage(&scratchbuf);
	return 0;
}

/*
 * step1 : get frame
 *
 */
int capture_get_frame(IplImage* srcframe, int frame_idx) {

	if (cvGrabFrame(videocam)) {
		/* how do I use the srcframe information? */
		srcframe = cvRetrieveFrame(videocam, frame_idx);
		if (!srcframe)
			return -EINVAL;
		
		printf("Captured %dth image(%p): %dx%d with [%d channels,"
		       "%d step, %p data.\n",
		       frame_idx, srcframe, srcframe->height,
		       srcframe->width, srcframe->nChannels,
		       srcframe->widthStep, srcframe->imageData);
		

		/* display captured frame */
		cvShowImage("FLL", (CvArr*)srcframe);
		return 0;
	}
	else
		return -ENODEV;
}

/*
 * step2: set detector (detection on gray img)
 *
 */
int capture_configure_detector(const IplImage* const srcframe,
			       IplImage* dstframe,
			       enum capture_detector_t cdt) {
	
	dstframe = cvCreateImage(cvSize(srcframe->width, srcframe->height),
				 srcframe->depth, srcframe->nChannels);
	if (!dstframe)
		return -ENOMEM;
	
	/* grey image might only be needed for Haar */
	if (cdt == CDT_HAAR) {
		cvCvtColor(srcframe->imageData, dstframe->imageData,
			   CV_BGR2GRAY);
		cvNamedWindow("FLL Grey", CV_WINDOW_AUTOSIZE);
		cvShowImage("FLL Grey", (CvArr*)dstframe);
	}
	
	return 0;
}		
/*
 * step3: run detector
 *
 */
static CvSeq* capture_run_Haar_detector(IplImage* frame,
					CvMemStorage* const buffer)  { 
	CvSeq* faces;
	
	cvClearMemStorage(buffer);
	/* using Haar Classifier */
	faces = cvHaarDetectObjects(frame, cdtHaar_detector, buffer,
				    1.1, /*default scale factor*/
				    3, /*default min neighbors*/
				    CV_HAAR_DO_CANNY_PRUNING,
				    cvSize(0, 0), /*min size*/
				    cvSize(40, 40) /*max size*/
		); 
		return faces;  
}

static CvSeq* capture_run_latentSVM_detector(IplImage* frame,
					     CvMemStorage* const buffer) {
	CvSeq* faces;

	cvClearMemStorage(buffer);
	/* using LSVM classifier */
	faces = cvLatentSvmDetectObjects(frame, cdtSVM_detector, buffer,
		0.15f, /* default overlap threshold */
		-1     /* default number of threads */
		);
	return faces;
}

/*
 * step4: draw a rectangle around the detected face(s)
 *
 */
static int capture_overlay_bboxes(CvSeq* faces, IplImage* img, int scale) {

	int i;
	CvPoint pt1, pt2;
	
	for (i = 0; i < (faces? faces->total : 0); i++)
	{
                /* Create a new rectangle for drawing the face */
		CvRect* r = (CvRect*)cvGetSeqElem(faces, i);

		/* Find the dimensions of the face,and scale it if necessary */
		pt1.x = r->x*scale;
		pt2.x = (r->x+r->width)*scale;
		pt1.y = r->y*scale;
		pt2.y = (r->y+r->height)*scale;

		/* Draw the rectangle in the input image */
		cvRectangle( img, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
	}
	cvShowImage("FLL Grey", (CvArr*)img);
	return 0;
}


/*
 * step 5: provide face position for system to take action on servos.
 *
 */


int capture_process(enum capture_detector_t cdt, int scale) {
	IplImage* srcframe, *dstframe;
	CvSeq* faces;
	int nrframes, ret;
	
	ret = capture_initialize(cdt);
	if (ret < 0)
		return ret;
	srcframe = dstframe = 0;	
	for (nrframes=0;;nrframes++) {
		ret = capture_get_frame(srcframe,nrframes);
		if (ret < 0)
			goto capture_error;
		capture_configure_detector(srcframe, dstframe, cdt);
		if (cdt == CDT_HAAR)
			faces = capture_run_Haar_detector(dstframe, scratchbuf);
		else if (cdt == CDT_LSVM)
			faces = capture_run_latentSVM_detector(dstframe, scratchbuf);
		else
			break;
		ret = capture_overlay_bboxes(faces, dstframe, scale);
	}
	return 0;
capture_error:
	capture_tear_down(dstframe);
	return ret;
}

#else

int capture_process(enum capture_detector_t cdt, int scale) {
	return -ENODEV;
}

#endif
