/**
 * @file facelockedloop/capture.c
 * @brief Face detection module, based on OpenCV libraries.
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <errno.h>
#include <stdio.h>
#include <malloc.h>

#include "capture.h"
#include "store.h"

#if defined(HAVE_OPENCV2)

#include "highgui/highgui_c.h"
#include "imgproc/imgproc_c.h"
#include "objdetect/objdetect.hpp"

/*
 * cascade_xml is the trained detector filter definition, which loads 
 * from a file.
 */
static const char* cascade_xml = "../cascade_frontalfrace_default.xml";
static CvLatentSvmDetector* cdtSVM_det;
static CvHaarClassifierCascade* cdtHaar_det;
static CvCapture* videocam;
static CvMemStorage* scratchbuf;

static int capture_initialize(int vindex, enum capture_detector_t cdt);
static int capture_get_frame(IplImage* srcframe, int frame_idx);
static int capture_tear_down(IplImage* dstframe, enum capture_detector_t cdt);
static int capture_configure_detector(const IplImage* srcframe,
				      IplImage* dstframe,
				      enum capture_detector_t cdt);

static CvSeq* capture_run_Haar_detector(IplImage* frame,
					CvMemStorage* const buffer);
static CvSeq* capture_run_latentSVM_detector(IplImage* frame,
					     CvMemStorage* const buffer);
static int capture_overlay_bboxes(CvSeq* faces, IplImage* img, int scale);


static int capture_initialize(int vindex, enum capture_detector_t cdt)
{
	switch(cdt) {
	case CDT_HAAR:
		cdtHaar_det =
			(CvHaarClassifierCascade*)cvLoad(cascade_xml, 0, 0, 0 );
		if (!cdtHaar_det)
			return -ENOENT;
		break;
	case CDT_LSVM:
		cdtSVM_det = cvLoadLatentSvmDetector(cascade_xml);
		if (!cdtSVM_det)
			return -ENOENT;
		break;
	default:
		return -EINVAL;
	};
	
	videocam = cvCreateCameraCapture(CV_CAP_ANY+vindex); 
	if (!videocam)
		return -ENODEV;

	scratchbuf = cvCreateMemStorage(0); /*block_size: 0->64K*/
	if (!scratchbuf)
		return -ENOMEM;

	/* create display window */
	cvNamedWindow("FLL", CV_WINDOW_AUTOSIZE);
	return 0;
}

static int capture_tear_down(IplImage* dstframe, enum capture_detector_t cdt) {
	cvDestroyWindow("FLL");
	if (cdt == CDT_HAAR)
		cvDestroyWindow("FLL Grey");
	if (dstframe)
		cvReleaseImage(&dstframe);
	if (scratchbuf)
		cvReleaseMemStorage(&scratchbuf);
	return 0;
}

static int capture_get_frame(IplImage* srcframe, int frame_idx)
{
	if (!videocam)
		return -ENODEV;

	if (cvGrabFrame(videocam)) {
		srcframe = cvRetrieveFrame(videocam, frame_idx);
		if (!srcframe)
			return -EIO;
		
		printf("Captured %dth image(%p): %dx%d with [%d channels,"
		       "%d step, %p data.\n",
		       frame_idx, srcframe, srcframe->height,
		       srcframe->width, srcframe->nChannels,
		       srcframe->widthStep, srcframe->imageData);
		cvShowImage("FLL", (CvArr*)srcframe);
	}
	return 0;
}

static int capture_configure_detector(const IplImage* const srcframe,
			       IplImage* dstframe,
			       enum capture_detector_t cdt)
{
	dstframe = cvCreateImage(cvSize(srcframe->width, srcframe->height),
				 srcframe->depth, srcframe->nChannels);
	if (!dstframe)
		return -ENOMEM;
	
	/* grey image only be needed for Haar */
	if (cdt == CDT_HAAR) {
		cvCvtColor(srcframe->imageData, dstframe->imageData,
			   CV_BGR2GRAY);
		cvNamedWindow("FLL Grey", CV_WINDOW_AUTOSIZE);
		cvShowImage("FLL Grey", (CvArr*)dstframe);
	}
	return 0;
}		

static CvSeq* capture_run_Haar_detector(IplImage* frame,
					CvMemStorage* const buf)
{ 
	CvSeq* faces;
	
	if (!buf) 
		return 0;

	cvClearMemStorage(buf);	
	faces = cvHaarDetectObjects(frame, cdtHaar_det, buf,
				    1.1, /*default scale factor*/
				    3,   /*default min neighbors*/
				    CV_HAAR_DO_CANNY_PRUNING,
				    cvSize(60, 60),  /*min size*/
				    cvSize(180, 180) /*max size*/
		); 
	return faces;  
}

static CvSeq* capture_run_latentSVM_detector(IplImage* frame,
					     CvMemStorage* const buf)
{
	CvSeq* faces;
	
	if (!buf) 
		return 0;

	cvClearMemStorage(buf);	
	faces = cvLatentSvmDetectObjects(frame, cdtSVM_det, buf,
		0.15f, /* default overlap threshold */
		-1     /* default number of threads */
		);
	return faces;
}

static CvSeq* capture_run_detector(IplImage* frame, CvMemStorage* const buf,
				   enum capture_detector_t cdt)
{
	CvSeq* faces;

	if (!buf)
		return 0;

	cvClearMemStorage(buf);
	switch(cdt) {
	case CDT_HAAR:
		faces = cvHaarDetectObjects(frame, cdtHaar_det, buf,
					    1.2, /*default scale factor: 1.1*/
					    2,   /*default min neighbors: 3*/
					    CV_HAAR_DO_CANNY_PRUNING,
					    cvSize(10, 10),  /*min size*/
					    cvSize(160, 160) /*max size*/	); 
		break;
	case CDT_LSVM:
		faces =	cvLatentSvmDetectObjects(frame, cdtSVM_det,
						 buf,
						 0.15f, /* overlap threshold */
						 -1     /* threads number*/ );
		break;
	default:
		faces = 0;
	};
	if (!faces)
		printf("No face, cdt=%d.\n", cdt);
	return faces;
}

static int capture_overlay_bboxes(CvSeq* faces, IplImage* img, int scale)
{
	int i;
	CvPoint ptA, ptB;
	struct store_box *pos;
	if (!faces)
		return -EINVAL;
	pos = memalign(sizeof(struct store_box), faces->total);
	printf("%d faces.\n", faces->total);
	for (i = 0; i < faces->total; i++)
	{
		CvRect* rAB = (CvRect*)cvGetSeqElem(faces, i);
		ptA.x = rAB->x * scale;
		ptB.x = (rAB->x + rAB->width)*scale;
		ptA.y = rAB->y*scale;
		ptB.y = (rAB->y+rAB->height)*scale;
		cvRectangle(img, ptA, ptB, CV_RGB(255,0,0), 3, 8, 0 );
		printf("(%d,%d) and (%d,%d).\n", ptA.x, ptA.y, ptB.x, ptB.y);
		if (!pos)
			continue;
		pos[i].ptA_x = ptA.x;
		pos[i].ptA_y = ptA.y;
		pos[i].ptB_x = ptA.x;
		pos[i].ptB_y = ptA.y;
	}
	cvShowImage("FLL detection", (CvArr*)img);
	return 0;
}

int capture_process(int vindex, enum capture_detector_t cdt, int scale)
{
	IplImage* srcframe, *dstframe;
	CvSeq* faces;
	int n, ret;
	
	ret = capture_initialize(vindex, cdt);
	if (ret < 0)
		return ret;

	srcframe = 0;
	dstframe = 0;	
	for (n=0; ; n++) {
		printf("%s: step1 get frame.\n", __func__);
		ret = capture_get_frame(srcframe, n);
		if (ret < 0)
			break;

		printf("%s: step2 configure.\n", __func__);
		ret = capture_configure_detector(srcframe, dstframe, cdt);
		if (ret < 0)
			break;
		printf("%s: step3 run detector on frame.\n", __func__);
		faces = capture_run_detector(dstframe, scratchbuf, cdt);
		if (!faces)
			continue;
		printf("%s: step4 draw bboxes on frame.\n", __func__);
		ret = capture_overlay_bboxes(faces, dstframe, scale);
	}

	capture_tear_down(dstframe, cdt);
	return ret;
}

#else

int capture_process(enum capture_detector_t cdt, int scale)
{
	return -ENODEV;
}

int capture_get_facecount(void)
{
	return 0;
}

int capture_read_locations(struct store_box *faceset)
{
	faceset = 0;
	return -ENODEV;
}
#endif /*HAVE_OPENCV2*/
