/**
 * @file facelockedloop/detect.c
 * @brief Face detection stage, based on OpenCV libraries.
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <errno.h>
#include <stdio.h>
#include <malloc.h>
#include "detect.h"
#include "store.h"
#include "kernel_utils.h"

#if defined(HAVE_OPENCV2)
#include "highgui/highgui_c.h"
#include "imgproc/imgproc_c.h"
#include "objdetect/objdetect.hpp"

static CvSeq* detect_run_Haar_algorithm(IplImage* frame,
					CvMemStorage* const buffer,
					void *algo);
static CvSeq* detect_run_latentSVM_algorithm(IplImage* frame,
					     CvMemStorage* const buffer,
					     void *algo);
static int detect_store(CvSeq* faces, IplImage* img, int scale);
#endif

static void detect_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe);
static void detect_stage_down(struct stage *stg);
static int detect_stage_run(struct stage *stg);
static void detect_stage_wait(struct stage *stg);
static void detect_stage_go(struct stage *stg);

static struct stage_ops detect_ops = {
	.up = detect_stage_up, 
	.down = detect_stage_down,
	.run = detect_stage_run,
	.wait = detect_stage_wait,
	.go = detect_stage_go,
};

static void detect_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe)
{
	stage_up(stg, p, o, pipe);
	pipeline_register(pipe, stg);
	
}

static void detect_stage_down(struct stage *stg)
{
	struct detector *algo;

	algo = container_of(stg, struct detector, step);
	detect_teardown(algo);
	stage_down(stg);
	pipeline_deregister(stg->pipeline, stg);
}

static int detect_stage_run(struct stage *stg)
{
	struct detector *algo;

	algo = container_of(stg, struct detector, step);
	return detect_run(algo);
}

static void detect_stage_wait(struct stage *stg)
{
	stage_wait(stg);
}

static void detect_stage_go(struct stage *stg)
{
	stage_go(stg);
}


#if defined(HAVE_OPENCV2)

/*
 * cascade_xml is the trained detector filter definition, which loads 
 * from a file.
 */
int detect_initialize(struct detector *d, struct detector_params *p,
		      struct pipeline *pipe)
{
	struct stage_params stgparams;
	CvLatentSvmDetector* cdtSVM_det;
	CvHaarClassifierCascade* cdtHaar_det;
	int ret = 0;

	stgparams.nth_stage = DETECTION_STAGE;
	stgparams.data_in = NULL;
	stgparams.data_out = NULL;

	d->params = *p;
	
	switch(d->params.odt) {
	case CDT_HAAR:
		if (p->cascade_xml == NULL)
			p->cascade_xml = "../cascade_frontalfrace_default.xml";
		cdtHaar_det =
			(CvHaarClassifierCascade*)cvLoad(d->params.cascade_xml, 0, 0, 0 );
		if (!cdtHaar_det)
			return -ENOENT;
		d->params.algorithm = (void*)cdtHaar_det;
		break;
	case CDT_LSVM:
		if (p->cascade_xml == NULL)
			p->cascade_xml = "../cascade_frontalfrace_default.xml";
		cdtSVM_det = cvLoadLatentSvmDetector(d->params.cascade_xml);
		if (!cdtSVM_det)
			return -ENOENT;
		d->params.algorithm = (void*)cdtSVM_det;
		break;
	default:
		return -EINVAL;
	};

	d->params.scratchbuf = cvCreateMemStorage(0); /*block_size: 0->64K*/
	if (d->params.scratchbuf == NULL)
		return -ENOMEM;

	detect_stage_up(&d->step, &stgparams, &detect_ops, pipe);
	return ret;
}

void detect_teardown(struct detector *d)
{
	if (d->params.odt == CDT_HAAR)
		cvDestroyWindow("FLL Grey");
	if (d->params.dstframe)
		cvReleaseImage(&(d->params.dstframe));
	if (d->params.scratchbuf)
		cvReleaseMemStorage(&(d->params.scratchbuf));
}

int detect_run(struct detector *d)
{
	CvSeq* faces;

	if (!d->params.scratchbuf)
		return -ENOMEM;

	cvClearMemStorage(d->params.scratchbuf);
	d->params.dstframe = cvCreateImage(cvSize(d->params.srcframe->width,
						  d->params.srcframe->height),
					   d->params.srcframe->depth,
					   d->params.srcframe->nChannels);
	if (d->params.dstframe == NULL)
		return -ENOMEM;

	switch(d->params.odt) {
	case CDT_HAAR:
		/* grey image only be needed for Haar */
		cvCvtColor(d->params.srcframe->imageData,
			   d->params.dstframe->imageData,
			   CV_BGR2GRAY);
		cvNamedWindow("FLL Grey", CV_WINDOW_AUTOSIZE);
		cvShowImage("FLL Grey", (CvArr*)(d->params.dstframe));
		faces = cvHaarDetectObjects(d->params.dstframe,
					    (CvHaarClassifierCascade*)(
						    d->params.algorithm),
					    d->params.scratchbuf,
					    1.2, /*default scale factor: 1.1*/
					    2,   /*default min neighbors: 3*/
					    CV_HAAR_DO_CANNY_PRUNING,
					    cvSize(60, 60),  /*min size*/
					    cvSize(180, 180) /*max size*/	); 
		break;
	case CDT_LSVM:
		faces =	cvLatentSvmDetectObjects(d->params.dstframe,
						 (CvLatentSvmDetector*)(
							 d->params.algorithm),
						 d->params.scratchbuf,
						 0.15f, /* overlap threshold */
						 -1     /* threads number*/ );
		break;
	default:
		faces = 0;
	};
	if (!faces)
		printf("No face, cdt=%d.\n", d->params.odt);
	else
		detect_store(faces, d->params.dstframe, 1);
	return 0;

}

static CvSeq* detect_run_Haar_algorithm(IplImage* frame,
					CvMemStorage* const buf,
					void *algo)
{ 
	CvSeq* faces;
	
	if (!buf) 
		return 0;

	cvClearMemStorage(buf);	
	faces = cvHaarDetectObjects(frame,
				    (CvHaarClassifierCascade*)algo,
				    buf,
				    1.1, /*default scale factor*/
				    3,   /*default min neighbors*/
				    CV_HAAR_DO_CANNY_PRUNING,
				    cvSize(60, 60),  /*min size*/
				    cvSize(180, 180) /*max size*/
		); 
	return faces;  
}

static CvSeq* detect_run_latentSVM_algorithm(IplImage* frame,
					     CvMemStorage* const buf,
					     void *algo)
{
	CvSeq* faces;
	
	if (!buf) 
		return 0;

	cvClearMemStorage(buf);	
	faces = cvLatentSvmDetectObjects(frame,
					 (CvLatentSvmDetector*)algo,
					 buf,
					 0.15f, /* default overlap threshold */
					 -1     /* default number of threads */
		);
	return faces;
}

static int detect_store(CvSeq* faces, IplImage* img, int scale)
{
	int i;
	CvPoint ptA, ptB;
	struct store_box *bbpos;
	if (!faces)
		return -EINVAL;
	
	bbpos = memalign(sizeof(struct store_box), faces->total);
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
		if (!bbpos)
			continue;
		bbpos[i].ptA_x = ptA.x;
		bbpos[i].ptA_y = ptA.y;
		bbpos[i].ptB_x = ptA.x;
		bbpos[i].ptB_y = ptA.y;
	}
	cvShowImage("FLL detection", (CvArr*)img);
	return 0;
}

#else

int detect_initialize(struct detector *d, struct detector_params *p)
{
	return -ENODEV;
}
	
int detect_run(struct detector *d)
{
	return -EINVAL;
}
	
void detect_teardown(struct detector *d)
{
	return;
}

#endif 

int detect_print_stats(struct detector *d)
{
	return 0;
}

int detect_get_objcount(struct detector *d)
{
	return 0;
}

