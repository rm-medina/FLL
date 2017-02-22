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
//#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui_c.h"
//#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
//#include "opencv2/objdetect.hpp"
#include "opencv2/objdetect/objdetect.hpp"

static CvSeq* detect_run_Haar_algorithm(IplImage* frame,
					CvMemStorage* const buffer,
					void *algo);
static CvSeq* detect_run_latentSVM_algorithm(IplImage* frame,
					     CvMemStorage* const buffer,
					     void *algo);
static struct store_box* detect_store(CvSeq* faces, IplImage* img, int scale);
#endif

static void detect_stage_up(struct stage *stg, struct stage_params *p,
			     struct stage_ops *o,struct pipeline *pipe);
static void detect_stage_down(struct stage *stg);
static int detect_stage_run(struct stage *stg);
static void detect_stage_wait(struct stage *stg);
static void detect_stage_go(struct stage *stg);
static int detect_stage_output(struct stage *stg, void* it);
static int detect_stage_input(struct stage *stg, void** it);

static struct stage_ops detect_ops = {
	.up = detect_stage_up, 
	.down = detect_stage_down,
	.run = detect_stage_run,
	.wait = detect_stage_wait,
	.go = detect_stage_go,
	.output = detect_stage_output,
	.input = detect_stage_input,
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
	int ret;
	algo = container_of(stg, struct detector, step);
	if (!algo)
		return -EINVAL;
	
	ret = detect_run(algo);
	/* pass only first face detected to next stage */
	stg->params.data_out = &algo->params.faceboxs[0];
	stg->stats.ofinterest = algo->stats.facecount;
	
	return ret;
}

static void detect_stage_wait(struct stage *stg)
{
	stage_wait(stg);
}

static void detect_stage_go(struct stage *stg)
{
	stage_go(stg);
}

static int detect_stage_output(struct stage *stg, void* it)
{
	return stage_output(stg, stg->params.data_out);
}

static int detect_stage_input(struct stage *stg, void **it)
{
	void *itin = NULL;
	struct detector *algo;

	algo = container_of(stg, struct detector, step);
	stage_input(stg, &itin);

	algo->params.srcframe = itin;
	return 0;
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

	cvNamedWindow("FLL detection", CV_WINDOW_AUTOSIZE);

	d->params.scratchbuf = cvCreateMemStorage(0); /*block_size: 0->64K*/
	if (d->params.scratchbuf == NULL)
		return -ENOMEM;
	
	switch(d->params.odt) {
	case CDT_HAAR:
		if (p->cascade_xml == NULL)
			p->cascade_xml = "haarcascade_frontalface_default.xml";
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



	detect_stage_up(&d->step, &stgparams, &detect_ops, pipe);
	return ret;
}

void detect_teardown(struct detector *d)
{
	cvDestroyWindow("FLL detection");

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


	d->params.dstframe = cvCreateImage(cvSize(d->params.srcframe->width,
						  d->params.srcframe->height),
					   d->params.srcframe->depth, 1);
	if (d->params.dstframe == NULL)
		return -ENOMEM;

	switch(d->params.odt) {
	case CDT_HAAR:
		/* grey image only be needed for Haar */
		cvCvtColor(d->params.srcframe,
			   d->params.dstframe,
			   CV_BGR2GRAY);
		cvClearMemStorage(d->params.scratchbuf);
		faces = cvHaarDetectObjects(d->params.dstframe,
					    (CvHaarClassifierCascade*)(
						    d->params.algorithm),
					    d->params.scratchbuf,
					    1.2, /*default scale factor: 1.1*/
					    2,   /*default min neighbours: 3*/
					    CV_HAAR_DO_CANNY_PRUNING |
					    CV_HAAR_FIND_BIGGEST_OBJECT,
					    cvSize(d->params.min_size,
						   d->params.min_size),  
					    cvSize(d->params.max_size,
						   d->params.max_size) ); 
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
	if (!faces) {
		printf("No face struct, cdt=%d.\n", d->params.odt);
		d->stats.facecount = 0;
	}
	else {
		d->params.faceboxs = detect_store(faces, d->params.dstframe, 1);
		d->stats.facecount = faces->total;
	}
	
	cvShowImage("FLL detection", (CvArr*)(d->params.dstframe));
	cvWaitKey(10);
	cvReleaseImage(&d->params.dstframe);
	d->params.dstframe = NULL;
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
				    1.2, /*default scale factor: 1.1*/
				    2,   /*default min neighbors: 3*/
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

static struct store_box* detect_store(CvSeq* faces, IplImage* img, int scale)
{
	int i;
	CvPoint ptA, ptB;
	struct store_box *bbpos;

	if (!faces || (faces->total ==0))
		return NULL;
	
	bbpos = calloc(faces->total, sizeof(struct store_box));
	if (!bbpos)
		return NULL;
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
		bbpos[i].ptB_x = ptB.x;
		bbpos[i].ptB_y = ptB.y;
	}
	cvShowImage("FLL detection", (CvArr*)img);
	return bbpos;
}

#else

int detect_initialize(struct detector *d, struct detector_params *p, struct pipeline *pipe)
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

