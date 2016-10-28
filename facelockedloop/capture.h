#ifndef __CAPTURE_H_
#define __CAPTURE_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(HAVE_OPENCV2)
#include "highgui/highgui_c.h"

struct imager_params {
	char* name;
	int vididx;
	int frameidx;
	IplImage* frame;
	CvCapture* videocam;
};

#else
struct imager_params {
	char* name;
	int vididx;
	int frameidx;
	void* frame;
	void* videocam;
};

#endif

struct imager_stats {
	int tally;
	int fps;
};

struct imager {
	struct stage *step;
	struct imager_params params;
	struct imager_stats stats;
	int status;
};

int capture_initialize(struct imager *i, struct imager_params *p);
int capture_run(struct imager *i);
void capture_teardown(struct imager *i);
int capture_get_imgcount(struct imager *i);
int capture_print_stats (struct imager *i);
  
#ifdef __cplusplus
}
#endif

#endif
