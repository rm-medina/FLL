#ifndef __DETECT_H_
#define __DETECT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "pipeline.h"

#if defined(HAVE_OPENCV2)
#include "highgui/highgui_c.h"
#endif
  
enum object_detector_t {
	CDT_HAAR = 0,
	CDT_LSVM = 1,
	CDT_UNKNOWN = 2,
};

#if defined(HAVE_OPENCV2)

struct detector_params {
	enum object_detector_t odt;
	char *cascade_xml;
	IplImage* srcframe;
	IplImage* dstframe;
	void *algorithm;
	CvMemStorage* scratchbuf;
	struct store_box *faceboxs;
	int min_size;
	int max_size;
};

#else
struct detector_params {
	enum object_detector_t odt;
	char *cascade_xml;
	void* srcframe;
	void* dstframe;
	void *algorithm;
	void *scratchbuf;
	struct store_box *faceboxs;
	int min_size;
	int max_size;
};

#endif

struct detector_stats {
	int frameidx;
	int facecount;
};

struct detector {
	struct stage step;
	struct detector_params params;
	struct detector_stats stats;
	int status;
};
  
int detect_initialize(struct detector *d, struct detector_params *p,
		      struct pipeline *pipe);
void detect_teardown(struct detector *d);
int detect_run(struct detector *d);
int detect_get_objcount(struct detector *d);
int detect_print_stats(struct detector *d);

#ifdef __cplusplus
}
#endif

#endif  /* __DETECT_H_ */
