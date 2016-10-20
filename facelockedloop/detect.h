#ifndef __DETECT_H_
#define __DETECT_H_

#ifdef __cplusplus
extern "C" {
#endif

enum object_detector_t {
	CDT_HAAR = 0,
	CDT_LSVM = 1,
	CDT_UNKNOWN = 2,
};

struct store_box;
	
#if defined(HAVE_OPENCV2)
	int detect_initialize(IplImage* srcframe, enum object_detector_t cdt);
#else
	int detect_initialize(void* srcframe, enum object_detector_t cdt);
#endif
	
int detect_teardown(enum object_detector_t cdt);

int detect_run(struct store_box *boxes, const IplImage* srcframe,
	       enum object_detector_t cdt);

int detect_get_objcount(void);

#ifdef __cplusplus
}
#endif
