#ifndef __CAPTURE_H_
#define __CAPTURE_H_

#ifdef __cplusplus
extern "C" {
#endif

enum capture_detector_t {
	CDT_HAAR = 0,
	CDT_LSVM = 1,
	CDT_UNKNOWN = 2,
};

int capture_process(int vindex, enum capture_detector_t cdt, int scale);
int capture_get_facecount(void);

#ifdef __cplusplus
}
#endif

#endif