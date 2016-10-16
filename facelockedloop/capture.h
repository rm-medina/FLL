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

struct capture_box {
	int ptA_x;
	int ptA_y;
	int ptB_x;
	int ptB_y;
};

int capture_process(enum capture_detector_t cdt, int scale);
int capture_get_facecount(void);
int capture_read_locations(struct capture_box *face);

#ifdef __cplusplus
}
#endif

#endif
