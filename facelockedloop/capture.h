#ifndef __CAPTURE_H_
#define __CAPTURE_H_

#ifdef __cplusplus
extern "C" {
#endif

int capture_initialize(int vindex);
	
#if defined(HAVE_OPENCV2)
	int capture_run(IplImage* srcframe, int frame_idx);
#else 
	int capture_run(void* srcframe, int frame_idx);
#endif
	
void capture_teardown(void);

int capture_get_vindex(void);

#ifdef __cplusplus
}
#endif

#endif
