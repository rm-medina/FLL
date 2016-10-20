#if defined(HAVE_OPENCV2)

#include "highgui/highgui_c.h"
#include "imgproc/imgproc_c.h"
#include "objdetect/objdetect.hpp"

int capture_process(int vindex, enum object_detector_t cdt, int scale)
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
		ret = capture_run(srcframe, n);
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
