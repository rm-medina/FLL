#ifndef __TRACK_H_
#define __TRACK_H_

#include "pipeline.h"
#include "servolib.h"
#include "store.h"

#ifdef __cplusplus
extern "C" {
#endif



struct tracker_stats {
	struct servo_stats pan_stats;
	struct servo_stats tilt_stats;
};

struct tracker_params {
	const char *name;
	int dev;
	int pan_tgt;
	int tilt_tgt;
	struct servo_params pan_params;
	struct servo_params tilt_params;
	struct store_box bbox;
};

struct tracker {
	struct stage step;
	struct tracker_params params;
	struct tracker_stats stats;
	int status;
};

int track_initialize(struct tracker *t, struct tracker_params *p,
		     struct pipeline *pipe);
void track_teardown(struct tracker *t);
int track_run(struct tracker *t);
int track_print_stats(struct tracker *t);
  
#ifdef __cplusplus
}
#endif

#endif /* __TRACK_H_ */
