#ifndef __TRACK_H_
#define __TRACK_H_

#include "servolib.h"

#ifdef __cplusplus
extern "C" {
#endif


struct tracker_stats {
	struct servo_stats pan_stats;
	struct servo_stats tilt_stats;
};

struct tracker_params {
	int pan_tgt;
	int tilt_tgt;
	struct servo_params pan_params;
	struct servo_params tilt_params;
};

struct tracker {
	struct stage *step;
	struct tracker_params params;
	struct tracker_stats stats;
	int dev;
	int status;
};

int track_initialize(struct tracker *t, struct tracker_params *p);
int track_teardown(struct tracker *t);
int track_run(struct tracker *t);
int track_get_max_abse(struct tracker *t);
int track_print_stats(struct tracker *t);
  
#ifdef __cplusplus
}
#endif

#endif /* __TRACK_H_ */
