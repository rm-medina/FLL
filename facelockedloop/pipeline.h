#ifndef __PIPELINE_H_
#define __PIPELINE_H_

#include <pthread.h>
#include <semaphore.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PIPELINE_MAX_STAGE 3
#define CAPTURE_STAGE 0
#define DETECTION_STAGE 1
#define TRACKING_STAGE 2

struct pipeline;
struct stage;


#define STAGE_ABRT 0x1 /*abort received*/
	
struct stage_params {
	const char *name;
	int nth_stage;
	void *data_in;
	void *data_out;
};

struct stage_ops {
	void (*up)(struct stage *stg, struct stage_params *p,
		  struct stage_ops *o, struct pipeline *pipe);
	int (*run)(struct stage *stg);
	void (*down)(struct stage *stg);
	void (*go)(struct stage *stg);
	void (*wait)(struct stage *stg);
	int (*output)(struct stage *stg, void *it);
	int (*input)(struct stage *stg, void **it);
	int (*count)(struct stage *step);
	void (*printstats)(struct stage *step);
};

struct stage {
	struct stage *self;
	struct stage_ops *ops;
	struct stage_params params;
	struct pipeline *pipeline;
	struct stage* next;
	struct timespec duration;
	pthread_t worker;
	pthread_mutex_t lock;
	pthread_cond_t sync;
	sem_t nowait;
	sem_t done;
	int flags;
	struct performance {
		struct timespec lastrun;
		struct timespec overall;
		unsigned long ofinterest;
		unsigned long persecond;
	} stats;
};

void stage_up(struct stage *stg,  struct stage_params *p,
	     struct stage_ops *o, struct pipeline *pipe);
void stage_down(struct stage *stg);	
void stage_go(struct stage *stg);
void stage_wait(struct stage *stg); 
int stage_output(struct stage *stg, void *it);
int stage_input(struct stage *stg, void **it);
void stage_printstats(struct stage *stg);

struct pipeline {
	struct stage *stgs[PIPELINE_MAX_STAGE];
	int count;
	int status;
};

void pipeline_init(struct pipeline *pipe);
int pipeline_register(struct pipeline *pipe, struct stage *stg);
int pipeline_deregister(struct pipeline *pipe, struct stage *stg);
void pipeline_teardown(struct pipeline *pipe);
int pipeline_run(struct pipeline *pipe);
int pipeline_pause(struct pipeline *pipe);
int pipeline_printstats(struct pipeline *pipe);
int pipeline_getcount(struct pipeline *pipe);
void pipeline_terminate(struct pipeline *pipe, int reason);
	
#ifdef __cplusplus
}
#endif

#endif  /* __PIPELINE_H_ */
