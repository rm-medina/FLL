#ifndef __PIPELINE_H_
#define __PIPELINE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define PIPELINE_MAX_STAGE 3
#define CAPTURE_STAGE 0
#define DETECTIGON_STAGE 1
#define TRACKING_STAGE 2

struct detector;
struct pipeline {
	struct stage stgs[PIPELINE_MAX_STAGE];
	int count;
	int status;
};

void pipeline_init(struct pipeline *pipe);
int pipeline_register(struct pipeline *pipe, struct stage *stg);
void pipeline_teardown(struct pipeline *pipe);
int pipeline_run(struct pipeline *pipe);
int pipeline_printstats(struct pipeline *pipe);
int pipeline_getcount(struct pipeline *pipe);

struct stage_ops {
	int (*up)(struct stage *stg, const struct stage_params *p,
		  struct stage_ops *o, struct pipeline *pipe);
	int (*run)(struct stage *stg);
	void (*down)(struct stage *stg);
	void (*go)(struct stage *stg);
	void (*wait)(struct stage *stg);
	int (*passit)(struct stage *stg, void *it);
	int (*processit)(struct stage *stg, void **it);
	int (*count)(struct stage *step);
	void (*printstats)(struct stage *step);
};

struct stage_params {
	int nth_stage;
	void *data_in;
	void *data_out;
};

struct stage {
	struct stage *self;
	struct stage_ops *ops;
	struct stage_params *params;
	struct pipeline *pipeline;
	struct stage* next;
	struct timespec duration;
	pthread_t worker;
	sem_t *nowait;
	sem_t *done;
	struct performance {
		struct timespec lastrun;
		unsigned long nofinterest;
		unsigned long persecond;
	} stats;
};

int stage_up(struct stage *stg, const struct stage_params *p,
	     struct stage_ops *o, struct pipeline *pipe);
int stage_down(struct stage *stg);	
void stage_go(struct stage *stg);
void stage_wait(struct stage *stg); 
int stage_passit(struct stage *stg, void *it);
int stage_processit(struct stage *stg, void **it);
void stage_printstats(struct stage *stg);
	
#ifdef __cplusplus
}
#endif

#endif  /* __PIPELINE_H_ */
